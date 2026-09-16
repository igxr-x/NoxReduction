/*
 * Bridge CAN "man-in-the-middle" - LilyGO T-2CAN (ESP32-S3) - SEM FIO
 * ==================================================================
 * Mesma logica do firmware do Arduino Nano / ESP32-C3, porem:
 *   - Um lado do barramento usa o MCP2515 (SPI) e o OUTRO lado usa o
 *     controlador CAN nativo (TWAI) do ESP32-S3. A T-2CAN tem os dois.
 *   - A configuracao/telemetria NAO vai mais pela serial: o ESP32 cria
 *     um Wi-Fi proprio (Access Point "BridgeCAN") e serve uma pagina web.
 *     Conecte o celular/notebook nessa rede e abra http://192.168.4.1
 *
 * Papel de cada canal (o barramento continua CORTADO em dois trechos):
 *   MCP2515 (SPI)  ->  LEITURA  (lado do sensor/ECU original)
 *   TWAI nativo    ->  SAIDA    (lado do diagnostico)
 *
 *   [Sensor/ECU] --> (MCP2515 LE) [T-2CAN corrige] (TWAI ENVIA) --> [Diag]
 *   Outros IDs: repassados nos dois sentidos, sem alteracao.
 *
 * TODA a matematica de correcao (ganho proporcional unico, trigger de O2,
 * piso oscilante, correcao proporcional a faixa, rampa e override do pino
 * de hardware) e' IDENTICA ao firmware original. So mudou a fiacao CAN e a
 * comunicacao (serial -> WebSocket sobre Wi-Fi). O "protocolo" de texto e'
 * o mesmo (GET/PCT/TRG/FLR/RMP/D3/PRP e as respostas CFG/T), apenas trafega
 * por WebSocket em vez da porta serial.
 *
 * Pinos da LilyGO T-2CAN (V1.0, cristal do MCP2515 = 16 MHz):
 *   SPI:  SCLK=12  MOSI=11  MISO=13
 *   MCP2515: CS=10  RST=9  INT=8   (INT nao e' usado - fazemos polling)
 *   TWAI (CAN nativo): TX=7  RX=6
 *   BOOT=0
 *   D3 override -> GPIO4 (INPUT_PULLUP; opcional - se nao ligar, fica solto)
 */

#include <Arduino.h>
#include <math.h>
#include <SPI.h>
#include <mcp_can.h>
#include <EEPROM.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include "driver/twai.h"

// ---------------------------------------------------------------------------
// Pinos da T-2CAN
// ---------------------------------------------------------------------------
const uint8_t PIN_SCLK = 12, PIN_MOSI = 11, PIN_MISO = 13;
const uint8_t MCP_CS = 10, MCP_RST = 9;    // MCP_INT (8) nao usado (polling)
const gpio_num_t TWAI_TX = GPIO_NUM_7, TWAI_RX = GPIO_NUM_6;
const uint8_t D3_PIN = 4;                  // override por hardware (opcional)

// ---------------------------------------------------------------------------
// Wi-Fi (Access Point proprio - nao precisa de roteador no veiculo)
// ---------------------------------------------------------------------------
const char *AP_SSID = "Starlink Labs";
const char *AP_PASS = "11021992";          // >= 8 caracteres; troque se quiser

const uint32_t TARGET_ID  = 0x18F00F52;    // frame a corrigir (estendido)
const uint16_t TELEM_MS   = 100;           // periodo da telemetria
const uint32_t CFG_MAGIC  = 0x43414E37;    // "CAN7" - marca de EEPROM valida (cresceu: piso 2 + X/Y)

// ---- Calibracao FIXA raw -> diag (raw0, dref, rref) ----
const float NOX_RAW0 = 4000.0f,  NOX_DREF = 100.0f, NOX_RREF = 6000.0f;   // NOx
const float O2_RAW0  = 23500.0f, O2_DREF  = 9.5f,   O2_RREF  = 42000.0f;  // O2

// ---- Oscilacao do piso de O2 (evita linha reta no grafico) ----
const uint32_t OSC_PERIOD_MS = 2500;
const float    OSC_AMP_FRAC  = 0.06f;
const float    TWO_PI_F      = 6.2831853f;

MCP_CAN CAN_MCP(MCP_CS);          // lado do sensor/ECU (LEITURA)
WebServer http(80);               // serve a pagina web
WebSocketsServer ws(81);          // config + telemetria em tempo real

struct Config {
  uint32_t magic;
  float    pct;            // ganho proporcional unico (%): O2 +g, NOx -g
  float    o2_trigger;     // corrige quando O2(diag) < o2_trigger
  float    o2_floor;       // O2 corrigido nao deve baixar disso (0 = desliga)
  uint32_t ramp_ms;        // tempo de rampa (ms) de 0 a 100%
  uint8_t  d3_low_bypass;  // 1: D3 baixo -> bypass ; 0: D3 baixo -> usa trigger
  uint8_t  prop_band;      // 1: ganho proporcional a faixa [piso, trigger]
  float    o2_floor2;      // Piso 2: piso mais baixo, atingido no ciclo (0 = desliga)
  uint32_t floor_hold_ms;  // tempo X: quanto segura no Piso 2 (conta ao atingi-lo)
  uint32_t floor_period_ms;// tempo Y: quanto segura no Piso 1 entre mergulhos
};

Config cfg;

// Estado ao vivo
uint16_t s1Read = 0, s1Corr = 0, s2Read = 0, s2Corr = 0;
bool hasData = false;
float lastO2Diag = 1e9f;
float k = 0.0f;
bool correctActive = false;
unsigned long lastTelem = 0;
unsigned long lastRamp = 0;

// Ciclo do Piso 2 (Piso1 <-> Piso2 no tempo, so quando corrigindo)
enum FloorPhase { FP_HOLD1, FP_DOWN, FP_HOLD2, FP_UP };
uint8_t floorPhase = FP_HOLD1;
float activeFloor = 0.0f;            // piso em vigor no instante (o que o reforco usa)
unsigned long floorPhaseT0 = 0;      // inicio da fase atual (p/ contar X e Y)
unsigned long lastFloorMs = 0;

// Prototipos
void loadConfig();
void saveConfig();
void sendConfig();
void handleLine(char *line);
uint16_t correct(uint16_t raw, float raw0, float signedPct);
float rawToDiag(uint16_t raw, float raw0, float dref, float rref);
void updateRamp();
void updateFloorCycle();

// Abstracao dos dois canais CAN (assinatura unica p/ o bridge) --------------
typedef bool (*RecvFn)(uint32_t &id, uint8_t &ext, uint8_t &len, uint8_t *buf);
typedef void (*SendFn)(uint32_t id, uint8_t ext, uint8_t len, uint8_t *buf);

bool mcpRecv(uint32_t &id, uint8_t &ext, uint8_t &len, uint8_t *buf) {
  if (CAN_MCP.checkReceive() != CAN_MSGAVAIL) return false;
  unsigned long lid = 0; uint8_t e = 0, l = 0;
  if (CAN_MCP.readMsgBuf(&lid, &e, &l, buf) != CAN_OK) return false;
  id = lid; ext = e; len = l; return true;
}
void mcpSend(uint32_t id, uint8_t ext, uint8_t len, uint8_t *buf) {
  CAN_MCP.sendMsgBuf(id, ext, len, buf);
}
bool twaiRecv(uint32_t &id, uint8_t &ext, uint8_t &len, uint8_t *buf) {
  twai_message_t m;
  if (twai_receive(&m, 0) != ESP_OK) return false;
  id = m.identifier; ext = m.extd; len = m.data_length_code;
  for (int i = 0; i < len && i < 8; i++) buf[i] = m.data[i];
  return true;
}
void twaiSend(uint32_t id, uint8_t ext, uint8_t len, uint8_t *buf) {
  twai_message_t m; memset(&m, 0, sizeof(m));
  m.identifier = id; m.extd = ext ? 1 : 0; m.data_length_code = len;
  for (int i = 0; i < len && i < 8; i++) m.data[i] = buf[i];
  twai_transmit(&m, pdMS_TO_TICKS(2));
}

void pumpBus(RecvFn recv, SendFn send, bool applyCorrection);
void onWsEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length);

// Pagina web (config + telemetria) - servida da flash ----------------------
extern const char INDEX_HTML[] PROGMEM;

void setup() {
  Serial.begin(115200);
  pinMode(D3_PIN, INPUT_PULLUP);
  pinMode(MCP_RST, OUTPUT); digitalWrite(MCP_RST, HIGH);  // MCP2515 fora do reset

  EEPROM.begin(sizeof(Config) + 16);   // EEPROM emulada na flash do ESP32
  loadConfig();

  // ---- MCP2515 (lado do sensor/ECU) : 250 kbps, cristal 16 MHz ----
  SPI.begin(PIN_SCLK, PIN_MISO, PIN_MOSI, MCP_CS);
  bool okMcp = (CAN_MCP.begin(MCP_ANY, CAN_250KBPS, MCP_16MHZ) == CAN_OK);
  while (!okMcp) {
    Serial.println(F("ERRO MCP2515 (verifique cristal 16MHz e fiacao)"));
    delay(1000);
    okMcp = (CAN_MCP.begin(MCP_ANY, CAN_250KBPS, MCP_16MHZ) == CAN_OK);
  }
  CAN_MCP.setMode(MCP_NORMAL);

  // ---- TWAI / CAN nativo (lado do diagnostico) : 250 kbps ----
  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(TWAI_TX, TWAI_RX, TWAI_MODE_NORMAL);
  twai_timing_config_t  t = TWAI_TIMING_CONFIG_250KBITS();
  twai_filter_config_t  f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  if (twai_driver_install(&g, &t, &f) != ESP_OK || twai_start() != ESP_OK) {
    Serial.println(F("ERRO TWAI (CAN nativo) - verifique TX=7/RX=6"));
  }

  // ---- Wi-Fi Access Point + servidores web ----
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.print(F("Wi-Fi AP \"")); Serial.print(AP_SSID);
  Serial.print(F("\" senha \"")); Serial.print(AP_PASS);
  Serial.print(F("\"  ->  http://")); Serial.println(WiFi.softAPIP());

  http.on("/", []() { http.send_P(200, "text/html", INDEX_HTML); });
  http.begin();
  ws.begin();
  ws.onEvent(onWsEvent);

  lastRamp = millis();
  lastFloorMs = floorPhaseT0 = millis();
  activeFloor = cfg.o2_floor;
  Serial.println(F("PRONTO: bridge CAN 250kbps (MCP2515=leitura  TWAI=saida)"));
}

void loop() {
  http.handleClient();
  ws.loop();
  updateRamp();
  updateFloorCycle();

  // Sensor(MCP2515) -> Diagnostico(TWAI): corrige o TARGET_ID
  pumpBus(mcpRecv, twaiSend, true);
  // Diagnostico(TWAI) -> Sensor(MCP2515): repasse puro
  pumpBus(twaiRecv, mcpSend, false);

  unsigned long now = millis();
  if (hasData && now - lastTelem >= TELEM_MS) {
    lastTelem = now;
    int kmil = (int)(k * 1000.0f + 0.5f);
    char line[64];
    snprintf(line, sizeof(line), "T %u %u %u %u %d %d",
             s1Read, s1Corr, s2Read, s2Corr, correctActive ? 1 : 0, kmil);
    ws.broadcastTXT(line);
  }
}

// Atualiza o alvo (trigger de O2 + override do D3) e faz a rampa de 'k'.
void updateRamp() {
  unsigned long now = millis();
  unsigned long dt = now - lastRamp;
  lastRamp = now;

  bool d3low = (digitalRead(D3_PIN) == LOW);
  bool target;
  if (d3low && cfg.d3_low_bypass) {
    target = false;                          // D3 baixo em modo Bypass: forca bypass total
  } else {
    // Normal (D3 solto) e tambem D3 baixo em modo "Corrigindo": segue o trigger de
    // O2 - so corrige quando O2 < trigger, entrando/saindo do bypass por ele.
    target = (lastO2Diag < cfg.o2_trigger);
  }
  correctActive = target;

  float goal = target ? 1.0f : 0.0f;
  if (cfg.ramp_ms == 0) {
    k = goal;
  } else {
    float step = (float)dt / (float)cfg.ramp_ms;
    if (k < goal)      { k += step; if (k > goal) k = goal; }
    else if (k > goal) { k -= step; if (k < goal) k = goal; }
  }
}

// Ciclo do Piso 2: enquanto esta corrigindo (dentro do trigger) e o Piso 2 for
// valido (0 < piso2 < piso1), o piso em vigor (activeFloor) oscila NO TEMPO:
//   HOLD1 -> segura no Piso 1 por Y (floor_period_ms)
//   DOWN  -> desce em rampa ate o Piso 2
//   HOLD2 -> segura no Piso 2 por X (floor_hold_ms; conta a partir de atingi-lo)
//   UP    -> sobe em rampa de volta ao Piso 1 -> recomeca (HOLD1)
// Fora do trigger (bypass) o ciclo pausa/zera: volta pro Piso 1. A rampa entre
// pisos reusa o mesmo tempo de rampa (ramp_ms). A oscilacao senoidal pequena do
// piso continua sendo aplicada POR CIMA de activeFloor (em pumpBus).
void updateFloorCycle() {
  unsigned long now = millis();
  unsigned long dt = now - lastFloorMs;
  lastFloorMs = now;

  float piso1 = cfg.o2_floor;
  float piso2 = cfg.o2_floor2;
  bool cycleOn = correctActive && piso1 > 0.0f && piso2 > 0.0f && piso2 < piso1;

  if (!cycleOn) {                 // sem ciclo: fica no Piso 1 (comportamento antigo)
    floorPhase = FP_HOLD1;
    floorPhaseT0 = now;
    activeFloor = piso1;
    return;
  }

  // passo da rampa do piso (unidades de O2/ms); ramp_ms==0 -> transicao instantanea
  float step = (cfg.ramp_ms == 0) ? 1e9f
             : (piso1 - piso2) * (float)dt / (float)cfg.ramp_ms;

  switch (floorPhase) {
    case FP_HOLD1:
      activeFloor = piso1;
      if (now - floorPhaseT0 >= cfg.floor_period_ms) floorPhase = FP_DOWN;  // passou Y
      break;
    case FP_DOWN:
      activeFloor -= step;
      if (activeFloor <= piso2) {              // atingiu o Piso 2
        activeFloor = piso2;
        floorPhase = FP_HOLD2;
        floorPhaseT0 = now;                    // X so comeca a contar agora
      }
      break;
    case FP_HOLD2:
      activeFloor = piso2;
      if (now - floorPhaseT0 >= cfg.floor_hold_ms) floorPhase = FP_UP;      // passou X
      break;
    case FP_UP:
      activeFloor += step;
      if (activeFloor >= piso1) {              // voltou ao Piso 1
        activeFloor = piso1;
        floorPhase = FP_HOLD1;
        floorPhaseT0 = now;                    // recomeca Y
      }
      break;
  }
}

// Le todos os frames de "recv" e repassa para "send". Se applyCorrection e o
// frame for o TARGET_ID, aplica a correcao antes de reenviar. (IDENTICO ao
// original - so a origem/destino agora sao funcoes de canal.)
void pumpBus(RecvFn recv, SendFn send, bool applyCorrection) {
  uint32_t id; uint8_t ext = 0, len = 0, buf[8];
  while (recv(id, ext, len, buf)) {
    if (applyCorrection && ext && id == TARGET_ID && len >= 4) {
      uint16_t r1 = (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);   // NOx
      uint16_t r2 = (uint16_t)buf[2] | ((uint16_t)buf[3] << 8);   // O2

      float o2d = rawToDiag(r2, O2_RAW0, O2_DREF, O2_RREF);
      lastO2Diag = o2d;

      float g = cfg.pct;

      if (cfg.prop_band && cfg.o2_floor > 0.0f && cfg.o2_trigger > cfg.o2_floor) {
        float p = (cfg.o2_trigger - o2d) / (cfg.o2_trigger - cfg.o2_floor);
        if (p < 0.0f) p = 0.0f;
        if (p > 1.0f) p = 1.0f;
        g = cfg.pct * p;
      }

      if (activeFloor > 0.0f && o2d > 0.01f) {
        float ph  = (float)(millis() % OSC_PERIOD_MS) / (float)OSC_PERIOD_MS;
        float osc = 0.5f + 0.5f * sin(TWO_PI_F * ph);
        float tgt = activeFloor * (1.0f + OSC_AMP_FRAC * osc);  // senoide por cima do piso ativo
        float gNeeded = 100.0f * (tgt / o2d - 1.0f);
        if (gNeeded > g) g = gNeeded;
      }
      g *= k;

      uint16_t c1 = correct(r1, NOX_RAW0, -g); // NOx: diminui
      uint16_t c2 = correct(r2, O2_RAW0,  +g); // O2 : aumenta
      buf[0] = c1 & 0xFF; buf[1] = (c1 >> 8) & 0xFF;
      buf[2] = c2 & 0xFF; buf[3] = (c2 >> 8) & 0xFF;

      s1Read = r1; s1Corr = c1;
      s2Read = r2; s2Corr = c2;
      hasData = true;
    }
    send(id, ext, len, buf);
  }
}

uint16_t correct(uint16_t raw, float raw0, float signedPct) {
  float out = raw0 + ((float)raw - raw0) * (1.0f + signedPct / 100.0f);
  if (out < 0)       out = 0;
  if (out > 65535.0) out = 65535.0;
  return (uint16_t)(out + 0.5f);
}

float rawToDiag(uint16_t raw, float raw0, float dref, float rref) {
  float den = rref - raw0;
  if (den == 0) den = 1;
  return ((float)raw - raw0) * dref / den;
}

// ---- EEPROM ----
void loadConfig() {
  EEPROM.get(0, cfg);
  if (cfg.magic != CFG_MAGIC) {
    cfg.magic         = CFG_MAGIC;
    cfg.pct           = 0;
    cfg.o2_trigger    = 5;
    cfg.o2_floor      = 3;
    cfg.ramp_ms       = 2000;
    cfg.d3_low_bypass = 1;
    cfg.prop_band     = 0;
    cfg.o2_floor2     = 0;        // Piso 2 desligado por padrao (ciclo inativo)
    cfg.floor_hold_ms   = 5000;   // X: 5 s no Piso 2
    cfg.floor_period_ms = 60000;  // Y: 60 s no Piso 1 entre mergulhos
    saveConfig();
  }
}

void saveConfig() {
  EEPROM.put(0, cfg);
  EEPROM.commit();      // no ESP32 nada e' gravado ate' o commit
}

// Envia a config atual para TODOS os clientes web conectados.
void sendConfig() {
  char line[128];
  snprintf(line, sizeof(line), "CFG %.3f,%.3f,%.3f,%lu,%u,%u,%.3f,%lu,%lu",
           cfg.pct, cfg.o2_trigger, cfg.o2_floor,
           (unsigned long)cfg.ramp_ms, cfg.d3_low_bypass, cfg.prop_band,
           cfg.o2_floor2, (unsigned long)cfg.floor_hold_ms,
           (unsigned long)cfg.floor_period_ms);
  ws.broadcastTXT(line);
}

// ---- Comandos (mesmos do firmware serial, agora via WebSocket) ----
void handleLine(char *line) {
  if (strcmp(line, "GET") == 0) { sendConfig(); return; }

  if (strncmp(line, "PCT:", 4) == 0) { cfg.pct        = atof(line + 4); saveConfig(); sendConfig(); return; }
  if (strncmp(line, "TRG:", 4) == 0) { cfg.o2_trigger = atof(line + 4); saveConfig(); sendConfig(); return; }
  if (strncmp(line, "FLR:", 4) == 0) { cfg.o2_floor   = atof(line + 4); saveConfig(); sendConfig(); return; }
  if (strncmp(line, "RMP:", 4) == 0) { cfg.ramp_ms    = (uint32_t)atol(line + 4); saveConfig(); sendConfig(); return; }
  if (strncmp(line, "D3:", 3) == 0)  { cfg.d3_low_bypass = (atoi(line + 3) != 0) ? 1 : 0; saveConfig(); sendConfig(); return; }
  if (strncmp(line, "PRP:", 4) == 0) { cfg.prop_band     = (atoi(line + 4) != 0) ? 1 : 0; saveConfig(); sendConfig(); return; }
  if (strncmp(line, "FL2:", 4) == 0) { cfg.o2_floor2       = atof(line + 4); saveConfig(); sendConfig(); return; }
  if (strncmp(line, "HLD:", 4) == 0) { cfg.floor_hold_ms   = (uint32_t)atol(line + 4); saveConfig(); sendConfig(); return; }
  if (strncmp(line, "PER:", 4) == 0) { cfg.floor_period_ms = (uint32_t)atol(line + 4); saveConfig(); sendConfig(); return; }
}

// Evento do WebSocket: ao conectar manda a config; texto recebido = comando.
void onWsEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length) {
  if (type == WStype_CONNECTED) {
    sendConfig();
  } else if (type == WStype_TEXT) {
    char buf[48];
    size_t n = length < sizeof(buf) - 1 ? length : sizeof(buf) - 1;
    memcpy(buf, payload, n);
    buf[n] = '\0';
    handleLine(buf);
  }
}

// ---------------------------------------------------------------------------
// Pagina web (mesma UI do app.py, agora no navegador via WebSocket)
// ---------------------------------------------------------------------------
const char INDEX_HTML[] PROGMEM = R"HTMLPAGE(<!DOCTYPE html>
<html lang="pt-br"><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Bridge CAN 18F00F52</title>
<style>
  :root{ --bg:#12151a; --card:#1c2129; --ink:#e8ebf0; --mut:#9aa4b2; --line:#2b323d; }
  *{box-sizing:border-box}
  body{margin:0;background:var(--bg);color:var(--ink);font:15px/1.4 "Segoe UI",system-ui,sans-serif}
  .wrap{max-width:640px;margin:0 auto;padding:14px}
  h1{font-size:17px;margin:6px 0 12px}
  .conn{font-size:13px;color:var(--mut)}
  .conn b{color:#e04}
  .conn.ok b{color:#2ecc71}
  .banner{border-radius:10px;padding:16px;text-align:center;font-size:22px;font-weight:800;
          letter-spacing:.5px;background:#333;color:#bbb;margin:10px 0}
  .card{background:var(--card);border:1px solid var(--line);border-radius:10px;padding:12px;margin:10px 0}
  .card h2{font-size:13px;text-transform:uppercase;letter-spacing:.5px;color:var(--mut);margin:0 0 10px}
  .read{display:flex;justify-content:space-between;font-family:Consolas,monospace;padding:3px 0}
  .read .v{font-weight:700}
  .read .c{color:#2ecc71}
  .row{display:flex;align-items:center;gap:8px;margin:8px 0;flex-wrap:wrap}
  .row label{flex:1;min-width:150px;color:var(--mut)}
  input,select{background:#0e1116;color:var(--ink);border:1px solid var(--line);border-radius:6px;
               padding:6px 8px;font-size:14px}
  input{width:90px}
  button{background:#2b6cff;color:#fff;border:0;border-radius:6px;padding:6px 12px;cursor:pointer;font-size:14px}
  button:hover{filter:brightness(1.1)}
  button.sec{background:#333c4a}
  .hint{color:var(--mut);font-size:12px}
  .warn{color:#ffb020;font-size:13px;min-height:16px;margin-top:4px}
  canvas{width:100%;height:170px;background:#0e1116;border-radius:8px}
</style></head>
<body><div class="wrap">
  <h1>Bridge CAN 18F00F52 &mdash; corretor de sensores</h1>
  <div id="conn" class="conn">status: <b>conectando&hellip;</b></div>

  <div id="banner" class="banner">AGUARDANDO</div>

  <div class="card">
    <h2>Leitura (bancada)</h2>
    <div class="read"><span>Oxigenio (O2) &mdash; Lido</span><span class="v" id="o2r">---</span></div>
    <div class="read"><span>Oxigenio (O2) &mdash; Corrigido</span><span class="v c" id="o2c">---</span></div>
    <div class="read"><span>NOx (ppm) &mdash; Lido</span><span class="v" id="nor">---</span></div>
    <div class="read"><span>NOx (ppm) &mdash; Corrigido</span><span class="v c" id="noc">---</span></div>
    <canvas id="chart" width="600" height="170"></canvas>
    <div class="hint">grafico do O2 &mdash; azul = lido, verde = corrigido (ultimas amostras)</div>
  </div>

  <div class="card">
    <h2>Configuracoes (gravam na EEPROM)</h2>
    <div class="row"><label>Ganho proporcional (%) &mdash; O2 +X% / NOx &minus;X%</label>
      <input id="pct" type="number" step="any"><button onclick="ap('PCT',pct.value)">Aplicar</button></div>
    <div class="row"><label>Trigger O2 (corrige se O2 &lt;)</label>
      <input id="trg" type="number" step="any"><button onclick="apTrg()">Aplicar</button></div>
    <div class="row"><label>Piso 1 O2 (nao baixar de; 0 = desliga)</label>
      <input id="flr" type="number" step="any"><button onclick="apFlr()">Aplicar</button></div>
    <div class="row"><label>Piso 2 O2 (mais baixo; 0 = sem ciclo)</label>
      <input id="flr2" type="number" step="any"><button onclick="apFlr2()">Aplicar</button></div>
    <div class="row"><label>Tempo X &mdash; segura no Piso 2 (ms)</label>
      <input id="hld" type="number" step="1"><button onclick="ap('HLD',Math.round(hld.value||0))">Aplicar</button></div>
    <div class="row"><label>Tempo Y &mdash; intervalo no Piso 1 (ms)</label>
      <input id="per" type="number" step="1"><button onclick="ap('PER',Math.round(per.value||0))">Aplicar</button></div>
    <div class="row"><label>Rampa (ms)</label>
      <input id="rmp" type="number" step="1"><button onclick="ap('RMP',Math.round(rmp.value||0))">Aplicar</button></div>
    <div class="row"><label>Correcao</label>
      <select id="prp"><option value="0">Fixa (piso oscila)</option><option value="1">Proporcional a faixa</option></select>
      <button onclick="apPrp()">Aplicar</button></div>
    <div class="row"><label>D3 negativo (pino em nivel baixo)</label>
      <select id="d3"><option value="1">Bypass</option><option value="0">Corrigindo</option></select>
      <button onclick="ap('D3',d3.value)">Aplicar</button></div>
    <div id="warn" class="warn"></div>
  </div>

  <div class="card">
    <h2>Log (gravado no navegador)</h2>
    <div class="row">
      <button id="logbtn" class="sec" onclick="toggleLog()">Gravar log</button>
      <span id="logst" class="hint">log parado</span>
    </div>
    <div class="hint">gera um .csv (mesmas colunas do app antigo) para abrir no log_viewer.html</div>
  </div>
</div>
<script>
// Calibracao FIXA raw -> valor exibido (igual ao firmware)
const CAL_O2=[23500,9.5,42000], CAL_NOX=[4000,100,6000];
const d2=(raw,c)=>(raw-c[0])*c[1]/((c[2]-c[0])||1);
const $=id=>document.getElementById(id);

let ws, hist=[]; // hist: {r,c} do O2 p/ o grafico
let log=null, logT0=0, logN=0, rows=[];

function connect(){
  ws=new WebSocket('ws://'+location.hostname+':81/');
  ws.onopen =()=>{ setConn(true); ws.send('GET'); };
  ws.onclose=()=>{ setConn(false); setTimeout(connect,1500); };
  ws.onmessage=e=>handle(e.data);
}
function setConn(ok){ const c=$('conn'); c.className='conn'+(ok?' ok':'');
  c.innerHTML='status: <b>'+(ok?'conectado':'reconectando...')+'</b>'; }

function handle(line){
  if(line.startsWith('T ')){
    const p=line.split(' '); if(p.length!==7) return;
    const s1r=+p[1],s1c=+p[2],s2r=+p[3],s2c=+p[4],st=+p[5],kmil=+p[6];
    const o2r=d2(s2r,CAL_O2),o2c=d2(s2c,CAL_O2),nor=d2(s1r,CAL_NOX),noc=d2(s1c,CAL_NOX);
    $('o2r').textContent=o2r.toFixed(2); $('o2c').textContent=o2c.toFixed(2);
    $('nor').textContent=nor.toFixed(1); $('noc').textContent=noc.toFixed(1);
    banner(st,kmil);
    hist.push({r:o2r,c:o2c}); if(hist.length>240)hist.shift(); draw();
    if(log) logSample(o2r,o2c,nor,noc,st,kmil);
  } else if(line.startsWith('CFG ')){
    const v=line.slice(4).split(',').map(Number);
    $('pct').value=v[0]; $('trg').value=v[1]; $('flr').value=v[2];
    $('rmp').value=Math.round(v[3]);
    $('d3').value = v[4]?'1':'0';               // 1=Bypass, 0=Corrigindo
    if(v.length>=6) $('prp').value = v[5]?'1':'0';
    if(v.length>=9){ $('flr2').value=v[6]; $('hld').value=Math.round(v[7]); $('per').value=Math.round(v[8]); }
    checkBand();
  }
}
function banner(st,kmil){
  const b=$('banner');
  if(st===1){ let t='CORRIGINDO'; if(kmil<999)t+=' (rampa '+(kmil/10).toFixed(0)+'%)';
              b.textContent=t; b.style.background='#0a7d2c'; b.style.color='#fff'; }
  else{ let t='BYPASS'; if(kmil>1)t+=' (rampa '+(kmil/10).toFixed(0)+'%)';
        b.textContent=t; b.style.background='#b06a00'; b.style.color='#fff'; }
}
function ap(cmd,val){ if(ws&&ws.readyState===1) ws.send(cmd+':'+val); }
function apTrg(){ ap('TRG',$('trg').value); checkBand(); }
function apFlr(){ ap('FLR',$('flr').value); checkBand(); }
function apFlr2(){ ap('FL2',$('flr2').value); checkBand(); }
function apPrp(){ ap('PRP',$('prp').value); if($('prp').value==='1')checkBand(); }
function checkBand(){
  const trg=+$('trg').value, flr=+$('flr').value, flr2=+$('flr2').value;
  let msg='';
  if(flr>0 && flr>trg)
    msg='Atencao: Piso 1 ('+flr+') maior que o trigger ('+trg+') - faixa invalida (use piso < trigger).';
  else if(flr2>0 && flr2>=flr)
    msg='Atencao: Piso 2 ('+flr2+') deve ser MENOR que o Piso 1 ('+flr+') - o ciclo so vale com Piso 2 < Piso 1.';
  $('warn').textContent=msg;
}

// grafico simples do O2 (lido x corrigido)
function draw(){
  const cv=$('chart'), x=cv.getContext('2d'), W=cv.width, H=cv.height;
  x.clearRect(0,0,W,H); if(hist.length<2)return;
  let lo=1e9,hi=-1e9; hist.forEach(p=>{lo=Math.min(lo,p.r,p.c);hi=Math.max(hi,p.r,p.c);});
  if(hi-lo<0.5){hi+=0.5;lo-=0.5;} const pad=(hi-lo)*0.1; lo-=pad; hi+=pad;
  const X=i=>i/(hist.length-1)*W, Y=v=>H-(v-lo)/(hi-lo)*H;
  const plot=(key,col)=>{ x.strokeStyle=col; x.lineWidth=2; x.beginPath();
    hist.forEach((p,i)=>{ i?x.lineTo(X(i),Y(p[key])):x.moveTo(X(i),Y(p[key])); }); x.stroke(); };
  plot('r','#3b82f6'); plot('c','#2ecc71');
}

// log CSV no navegador
function toggleLog(){ log? stopLog(): startLog(); }
function startLog(){ rows=[]; logN=0; logT0=performance.now(); log=true;
  $('logbtn').textContent='Parar log'; $('logst').textContent='gravando... (0)'; }
function logSample(o2r,o2c,nor,noc,st,kmil){
  const t=(performance.now()-logT0)/1000, trg=+$('trg').value||0,
        flr=+$('flr').value||0, flr2=+$('flr2').value||0;
  rows.push([t.toFixed(3),o2r.toFixed(3),o2c.toFixed(3),nor.toFixed(3),noc.toFixed(3),
             trg,flr,flr2,st,(kmil/10).toFixed(1)].join(','));
  if(++logN%5===0) $('logst').textContent='gravando... ('+logN+')';
}
function stopLog(){ log=false; $('logbtn').textContent='Gravar log';
  const head='t_s,o2_read,o2_corr,nox_read,nox_corr,o2_trigger,o2_floor,o2_floor2,state,ramp_pct\n';
  const blob=new Blob([head+rows.join('\n')+'\n'],{type:'text/csv'});
  const a=document.createElement('a'); a.href=URL.createObjectURL(blob);
  const d=new Date(), p=n=>String(n).padStart(2,'0');
  a.download='log_'+d.getFullYear()+p(d.getMonth()+1)+p(d.getDate())+'_'+
             p(d.getHours())+p(d.getMinutes())+p(d.getSeconds())+'.csv';
  a.click(); $('logst').textContent='salvo ('+logN+' amostras)'; }

connect();
</script></body></html>)HTMLPAGE";
