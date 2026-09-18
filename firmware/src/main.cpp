/*
 * Bridge CAN "man-in-the-middle" - Arduino Nano + 2x MCP2515 (8 MHz)
 * ==================================================================
 * Fica NO MEIO do barramento (o barramento precisa ser CORTADO em dois
 * trechos). Repassa TODO o trafego de forma transparente nos dois
 * sentidos, EXCETO o frame 0x18F00F52, que e' lido, corrigido e reenviado.
 *
 * Papel de cada modulo:
 *   MCP2515 com CS no pino 10  ->  LEITURA  (lado do sensor/ECU original)
 *   MCP2515 com CS no pino  9  ->  SAIDA    (lado do diagnostico)
 *
 *   [Sensor/ECU] --> (CS10 LE) [Nano corrige] (CS9 ENVIA) --> [Diagnostico]
 *   Outros IDs: repassados nos dois sentidos, sem alteracao.
 *
 * Sensores no frame 0x18F00F52 (little-endian):
 *   Sensor 1 = DATA[0..1] = NOx (ppm)
 *   Sensor 2 = DATA[2..3] = Oxigenio (O2)
 *
 * A calibracao raw->diag e' FIXA (constantes abaixo) - so serve para exibir
 * o valor final e para avaliar os triggers de O2 em unidade de diagnostico.
 *
 * Correcao PROPORCIONAL (ganho unico):
 *   Um unico percentual "pct" controla os dois sensores:
 *     O2  (sensor 2): +g%  (aumenta)
 *     NOx (sensor 1): -g%  (diminui)
 *   raw_out = raw0 + (raw_lido - raw0) * (1 + (sinal*g*k)/100)
 *   'k' e' o fator de rampa 0..1 (ver abaixo).
 *
 * TRIGGER (atrelado ao oxigenio):
 *   A correcao so atua quando o O2 (diag) fica ABAIXO de 'o2_trigger'.
 *   Acima disso -> bypass (sem correcao).
 *
 * PISO DE O2 (segundo trigger, valor minimo de saida):
 *   Quando a correcao base nao segura o O2 corrigido acima de 'o2_floor',
 *   o ganho efetivo AUMENTA dinamicamente (O2 e NOx juntos) para o O2
 *   corrigido NAO baixar desse valor. Em vez de travar numa linha reta,
 *   o alvo OSCILA um pouco acima do piso (ver OSC_* abaixo).
 *
 * CORRECAO PROPORCIONAL A FAIXA (prop_band):
 *   Com prop_band = 1, o ganho e' proporcional a posicao do O2 LIDO dentro
 *   da faixa [piso, trigger]:
 *     O2 no trigger -> ganho 0 (correcao minima)
 *     O2 no piso    -> ganho cheio (cfg.pct)
 *     abaixo do piso -> satura no ganho cheio.
 *   Quanto mais perto do piso, MAIOR a correcao; mais perto do trigger,
 *   MENOR. Requer piso > 0 e trigger > piso (senao vira ganho fixo).
 *   O reforco do piso (oscilando) continua valendo POR CIMA: mesmo nesse
 *   modo, o O2 corrigido nao baixa do piso e o alvo segue oscilando.
 *
 * RAMPA:
 *   Ao ativar/desativar o trigger, o fator 'k' sobe/desce gradualmente
 *   (leva 'ramp_ms' milissegundos de 0 a 100%). ramp_ms = 0 -> instantaneo.
 *
 * PINO D3 (entrada, INPUT_PULLUP -> "negativo" = ligado ao GND = LOW):
 *   Override por hardware. Quando D3 esta em nivel BAIXO (negativo):
 *     d3_low_bypass = 1 -> forca BYPASS (totalmente sem correcao)
 *     d3_low_bypass = 0 -> forca CORRIGINDO (atua a correcao)
 *   Quando D3 esta em nivel ALTO (solto) -> vale o trigger de O2.
 *
 * Persistencia: config na EEPROM (sobrevive a desligar o Nano).
 *
 * CICLO DE PISO (Piso 2, oscilacao lenta no tempo):
 *   Alem do Piso 1 (o2_floor), existe um Piso 2 (o2_floor2) MAIS BAIXO. Quando
 *   esta corrigindo e 0 < Piso2 < Piso1, o piso em vigor (activeFloor) oscila
 *   no tempo: segura no Piso 1 por Y ms (floor_period_ms), desce em rampa ate'
 *   o Piso 2, segura la por X ms (floor_hold_ms) e volta ao Piso 1. Fora do
 *   trigger (bypass) o ciclo pausa e volta ao Piso 1. A senoide pequena do piso
 *   (OSC_*) continua sendo aplicada POR CIMA de activeFloor. Piso2 = 0 desliga
 *   o ciclo (fica so no Piso 1, comportamento antigo).
 *
 * Protocolo serial (115200):
 *   PC -> Nano:
 *     GET            pede a config atual
 *     PCT:<v>        ganho proporcional unico (ex.: PCT:20)
 *     TRG:<v>        limite de O2 do trigger (diag)
 *     FLR:<v>        piso 1 de O2 (valor minimo de saida)
 *     RMP:<v>        tempo de rampa em milissegundos
 *     D3:<0|1>       modo do D3 baixo (0=corrige, 1=bypass)
 *     PRP:<0|1>      correcao proporcional a faixa (1=on, 0=piso oscila)
 *     FL2:<v>        piso 2 de O2 (mais baixo; 0 = sem ciclo)
 *     HLD:<v>        tempo X: quanto segura no Piso 2 (ms)
 *     PER:<v>        tempo Y: quanto segura no Piso 1 entre mergulhos (ms)
 *   Nano -> PC:
 *     CFG pct,o2_trigger,o2_floor,ramp_ms,d3mode,prop_band,o2_floor2,hold_ms,period_ms
 *     T <s1_lido> <s1_corr> <s2_lido> <s2_corr> <estado> <k_milesimos>
 *        estado: 0=BYPASS 1=CORRIGINDO ; k_milesimos: 0..1000
 */

#include <Arduino.h>
#include <math.h>
#include <SPI.h>
#include <mcp_can.h>
#include <EEPROM.h>

// ---- Pinos ----
#if defined(ARDUINO_ARCH_ESP32)
  // ESP32-C3-MINI-1 (RISC-V). SPI compartilhado (default do C3):
  //   SCK = GPIO4 , MISO = GPIO5 , MOSI = GPIO6
  // Ligue os DOIS modulos MCP2515 nesses 3 fios; so o CS e' separado.
  // ATENCAO 3.3V: o MISO do modulo (5V) precisa de level-shift/divisor,
  // ou alimente o MCP2515 em 3.3V (e o TJA1050 em 5V). Ver README.
  const uint8_t CS_READ = 7;    // LEITURA (sensor/ECU)  -> GPIO7
  const uint8_t CS_SEND = 10;   // SAIDA (diagnostico)   -> GPIO10
  const uint8_t D3_PIN  = 3;    // override (INPUT_PULLUP) -> GPIO3
#else
  // Arduino Nano (5V) - pinagem original
  const uint8_t CS_READ = 10;   // LEITURA (sensor/ECU)
  const uint8_t CS_SEND = 9;    // SAIDA (diagnostico)
  const uint8_t D3_PIN  = 3;    // entrada de override (INPUT_PULLUP)
#endif

const uint32_t TARGET_ID  = 0x18F00F52;  // frame a corrigir (estendido)
const uint16_t TELEM_MS   = 100;         // periodo da telemetria
const uint32_t CFG_MAGIC  = 0x43414E37;  // "CAN7" - marca de EEPROM valida (cresceu: piso 2 + X/Y)

// ---- Calibracao FIXA raw -> diag (raw0, dref, rref) ----
const float NOX_RAW0 = 4000.0f,  NOX_DREF = 100.0f, NOX_RREF = 6000.0f;   // NOx
const float O2_RAW0  = 23500.0f, O2_DREF  = 9.5f,   O2_RREF  = 42000.0f;  // O2

// ---- Oscilacao do piso de O2 (evita linha reta no grafico) ----
const uint32_t OSC_PERIOD_MS = 2500;    // periodo da oscilacao
const float    OSC_AMP_FRAC  = 0.06f;   // amplitude = 6% acima do piso
const float    TWO_PI_F      = 6.2831853f;

MCP_CAN CAN_READ(CS_READ);
MCP_CAN CAN_SEND(CS_SEND);

struct Config {
  uint32_t magic;
  float    pct;            // ganho proporcional unico (%): O2 +g, NOx -g
  float    o2_trigger;     // corrige quando O2(diag) < o2_trigger
  float    o2_floor;       // O2 corrigido nao deve baixar disso (0 = desliga)
  uint32_t ramp_ms;        // tempo de rampa (ms) de 0 a 100%
  uint8_t  d3_low_bypass;  // 1: D3 baixo -> bypass ; 0: D3 baixo -> corrige
  uint8_t  prop_band;      // 1: ganho proporcional a faixa [piso, trigger]
  float    o2_floor2;      // Piso 2: piso mais baixo, atingido no ciclo (0 = desliga)
  uint32_t floor_hold_ms;  // tempo X: quanto segura no Piso 2 (conta ao atingi-lo)
  uint32_t floor_period_ms;// tempo Y: quanto segura no Piso 1 entre mergulhos
};

Config cfg;

// Estado ao vivo
uint16_t s1Read = 0, s1Corr = 0, s2Read = 0, s2Corr = 0;
bool hasData = false;
float lastO2Diag = 1e9f;      // O2 lido em diag (inicia "alto" -> bypass)
float k = 0.0f;               // fator de rampa 0..1
bool correctActive = false;   // alvo atual (trigger/D3): corrigindo?
unsigned long lastTelem = 0;
unsigned long lastRamp = 0;

// Ciclo do Piso 2 (Piso1 <-> Piso2 no tempo, so quando corrigindo)
enum FloorPhase { FP_HOLD1, FP_DOWN, FP_HOLD2, FP_UP };
uint8_t floorPhase = FP_HOLD1;
float activeFloor = 0.0f;            // piso em vigor no instante (o que o reforco usa)
unsigned long floorPhaseT0 = 0;      // inicio da fase atual (p/ contar X e Y)
unsigned long lastFloorMs = 0;

// Buffer serial
char rxBuf[48];
uint8_t rxLen = 0;

// Prototipos
void loadConfig();
void saveConfig();
void sendConfig();
void handleSerial();
void handleLine(char *line);
void pumpBus(MCP_CAN &in, MCP_CAN &out, bool applyCorrection);
uint16_t correct(uint16_t raw, float raw0, float signedPct);
float rawToDiag(uint16_t raw, float raw0, float dref, float rref);
void updateRamp();
void updateFloorCycle();

void setup() {
  Serial.begin(115200);
  pinMode(D3_PIN, INPUT_PULLUP);
#if defined(ARDUINO_ARCH_ESP32)
  // No ESP32 a EEPROM e' emulada na flash: precisa reservar espaco antes de usar
  EEPROM.begin(sizeof(Config) + 16);
  // Fixa os pinos SPI do C3 (SCK, MISO, MOSI) antes de iniciar os MCP2515
  SPI.begin(4, 5, 6);
#endif
  loadConfig();

  bool okR = (CAN_READ.begin(MCP_ANY, CAN_250KBPS, MCP_8MHZ) == CAN_OK);
  bool okS = (CAN_SEND.begin(MCP_ANY, CAN_250KBPS, MCP_8MHZ) == CAN_OK);
  while (!okR || !okS) {
    Serial.print(F("ERRO MCP2515: LEITURA="));
    Serial.print(okR ? F("OK") : F("FALHA"));
    Serial.print(F(" SAIDA="));
    Serial.println(okS ? F("OK") : F("FALHA"));
    delay(1000);
    okR = (CAN_READ.begin(MCP_ANY, CAN_250KBPS, MCP_8MHZ) == CAN_OK);
    okS = (CAN_SEND.begin(MCP_ANY, CAN_250KBPS, MCP_8MHZ) == CAN_OK);
  }
  CAN_READ.setMode(MCP_NORMAL);
  CAN_SEND.setMode(MCP_NORMAL);

  lastRamp = millis();
  lastFloorMs = floorPhaseT0 = millis();
  activeFloor = cfg.o2_floor;
  Serial.println(F("PRONTO: bridge CAN 250kbps ativo (CS10=leitura CS9=saida)"));
  sendConfig();
}

void loop() {
  updateRamp();
  updateFloorCycle();

  // Sensor -> Diagnostico (corrige o TARGET_ID)
  pumpBus(CAN_READ, CAN_SEND, true);
  // Diagnostico -> Sensor (repasse puro)
  pumpBus(CAN_SEND, CAN_READ, false);

  handleSerial();

  unsigned long now = millis();
  if (hasData && now - lastTelem >= TELEM_MS) {
    lastTelem = now;
    int kmil = (int)(k * 1000.0f + 0.5f);
    Serial.print(F("T "));
    Serial.print(s1Read); Serial.print(' ');
    Serial.print(s1Corr); Serial.print(' ');
    Serial.print(s2Read); Serial.print(' ');
    Serial.print(s2Corr); Serial.print(' ');
    Serial.print(correctActive ? 1 : 0); Serial.print(' ');
    Serial.println(kmil);
  }
}

// Atualiza o alvo (trigger de O2 + override do D3) e faz a rampa de 'k'.
void updateRamp() {
  unsigned long now = millis();
  unsigned long dt = now - lastRamp;
  lastRamp = now;

  bool d3low = (digitalRead(D3_PIN) == LOW);
  bool target;
  if (d3low) {
    target = cfg.d3_low_bypass ? false : true;   // override por hardware
  } else {
    target = (lastO2Diag < cfg.o2_trigger);       // trigger de oxigenio
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

// Le todos os frames de "in" e repassa para "out". Se applyCorrection e o
// frame for o TARGET_ID, aplica a correcao antes de reenviar.
void pumpBus(MCP_CAN &in, MCP_CAN &out, bool applyCorrection) {
  while (in.checkReceive() == CAN_MSGAVAIL) {
    unsigned long id;
    uint8_t ext = 0, len = 0, buf[8];
    if (in.readMsgBuf(&id, &ext, &len, buf) != CAN_OK) break;

    if (applyCorrection && ext && id == TARGET_ID && len >= 4) {
      uint16_t r1 = (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);   // NOx
      uint16_t r2 = (uint16_t)buf[2] | ((uint16_t)buf[3] << 8);   // O2

      float o2d = rawToDiag(r2, O2_RAW0, O2_DREF, O2_RREF);
      lastO2Diag = o2d;                       // alimenta o trigger

      // ganho base ajustado pelo piso de O2
      float g = cfg.pct;

      // Modo proporcional a faixa: escala o ganho pela posicao do O2 lido
      // dentro de [piso, trigger] -> no trigger 0 ; no piso pct ; abaixo do
      // piso satura em pct. Mais perto do piso, maior a correcao.
      if (cfg.prop_band && cfg.o2_floor > 0.0f && cfg.o2_trigger > cfg.o2_floor) {
        float p = (cfg.o2_trigger - o2d) / (cfg.o2_trigger - cfg.o2_floor);
        if (p < 0.0f) p = 0.0f;
        if (p > 1.0f) p = 1.0f;
        g = cfg.pct * p;
      }

      // Reforco do piso (com oscilacao): vale nos DOIS modos. Sobe o ganho
      // para o O2 corrigido NAO baixar do piso EM VIGOR (activeFloor, que o
      // ciclo do Piso 2 move no tempo), com o alvo oscilando um pouco acima
      // dele - inclusive quando a escala proporcional deixaria o ganho baixo
      // demais perto do trigger.
      if (activeFloor > 0.0f && o2d > 0.01f) {
        float ph  = (float)(millis() % OSC_PERIOD_MS) / (float)OSC_PERIOD_MS;
        float osc = 0.5f + 0.5f * sin(TWO_PI_F * ph);            // 0..1
        float tgt = activeFloor * (1.0f + OSC_AMP_FRAC * osc);   // piso..piso+6%
        float gNeeded = 100.0f * (tgt / o2d - 1.0f);
        if (gNeeded > g) g = gNeeded;         // reforca para nao baixar do piso
      }
      g *= k;                                  // aplica a rampa

      uint16_t c1 = correct(r1, NOX_RAW0, -g); // NOx: diminui
      uint16_t c2 = correct(r2, O2_RAW0,  +g); // O2 : aumenta
      buf[0] = c1 & 0xFF; buf[1] = (c1 >> 8) & 0xFF;
      buf[2] = c2 & 0xFF; buf[3] = (c2 >> 8) & 0xFF;

      s1Read = r1; s1Corr = c1;
      s2Read = r2; s2Corr = c2;
      hasData = true;
    }
    out.sendMsgBuf(id, ext, len, buf);
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
    cfg.pct           = 0;      // ganho proporcional
    cfg.o2_trigger    = 5;      // corrige quando O2 < 5
    cfg.o2_floor      = 3;      // O2 corrigido nao baixa de 3 (0 = desliga)
    cfg.ramp_ms       = 2000;   // 2 s de rampa
    cfg.d3_low_bypass = 1;      // D3 baixo -> bypass
    cfg.prop_band     = 0;      // 0 = piso oscila ; 1 = proporcional a faixa
    cfg.o2_floor2       = 0;    // Piso 2 desligado por padrao (ciclo inativo)
    cfg.floor_hold_ms   = 5000; // X: 5 s no Piso 2
    cfg.floor_period_ms = 60000;// Y: 60 s no Piso 1 entre mergulhos
    saveConfig();
  }
}

void saveConfig() {
  EEPROM.put(0, cfg);   // put usa update: so grava bytes que mudaram
#if defined(ARDUINO_ARCH_ESP32)
  EEPROM.commit();      // no ESP32 nada e' gravado ate' o commit
#endif
}

void sendConfig() {
  Serial.print(F("CFG "));
  Serial.print(cfg.pct, 3);        Serial.print(',');
  Serial.print(cfg.o2_trigger, 3); Serial.print(',');
  Serial.print(cfg.o2_floor, 3);   Serial.print(',');
  Serial.print(cfg.ramp_ms);       Serial.print(',');
  Serial.print(cfg.d3_low_bypass); Serial.print(',');
  Serial.print(cfg.prop_band);       Serial.print(',');
  Serial.print(cfg.o2_floor2, 3);    Serial.print(',');
  Serial.print(cfg.floor_hold_ms);   Serial.print(',');
  Serial.println(cfg.floor_period_ms);
}

// ---- Comandos serial ----
void handleSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (rxLen > 0) {
        rxBuf[rxLen] = '\0';
        handleLine(rxBuf);
        rxLen = 0;
      }
    } else if (rxLen < sizeof(rxBuf) - 1) {
      rxBuf[rxLen++] = c;
    } else {
      rxLen = 0;
    }
  }
}

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
