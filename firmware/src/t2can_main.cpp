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
 *   >>> CAN A <-> CAN B INVERTIDO (a entrada do sensor mudou de lado) <<<
 *   TWAI nativo    ->  LEITURA  (lado do sensor/ECU original)
 *   MCP2515 (SPI)  ->  SAIDA    (lado do diagnostico)
 *
 *   [Sensor/ECU] --> (TWAI LE) [T-2CAN corrige] (MCP2515 ENVIA) --> [Diag]
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
// Entrada extra (INPUT_PULLUP; "negativo" = aterrar o pino = LOW):
const uint8_t RESET_PIN    = 15;  // negativo -> zera o contador de FURADAS do Piso 1
const uint8_t BOLINHAS_PIN = 16;  // SAIDA: normalmente HIGH; pulso negativo (LOW 500ms) a cada FURADA do Piso 1

// ---------------------------------------------------------------------------
// Wi-Fi (Access Point proprio - nao precisa de roteador no veiculo)
// ---------------------------------------------------------------------------
const char *AP_SSID = "Starlink Labs";
const char *AP_PASS = "11021992";          // >= 8 caracteres; troque se quiser

const uint32_t TARGET_ID  = 0x18F00F52;    // frame a corrigir (estendido)
const uint16_t TELEM_MS   = 100;           // periodo da telemetria
const uint32_t CAN_OK_MS  = 1500;          // sem frame nesse tempo -> CAN marcada "sem sinal"
const uint32_t CFG_MAGIC  = 0x43414E39;    // "CAN9" - marca de EEPROM valida (cresceu: can_timeout_ms)

// ---- Calibracao FIXA raw -> diag (raw0, dref, rref) ----
const float NOX_RAW0 = 4000.0f,  NOX_DREF = 100.0f, NOX_RREF = 6000.0f;   // NOx
const float O2_RAW0  = 23500.0f, O2_DREF  = 9.5f,   O2_RREF  = 42000.0f;  // O2

// ---- Oscilacao do piso de O2 (evita linha reta no grafico) ----
// Antes era quase uma senoide. Agora e' DOMINADA por ruido suave (value noise)
// com passo ALEATORIO + uma oitava rapida de textura por cima, deixando so um
// residuo de senoide. Fica bem menos previsivel/periodica, mas continua suave
// (smoothstep, sem saltos), com a MESMA amplitude (osc 0..1 aplicado como
// OSC_AMP_FRAC acima do piso) e sempre >= piso.
const uint32_t OSC_PERIOD_MS    = 2500;  // periodo da senoide residual
const float    OSC_AMP_FRAC     = 0.06f; // amplitude = 6% acima do piso
const uint32_t OSC_NOISE_MIN_MS = 900;   // passo do ruido lento: sorteado nessa faixa
const uint32_t OSC_NOISE_MAX_MS = 1600;  //   (passo variavel quebra a periodicidade)
const uint32_t OSC_FAST_MS      = 450;   // passo da oitava rapida (textura)
const float    OSC_SINE_MIX     = 0.25f; // peso da senoide residual; resto e' ruido lento
const float    OSC_FAST_AMP     = 0.22f; // amplitude da oitava rapida somada por cima
const float    TWO_PI_F         = 6.2831853f;

// ---- Oscilacao do NOx CONGELADO (quando bate o piso) ----
// Enquanto o reforco do piso atua (ver pumpBus), a conta -g levaria o NOx a
// valores loucos (ate negativos). Em vez disso o NOx fica preso no ultimo valor
// corrigido ANTES de bater o piso; esta oscilacao evita a linha reta no grafico.
const float    NOX_OSC_FRAC     = 0.02f; // amplitude = +/- 2% do valor congelado

// Um canal de value-noise suave: interpola (smoothstep) entre alvos aleatorios
// em [0,1]. "stepMs" e' re-sorteado a cada alvo entre [minMs,maxMs] (min==max ->
// passo fixo) para nao ficar periodico. O estado do canal vem por referencia
// (static no chamador), permitindo varios canais independentes.
float valueNoise(unsigned long now, unsigned long &t0, uint32_t &stepMs,
                 float &from, float &to, uint32_t minMs, uint32_t maxMs) {
  if (now - t0 >= stepMs) {
    t0     = now;
    from   = to;
    to     = (float)random(0, 1001) / 1000.0f;
    stepMs = (minMs >= maxMs) ? minMs
             : (uint32_t)random((long)minMs, (long)maxMs + 1);
  }
  float u = (float)(now - t0) / (float)stepMs;
  if (u > 1.0f) u = 1.0f;
  float s = u * u * (3.0f - 2.0f * u);                       // smoothstep
  return from + (to - from) * s;                             // 0..1
}

// Gera a oscilacao do piso (0..1): ruido lento (passo aleatorio) dominante +
// residuo de senoide + oitava rapida somada por cima. Clampa em [0,1].
float floorOsc() {
  unsigned long now = millis();

  float ph   = (float)(now % OSC_PERIOD_MS) / (float)OSC_PERIOD_MS;
  float sine = 0.5f + 0.5f * sin(TWO_PI_F * ph);             // 0..1

  static unsigned long sT0 = 0, fT0 = 0;
  static uint32_t sStep = OSC_NOISE_MIN_MS, fStep = OSC_FAST_MS;
  static float sFrom = 0.5f, sTo = 0.5f, fFrom = 0.5f, fTo = 0.5f;

  float slow = valueNoise(now, sT0, sStep, sFrom, sTo, OSC_NOISE_MIN_MS, OSC_NOISE_MAX_MS);
  float fast = valueNoise(now, fT0, fStep, fFrom, fTo, OSC_FAST_MS, OSC_FAST_MS);

  float osc = OSC_SINE_MIX * sine + (1.0f - OSC_SINE_MIX) * slow;  // base 0..1
  osc += OSC_FAST_AMP * (fast - 0.5f);                            // textura +/-
  if (osc < 0.0f) osc = 0.0f;
  if (osc > 1.0f) osc = 1.0f;
  return osc;
}

// Oscilacao do NOx congelado (0..1). Canal de value-noise PROPRIO (nao divide
// estado com o floorOsc do O2) para o NOx nao ficar oscilando colado no O2.
float noxOsc() {
  unsigned long now = millis();
  static unsigned long nT0 = 0;
  static uint32_t nStep = OSC_NOISE_MIN_MS;
  static float nFrom = 0.5f, nTo = 0.5f;
  return valueNoise(now, nT0, nStep, nFrom, nTo, OSC_NOISE_MIN_MS, OSC_NOISE_MAX_MS);
}

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
  uint32_t max_pierces;    // limite de FURADAS do Piso 1 (0 = sem limite); zera com RESET
  uint32_t can_timeout_ms; // sem frame do sensor por mais que isso -> marca TIMEOUT no log (0 = desliga)
};

Config cfg;

// Estado ao vivo
uint16_t s1Read = 0, s1Corr = 0, s2Read = 0, s2Corr = 0;
bool hasData = false;
uint16_t noxHeldRaw = 0;    // ultimo NOx corrigido (raw) antes de bater o piso
bool     noxHeld    = false;// ja ha um valor memorizado p/ congelar?
unsigned long lastSensorMs = 0;      // millis() do ultimo frame do sensor (TARGET_ID)
float lastO2Diag = 1e9f;
float k = 0.0f;
bool correctActive = false;
unsigned long lastTelem = 0;
unsigned long lastRamp = 0;

// Diagnostico do lado TWAI (CAN B / diagnostico): conta os frames recebidos do
// painel e deixa curar bus-off. Se twaiRxCount NAO sobe, o TWAI nao esta
// recebendo nada do lado B (HW/fiacao/transceiver ou bus-off).
volatile uint32_t twaiRxCount = 0;
unsigned long lastTwaiLog = 0;

// "Saude" de cada barramento para a UI: guarda o millis() do ultimo frame
// recebido em cada canal fisico. CAN A = MCP2515, CAN B = TWAI. Se nao chega
// frame ha mais que CAN_OK_MS, a CAN e' marcada como "sem sinal" na pagina.
volatile uint32_t mcpRxCount = 0;    // frames recebidos no MCP2515 (CAN A)
unsigned long lastMcpRxMs  = 0;      // millis() do ultimo frame no MCP2515 (CAN A)
unsigned long lastTwaiRxMs = 0;      // millis() do ultimo frame no TWAI    (CAN B)

// Recuperacao do MCP2515 (equivalente ao twaiHealth do lado TWAI). Com o CAN
// INVERTIDO o MCP virou o lado que TRANSMITE (diagnostico): se o tester nao da
// ACK (key-on / desconectado), os 3 buffers de TX enchem, o sendMsg fica girando
// ate TIMEOUTVALUE e o loop() nao roda o mcpRecv -> CAN A fica muda. O one-shot
// (setup) impede a tempestade de retransmissao; isto aqui cura o bus-off residual.
uint32_t mcpRecoverCount = 0;        // quantas vezes o MCP foi re-inicializado
unsigned long lastMcpHealth = 0;     // ultimo check de saude do MCP (throttle do SPI)

// ONE-SHOT DINAMICO do MCP (lado que TRANSMITE para o diagnostico).
// O one-shot tenta cada frame 1x e, falhando (perda de arbitragem / erro de bit),
// DESCARTA sem retransmitir. Isso evita a tempestade de retransmissao no key-on
// (tester sem dar ACK), mas EM MARCHA derruba o frame da sonda de vez em quando
// quando ele perde arbitragem no barramento cheio -> o tester estoura o timeout e
// acusa "sonda desconectada". Solucao: so manter o one-shot ENQUANTO o tester nao
// foi confirmado. Assim que chega trafego pelo lado MCP (= ha outro no' ativo, logo
// ha ACK e pode haver arbitragem concorrente), desliga o one-shot e volta a
// retransmitir normalmente. O mcpHealth (bus-off) continua sendo a rede de seguranca.
bool mcpOneShot = true;              // estado atual do bit OSM (setup liga = true)
const uint32_t MCP_ACK_ALIVE_MS = 500; // RX recente no MCP => tester vivo (ACK disponivel)

// Ciclo do Piso 2 (Piso1 <-> Piso2 no tempo, so quando corrigindo)
//   FP_WAIT: venceu o tempo Y -> AGUARDA o O2 encostar/furar o Piso 1 p/ mergulhar
enum FloorPhase { FP_HOLD1, FP_WAIT, FP_DOWN, FP_HOLD2, FP_UP };
uint8_t floorPhase = FP_HOLD1;
float activeFloor = 0.0f;            // piso em vigor no instante (o que o reforco usa)
unsigned long floorPhaseT0 = 0;      // inicio da fase atual (usado p/ contar X no Piso 2)
unsigned long yT0 = 0;               // inicio da contagem Y (Piso 1); so zera ao FURAR
unsigned long lastFloorMs = 0;
bool floorBiting = false;            // O2 corrigido batendo no Piso 1 (reforco atuando)

// Contadores (vao para a telemetria/log)
uint32_t biteCount = 0;              // quantas vezes BATEU no Piso 1 (borda de floorBiting)
uint32_t pierceCount = 0;            // quantas vezes FUROU o Piso 1 (mergulhos iniciados)
bool pierceBlocked = false;          // atingiu max_pierces -> segura no Piso 1 ate o RESET

// Entrada RESET (borda de descida = evento) + pulso latched p/ a telemetria
bool prevResetLow = false;
bool resetEvent = false;             // pulso: RESET disparado desde o ultimo telem

// Saida BOLINHAS: fica HIGH e cai p/ LOW por 500ms a cada FURADA do Piso 1.
const uint32_t BOLINHAS_PULSE_MS = 500;
unsigned long bolinhasPulseUntil = 0;  // millis() ate quando o pino fica LOW (0 = HIGH)
uint32_t prevPierceCount = 0;          // detecta nova furada p/ disparar o pulso

// Rede (Wi-Fi AP + web). Sobe SO DEPOIS do CAN ja estar repassando os frames,
// pra nao "roubar" a janela de key-on: o sensor NOx precisa do handshake de
// aquecimento (comando da ECU) logo ao ligar, e com o barramento CORTADO esse
// frame so cruza se o bridge ja estiver no ar. Bloquear o setup() subindo o
// softAP antes deixava os dois lados desconectados nesse instante critico.
bool netStarted = false;               // Wi-Fi/web ja foi inicializado?
unsigned long canReadyMs = 0;          // millis() em que o CAN ficou pronto
const uint32_t NET_START_DELAY_MS = 2500;  // ms repassando o CAN antes de subir o Wi-Fi

// Prototipos
void loadConfig();
void saveConfig();
void sendConfig();
void handleLine(char *line);
uint16_t correct(uint16_t raw, float raw0, float signedPct);
float rawToDiag(uint16_t raw, float raw0, float dref, float rref);
void updateRamp();
void updateFloorCycle();
void updateInputs();
void resetPierces();
void startNetwork();
void twaiHealth();
void mcpHealth();

// Abstracao dos dois canais CAN (assinatura unica p/ o bridge) --------------
typedef bool (*RecvFn)(uint32_t &id, uint8_t &ext, uint8_t &len, uint8_t *buf);
typedef void (*SendFn)(uint32_t id, uint8_t ext, uint8_t len, uint8_t *buf);

bool mcpRecv(uint32_t &id, uint8_t &ext, uint8_t &len, uint8_t *buf) {
  if (CAN_MCP.checkReceive() != CAN_MSGAVAIL) return false;
  unsigned long lid = 0; uint8_t e = 0, l = 0;
  if (CAN_MCP.readMsgBuf(&lid, &e, &l, buf) != CAN_OK) return false;
  mcpRxCount++; lastMcpRxMs = millis();  // chegou frame no lado CAN A (MCP2515)
  id = lid; ext = e; len = l; return true;
}
void mcpSend(uint32_t id, uint8_t ext, uint8_t len, uint8_t *buf) {
  CAN_MCP.sendMsgBuf(id, ext, len, buf);
}
bool twaiRecv(uint32_t &id, uint8_t &ext, uint8_t &len, uint8_t *buf) {
  twai_message_t m;
  if (twai_receive(&m, 0) != ESP_OK) return false;
  twaiRxCount++; lastTwaiRxMs = millis(); // chegou frame no lado CAN B (TWAI)
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

// Cura o TWAI se ele cair em bus-off (ex.: transmitiu dado do sensor no key-on
// sem ninguem dando ACK ainda no lado do diagnostico). Sem isso o controlador
// para TX e RX DE VEZ ate reiniciar - e o lado CAN B fica mudo o resto da
// sessao. Chamado a cada volta do loop().
void twaiHealth() {
  twai_status_info_t st;
  if (twai_get_status_info(&st) != ESP_OK) return;
  if (st.state == TWAI_STATE_BUS_OFF) {
    twai_initiate_recovery();            // inicia a recuperacao (128x11 bits recessivos)
  } else if (st.state == TWAI_STATE_STOPPED) {
    twai_start();                        // recuperou -> religa o controlador
  }
}

// Cura o MCP2515 se ele cair em bus-off (TEC=255) - o lado CAN A, que agora
// TRANSMITE para o tester. Diferente do TWAI, o MCP nao tem recuperacao propria
// exposta, entao re-inicializamos o controlador (reset + reconfigura). Checado
// ~10x/s porque cada getError/errorCount e' um acesso SPI. O one-shot (setup)
// evita a tempestade que leva ao bus-off; aqui e' a rede de seguranca.
void mcpHealth() {
  unsigned long now = millis();
  if (now - lastMcpHealth < 100) return;
  lastMcpHealth = now;
  uint8_t eflg = CAN_MCP.getError();           // EFLG: TXBO = bus-off
  if ((eflg & MCP_EFLG_TXBO) || CAN_MCP.errorCountTX() >= 255) {
    CAN_MCP.begin(MCP_ANY, CAN_250KBPS, MCP_16MHZ);  // soft-reset + reconfigura
    CAN_MCP.setMode(MCP_NORMAL);
    CAN_MCP.enOneShotTX();                            // reset volta ao estado seguro
    mcpOneShot = true;                               // (one-shot ligado)
    mcpRecoverCount++;
    return;                                           // so recupera nesta volta
  }

  // One-shot dinamico: tester confirmado (RX recente no MCP = ha no' ativo dando
  // ACK e podendo concorrer na arbitragem) -> DESLIGA o one-shot p/ retransmitir e
  // nao perder frame da sonda. Sem trafego ha um tempo -> religa (protege o key-on).
  bool testerAlive = (lastMcpRxMs != 0) && (now - lastMcpRxMs < MCP_ACK_ALIVE_MS);
  bool wantOneShot = !testerAlive;
  if (wantOneShot != mcpOneShot) {
    if (wantOneShot) CAN_MCP.enOneShotTX();
    else             CAN_MCP.disOneShotTX();
    mcpOneShot = wantOneShot;
  }
}

void pumpBus(RecvFn recv, SendFn send, bool applyCorrection);
void onWsEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length);

// Pagina web (config + telemetria) - servida da flash ----------------------
extern const char INDEX_HTML[] PROGMEM;

void setup() {
  Serial.begin(115200);
  pinMode(D3_PIN, INPUT_PULLUP);
  pinMode(RESET_PIN, INPUT_PULLUP);
  pinMode(BOLINHAS_PIN, OUTPUT); digitalWrite(BOLINHAS_PIN, HIGH);  // saida: repouso em HIGH
  prevResetLow = (digitalRead(RESET_PIN) == LOW);        // nao dispara evento ao ligar
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
  // ONE-SHOT: com o CAN invertido o MCP e' o lado que TRANSMITE para o tester.
  // Sem one-shot, um frame sem ACK (key-on / tester desligado) e' retransmitido
  // sem parar: os 3 buffers de TX enchem, o sendMsg gira ate TIMEOUTVALUE e o
  // loop() para de ler o outro lado -> CAN A fica muda. Em one-shot cada frame e'
  // tentado 1x e, falhando, o buffer e' liberado na hora (sem travar o loop).
  // Ligamos AQUI (estado seguro p/ o key-on). Depois o mcpHealth() o DESLIGA
  // assim que o tester for confirmado (RX no MCP), p/ nao perder frame da sonda
  // por perda de arbitragem em marcha. Ver mcpOneShot / MCP_ACK_ALIVE_MS.
  CAN_MCP.enOneShotTX();
  mcpOneShot = true;

  // ---- TWAI / CAN nativo (lado do diagnostico) : 250 kbps ----
  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(TWAI_TX, TWAI_RX, TWAI_MODE_NORMAL);
  twai_timing_config_t  t = TWAI_TIMING_CONFIG_250KBITS();
  twai_filter_config_t  f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  // Repete ate' conseguir (igual ao MCP). Antes era so um println e o codigo
  // SEGUIA com o TWAI morto -> lado CAN B mudo sem aviso em operacao normal.
  bool okTwai = (twai_driver_install(&g, &t, &f) == ESP_OK) && (twai_start() == ESP_OK);
  while (!okTwai) {
    Serial.println(F("ERRO TWAI (CAN nativo) - verifique TX=7/RX=6 e o transceiver"));
    twai_stop();                 // limpa um install parcial antes de tentar de novo
    twai_driver_uninstall();
    delay(1000);
    okTwai = (twai_driver_install(&g, &t, &f) == ESP_OK) && (twai_start() == ESP_OK);
  }

  // NAO sobe o Wi-Fi/web aqui: isso fica para startNetwork(), chamado pelo loop()
  // so depois do CAN ja estar repassando (ver NET_START_DELAY_MS). Subir o softAP
  // dentro do setup() deixava os dois lados do barramento CORTADO desconectados
  // justo na janela de key-on, perdendo o handshake de aquecimento do sensor NOx.

  lastRamp = millis();
  lastSensorMs = millis();
  lastFloorMs = floorPhaseT0 = yT0 = millis();
  activeFloor = cfg.o2_floor;
  randomSeed(micros());   // semente p/ a oscilacao aleatoria do piso
  canReadyMs = millis();  // CAN pronto: a partir daqui o loop() ja repassa frames
  Serial.println(F("PRONTO: bridge CAN 250kbps (MCP2515=leitura  TWAI=saida)"));
}

// Sobe o Wi-Fi AP + servidor web + WebSocket. Chamado UMA vez pelo loop(), apos
// NET_START_DELAY_MS repassando o CAN (fora da janela de key-on do sensor NOx).
void startNetwork() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.print(F("Wi-Fi AP \"")); Serial.print(AP_SSID);
  Serial.print(F("\" senha \"")); Serial.print(AP_PASS);
  Serial.print(F("\"  ->  http://")); Serial.println(WiFi.softAPIP());

  http.on("/", []() { http.send_P(200, "text/html", INDEX_HTML); });
  http.begin();
  ws.begin();
  ws.onEvent(onWsEvent);
  netStarted = true;
}

void loop() {
  unsigned long now = millis();
  // Sobe o Wi-Fi/web so depois de NET_START_DELAY_MS repassando o CAN, para nao
  // ocupar o setup/loop com o softAP durante a janela de key-on do sensor NOx.
  if (!netStarted && now - canReadyMs >= NET_START_DELAY_MS) startNetwork();
  if (netStarted) { http.handleClient(); ws.loop(); }

  updateInputs();
  updateRamp();
  updateFloorCycle();

  // [CAN A<->B INVERTIDO] Sensor(TWAI) -> Diagnostico(MCP2515): corrige o TARGET_ID
  pumpBus(twaiRecv, mcpSend, true);
  // [CAN A<->B INVERTIDO] Diagnostico(MCP2515) -> Sensor(TWAI): repasse puro
  pumpBus(mcpRecv, twaiSend, false);
  twaiHealth();                          // cura bus-off do lado CAN B (TWAI)
  mcpHealth();                           // cura bus-off do lado CAN A (MCP2515)

  now = millis();
  // Diagnostico do lado CAN B no serial (1x/s), independente do sensor ja estar
  // ligado. rx = frames recebidos do painel; estado 0=STOPPED 1=RUNNING
  // 2=BUS_OFF 3=RECOVERING. Se rx NAO sobe, o TWAI nao recebe nada do painel.
  if (now - lastTwaiLog >= 1000) {
    lastTwaiLog = now;
    twai_status_info_t st;
    if (twai_get_status_info(&st) == ESP_OK) {
      Serial.printf("TWAI rx=%lu estado=%d tx_err=%lu rx_err=%lu tx_fail=%lu\n",
                    (unsigned long)twaiRxCount, (int)st.state,
                    (unsigned long)st.tx_error_counter,
                    (unsigned long)st.rx_error_counter,
                    (unsigned long)st.tx_failed_count);
    }
    // Diagnostico do MCP (CAN A): rx = frames recebidos; eflg = registrador de
    // erro (0x20=bus-off); tec/rec = contadores de erro TX/RX; recover = quantas
    // vezes o MCP foi re-inicializado. Se rx NAO sobe, CAN A nao recebe nada.
    // oneshot=1 -> protegendo key-on (descarta sem retransmitir); =0 -> tester
    // confirmado, retransmitindo normal (esperado EM MARCHA).
    Serial.printf("MCP  rx=%lu eflg=0x%02X tec=%u rec=%u recover=%lu oneshot=%d\n",
                  (unsigned long)mcpRxCount, CAN_MCP.getError(),
                  CAN_MCP.errorCountTX(), CAN_MCP.errorCountRX(),
                  (unsigned long)mcpRecoverCount, mcpOneShot ? 1 : 0);
  }
  // Envia telemetria assim que a rede sobe (nao espera o 1o frame do sensor):
  // assim o status "ok" de cada CAN ja aparece na pagina mesmo sem dados ainda.
  if (netStarted && now - lastTelem >= TELEM_MS) {
    lastTelem = now;
    int kmil = (int)(k * 1000.0f + 0.5f);
    int afMil = (int)(activeFloor * 1000.0f + 0.5f);     // piso em vigor (x1000)
    unsigned long remain = 0;                            // tempo restante na fase
    if (floorPhase == FP_HOLD1 && !pierceBlocked) {      // contagem Y no Piso 1 (yT0 proprio)
      unsigned long el = now - yT0;
      remain = (el < cfg.floor_period_ms) ? cfg.floor_period_ms - el : 0;
    } else if (floorPhase == FP_HOLD2) {                 // contagem X no Piso 2
      unsigned long el = now - floorPhaseT0;
      remain = (el < cfg.floor_hold_ms) ? cfg.floor_hold_ms - el : 0;
    }
    int rstP = resetEvent ? 1 : 0;
    resetEvent = false;                                  // consome o pulso
    // TIMEOUT: sem frame do sensor ha mais que can_timeout_ms (0 = desligado)
    int toP = (cfg.can_timeout_ms > 0 && (now - lastSensorMs) > cfg.can_timeout_ms) ? 1 : 0;
    // Status "ok" de cada barramento: recebeu frame ha menos de CAN_OK_MS?
    int caOk = (lastMcpRxMs  && (now - lastMcpRxMs)  < CAN_OK_MS) ? 1 : 0; // CAN A (MCP2515)
    int cbOk = (lastTwaiRxMs && (now - lastTwaiRxMs) < CAN_OK_MS) ? 1 : 0; // CAN B (TWAI)
    char line[140];
    snprintf(line, sizeof(line), "T %u %u %u %u %d %d %u %d %lu %lu %lu %d %d %d %d %d %d",
             s1Read, s1Corr, s2Read, s2Corr, correctActive ? 1 : 0, kmil,
             floorPhase, afMil, remain, (unsigned long)biteCount,
             (unsigned long)pierceCount, pierceBlocked ? 1 : 0, rstP, toP,
             caOk, cbOk, hasData ? 1 : 0);
    ws.broadcastTXT(line);
  }
}

// Le a entrada RESET (INPUT_PULLUP -> ativa em nivel BAIXO; dispara na BORDA de
// descida) que zera as furadas. Em seguida gera a saida BOLINHAS: a cada nova
// FURADA do Piso 1 (pierceCount subiu) da um pulso LOW de 500ms; repouso em HIGH.
void updateInputs() {
  bool rLow = (digitalRead(RESET_PIN) == LOW);
  if (rLow && !prevResetLow) resetPierces();
  prevResetLow = rLow;

  unsigned long now = millis();
  if (pierceCount != prevPierceCount) {       // nova furada (ou RESET zerou o contador)
    if (pierceCount > prevPierceCount) bolinhasPulseUntil = now + BOLINHAS_PULSE_MS;
    prevPierceCount = pierceCount;
  }
  digitalWrite(BOLINHAS_PIN, (bolinhasPulseUntil && now < bolinhasPulseUntil) ? LOW : HIGH);
}

// Zera o contador de furadas e destrava o ciclo (pode furar de novo). Marca o
// evento RESET no log. Chamado pelo pino RESET (negativo) ou pelo comando RSF.
void resetPierces() {
  pierceCount = 0;
  pierceBlocked = false;
  resetEvent = true;
}

// Atualiza o alvo (trigger de O2 + override do D3) e faz a rampa de 'k'.
void updateRamp() {
  unsigned long now = millis();
  unsigned long dt = now - lastRamp;
  lastRamp = now;

  bool d3low = (digitalRead(D3_PIN) == LOW);
  // O D3 e' um HABILITADOR externo da correcao; o modo escolhe a polaridade:
  //   "Bypass"     (d3_low_bypass=1): D3 negativo DESLIGA (bypass) ; solto -> habilita.
  //   "Corrigindo" (d3_low_bypass=0): D3 negativo HABILITA         ; solto -> bypass.
  // Quando habilitado, segue o trigger de O2 (so corrige com O2 < trigger).
  bool d3allow = cfg.d3_low_bypass ? !d3low : d3low;
  bool target = d3allow && (lastO2Diag < cfg.o2_trigger);
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
// valido (0 < piso2 < piso1), o piso em vigor (activeFloor) mergulha NO TEMPO:
//   HOLD1 -> segura no Piso 1 contando Y (floor_period_ms). Vencido Y -> WAIT.
//   WAIT  -> AGUARDA (segue no Piso 1) ate' o O2 encostar/FURAR o Piso 1
//            (floorBiting). Ao furar: mergulha (DOWN) e REINICIA Y nesse instante.
//   DOWN  -> tira a correcao em rampa ate o Piso 2.
//   HOLD2 -> segura no Piso 2 por X (floor_hold_ms; conta a partir de atingi-lo)
//   UP    -> incrementa a correcao em rampa, subindo ate o Piso 1 -> HOLD1.
// O contador Y (yT0) so REINICIA na furada CONTROLADA (corrigindo). Em bypass o Y
// NAO reinicia e CONTINUA contando ate "aguardando"; la a furada vale NA HORA que o
// O2 cruza o Piso 1 (IGNORA o Y - ver pumpBus): conta e pulsa a saida BOLINHAS, sem
// mergulhar. A rampa entre pisos reusa ramp_ms; a oscilacao do piso vem POR CIMA.
void updateFloorCycle() {
  unsigned long now = millis();
  unsigned long dt = now - lastFloorMs;
  lastFloorMs = now;

  float piso1 = cfg.o2_floor;
  float piso2 = cfg.o2_floor2;
  bool cycleValid = piso1 > 0.0f && piso2 > 0.0f && piso2 < piso1;  // Piso 2 configurado

  // Atingiu o limite de furadas? (0 = sem limite). Enquanto travado, nao mergulha
  // (fica AGUARDANDO no Piso 1) ate' vir o RESET (pino negativo ou comando RSF).
  bool canPierce = (cfg.max_pierces == 0) || (pierceCount < cfg.max_pierces);
  pierceBlocked = !canPierce;

  if (!cycleValid) {              // Piso 2 desligado -> sem ciclo, fica no Piso 1
    floorPhase = FP_HOLD1;
    yT0 = now;                    // sem ciclo o Y nao conta
    activeFloor = piso1;
    return;
  }

  // Em BYPASS (fora do trigger) a FURADA ainda vale (conta e pulsa a saida
  // BOLINHAS), mas o MERGULHO em si nao e' aplicado: aborta um mergulho em curso
  // de volta ao Piso 1. O contador Y segue correndo (FP_HOLD1 abaixo).
  if (!correctActive &&
      (floorPhase == FP_DOWN || floorPhase == FP_HOLD2 || floorPhase == FP_UP))
    floorPhase = FP_HOLD1;

  // passo da rampa do piso (unidades de O2/ms); ramp_ms==0 -> transicao instantanea
  float step = (cfg.ramp_ms == 0) ? 1e9f
             : (piso1 - piso2) * (float)dt / (float)cfg.ramp_ms;

  switch (floorPhase) {
    case FP_HOLD1:
      activeFloor = piso1;
      if (now - yT0 >= cfg.floor_period_ms) floorPhase = FP_WAIT;  // venceu Y (conta ate em bypass) -> aguarda furar
      break;
    case FP_WAIT:                              // AGUARDA o O2 encostar/furar o Piso 1
      activeFloor = piso1;
      if (correctActive && canPierce && floorBiting) {  // furada CONTROLADA (corrigindo): respeita o Y
        floorPhase = FP_DOWN;
        pierceCount++;                         // conta a furada + dispara o pulso de saida
        yT0 = now;                             // reinicia Y JA no instante da furada
      }
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
      if (activeFloor >= piso1) {              // voltou ao Piso 1 (Y segue correndo desde a furada)
        activeFloor = piso1;
        floorPhase = FP_HOLD1;
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

      // Ganho BASE do NOx (ja com a rampa), ANTES do reforco do piso: o NOx so
      // usa esse ganho. O reforco do piso e' exclusivo do O2 (ver abaixo).
      float gNox = g * k;

      // "Batendo no Piso 1": O2 corrigido so pela correcao BASE (sem o reforco,
      // ja com a rampa) ficaria abaixo do Piso 1 -> o reforco precisa atuar.
      // (rawToDiag e' linear -> O2 corrigido diag = o2d * (1 + ganho/100).)
      float o2baseCorr = o2d * (1.0f + (g * k) / 100.0f);
      bool nowBiting = (cfg.o2_floor > 0.0f) && (o2baseCorr < cfg.o2_floor);
      if (nowBiting && !floorBiting) {              // borda: comecou a bater no Piso 1
        biteCount++;
        // Em BYPASS a furada vale NA HORA, IGNORANDO o tempo Y: conta e pulsa a
        // saida BOLINHAS. Nao mergulha (nao ha correcao em bypass) nem reseta o Y.
        if (!correctActive && ((cfg.max_pierces == 0) || (pierceCount < cfg.max_pierces)))
          pierceCount++;
      }
      floorBiting = nowBiting;

      bool floorBoost = false;   // reforco do piso atuando? (empurra o O2 p/ o piso)
      if (activeFloor > 0.0f && o2d > 0.01f) {
        float tgt = activeFloor * (1.0f + OSC_AMP_FRAC * floorOsc());  // por cima do piso ativo
        float gNeeded = 100.0f * (tgt / o2d - 1.0f);
        if (gNeeded > g) { g = gNeeded; floorBoost = true; }
      }
      g *= k;

      // O2: mantem TODA a logica (ganho base + reforco do piso + rampa).
      uint16_t c2 = correct(r2, O2_RAW0, +g); // O2 : aumenta

      // NOx: so o ganho BASE. Enquanto o reforco do piso atua, a conta -g levaria
      // o NOx a valores loucos (ate negativos). Em vez disso CONGELA o ultimo NOx
      // corrigido antes de bater o piso e deixa ele OSCILANDO (so NOx). Fora do
      // reforco, corrige normalmente e memoriza o valor.
      uint16_t c1;
      if (floorBoost && noxHeld) {
        float heldDiag = rawToDiag(noxHeldRaw, NOX_RAW0, NOX_DREF, NOX_RREF);
        float diagOut  = heldDiag * (1.0f + NOX_OSC_FRAC * (2.0f * noxOsc() - 1.0f));
        float den = NOX_RREF - NOX_RAW0; if (den == 0.0f) den = 1.0f;
        float rawOut = NOX_RAW0 + diagOut * den / NOX_DREF;   // volta de diag -> raw
        if (rawOut < 0.0f)       rawOut = 0.0f;
        if (rawOut > 65535.0f)   rawOut = 65535.0f;
        c1 = (uint16_t)(rawOut + 0.5f);
      } else {
        c1 = correct(r1, NOX_RAW0, -gNox);   // NOx: diminui (correcao normal)
        noxHeldRaw = c1; noxHeld = true;     // memoriza o ultimo valor bom
      }
      buf[0] = c1 & 0xFF; buf[1] = (c1 >> 8) & 0xFF;
      buf[2] = c2 & 0xFF; buf[3] = (c2 >> 8) & 0xFF;

      s1Read = r1; s1Corr = c1;
      s2Read = r2; s2Corr = c2;
      hasData = true;
      lastSensorMs = millis();   // recebeu frame do sensor -> arma/rearma o timeout
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
    cfg.max_pierces     = 1;      // por padrao fura 1x e trava ate o RESET
    cfg.can_timeout_ms  = 2000;   // 2 s sem dados do sensor -> TIMEOUT
    saveConfig();
  }
}

void saveConfig() {
  EEPROM.put(0, cfg);
  EEPROM.commit();      // no ESP32 nada e' gravado ate' o commit
}

// Envia a config atual para TODOS os clientes web conectados.
void sendConfig() {
  char line[160];
  snprintf(line, sizeof(line), "CFG %.3f,%.3f,%.3f,%lu,%u,%u,%.3f,%lu,%lu,%lu,%lu",
           cfg.pct, cfg.o2_trigger, cfg.o2_floor,
           (unsigned long)cfg.ramp_ms, cfg.d3_low_bypass, cfg.prop_band,
           cfg.o2_floor2, (unsigned long)cfg.floor_hold_ms,
           (unsigned long)cfg.floor_period_ms, (unsigned long)cfg.max_pierces,
           (unsigned long)cfg.can_timeout_ms);
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
  if (strncmp(line, "MXF:", 4) == 0) { cfg.max_pierces     = (uint32_t)atol(line + 4); saveConfig(); sendConfig(); return; }
  if (strncmp(line, "TMO:", 4) == 0) { cfg.can_timeout_ms  = (uint32_t)atol(line + 4); saveConfig(); sendConfig(); return; }
  if (strcmp(line, "RSF") == 0)      { resetPierces(); return; }   // zera furadas (botao da UI)
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
  .cans{display:flex;gap:8px;margin:8px 0}
  .pill{font-size:12px;font-weight:700;padding:3px 10px;border-radius:999px;border:1px solid var(--line)}
  .pill.ok{background:rgba(46,204,113,.15);color:#2ecc71;border-color:#2ecc71}
  .pill.bad{background:rgba(224,16,16,.15);color:#ff6b6b;border-color:#e04}
  .tabs{display:flex;gap:6px;margin:12px 0 0}
  .tab{background:#333c4a;color:var(--ink);border:0;border-radius:8px 8px 0 0;padding:8px 14px;cursor:pointer;font-size:14px;opacity:.55}
  .tab.active{background:var(--card);opacity:1}
  .pane.hidden{display:none}
</style></head>
<body><div class="wrap">
  <h1>Bridge CAN 18F00F52 &mdash; corretor de sensores</h1>
  <div id="conn" class="conn">status: <b>conectando&hellip;</b></div>

  <div class="cans">
    <span id="cana" class="pill bad">CAN A: &mdash;</span>
    <span id="canb" class="pill bad">CAN B: &mdash;</span>
  </div>

  <div class="tabs">
    <button id="tb-main" class="tab active" onclick="showTab('main')">Principal</button>
    <button id="tb-adv"  class="tab"        onclick="showTab('adv')">Op&ccedil;&otilde;es avan&ccedil;adas</button>
  </div>

  <div id="pane-main" class="pane">
  <div id="banner" class="banner">AGUARDANDO</div>

  <div class="card">
    <h2>Leitura (bancada)</h2>
    <div class="read"><span>Oxigenio (O2) &mdash; Lido</span><span class="v" id="o2r">---</span></div>
    <div class="read"><span>Oxigenio (O2) &mdash; Corrigido</span><span class="v c" id="o2c">---</span></div>
    <div class="read"><span>NOx (ppm) &mdash; Lido</span><span class="v" id="nor">---</span></div>
    <div class="read"><span>NOx (ppm) &mdash; Corrigido</span><span class="v c" id="noc">---</span></div>
    <canvas id="chart" width="600" height="170"></canvas>
    <div class="hint">grafico do O2 &mdash; azul = lido, verde = corrigido &middot;
      marcas: <b style="color:#e04">furou</b> / <b style="color:#ffb020">RESET</b></div>
  </div>

  <div class="card">
    <h2>Ciclo do piso</h2>
    <div class="read"><span>Bateu no Piso 1 (vezes)</span><span class="v" id="bite">0</span></div>
    <div class="read"><span>Furou o Piso 1 (vezes)</span><span class="v c" id="pierce">0</span></div>
    <div class="read"><span>Piso em vigor / fase</span><span class="v" id="cyc">---</span></div>
    <div class="read"><span>Ultimo evento</span><span class="v" id="evt">&mdash;</span></div>
  </div>

  <div class="card">
    <h2>Configuracoes (gravam na EEPROM)</h2>
    <div class="row"><label>Ganho proporcional (%) &mdash; O2 +X% / NOx &minus;X%</label>
      <input id="pct" type="number" step="any"></div>
    <div class="row"><label>Trigger O2 (corrige se O2 &lt;)</label>
      <input id="trg" type="number" step="any"></div>
    <div class="row"><label>Piso 1 O2 (nao baixar de; 0 = desliga)</label>
      <input id="flr" type="number" step="any"></div>
    <div class="row"><label>Piso 2 O2 (mais baixo; 0 = sem ciclo)</label>
      <input id="flr2" type="number" step="any"></div>
    <div class="row"><label>Tempo X &mdash; segura no Piso 2 (ms)</label>
      <input id="hld" type="number" step="1"></div>
    <div class="row"><label>Tempo Y &mdash; intervalo no Piso 1 (ms)</label>
      <input id="per" type="number" step="1"></div>
    <div class="row"><label>M&aacute;x. furadas do Piso 1 (0 = sem limite)</label>
      <input id="mxf" type="number" step="1"></div>
    <div class="row"><label>Timeout sem dados do sensor (ms) &mdash; 0 desliga</label>
      <input id="tmo" type="number" step="1"></div>
    <div class="row"><label>Rampa (ms)</label>
      <input id="rmp" type="number" step="1"></div>
    <div class="hint">Corre&ccedil;&atilde;o e IO4 ficam na aba &ldquo;Op&ccedil;&otilde;es avan&ccedil;adas&rdquo;.</div>
    <div class="row"><button onclick="applyAll()">Aplicar tudo</button>
      <button class="sec" onclick="resetFuros()">Zerar furadas</button>
      <span class="hint">grava os campos na EEPROM &middot; zerar libera novas furadas</span></div>
    <div id="warn" class="warn"></div>
  </div>

  <div class="card">
    <h2>Log (gravado no navegador)</h2>
    <div class="row">
      <button id="logbtn" class="sec" onclick="toggleLog()">Gravar log</button>
      <span id="logst" class="hint">log parado</span>
    </div>
    <div class="hint">gera um .csv para abrir no log_viewer.html (inclui a coluna de TIMEOUT)</div>
  </div>
  </div><!-- /pane-main -->

  <div id="pane-adv" class="pane hidden">
    <div class="card">
      <h2>Op&ccedil;&otilde;es avan&ccedil;adas (listas)</h2>
      <div class="hint" style="margin-bottom:10px">Separadas da tela principal para evitar toque acidental na correria.</div>
      <div class="row"><label>Correcao</label>
        <select id="prp"><option value="0">Fixa (piso oscila)</option><option value="1">Proporcional a faixa</option></select></div>
      <div class="row"><label>IO4 negativo (pino em nivel baixo)</label>
        <select id="d3"><option value="1">Bypass</option><option value="0">Corrigindo</option></select></div>
      <div class="row"><button onclick="applyAll()">Aplicar op&ccedil;&otilde;es</button>
        <span class="hint">grava Correcao e IO4 na EEPROM</span></div>
    </div>
  </div><!-- /pane-adv -->
</div>
<script>
// Calibracao FIXA raw -> valor exibido (igual ao firmware)
const CAL_O2=[23500,9.5,42000], CAL_NOX=[4000,100,6000];
const d2=(raw,c)=>(raw-c[0])*c[1]/((c[2]-c[0])||1);
const $=id=>document.getElementById(id);

let ws, hist=[]; // hist: {r,c,furou,rst} do O2 p/ o grafico
let log=null, logT0=0, logN=0, rows=[];
let lastPierce=0;                         // p/ detectar nova furada entre amostras
const PHN=['Piso 1','aguardando','descendo','Piso 2','subindo'];

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
    const p=line.split(' '); if(p.length<18) return;
    const s1r=+p[1],s1c=+p[2],s2r=+p[3],s2c=+p[4],st=+p[5],kmil=+p[6];
    const phase=+p[7], af=(+p[8])/1000, remain=+p[9], bite=+p[10], pierce=+p[11],
          blocked=+p[12], rst=+p[13], to=+p[14], caOk=+p[15], cbOk=+p[16], hd=+p[17];
    setPill($('cana'),'CAN A',caOk); setPill($('canb'),'CAN B',cbOk);
    banner(st,kmil,hd);
    if(!hd) return;          // sem dados do sensor ainda: so atualiza status das CANs
    const o2r=d2(s2r,CAL_O2),o2c=d2(s2c,CAL_O2),nor=d2(s1r,CAL_NOX),noc=d2(s1c,CAL_NOX);
    $('o2r').textContent=o2r.toFixed(2); $('o2c').textContent=o2c.toFixed(2);
    $('nor').textContent=nor.toFixed(1); $('noc').textContent=noc.toFixed(1);
    // readouts do ciclo do piso
    $('bite').textContent=bite;
    $('pierce').textContent=pierce + (blocked?'  (travado)':'');
    $('cyc').textContent=af.toFixed(2)+'  ['+(PHN[phase]||phase)+']'+
      ((phase===0||phase===3)&&remain>0?'  '+(remain/1000).toFixed(0)+'s':'');
    if(to) $('evt').textContent='TIMEOUT (sem dados do sensor)'; else if(rst) $('evt').textContent='RESET';
    const furou = pierce>lastPierce; lastPierce=pierce;
    hist.push({r:o2r,c:o2c,furou:furou,rst:!!rst,to:!!to});
    if(hist.length>240)hist.shift(); draw();
    if(log) logSample(o2r,o2c,nor,noc,st,kmil,phase,af,remain,bite,pierce,blocked,rst,to);
  } else if(line.startsWith('CFG ')){
    const v=line.slice(4).split(',').map(Number);
    $('pct').value=v[0]; $('trg').value=v[1]; $('flr').value=v[2];
    $('rmp').value=Math.round(v[3]);
    $('d3').value = v[4]?'1':'0';               // 1=Bypass, 0=Corrigindo
    if(v.length>=6) $('prp').value = v[5]?'1':'0';
    if(v.length>=9){ $('flr2').value=v[6]; $('hld').value=Math.round(v[7]); $('per').value=Math.round(v[8]); }
    if(v.length>=10) $('mxf').value=Math.round(v[9]);
    if(v.length>=11) $('tmo').value=Math.round(v[10]);
    checkBand();
  }
}
function banner(st,kmil,hd){
  const b=$('banner');
  if(!hd){ b.textContent='AGUARDANDO'; b.style.background='#333'; b.style.color='#bbb'; return; }
  if(st===1){ let t='CORRIGINDO'; if(kmil<999)t+=' (rampa '+(kmil/10).toFixed(0)+'%)';
              b.textContent=t; b.style.background='#0a7d2c'; b.style.color='#fff'; }
  else{ let t='BYPASS'; if(kmil>1)t+=' (rampa '+(kmil/10).toFixed(0)+'%)';
        b.textContent=t; b.style.background='#b06a00'; b.style.color='#fff'; }
}
// Status "ok" de cada barramento CAN (verde = recebendo frames; vermelho = sem sinal)
function setPill(el,name,ok){
  el.textContent=name+': '+(ok?'OK':'sem sinal');
  el.className='pill '+(ok?'ok':'bad');
}
// Alterna entre a aba Principal e a de Opcoes avancadas (listas protegidas)
function showTab(name){
  const main = name==='main';
  $('pane-main').classList.toggle('hidden',!main);
  $('pane-adv').classList.toggle('hidden',main);
  $('tb-main').classList.toggle('active',main);
  $('tb-adv').classList.toggle('active',!main);
}
function ap(cmd,val){ if(ws&&ws.readyState===1) ws.send(cmd+':'+val); }
// Botao unico: envia TODOS os campos (cada um grava na EEPROM no firmware).
function applyAll(){
  if(!ws||ws.readyState!==1){ $('warn').textContent='Sem conexao - reconectando...'; return; }
  ap('PCT',$('pct').value); ap('TRG',$('trg').value);
  ap('FLR',$('flr').value); ap('FL2',$('flr2').value);
  ap('HLD',Math.round($('hld').value||0)); ap('PER',Math.round($('per').value||0));
  ap('MXF',Math.round($('mxf').value||0));
  ap('TMO',Math.round($('tmo').value||0));
  ap('RMP',Math.round($('rmp').value||0));
  ap('PRP',$('prp').value); ap('D3',$('d3').value);
  checkBand();
}
// Zera o contador de furadas no firmware (libera furar de novo). Marca RESET no log.
function resetFuros(){ if(ws&&ws.readyState===1) ws.send('RSF'); }
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
  // faixa vermelha = TIMEOUT (sem dados do sensor)
  x.fillStyle='rgba(224,16,16,0.14)';
  hist.forEach((p,i)=>{ if(p.to){ const x0=X(i),x1=X(Math.min(i+1,hist.length-1)); x.fillRect(x0,0,(x1-x0)||2,H); } });
  const plot=(key,col)=>{ x.strokeStyle=col; x.lineWidth=2; x.beginPath();
    hist.forEach((p,i)=>{ i?x.lineTo(X(i),Y(p[key])):x.moveTo(X(i),Y(p[key])); }); x.stroke(); };
  plot('r','#3b82f6'); plot('c','#2ecc71');
  // marcas verticais dos eventos na linha do tempo
  const mark=(i,col)=>{ x.strokeStyle=col; x.lineWidth=1.5; x.beginPath();
    x.moveTo(X(i),0); x.lineTo(X(i),H); x.stroke(); };
  hist.forEach((p,i)=>{ if(p.rst)mark(i,'#ffb020'); if(p.furou)mark(i,'#e04'); });
}

// log CSV no navegador
function toggleLog(){ log? stopLog(): startLog(); }
function startLog(){ rows=[]; logN=0; logT0=performance.now(); log=true;
  $('logbtn').textContent='Parar log'; $('logst').textContent='gravando... (0)'; }
function logSample(o2r,o2c,nor,noc,st,kmil,phase,af,remain,bite,pierce,blocked,rst,to){
  const t=(performance.now()-logT0)/1000, trg=+$('trg').value||0,
        flr=+$('flr').value||0, flr2=+$('flr2').value||0,
        hld=Math.round($('hld').value||0), per=Math.round($('per').value||0);
  rows.push([t.toFixed(3),o2r.toFixed(3),o2c.toFixed(3),nor.toFixed(3),noc.toFixed(3),
             trg,flr,flr2,st,(kmil/10).toFixed(1),
             phase,af.toFixed(3),(remain/1000).toFixed(2),hld,per,
             bite,pierce,blocked,rst,to].join(','));
  if(++logN%5===0) $('logst').textContent='gravando... ('+logN+')';
}
function stopLog(){ log=false; $('logbtn').textContent='Gravar log';
  const head='t_s,o2_read,o2_corr,nox_read,nox_corr,o2_trigger,o2_floor,o2_floor2,state,ramp_pct,'+
             'phase,active_floor,phase_remain_s,hold_ms,period_ms,bite_count,pierce_count,blocked,reset,timeout\n';
  const blob=new Blob([head+rows.join('\n')+'\n'],{type:'text/csv'});
  const a=document.createElement('a'); a.href=URL.createObjectURL(blob);
  const d=new Date(), p=n=>String(n).padStart(2,'0');
  a.download='log_'+d.getFullYear()+p(d.getMonth()+1)+p(d.getDate())+'_'+
             p(d.getHours())+p(d.getMinutes())+p(d.getSeconds())+'.csv';
  a.click(); $('logst').textContent='salvo ('+logN+' amostras)'; }

connect();
</script></body></html>)HTMLPAGE";
