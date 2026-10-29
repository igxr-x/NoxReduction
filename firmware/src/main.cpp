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
 * CICLO DE PISO (Piso 2, mergulho lento no tempo):
 *   Alem do Piso 1 (o2_floor), existe um Piso 2 (o2_floor2) MAIS BAIXO. Quando
 *   esta corrigindo e 0 < Piso2 < Piso1, o piso em vigor (activeFloor) mergulha
 *   do Piso 1 ao Piso 2 no tempo. REGRA: so mergulha para o Piso 2 quando NAO
 *   esta "batendo no Piso 1" (ou seja, quando a correcao base ja segura o O2
 *   acima do Piso 1 sozinha, sem o reforco precisar atuar).
 *     HOLD1 -> segura no Piso 1 por Y ms (floor_period_ms). Vencido Y:
 *                - se NAO esta batendo no Piso 1 -> mergulha ja (DOWN).
 *                - se ESTA batendo no Piso 1    -> adia (WAIT): espera o valor
 *                  SAIR de cima do Piso 1 (parar de bater) e, na PROXIMA vez que
 *                  ele voltar a bater no Piso 1, aí sim mergulha (DOWN).
 *     DOWN  -> tira a correcao em rampa ate' bater no Piso 2.
 *     HOLD2 -> segura no Piso 2 por X ms (floor_hold_ms; conta ao atingi-lo).
 *     UP    -> incrementa a correcao em rampa, subindo OBRIGATORIAMENTE ate' o
 *              Piso 1 (ou acima) -> recomeca (HOLD1).
 *   Fora do trigger (bypass) o ciclo pausa e volta ao Piso 1. A oscilacao
 *   pequena do piso (OSC_*, agora meio aleatoria) continua POR CIMA de
 *   activeFloor. Piso2 = 0 desliga o ciclo (fica so no Piso 1).
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
 *     MXF:<v>        max. de FURADAS do Piso 1 antes de travar (0 = sem limite)
 *     RSF            zera o contador de furadas (libera furar de novo) = RESET
 *   Nano -> PC:
 *     CFG pct,o2_trigger,o2_floor,ramp_ms,d3mode,prop_band,o2_floor2,hold_ms,period_ms,max_pierces
 *     T <s1_lido> <s1_corr> <s2_lido> <s2_corr> <estado> <k_milesimos>
 *       <fase> <piso_vigor_x1000> <resta_ms> <bateu> <furou> <travado> <bolinhas> <reset>
 *        estado: 0=BYPASS 1=CORRIGINDO ; k_milesimos: 0..1000
 *        fase: 0=Piso1 1=aguardando 2=descendo 3=Piso2 4=subindo
 *        bateu/furou: contadores acumulados ; bolinhas/reset: pulso 0/1 no evento
 *
 * ENTRADAS EXTRAS (INPUT_PULLUP; "negativo" = aterrar o pino = LOW):
 *   RESET_PIN    -> na borda de descida zera as furadas (= comando RSF).
 *   BOLINHAS_PIN -> na borda de descida marca o evento "BOLINHAS" no log.
 *   (pinos definidos por placa no bloco de Pinos abaixo.)
 *
 * TRAVA DE FURADAS (max_pierces):
 *   Cada mergulho rumo ao Piso 2 conta uma "furada". Ao atingir max_pierces o
 *   ciclo trava no Piso 1 (nao mergulha mais) ate' chegar um RESET (pino ou RSF),
 *   que zera o contador e libera novas furadas. max_pierces = 0 -> sem limite.
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
  const uint8_t RESET_PIN    = 1; // negativo -> zera FURADAS do Piso 1 -> GPIO1
  const uint8_t BOLINHAS_PIN = 0; // negativo -> evento "BOLINHAS" no log -> GPIO0
#else
  // Arduino Nano (5V) - pinagem original
  const uint8_t CS_READ = 10;   // LEITURA (sensor/ECU)
  const uint8_t CS_SEND = 9;    // SAIDA (diagnostico)
  const uint8_t D3_PIN  = 3;    // entrada de override (INPUT_PULLUP)
  const uint8_t RESET_PIN    = 4; // negativo -> zera FURADAS do Piso 1
  const uint8_t BOLINHAS_PIN = 5; // negativo -> evento "BOLINHAS" no log
#endif

const uint32_t TARGET_ID  = 0x18F00F52;  // frame a corrigir (estendido)
const uint16_t TELEM_MS   = 100;         // periodo da telemetria
const uint32_t CFG_MAGIC  = 0x43414E38;  // "CAN8" - marca de EEPROM valida (cresceu: max_pierces)

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
  uint32_t max_pierces;    // limite de FURADAS do Piso 1 (0 = sem limite); zera com RESET
};

Config cfg;

// Estado ao vivo
uint16_t s1Read = 0, s1Corr = 0, s2Read = 0, s2Corr = 0;
bool hasData = false;
uint16_t noxHeldRaw = 0;    // ultimo NOx corrigido (raw) antes de bater o piso
bool     noxHeld    = false;// ja ha um valor memorizado p/ congelar?
float lastO2Diag = 1e9f;      // O2 lido em diag (inicia "alto" -> bypass)
float k = 0.0f;               // fator de rampa 0..1
bool correctActive = false;   // alvo atual (trigger/D3): corrigindo?
unsigned long lastTelem = 0;
unsigned long lastRamp = 0;

// Ciclo do Piso 2 (Piso1 <-> Piso2 no tempo, so quando corrigindo)
//   FP_WAIT: venceu Y batendo no Piso 1 -> aguarda sair do Piso 1 e voltar a bater
enum FloorPhase { FP_HOLD1, FP_WAIT, FP_DOWN, FP_HOLD2, FP_UP };
uint8_t floorPhase = FP_HOLD1;
float activeFloor = 0.0f;            // piso em vigor no instante (o que o reforco usa)
unsigned long floorPhaseT0 = 0;      // inicio da fase atual (p/ contar X e Y)
unsigned long lastFloorMs = 0;
bool floorBiting = false;            // O2 corrigido batendo no Piso 1 (reforco atuando)
bool floorReleased = false;          // no FP_WAIT: ja saiu de cima do Piso 1?

// Contadores (vao para a telemetria/log)
uint32_t biteCount = 0;              // quantas vezes BATEU no Piso 1 (borda de floorBiting)
uint32_t pierceCount = 0;            // quantas vezes FUROU o Piso 1 (mergulhos iniciados)
bool pierceBlocked = false;          // atingiu max_pierces -> segura no Piso 1 ate o RESET

// Entradas extras (borda de descida = evento) + pulsos latched p/ a telemetria
bool prevResetLow = false, prevBolinhasLow = false;
bool resetEvent = false;             // pulso: RESET disparado desde o ultimo telem
bool bolinhasEvent = false;          // pulso: BOLINHAS disparado desde o ultimo telem

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
void updateInputs();
void resetPierces();

void setup() {
  Serial.begin(115200);
  pinMode(D3_PIN, INPUT_PULLUP);
  pinMode(RESET_PIN, INPUT_PULLUP);
  pinMode(BOLINHAS_PIN, INPUT_PULLUP);
  prevResetLow    = (digitalRead(RESET_PIN) == LOW);     // nao dispara evento ao ligar
  prevBolinhasLow = (digitalRead(BOLINHAS_PIN) == LOW);
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
  randomSeed(micros());   // semente p/ a oscilacao aleatoria do piso
  Serial.println(F("PRONTO: bridge CAN 250kbps ativo (CS10=leitura CS9=saida)"));
  sendConfig();
}

void loop() {
  updateInputs();
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
    int afMil = (int)(activeFloor * 1000.0f + 0.5f);     // piso em vigor (x1000)
    unsigned long el = now - floorPhaseT0, remain = 0;   // tempo restante na fase
    if (floorPhase == FP_HOLD1 && !pierceBlocked) remain = (el < cfg.floor_period_ms) ? cfg.floor_period_ms - el : 0;
    else if (floorPhase == FP_HOLD2) remain = (el < cfg.floor_hold_ms) ? cfg.floor_hold_ms - el : 0;
    int bolP = bolinhasEvent ? 1 : 0, rstP = resetEvent ? 1 : 0;
    bolinhasEvent = resetEvent = false;                  // consome os pulsos
    Serial.print(F("T "));
    Serial.print(s1Read); Serial.print(' ');
    Serial.print(s1Corr); Serial.print(' ');
    Serial.print(s2Read); Serial.print(' ');
    Serial.print(s2Corr); Serial.print(' ');
    Serial.print(correctActive ? 1 : 0); Serial.print(' ');
    Serial.print(kmil);          Serial.print(' ');
    Serial.print(floorPhase);    Serial.print(' ');
    Serial.print(afMil);         Serial.print(' ');
    Serial.print(remain);        Serial.print(' ');
    Serial.print(biteCount);     Serial.print(' ');
    Serial.print(pierceCount);   Serial.print(' ');
    Serial.print(pierceBlocked ? 1 : 0); Serial.print(' ');
    Serial.print(bolP);          Serial.print(' ');
    Serial.println(rstP);
  }
}

// Le as entradas extras (RESET e BOLINHAS), ambas INPUT_PULLUP -> ativas em
// nivel BAIXO. Dispara na BORDA de descida (aterrou agora). RESET zera as
// furadas (liberando novos mergulhos); BOLINHAS so marca um evento no log.
void updateInputs() {
  bool rLow = (digitalRead(RESET_PIN) == LOW);
  if (rLow && !prevResetLow) resetPierces();
  prevResetLow = rLow;

  bool bLow = (digitalRead(BOLINHAS_PIN) == LOW);
  if (bLow && !prevBolinhasLow) bolinhasEvent = true;
  prevBolinhasLow = bLow;
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
// valido (0 < piso2 < piso1), o piso em vigor (activeFloor) mergulha NO TEMPO,
// mas so entra no Piso 2 quando NAO esta batendo no Piso 1 (floorBiting):
//   HOLD1 -> segura no Piso 1 por Y (floor_period_ms). Vencido Y:
//              nao batendo -> DOWN ja ; batendo -> FP_WAIT (adia o mergulho).
//   WAIT  -> segura no Piso 1 ate' SAIR de cima dele (parar de bater) e, na
//            PROXIMA vez que voltar a bater, mergulha (DOWN).
//   DOWN  -> tira a correcao em rampa ate o Piso 2.
//   HOLD2 -> segura no Piso 2 por X (floor_hold_ms; conta a partir de atingi-lo)
//   UP    -> incrementa a correcao em rampa, subindo ate o Piso 1 -> HOLD1.
// Fora do trigger (bypass) o ciclo pausa/zera: volta pro Piso 1. A rampa entre
// pisos reusa o mesmo tempo de rampa (ramp_ms). A oscilacao pequena do piso
// continua sendo aplicada POR CIMA de activeFloor (em pumpBus).
void updateFloorCycle() {
  unsigned long now = millis();
  unsigned long dt = now - lastFloorMs;
  lastFloorMs = now;

  float piso1 = cfg.o2_floor;
  float piso2 = cfg.o2_floor2;
  bool cycleOn = correctActive && piso1 > 0.0f && piso2 > 0.0f && piso2 < piso1;

  // Atingiu o limite de furadas? (0 = sem limite). Enquanto travado, segura no
  // Piso 1 e nao mergulha ate' vir o RESET (pino negativo ou comando RSF).
  bool canPierce = (cfg.max_pierces == 0) || (pierceCount < cfg.max_pierces);
  pierceBlocked = !canPierce;

  if (!cycleOn) {                 // sem ciclo: fica no Piso 1 (comportamento antigo)
    floorPhase = FP_HOLD1;
    floorPhaseT0 = now;
    floorReleased = false;
    activeFloor = piso1;
    return;
  }

  // passo da rampa do piso (unidades de O2/ms); ramp_ms==0 -> transicao instantanea
  float step = (cfg.ramp_ms == 0) ? 1e9f
             : (piso1 - piso2) * (float)dt / (float)cfg.ramp_ms;

  switch (floorPhase) {
    case FP_HOLD1:
      activeFloor = piso1;
      if (now - floorPhaseT0 >= cfg.floor_period_ms) {       // venceu Y
        if (!canPierce) {
          floorPhaseT0 = now;                                // travado: segura no Piso 1
        } else if (!floorBiting) {
          floorPhase = FP_DOWN; pierceCount++;               // livre -> FUROU (mergulha)
        } else {
          floorPhase = FP_WAIT;                              // batendo -> adia
          floorReleased = false;
        }
      }
      break;
    case FP_WAIT:
      activeFloor = piso1;                                   // segue no Piso 1
      if (!floorReleased) {
        if (!floorBiting) floorReleased = true;              // saiu de cima do Piso 1
      } else if (floorBiting) {
        floorPhase = FP_DOWN; pierceCount++;                 // voltou a bater -> FUROU
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
      if (activeFloor >= piso1) {              // voltou ao Piso 1 (ou acima)
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

      // Ganho BASE do NOx (ja com a rampa), ANTES do reforco do piso: o NOx so
      // usa esse ganho. O reforco do piso e' exclusivo do O2 (ver abaixo).
      float gNox = g * k;

      // "Batendo no Piso 1": O2 corrigido so pela correcao BASE (sem o reforco,
      // ja com a rampa) ficaria abaixo do Piso 1 -> o reforco precisa atuar.
      // (rawToDiag e' linear -> O2 corrigido diag = o2d * (1 + ganho/100).)
      float o2baseCorr = o2d * (1.0f + (g * k) / 100.0f);
      bool nowBiting = (cfg.o2_floor > 0.0f) && (o2baseCorr < cfg.o2_floor);
      if (nowBiting && !floorBiting) biteCount++;   // conta a BORDA (comecou a bater)
      floorBiting = nowBiting;

      // Reforco do piso (com oscilacao): vale nos DOIS modos. Sobe o ganho
      // para o O2 corrigido NAO baixar do piso EM VIGOR (activeFloor, que o
      // ciclo do Piso 2 move no tempo), com o alvo oscilando um pouco acima
      // dele - inclusive quando a escala proporcional deixaria o ganho baixo
      // demais perto do trigger.
      bool floorBoost = false;   // reforco do piso atuando? (empurra o O2 p/ o piso)
      if (activeFloor > 0.0f && o2d > 0.01f) {
        float tgt = activeFloor * (1.0f + OSC_AMP_FRAC * floorOsc()); // piso..piso+6%
        float gNeeded = 100.0f * (tgt / o2d - 1.0f);
        if (gNeeded > g) { g = gNeeded; floorBoost = true; } // reforca para nao baixar do piso
      }
      g *= k;                                  // aplica a rampa

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
    cfg.max_pierces     = 1;    // por padrao fura 1x e trava ate o RESET
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
  Serial.print(cfg.floor_period_ms); Serial.print(',');
  Serial.println(cfg.max_pierces);
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
  if (strncmp(line, "MXF:", 4) == 0) { cfg.max_pierces     = (uint32_t)atol(line + 4); saveConfig(); sendConfig(); return; }
  if (strcmp(line, "RSF") == 0)      { resetPierces(); return; }   // zera furadas (botao da UI)
}
