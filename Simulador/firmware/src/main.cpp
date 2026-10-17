/*
 * Ponte USB -> CAN para Arduino Nano + MCP2515
 * -------------------------------------------------
 * Hardware:
 *   - Arduino Nano
 *   - Modulo CAN MCP2515 com cristal de 8 MHz
 *   - Barramento CAN a 250 kbps
 *
 * Ligacao (SPI padrao do Nano):
 *   MCP2515  ->  Nano
 *   VCC      ->  5V
 *   GND      ->  GND
 *   CS       ->  D10
 *   SO (MISO)->  D12
 *   SI (MOSI)->  D11
 *   SCK      ->  D13
 *   INT      ->  D2   (nao usado para TX, opcional)
 *
 * Build/gravacao com PlatformIO:
 *   pio run -t upload                 (bootloader NEW - esta placa/CH340)
 *   pio run -e nano_old -t upload     (Nano com bootloader antigo)
 *
 * Este firmware e' um transmissor "burro": recebe os 8 bytes prontos do
 * app pela serial e apenas os retransmite na CAN. Toda a logica de
 * sensores/calibracao fica no app (permite recalibrar sem regravar).
 *
 * Protocolo serial (PC -> Nano), 115200 baud:
 *   Uma linha "F" + 16 digitos hex + '\n'  = os 8 bytes de DATA.
 *   Ex.: "F803EF1A2559F1FFF\n" -> 80 3E F1 A2 55 9F 1F FF
 *
 * Frame CAN transmitido (a cada 20 ms = 50 Hz):
 *   ID  : 0x18F00F52 (estendido, 29 bits)
 *   DLC : 8
 *   DATA: os 8 bytes recebidos (padrao inicial: 80 3E F1 A2 55 9F 1F FF)
 */

#include <Arduino.h>
#include <SPI.h>
#include <mcp_can.h>

// ---- Configuracao ----
const uint8_t  CS_PIN     = 10;          // pino CS do MCP2515
const uint32_t CAN_ID     = 0x18F00F52;  // ID estendido (J1939)
const uint16_t TX_PERIOD  = 20;          // ms entre frames (20 ms = 50 Hz)

MCP_CAN CAN0(CS_PIN);

// Bytes atuais do frame (atualizados pela serial). Valor inicial = exemplo.
uint8_t frameData[8] = {0x80, 0x3E, 0xF1, 0xA2, 0x55, 0x9F, 0x1F, 0xFF};

// Buffer de recepcao serial
char     rxBuf[24];
uint8_t  rxLen = 0;

unsigned long lastTx = 0;

// Prototipos
void readSerial();
void parseLine(char *line);
int  hexNibble(char c);
void sendFrame();

void setup() {
  Serial.begin(115200);

  // Inicializa o MCP2515: aceita qualquer ID, 250 kbps, cristal de 8 MHz
  while (CAN0.begin(MCP_ANY, CAN_250KBPS, MCP_8MHZ) != CAN_OK) {
    Serial.println(F("ERRO: MCP2515 nao inicializou (verifique fiacao/cristal)"));
    delay(1000);
  }
  CAN0.setMode(MCP_NORMAL);   // modo normal (transmite de verdade no barramento)

  Serial.println(F("PRONTO: ponte USB->CAN 250kbps ativa"));
}

void loop() {
  readSerial();

  unsigned long now = millis();
  if (now - lastTx >= TX_PERIOD) {
    lastTx = now;
    sendFrame();
  }
}

// Le linhas "F<16 hex>\n" da serial e atualiza os 8 bytes
void readSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (rxLen > 0) {
        rxBuf[rxLen] = '\0';
        parseLine(rxBuf);
        rxLen = 0;
      }
    } else if (rxLen < sizeof(rxBuf) - 1) {
      rxBuf[rxLen++] = c;
    } else {
      rxLen = 0;  // linha grande demais: descarta
    }
  }
}

void parseLine(char *line) {
  // Espera 'F' seguido de exatamente 16 digitos hex (8 bytes)
  if (line[0] != 'F' && line[0] != 'f') return;
  if (strlen(line) != 17) return;

  uint8_t tmp[8];
  for (uint8_t i = 0; i < 8; i++) {
    int hi = hexNibble(line[1 + i * 2]);
    int lo = hexNibble(line[2 + i * 2]);
    if (hi < 0 || lo < 0) return;         // caractere invalido: ignora a linha
    tmp[i] = (uint8_t)((hi << 4) | lo);
  }
  memcpy(frameData, tmp, 8);

  // Eco de confirmacao (ajuda no debug do app)
  Serial.print(F("OK "));
  for (uint8_t i = 0; i < 8; i++) {
    if (frameData[i] < 0x10) Serial.print('0');
    Serial.print(frameData[i], HEX);
    if (i < 7) Serial.print(' ');
  }
  Serial.println();
}

int hexNibble(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return -1;
}

void sendFrame() {
  // sendMsgBuf(id, ext=1, len=8, buf) -> ID estendido de 29 bits
  CAN0.sendMsgBuf(CAN_ID, 1, 8, frameData);
}
