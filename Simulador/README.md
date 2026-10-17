# Controle de Sensores CAN — Arduino Nano + MCP2515

App de PC com dois sliders que controla, em tempo real, dois "sensores"
codificados dentro de um frame CAN transmitido por um Arduino Nano.

- **Barramento:** CAN 250 kbps, MCP2515 com cristal de **8 MHz**
- **Frame:** `ID 18F00F52` (estendido/J1939), `DLC 8`
- **DATA:** `[s1_lo][s1_hi][s2_lo][s2_hi] 55 9F 1F FF`

## Lógica dos sensores (little-endian, "de trás pra frente")

O valor bruto (raw) de cada sensor são 2 bytes em little-endian:

| Sensor | Bytes DATA | Raw = valor de 16 bits  |
|--------|------------|-------------------------|
| 1      | `[0] [1]`  | `DATA[1]<<8 \| DATA[0]` |
| 2      | `[2] [3]`  | `DATA[3]<<8 \| DATA[2]` |

Exemplo: `80 3E` → `0x3E80` = **16000**. Os bytes `DATA[4..7]` (padrão
`55 9F 1F FF`) são editáveis individualmente no app.

### Calibração diagnóstico → raw (linear, 2 pontos)

Os sliders trabalham em **valor de diagnóstico**; o app converte para raw:

```
raw = raw0 + (raw_ref - raw0) / diag_ref * diag
```

| Sensor | diag 0 → raw | diag ref → raw     | Slider (diag) |
|--------|--------------|--------------------|---------------|
| 1      | 4000         | 100 → 6000         | 0 .. 400      |
| 2      | 23500        | 9.5 → 42000        | 0 .. 23       |

Os 3 parâmetros de calibração são editáveis na tela (dá pra refinar sem
regravar o Arduino). O raw é limitado a `0..65535` (2 bytes); no sensor 2,
acima de diag ≈ 21,6 o raw estoura esse limite e o app avisa em vermelho.

## Arquitetura

```
[App Python]  --USB serial (115200)-->  [Arduino Nano + MCP2515]  --CAN 250k-->  [barramento / diagnóstico]
```

O app faz toda a lógica (calibração + montagem dos 8 bytes) e envia o frame
pronto como `"F"+16 hex+"\n"`. O Arduino é um transmissor "burro": apenas
**retransmite os 8 bytes continuamente a 50 Hz (a cada 20 ms)**, mantendo o
sinal "vivo" pro diagnóstico.

## 1. Firmware do Arduino (PlatformIO)

O firmware é um projeto PlatformIO em `firmware/`. A biblioteca `mcp_can` é
baixada automaticamente na primeira compilação (já configurada no
`platformio.ini`).

Confira antes a ligação do módulo (CS no **D10**; SPI: MISO=D12, MOSI=D11,
SCK=D13). Depois, dentro da pasta `firmware/`:

```bash
pio run                                # só compila (bootloader NEW - padrão)
pio run -t upload                      # compila e grava
pio run -e nano_old -t upload          # Nano com bootloader antigo
pio run -t upload --upload-port COM6   # força a porta
pio device monitor                     # abre o monitor serial (115200)
```

> **Bootloader:** esta placa (chip CH340) usa o **bootloader NOVO (115200)** →
> env `nano_new`, que já é o padrão. Se um dia usar um Nano antigo e der
> `not in sync`/timeout, troque para `-e nano_old`.
>
> **Porta:** informe a porta do Arduino (aqui é a **COM6**, USB-SERIAL CH340).
> Sem `--upload-port` o PlatformIO pode tentar uma porta Bluetooth por engano.

### Atalho: `gravar_firmware.bat`

Na raiz do projeto há um `.bat` que chama o `pio` pra você (duplo-clique ou CMD):

```bat
gravar_firmware.bat            :: bootloader NEW (esta placa), porta automatica
gravar_firmware.bat COM6       :: forca a porta COM6
gravar_firmware.bat COM6 old   :: bootloader antigo
gravar_firmware.bat - old      :: bootloader antigo, porta automatica
```

No monitor serial (115200) deve aparecer `PRONTO: ponte USB->CAN 250kbps ativa`.

> Se aparecer `ERRO: MCP2515 nao inicializou`, revise a fiação e confirme que o
> módulo realmente usa cristal de **8 MHz** (alguns vêm com 16 MHz — nesse caso
> troque `MCP_8MHZ` por `MCP_16MHZ` em `firmware/src/main.cpp`).

## 2. App do PC

```bash
pip install pyserial
python app.py
```

1. Feche o Monitor Serial da IDE Arduino (a porta não pode estar ocupada).
2. Selecione a porta COM do Nano (aqui **COM6**) e clique **Conectar**.
3. Mova os sliders (em valor de **diagnóstico**). Ao lado aparece o **raw**
   resultante (decimal + hex) e os **dois bytes** que vão no frame.
4. Ajuste os campos de **Calibração** se precisar refinar a reta.
5. Edite os **bytes livres DATA[4..7]** em hex e clique **Aplicar** para
   investigar o que cada um faz no diagnóstico.
6. Compare com o que seu diagnóstico lê no barramento para validar a lógica.

## Protocolo serial

- PC → Nano: `F` + 16 dígitos hex + `\n` = os 8 bytes de DATA.
  Ex.: `F701710A4559F1FFF\n` → `70 17 10 A4 55 9F 1F FF`.
- Nano → PC: eco `OK 70 17 10 A4 55 9F 1F FF` (útil para debug).
- Toda a calibração/limite fica no app; o firmware só retransmite os 8 bytes.
