# Bridge CAN corretor — Arduino Nano + 2× MCP2515

Dispositivo "man-in-the-middle" que fica **no meio do barramento CAN**, repassa
todo o tráfego de forma transparente e **intercepta só o frame `18F00F52`**,
aplicando uma **correção proporcional única** (ganho de **X%**). Um app de PC
mostra o valor **lido × corrigido** e o status **BYPASS / CORRIGINDO**, e ajusta
o ganho, o trigger, a rampa e o modo do D3 — tudo **gravado na EEPROM**
(sobrevive ao desligar o Arduino).

- **Barramento:** CAN 250 kbps, MCP2515 com cristal de **8 MHz** (dois módulos)
- **Frame corrigido:** `ID 18F00F52` (estendido/J1939), `DLC 8`
- **Sensores:** Sensor 1 = `DATA[0..1]` = **NOx (ppm)**, Sensor 2 = `DATA[2..3]`
  = **Oxigênio (O2)** (little-endian)

## Como funciona a correção

- **Ganho proporcional único:** um só percentual `X%` controla os dois sensores —
  **O2 recebe +X%** (sobe) e **NOx recebe −X%** (desce).
- **Trigger de O2:** a correção só atua quando o **O2 fica ABAIXO** do limite
  configurado. Acima disso → **bypass** (frame passa intacto).
- **Piso de O2 (2º limite):** se a correção base não segurar o O2 corrigido
  acima do piso, o ganho **aumenta dinamicamente** (O2 e NOx juntos) para o O2
  **não baixar desse valor**. Em vez de virar uma linha reta, o alvo **oscila**
  um pouco acima do piso (padrão: período 2,5 s, amplitude 6 %). `Piso = 0`
  desliga esse reforço.
- **Correção proporcional à faixa (modo alternativo):** quando ligada, o ganho
  vira **proporcional à posição do O2 lido dentro da faixa `[piso, trigger]`**:
  perto do **trigger** a correção é **mínima (0 %)**, perto do **piso** é
  **máxima (o ganho cheio)**, interpolando linearmente no meio; abaixo do piso
  satura no ganho cheio. Exige `trigger > piso > 0`. O **reforço oscilante do
  piso continua valendo por cima** — mesmo nesse modo o O2 corrigido **não
  baixa do piso** e o alvo segue **oscilando**. Se o **piso ficar ≥ que o
  trigger**, o app mostra um **alerta** (a faixa fica inválida).
- **Rampa:** ao ativar/desativar o trigger, o fator de correção sobe/desce
  gradualmente (leva `ramp_ms` milissegundos de 0 → 100%) para o valor corrigido
  não dar um salto brusco. `ramp_ms = 0` → troca instantânea.
- **Pino D3 (override por hardware, `INPUT_PULLUP`):** quando D3 chega em nível
  **baixo/negativo** (ligado ao GND), o modo configurável decide:
  **Bypass** (totalmente sem correção) ou **Corrigindo** (força a correção).
  Com D3 solto (alto), vale o trigger de O2.

```
raw_out = raw0 + (raw_lido - raw0) * (1 + (±X% · k)/100)
```

onde `k` (0..1) é o fator da rampa. O2 usa `+X%`, NOx usa `−X%`.

## Como ligar (IMPORTANTE: o barramento é cortado em dois)

```
[Sensor/ECU] --CAN--> (CS10 LÊ) [Arduino Nano +X%] (CS9 ENVIA) --CAN--> [Diagnóstico]
                <---------- outros IDs repassados nos dois sentidos ---------->
```

| Módulo MCP2515 | Pino CS | Papel      | Liga em                         |
|----------------|---------|------------|---------------------------------|
| Leitura        | **D10** | entrada    | lado do **sensor/ECU** original |
| Saída          | **D9**  | saída      | lado do **diagnóstico**         |

SPI compartilhado nos dois módulos: `SCK=D13  MOSI=D11  MISO=D12  VCC=5V  GND=GND`.
Só o pino **CS** é separado (D10 e D9). Os pinos INT não são usados (polling).

**Pino D3** = entrada de override (`INPUT_PULLUP`). Deixe **solto** para operação
normal (trigger de O2) ou **ligado ao GND** (negativo) para forçar o modo
configurado no app (Bypass ou Corrigindo).

> O barramento precisa ser **fisicamente aberto**: o trecho do sensor vai no
> módulo de leitura (CS10) e o trecho do diagnóstico no módulo de saída (CS9).
> Todo ID diferente de `18F00F52` é repassado intacto nos dois sentidos.

## Configurações (gravadas na EEPROM, editáveis pelo app)

| Config              | Padrão  | O que faz                                       |
|---------------------|---------|-------------------------------------------------|
| Ganho proporcional  | 0 %     | O2 `+X%`, NOx `−X%`                              |
| Trigger O2          | 5       | corrige quando O2 (diag) `<` este valor         |
| Piso O2             | 3       | O2 corrigido não baixa disso (oscila; 0 desliga)|
| Rampa               | 2000 ms | tempo de 0 → 100 % ao entrar/sair do trigger    |
| D3 negativo         | Bypass  | modo do D3 baixo: Bypass ou Corrigindo          |
| Correção            | Fixa    | Fixa (piso oscila) ou Proporcional à faixa      |

A **calibração** raw → valor exibido é **fixa** (constantes no firmware e no
app), usada só para exibir o valor final e avaliar os triggers de O2:

| Sensor       | raw@diag0 | diag ref → raw |
|--------------|-----------|----------------|
| 1 (NOx ppm)  | 4000      | 100 → 6000     |
| 2 (O2)       | 23500     | 9.5 → 42000    |

## 1. Firmware (PlatformIO)

Projeto em `firmware/`. As libs (`mcp_can`, `EEPROM`) são resolvidas
automaticamente. Dentro da pasta `firmware/`:

```bash
pio run                                # só compila (bootloader NEW - padrão)
pio run -t upload                      # compila e grava
pio run -e nano_old -t upload          # Nano com bootloader antigo
pio run -t upload --upload-port COM6   # força a porta
pio device monitor                     # monitor serial (115200)
```

> **Bootloader:** esta placa (chip CH340) usa o **bootloader NOVO (115200)** →
> env `nano_new` (padrão). Nano antigo com `not in sync`/timeout → `-e nano_old`.
>
> **Porta:** informe a porta do Arduino (USB-SERIAL CH340). Sem `--upload-port`
> o PlatformIO pode tentar uma porta Bluetooth por engano.

Na inicialização, o monitor mostra
`PRONTO: bridge CAN 250kbps ativo (CS10=leitura CS9=saida)`.

> Se aparecer `ERRO MCP2515: LEITURA=... SAIDA=...`, um dos módulos não
> respondeu — revise a fiação/CS e confirme que os módulos são de **8 MHz**
> (se forem de 16 MHz, troque `MCP_8MHZ` por `MCP_16MHZ` em
> `firmware/src/main.cpp`).

### Atalhos `.bat` (gravação por CMD / duplo-clique)

Todos auto-detectam a porta USB (ignoram Bluetooth); dá pra forçar passando a
porta como argumento.

```bat
gravar_firmware.bat        :: tenta os DOIS bootloaders (NEW e depois OLD)
gravar_new.bat             :: só bootloader NEW (115200)
gravar_old.bat             :: só bootloader OLD (57600)
gravar_firmware.bat COM6   :: força a porta
```

> **Não sincroniza / `not in sync` / timeout nos dois bootloaders?**
> É problema de **auto-reset**, não de bootloader. Tente:
> 1. **RESET manual:** inicie o upload e aperte+solte o botão RESET do Nano no
>    instante em que aparecer `Uploading`.
> 2. Confirme que grava pela **IDE Arduino** (valida se o board está saudável).
> 3. Teste outro **cabo USB** / verifique o driver CH340.

## 2. App do PC

```bash
pip install pyserial
python app.py
```

1. Feche o monitor serial da IDE (a porta não pode estar ocupada).
2. Selecione a porta COM do Nano e clique **Conectar** (o app pede a config).
3. O painel mostra ao vivo o valor final **Lido** e **Corrigido** do O2 e do NOx,
   e um status grande **BYPASS / CORRIGINDO** (com o % da rampa em transição).
4. Ajuste **Ganho / Trigger O2 / Piso O2 / Rampa (ms)** e clique **Aplicar**.
   Se o piso ficar acima do trigger, o app avisa com um alerta.
5. Em **Correção**, escolha **Fixa (piso oscila)** ou **Proporcional à faixa**
   e clique **Aplicar**.
6. Escolha o modo do **D3 negativo** (Bypass ou Corrigindo) e clique **Aplicar**.

## 3. Log e visualização gráfica

O Nano **não** guarda histórico (só tem ~1 KB de EEPROM, com desgaste de
escrita) — o log é gravado **no PC** enquanto o app está conectado.

1. Com o app conectado, clique **Gravar log**. Ele cria um `log_AAAAMMDD_HHMMSS.csv`
   na pasta **`logs/`** (criada ao lado do `app.py`) e grava cada amostra da
   telemetria (~10 Hz).
2. Clique **Parar log** (ou feche o app) para fechar o arquivo.
3. Abra o **`log_viewer.html`** (duplo-clique — não precisa de internet nem
   instalar nada) e arraste o `.csv` para dentro dele.

O visualizador mostra **tudo num único gráfico de eixo duplo** (O2 na escala da
esquerda, NOx ppm na direita), na mesma linha do tempo:

- **O2:** lido × corrigido, com as linhas do **trigger** (tracejada) e do
  **piso** (pontilhada);
- **NOx (ppm):** lido × corrigido;
- **faixas verdes** marcam os trechos em que o trigger estava ativo
  (**CORRIGINDO**); passe o mouse para ler os valores em cada instante.

Cada linha tem uma **caixa de seleção** no topo para mostrar/ocultar
individualmente (O2 lido, O2 corrigido, trigger, piso, NOx lido, NOx corrigido).

Colunas do CSV: `t_s, o2_read, o2_corr, nox_read, nox_corr, o2_trigger,
o2_floor, state, ramp_pct` (`state`: 0=bypass, 1=corrigindo; `t_s` em segundos
desde o início da gravação).

## Protocolo serial

- PC → Nano:
  - `GET` — pede a config atual.
  - `PCT:<v>` — ganho proporcional único (grava EEPROM).
  - `TRG:<v>` — limite de O2 do trigger (grava EEPROM).
  - `FLR:<v>` — piso de O2 / valor mínimo de saída (grava EEPROM).
  - `RMP:<v>` — tempo de rampa em milissegundos (grava EEPROM).
  - `D3:<0|1>` — modo do D3 baixo (`1`=Bypass, `0`=Corrigindo).
  - `PRP:<0|1>` — correção proporcional à faixa (`1`=on, `0`=piso oscila).
- Nano → PC:
  - `CFG pct,o2_trigger,o2_floor,ramp_ms,d3mode,prop_band` — config atual.
  - `T <s1_lido> <s1_corr> <s2_lido> <s2_corr> <estado> <k_milesimos>` —
    telemetria (~10 Hz); `estado`: 0=BYPASS 1=CORRIGINDO, `k`: 0..1000.
