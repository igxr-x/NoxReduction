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
  um pouco acima do piso (amplitude ≈ 6 %). A oscilação é **dominada por ruído
  suave com passo aleatório** (não mais uma senoide previsível), mantendo a
  amplitude e **nunca baixando do piso**. `Piso = 0` desliga esse reforço.
- **Piso 2 (ciclo temporizado):** um **segundo piso, mais baixo**, que o alvo
  passa a "bater" periodicamente **enquanto está corrigindo** (dentro do
  trigger). O ciclo é: segura no **Piso 1** por **Y** (`Tempo Y`) → **desce em
  rampa** até o **Piso 2** → conta **X** (`Tempo X`) **a partir do instante em
  que atinge o Piso 2** → **sobe em rampa** obrigatoriamente de volta ao Piso 1
  → recomeça, em loop. **Regra:** o mergulho ao Piso 2 só acontece quando **não
  está "batendo" no Piso 1** (quando a correção base já segura o O2 acima do
  Piso 1 sozinha). Se, ao vencer o Y, o valor estiver batendo no Piso 1, o
  mergulho é **adiado**: espera o valor **sair de cima do Piso 1** e, na
  **próxima** vez que voltar a bater, aí sim mergulha. A rampa entre os pisos
  reusa o mesmo `Rampa (ms)`. A **oscilação pequena (aleatória) continua
  aplicada por cima** do piso ativo no momento (Piso 1 ou Piso 2). Fora do
  trigger (bypass) o ciclo **pausa e volta ao Piso 1**. `Piso 2 = 0` (ou
  `Piso 2 ≥ Piso 1`) **desliga o ciclo**. Exige `0 < Piso 2 < Piso 1`.
- **Furadas do Piso 1 + trava e reset:** cada mergulho rumo ao Piso 2 conta uma
  **"furada"** do Piso 1. O campo **`Máx. furadas Piso 1`** limita quantas são
  permitidas (padrão **1**; `0` = sem limite). Ao atingir o limite, o ciclo
  **trava no Piso 1** (não mergulha mais) até chegar um **reset**. O reset vem
  de uma **entrada de hardware (nível baixo/negativo)** ou do botão **"Zerar
  furadas"** no app/página — ele zera o contador e **libera furar de novo**.
- **Entrada BOLINHAS:** uma segunda entrada de hardware (`INPUT_PULLUP`); quando
  recebe **negativo** (vai ao GND), registra um evento **`BOLINHAS`** no log.
- **Contadores no log:** o firmware conta e envia na telemetria **quantas vezes
  bateu no Piso 1** (reforço segurou o O2) e **quantas vezes furou o Piso 1**
  (mergulhos), além da **fase do ciclo** e dos **tempos restantes** — tudo vai
  para o log/CSV.
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
  - **Bypass:** força **bypass total** — tudo que entra sai igual, sem correção,
    ignorando o trigger enquanto D3 estiver baixo.
  - **Corrigindo:** **não força nada** — apenas **segue o trigger de O2 normal**
    (entra/sai do bypass pelo trigger), corrigindo **dentro do que foi
    configurado** em vez de corrigir o tempo todo. Fica igual à operação
    automática.

  Com D3 solto (alto), vale sempre o trigger de O2.

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

**Entradas extras** (também `INPUT_PULLUP`, ativas em nível **baixo/negativo**):

- **RESET de furadas = D4** — um pulso ao GND **zera o contador de furadas** do
  Piso 1 (libera o ciclo a furar de novo); equivale ao botão "Zerar furadas".
- **BOLINHAS = D5** — um pulso ao GND registra o evento **`BOLINHAS`** no log.

> O barramento precisa ser **fisicamente aberto**: o trecho do sensor vai no
> módulo de leitura (CS10) e o trecho do diagnóstico no módulo de saída (CS9).
> Todo ID diferente de `18F00F52` é repassado intacto nos dois sentidos.

## Configurações (gravadas na EEPROM, editáveis pelo app)

| Config              | Padrão  | O que faz                                       |
|---------------------|---------|-------------------------------------------------|
| Ganho proporcional  | 0 %     | O2 `+X%`, NOx `−X%`                              |
| Trigger O2          | 5       | corrige quando O2 (diag) `<` este valor         |
| Piso 1 O2           | 3       | O2 corrigido não baixa disso (oscila; 0 desliga)|
| Piso 2 O2           | 0       | piso mais baixo do ciclo (0 ou ≥Piso 1 = desliga)|
| Tempo X             | 5000 ms | quanto segura no Piso 2 (conta ao atingi-lo)    |
| Tempo Y             | 60000 ms| intervalo no Piso 1 entre mergulhos ao Piso 2   |
| Máx. furadas Piso 1 | 1       | quantas furadas antes de travar (0 = sem limite); reset libera |
| Rampa               | 2000 ms | tempo de 0 → 100 % ao entrar/sair; e rampa entre pisos |
| D3 negativo         | Bypass  | modo do D3 baixo: Bypass (força) ou Corrigindo (segue trigger) |
| Correção            | Fixa    | Fixa (piso oscila) ou Proporcional à faixa      |

A **calibração** raw → valor exibido é **fixa** (constantes no firmware e no
app), usada só para exibir o valor final e avaliar os triggers de O2:

| Sensor       | raw@diag0 | diag ref → raw |
|--------------|-----------|----------------|
| 1 (NOx ppm)  | 4000      | 100 → 6000     |
| 2 (O2)       | 23500     | 9.5 → 42000    |

## Versão sem fio — LilyGO T-2CAN (ESP32-S3)

Porta o **mesmo corretor** para a placa **LilyGO T-2CAN**, trocando a
comunicação por **Wi-Fi + página web**: não precisa mais do cabo USB nem do
app do PC para operar. A placa **cria a própria rede Wi-Fi** e serve uma página
que mostra a telemetria ao vivo e ajusta todos os parâmetros (ganho, trigger,
piso 1, **piso 2 + tempos X/Y do ciclo**, **máx. furadas**, rampa, correção e
D3), com botão **Zerar furadas** e **contadores ao vivo** (bateu/furou), tudo
gravado na EEPROM.

> A base da correção é a **mesma** do firmware do Arduino — mudaram a fiação CAN
> e o meio de comunicação. Firmware novo: `firmware/src/t2can_main.cpp`.
>
> O **ciclo de Piso 2** (tempos X/Y) agora existe **nas duas versões** — Nano
> (`firmware/src/main.cpp` + `app.py`) **e** T-2CAN. A **única** diferença de
> comportamento que resta é o **D3 "Corrigindo"**: na T-2CAN ele **segue o
> trigger de O2**; no firmware serial do Nano ele mantém o comportamento antigo
> (**força a correção** enquanto D3 estiver baixo).

### Diferença de hardware (importante)

A T-2CAN **não** tem dois MCP2515 iguais. Ela tem **dois canais CAN diferentes**:

| Canal | Controlador | Papel neste projeto | Liga em |
|-------|-------------|---------------------|---------|
| CAN A | **MCP2515** (SPI) | **Leitura** (corrige aqui) | lado do **sensor/ECU** |
| CAN B | **TWAI** (CAN nativo do ESP32-S3) | **Saída** | lado do **diagnóstico** |

O barramento continua **cortado em dois** — igual ao Nano. Só que agora um lado
entra no conector CAN do **MCP2515** e o outro no conector do **CAN nativo**.

Pinos usados pelo firmware (T-2CAN V1.0, **cristal do MCP2515 = 16 MHz**):
`SPI SCLK=12 MOSI=11 MISO=13` · `MCP2515 CS=10 RST=9` · `TWAI TX=7 RX=6` ·
`override físico = IO4` · `RESET de furadas = IO15` · `BOLINHAS = IO16`
(as três entradas são `INPUT_PULLUP`, ativas em nível baixo; deixe o override
solto para operação normal).

### Fiação do barramento (bornes da própria placa)

O barramento continua **cortado em dois trechos** (igual ao Nano). Ligue cada
trecho no borne parafusável correspondente:

| Lado do barramento | Borne na T-2CAN | Canal interno | Papel     |
|--------------------|-----------------|---------------|-----------|
| **Sensor / ECU**   | **CANHA / CANLA** | CAN A = MCP2515 | Leitura (corrige aqui) |
| **Diagnóstico**    | **CANHB / CANLB** | CAN B = TWAI    | Saída     |

- `CANH↔CANH` e `CANL↔CANL` de cada lado; terra do CAN (opcional) no `SGND`.
- **Alimentação:** USB-C **ou** o borne **DC 12–24 V** (pode puxar do veículo).
- Se a correção sair "invertida", troque os dois chicotes de lado nos bornes.

### Botão de override físico (o antigo D3) → **IO4**

O override por hardware (força Bypass/Corrigindo) está no pad **`IO4`** da placa
(`INPUT_PULLUP`, mesma lógica do D3 do Nano):

```
   [ interruptor de trava ]
   IO4  ───o     o───  GND
```

- **Fechado (IO4 no GND):** ativa o modo configurado no campo **"D3 negativo"**
  da página web (padrão de fábrica = **Bypass**).
- **Aberto (solto):** volta ao **trigger automático de O2**.
- Use um **interruptor de trava / chave gangorra** (fica ligado). Um botão de
  pressão momentâneo só forçaria o modo enquanto ficasse apertado.

**Entradas extras na T-2CAN** (`INPUT_PULLUP`, ativas em nível baixo):

- **`IO15` = RESET de furadas** — um **pulso ao GND** zera o contador de furadas
  do Piso 1 (destrava o ciclo). Aqui um **botão momentâneo** serve bem (basta um
  toque); também dá para usar o botão **"Zerar furadas"** da página.
- **`IO16` = BOLINHAS** — um **pulso ao GND** registra o evento `BOLINHAS` no
  log (igual ao Nano).

### Como usar (depois de gravar)

1. Ligue a T-2CAN (USB-C ou 12–24 V no conector DC).
2. No celular/notebook, conecte no Wi-Fi **`Starlink Labs`** — senha **`11021992`**.
3. Abra **`http://192.168.4.1`** no navegador.
4. A página mostra **BYPASS / CORRIGINDO**, os valores **Lido × Corrigido** de
   O2 e NOx (com gráfico ao vivo — com **marcas de furou / BOLINHAS / RESET**),
   os **contadores do ciclo** (bateu/furou) e todos os campos de configuração.
   Ajuste e clique **Aplicar tudo** — grava todos os campos na EEPROM na hora;
   **Zerar furadas** destrava o ciclo (libera furar de novo).
5. **Gravar log** salva um `.csv` (colunas completas: contadores, timers e
   eventos) direto no aparelho, que você abre no `log_viewer.html`.

> O SSID/senha ficam no topo do `t2can_main.cpp` (`AP_SSID` / `AP_PASS`) — a
> senha precisa ter no mínimo 8 caracteres.

### Gravar a T-2CAN pelo PlatformIO (passo a passo, sem experiência com ESP32)

É o **mesmo PlatformIO** que você já usava com o Arduino — só muda o *env*.

1. **Instale o PlatformIO Core** (uma vez só), se ainda não tiver:
   - Com Python: `pip install platformio` → o comando `pio` fica disponível no
     terminal. (Ou instale a extensão **PlatformIO IDE** no VS Code.)
   - Confira: `pio --version`.
2. **Conecte a T-2CAN no PC pelo cabo USB-C.** Ela usa **USB nativo** do ESP32-S3
   e aparece como uma porta **COM** nova (veja em Gerenciador de Dispositivos).
   Use um cabo USB-C **de dados** (alguns cabos são só de carga e não funcionam).
3. **Grave** (dentro da pasta do projeto):

   ```bash
   pio run -e t2can -t upload                 # compila e grava (env t2can e' o padrao)
   pio run -e t2can -t upload --upload-port COM7   # forca a porta, se preciso
   ```

   Na **primeira gravação**, o Windows baixa o toolchain do ESP32-S3 (demora um
   pouco só na primeira vez).
4. **Se travar em `Connecting...`:** ponha o S3 em modo de gravação manualmente —
   **segure o botão BOOT (GPIO0)**, **toque RESET**, **solte o BOOT** e rode o
   upload de novo.
5. **Ver o Wi-Fi/IP ao ligar (opcional):** `pio device monitor -e t2can`
   imprime `Wi-Fi AP "BridgeCAN" ... -> http://192.168.4.1`. (Só para conferir;
   a operação normal é toda pela página web, sem cabo.)

Ou use o atalho **`gravar_t2can.bat`** (duplo-clique ou por CMD), que
auto-detecta a porta e dá as dicas de recuperação:

```bat
gravar_t2can.bat          :: auto-detecta a porta
gravar_t2can.bat COM7     :: forca a porta
```

> **`ERRO MCP2515`** no monitor serial → confira a fiação do lado da leitura e
> que o cristal é de **16 MHz** (se por acaso for 8 MHz, troque `MCP_16MHZ` por
> `MCP_8MHZ` em `t2can_main.cpp`). **`ERRO TWAI`** → confira o lado do
> diagnóstico (TX=7/RX=6) e o bitrate 250 kbps.

---

## 1. Firmware do Arduino Nano / ESP32-C3 (PlatformIO)

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
4. Ajuste **Ganho / Trigger O2 / Piso 1 / Piso 2 / Tempo X / Tempo Y / Máx.
   furadas Piso 1 / Rampa (ms)**, o modo de **Correção** (Fixa ou Proporcional à
   faixa) e o modo do **D3 negativo** (Bypass ou Corrigindo). Um **único botão
   "Aplicar tudo"** grava **todos** os campos na EEPROM de uma vez. Se o Piso 1
   ficar acima do trigger, ou o Piso 2 ficar ≥ Piso 1, o app avisa com um alerta.
   **Piso 2 = 0** desliga o ciclo temporizado (fica só o Piso 1).
5. O painel **Ciclo do piso** mostra ao vivo **quantas vezes bateu** e **quantas
   furou** o Piso 1, a **fase / piso em vigor** e o **último evento**
   (BOLINHAS / RESET). O botão **"Zerar furadas"** manda o reset (destrava e
   libera furar de novo) — equivale a aterrar a entrada de RESET no hardware.

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

- **O2:** lido × corrigido, com as linhas de referência do **trigger**
  (tracejada laranja), do **piso 1** (pontilhada vermelha), do **piso 2**
  (pontilhada magenta) e do **piso em vigor** (roxo — o piso que o ciclo usa a
  cada instante);
- **NOx (ppm):** lido × corrigido;
- **faixas verdes** marcam os trechos em que o trigger estava ativo
  (**CORRIGINDO**);
- **linhas verticais** marcam os eventos na linha do tempo: **furou o piso 1**
  (vermelho), **BOLINHAS** (azul) e **RESET** (laranja) — assim dá para ver
  **quando** cada furada aconteceu (o resumo no topo conta o total de furadas).
  Passe o mouse para ler os valores, a fase e os contadores em cada instante.

Cada linha/evento tem uma **caixa de seleção** no topo para mostrar/ocultar
individualmente.

Colunas do CSV: `t_s, o2_read, o2_corr, nox_read, nox_corr, o2_trigger,
o2_floor, o2_floor2, state, ramp_pct, phase, active_floor, phase_remain_s,
hold_ms, period_ms, bite_count, pierce_count, blocked, bolinhas, reset`
(`state`: 0=bypass, 1=corrigindo; `phase`: 0=Piso 1, 1=aguardando, 2=descendo,
3=Piso 2, 4=subindo; `active_floor` = piso em vigor; `phase_remain_s` = tempo
restante da fase; `bite_count`/`pierce_count` = contadores acumulados de
"bateu"/"furou" o Piso 1; `blocked` = ciclo travado por limite de furadas;
`bolinhas`/`reset` = pulso 0/1 no evento; `t_s` em segundos desde o início). O
visualizador **ignora colunas ausentes**, então logs antigos continuam abrindo
normalmente.

## Protocolo serial

- PC → Nano:
  - `GET` — pede a config atual.
  - `PCT:<v>` — ganho proporcional único (grava EEPROM).
  - `TRG:<v>` — limite de O2 do trigger (grava EEPROM).
  - `FLR:<v>` — Piso 1 de O2 / valor mínimo de saída (grava EEPROM).
  - `RMP:<v>` — tempo de rampa em milissegundos (grava EEPROM).
  - `D3:<0|1>` — modo do D3 baixo (`1`=Bypass força; `0`=Corrigindo — no Nano
    serial força a correção, na T-2CAN segue o trigger).
  - `PRP:<0|1>` — correção proporcional à faixa (`1`=on, `0`=piso oscila).
  - `FL2:<v>` — Piso 2 de O2 (piso mais baixo do ciclo; `0` desliga).
  - `HLD:<ms>` — Tempo X: quanto segura no Piso 2.
  - `PER:<ms>` — Tempo Y: intervalo no Piso 1 entre mergulhos.
  - `MXF:<v>` — máx. de furadas do Piso 1 antes de travar (`0` = sem limite).
  - `RSF` — zera o contador de furadas (destrava; = pulso no pino de RESET).
- Nano → PC:
  - `CFG pct,o2_trigger,o2_floor,ramp_ms,d3mode,prop_band,o2_floor2,hold_ms,period_ms,max_pierces`
    — config atual (10 campos no firmware atual do Nano e da T-2CAN; firmwares
    antigos mandavam menos — os clientes ignoram os campos que faltam).
  - `T <s1_lido> <s1_corr> <s2_lido> <s2_corr> <estado> <k_milesimos> <fase> <piso_vigor×1000> <resta_ms> <bateu> <furou> <travado> <bolinhas> <reset>`
    — telemetria (~10 Hz). `estado`: 0=BYPASS 1=CORRIGINDO; `k`: 0..1000; `fase`:
    0=Piso 1, 1=aguardando, 2=descendo, 3=Piso 2, 4=subindo; `bateu`/`furou`:
    contadores acumulados; `travado`: 0/1 (limite de furadas atingido);
    `bolinhas`/`reset`: pulso 0/1 no evento.
