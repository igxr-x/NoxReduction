@echo off
REM ============================================================
REM  Grava o firmware da LilyGO T-2CAN (ESP32-S3) via PlatformIO.
REM  A placa aparece como porta COM (USB nativo) ao ligar no USB-C.
REM  Detecta a porta sozinho (ignora Bluetooth); da' pra forcar.
REM
REM  Uso:
REM     gravar_t2can.bat          (auto-detecta a porta)
REM     gravar_t2can.bat COM7     (forca a porta)
REM
REM  Se falhar em "Connecting...": segure BOOT (GPIO0), toque RESET,
REM  solte BOOT e rode de novo (poe o S3 em modo de gravacao).
REM ============================================================
setlocal
cd /d "%~dp0"

REM ---- Porta: argumento ou auto-deteccao ----
set "PORT=%~1"
if "%PORT%"=="" (
    powershell -NoProfile -Command "$m=Get-CimInstance Win32_PnPEntity | Where-Object {$_.Name -match '\(COM\d+\)' -and $_.Name -notmatch 'Bluetooth'} | Select-Object -First 1; if($m){[regex]::Match($m.Name,'COM\d+').Value}" > "%TEMP%\pio_port.txt"
    set /p PORT=<"%TEMP%\pio_port.txt"
)
if "%PORT%"=="" (
    echo Nenhuma porta USB encontrada. Conecte a T-2CAN ou informe manualmente:
    echo     gravar_t2can.bat COM7
    pause
    exit /b 1
)
echo Porta detectada: %PORT%
echo(

echo === Gravando firmware T-2CAN (ESP32-S3) ===
pio run -d firmware -e t2can -t upload --upload-port %PORT%
if %ERRORLEVEL%==0 goto :ok

echo(
echo *** FALHOU. ***
echo   1) Segure BOOT (GPIO0), toque RESET, solte BOOT e rode de novo.
echo   2) Confirme a porta certa e feche qualquer monitor serial aberto.
echo   3) Teste outro cabo USB-C (alguns cabos sao so de carga).
pause
exit /b 1

:ok
echo(
echo === CONCLUIDO! Firmware gravado em %PORT%. ===
echo Ligue a placa, conecte no Wi-Fi "Starlink Labs" (senha 11021992)
echo e abra http://192.168.4.1 no navegador.
pause
exit /b 0
