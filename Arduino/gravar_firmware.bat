@echo off
REM ============================================================
REM  Grava o firmware tentando OS DOIS bootloaders (NEW e OLD).
REM  Detecta sozinho a porta USB (ignora portas Bluetooth).
REM
REM  Uso:
REM     gravar_firmware.bat          (auto-detecta a porta)
REM     gravar_firmware.bat COM6     (forca a porta)
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
    echo Nenhuma porta USB encontrada. Conecte o Arduino ou informe manualmente:
    echo     gravar_firmware.bat COM6
    pause
    exit /b 1
)
echo Porta detectada: %PORT%
echo(

echo === Tentativa 1: bootloader NEW (115200) ===
pio run -d firmware -e nano_new -t upload --upload-port %PORT%
if %ERRORLEVEL%==0 goto :ok

echo(
echo === Tentativa 2: bootloader OLD (57600) ===
pio run -d firmware -e nano_old -t upload --upload-port %PORT%
if %ERRORLEVEL%==0 goto :ok

echo(
echo *** FALHOU nos dois bootloaders. ***
echo Provavel problema de AUTO-RESET. Tente:
echo   1) Segure o botao RESET do Nano e solte assim que aparecer "Uploading".
echo   2) Confirme a porta certa (chip CH340) e feche qualquer monitor serial.
echo   3) Teste outro cabo USB.
pause
exit /b 1

:ok
echo(
echo === CONCLUIDO! Firmware gravado com sucesso em %PORT%. ===
pause
exit /b 0
