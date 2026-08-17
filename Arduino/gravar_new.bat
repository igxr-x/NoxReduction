@echo off
REM ============================================================
REM  Grava SOMENTE com bootloader NEW (115200).
REM  Uso:  gravar_new.bat  [COM6]
REM ============================================================
setlocal
cd /d "%~dp0"

set "PORT=%~1"
if "%PORT%"=="" (
    powershell -NoProfile -Command "$m=Get-CimInstance Win32_PnPEntity | Where-Object {$_.Name -match '\(COM\d+\)' -and $_.Name -notmatch 'Bluetooth'} | Select-Object -First 1; if($m){[regex]::Match($m.Name,'COM\d+').Value}" > "%TEMP%\pio_port.txt"
    set /p PORT=<"%TEMP%\pio_port.txt"
)
if "%PORT%"=="" (
    echo Nenhuma porta USB encontrada. Use:  gravar_new.bat COM6
    pause
    exit /b 1
)
echo Porta: %PORT%   Bootloader: NEW (115200)
echo(

pio run -d firmware -e nano_new -t upload --upload-port %PORT%
if %ERRORLEVEL%==0 (
    echo(
    echo === CONCLUIDO! ===
) else (
    echo(
    echo *** FALHOU. Tente o gravar_old.bat ou segure RESET ao iniciar. ***
)
pause
exit /b %ERRORLEVEL%
