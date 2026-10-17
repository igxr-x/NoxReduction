@echo off
REM ============================================================
REM  Grava o firmware no Arduino Nano usando PlatformIO (pio).
REM
REM  Uso:
REM     gravar_firmware.bat                (bootloader NEW - esta placa/CH340)
REM     gravar_firmware.bat COM6           (forca a porta COM6)
REM     gravar_firmware.bat COM6 old       (bootloader OLD - Nano antigo)
REM     gravar_firmware.bat - old          (bootloader OLD, porta automatica)
REM ============================================================
setlocal
cd /d "%~dp0"

REM ---- Escolhe o ambiente (bootloader) ----
set "ENVNAME=nano_new"
if /I "%~2"=="old" set "ENVNAME=nano_old"

REM ---- Porta (opcional). Use "-" para deixar automatico ----
set "PORTARG="
if not "%~1"=="" if not "%~1"=="-" set "PORTARG=--upload-port %~1"

REM ---- Confere se o pio esta disponivel ----
where pio >nul 2>&1
if not %ERRORLEVEL%==0 (
    echo ERRO: PlatformIO ^(pio^) nao encontrado no PATH.
    echo Instale com:  pip install platformio
    goto :fail
)

echo(
echo === Gravando firmware CAN via PlatformIO ===
echo Ambiente: %ENVNAME%
if defined PORTARG ( echo Porta   : %~1 ) else ( echo Porta   : automatica )
echo(

pio run -d firmware -e %ENVNAME% -t upload %PORTARG%
if not %ERRORLEVEL%==0 goto :fail

echo(
echo === CONCLUIDO! Firmware gravado com sucesso. ===
echo Abra o app com:  python app.py
echo(
pause
exit /b 0

:fail
echo(
echo *** FALHA. Veja as mensagens acima. ***
echo Dica: se der "not in sync" ou timeout, tente o outro bootloader:
if /I "%ENVNAME%"=="nano_new" (
    echo     gravar_firmware.bat %~1 old
) else (
    echo     gravar_firmware.bat %~1
)
echo(
pause
exit /b 1
