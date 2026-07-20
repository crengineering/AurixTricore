@echo off
setlocal

set FLASHER=C:\Infineon\AURIX-Studio-1.10.32\tools\AurixFlasherSoftwareTool_v3.0.16\AURIXFlasher.exe
set HEX=C:\Users\chris\Projects\AurixTricore\TriCore Debug (TASKING)\AurixTricore.hex

if not exist "%HEX%" (
    echo Kein HEX-File gefunden. Erst bauen (build.bat oder IDE^).
    exit /b 1
)

echo === RESET: Target in sauberen Zustand bringen ===
"%FLASHER%" -erase off -prog off -ver off -connect 6 -start on

echo.
echo === ERASE: "%HEX%" ===
"%FLASHER%" -hex "%HEX%" -erase on -prog off -ver off -connect 6

if %ERRORLEVEL% NEQ 0 (
    echo Erase fehlgeschlagen.
    exit /b 1
)

echo.
echo === ERASE ERFOLGREICH ===
exit /b 0
