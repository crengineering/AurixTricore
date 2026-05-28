@echo off
setlocal

set FLASHER=C:\Infineon\AURIX-Studio-1.10.32\tools\AurixFlasherSoftwareTool_v3.0.16\AURIXFlasher.exe
set HEX=C:\Users\chris\Projects\AurixTricore\TriCore Debug (TASKING)\AurixTricore.hex

echo === FLASH ===
"%FLASHER%" -hex "%HEX%" -erase on -prog on -ver on

if %ERRORLEVEL% NEQ 0 (
    echo Flash fehlgeschlagen.
    pause
    exit /b 1
)

echo.
echo === FLASH ERFOLGREICH ===
exit /b 0