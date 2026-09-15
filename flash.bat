@echo off
setlocal

set FLASHER=C:\Infineon\AURIX-Studio-1.10.32\tools\AurixFlasherSoftwareTool_v3.0.16\AURIXFlasher.exe
set HEX=C:\Users\chris\Projects\AurixTricore\TriCore Debug (TASKING)\AurixTricore.hex

if not exist "%HEX%" (
    echo Kein HEX-File gefunden. Erst bauen (build.bat oder IDE^).
    exit /b 1
)

echo === FLASH: "%HEX%" ===
"%FLASHER%" -hex "%HEX%" -erase on -prog on -ver on

if %ERRORLEVEL% NEQ 0 (
    echo Flash fehlgeschlagen.
    exit /b 1
)

echo.
echo === FLASH ERFOLGREICH ===

rem --- confirm the flashed board actually carries Version.h's version ---
rem SYS1-001 B9, 2026-09-13: a build once printed "0 errors" while three
rem Version.h-dependent TUs stayed stale (amk missed the header-only
rem rebuild), so the board booted the PREVIOUS version despite a Pass
rem verdict from the flasher above. build.bat now forces those TUs to
rem recompile every time, but this is the check that fails loudly if that
rem is ever wrong again, instead of relying on a human to read it back.
rem
rem 2026-09-15: this check twice reported "Maximum CONNECT retries reached"
rem right after a flash, even though the board was fine -- v1.19.30 read
rem back seconds later by hand. The board had simply not finished its own
rem boot (reset -> SSW -> PLL init -> XCP task alive) when the first CONNECT
rem landed. A fixed boot delay plus a few retries fixes the race without
rem masking a real mismatch: a genuinely wrong version fails on every
rem attempt and still ends in FEHLGESCHLAGEN / exit 1.
echo.
echo === VERSION CHECK (XCP) ===
timeout /t 3 /nobreak >nul

set VERCHECK_ATTEMPT=0

:VERCHECK_RETRY
set /a VERCHECK_ATTEMPT+=1
python "%~dp0tools\xcp_verify_version.py"
if not errorlevel 1 goto VERCHECK_OK

if %VERCHECK_ATTEMPT% LSS 3 (
    echo Versionscheck Versuch %VERCHECK_ATTEMPT%/3 fehlgeschlagen -- Board evtl. noch am Booten, erneuter Versuch...
    timeout /t 2 /nobreak >nul
    goto VERCHECK_RETRY
)

echo.
echo === VERSION CHECK FEHLGESCHLAGEN -- geflashte Firmware stimmt nicht mit Version.h ueberein ===
exit /b 1

:VERCHECK_OK
exit /b 0
