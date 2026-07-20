@echo off
setlocal

rem =====================================================================
rem Build + flash + flight-controller replay smoke test, fully headless.
rem
rem Usage:
rem   test_replay.bat            incremental build, flash, smoke test
rem   test_replay.bat clean      clean build first (use after .h edits)
rem
rem For a real vector run afterwards:
rem   python tools\ctrl_replay_test.py --csv in.csv --expect ref.csv
rem =====================================================================

call "%~dp0build.bat" %1
if errorlevel 1 exit /b 1

call "%~dp0flash.bat"
if errorlevel 1 exit /b 1

echo === REPLAY SMOKE TEST ===
python "%~dp0tools\ctrl_replay_test.py"
if errorlevel 1 (
    echo === REPLAY TEST FEHLGESCHLAGEN ===
    exit /b 1
)

echo.
echo === REPLAY TEST ERFOLGREICH ===
exit /b 0
