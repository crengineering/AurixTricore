@echo off
setlocal

set ADS=C:\Infineon\AURIX-Studio-1.10.32
set WORKSPACE=C:\Users\chris\Projects
set PROJECT=AurixTricore

echo === BUILD ===
"%ADS%\AURIX-studioc.exe" ^
  -nosplash ^
  -data "%WORKSPACE%" ^
  -application com.tasking.managedbuilder.headlessbuild ^
  -cleanBuild "%PROJECT%" ^
  --launcher.ini "%ADS%\AURIX-studio-headless.ini"

if not exist "%WORKSPACE%\%PROJECT%\TriCore Debug (TASKING)\%PROJECT%.hex" (
    echo HEX nicht gefunden. Build fehlgeschlagen.
    pause
    exit /b 1
)

echo.
echo === BUILD ERFOLGREICH ===
exit /b 0