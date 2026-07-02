@echo off
setlocal

rem =====================================================================
rem Headless build for AurixTricore
rem
rem Uses a dedicated Eclipse workspace (ads_headless_ws) so it works even
rem while the ADS GUI is open. Do NOT start an IDE build while this runs.
rem
rem Known issue: Infineon's "booster" builder crashes headless ("Workbench
rem has not been created yet") because it needs the GUI. It is temporarily
rem removed from .project for the build; the standard CDT genmakebuilder
rem performs the actual amk build. .project is restored afterwards.
rem =====================================================================

set ADS=C:\Infineon\AURIX-Studio-1.10.32
set WORKSPACE=C:\Users\chris\Projects\ads_headless_ws
set PROJECT_DIR=C:\Users\chris\Projects\AurixTricore
set BUILD_DIR=%PROJECT_DIR%\TriCore Debug (TASKING)
set ELF=%BUILD_DIR%\AurixTricore.elf

rem --- import project into headless workspace on first run ---
set IMPORTARG=
if not exist "%WORKSPACE%\.metadata\.plugins\org.eclipse.core.resources\.projects\AurixTricore" (
    set IMPORTARG=-import "%PROJECT_DIR%"
)

rem --- strip GUI-only Infineon builders from .project (backup first) ---
copy /Y "%PROJECT_DIR%\.project" "%PROJECT_DIR%\.project.ads-backup" >nul
powershell -NoProfile -Command "$p='%PROJECT_DIR%\.project'; $x=[xml](Get-Content $p -Raw); $x.SelectNodes(\"//buildCommand[starts-with(name,'com.infineon.')]\") | ForEach-Object { [void]$_.ParentNode.RemoveChild($_) }; $x.Save($p)"
if errorlevel 1 (
    copy /Y "%PROJECT_DIR%\.project.ads-backup" "%PROJECT_DIR%\.project" >nul
    echo .project konnte nicht angepasst werden. Abbruch.
    exit /b 1
)

rem --- delete old artifact so a skipped build cannot look successful ---
if exist "%ELF%" del "%ELF%"

echo === BUILD ===
"%ADS%\AURIX-studioc.exe" --launcher.suppressErrors -nosplash ^
  -data "%WORKSPACE%" ^
  -application com.tasking.managedbuilder.headlessbuild ^
  %IMPORTARG% ^
  -build "AurixTricore/TriCore Debug .TASKING." ^
  --launcher.ini "%ADS%\AURIX-studio-headless.ini"

rem --- always restore the original .project for the IDE ---
copy /Y "%PROJECT_DIR%\.project.ads-backup" "%PROJECT_DIR%\.project" >nul
del "%PROJECT_DIR%\.project.ads-backup"

if not exist "%ELF%" (
    echo.
    echo === BUILD FEHLGESCHLAGEN: kein neues ELF ===
    exit /b 1
)

echo.
echo === BUILD ERFOLGREICH: "%ELF%" ===
exit /b 0
