@echo off
REM Host unit tests. Aus dem test/-Ordner heraus aufrufen.

cmake -S . -B build -G Ninja || exit /b 1
cmake --build build || exit /b 1
ctest --test-dir build --output-on-failure
exit /b %ERRORLEVEL%