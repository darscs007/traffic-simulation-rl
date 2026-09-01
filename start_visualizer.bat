@echo off
setlocal
cd /d "%~dp0"

netstat -ano | findstr /r /c:":8000 .*LISTENING" >nul
if not errorlevel 1 goto :open_browser

py -3 --version >nul 2>nul
if not errorlevel 1 (
  start "Traffic visualizer server" /min cmd /k py -3 -m http.server 8000 --bind 127.0.0.1
  goto :open_browser
)

python --version >nul 2>nul
if not errorlevel 1 (
  start "Traffic visualizer server" /min cmd /k python -m http.server 8000 --bind 127.0.0.1
  goto :open_browser
)

if exist "C:\msys64\ucrt64\bin\python.exe" (
  start "Traffic visualizer server" /min cmd /k "C:\msys64\ucrt64\bin\python.exe" -m http.server 8000 --bind 127.0.0.1
  goto :open_browser
)

echo Python 3 was not found on this system.
echo Please install Python from https://www.python.org/downloads/ and check "Add Python to PATH".
pause
exit /b 1

:open_browser
timeout /t 0.5 /nobreak >nul
start "" "http://localhost:8000/visualizer.html"
