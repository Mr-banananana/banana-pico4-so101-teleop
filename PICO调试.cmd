@echo off
cd /d "%~dp0"
if exist ".venv\Scripts\pythonw.exe" (
    start "" ".venv\Scripts\pythonw.exe" -m scripts.pico_debug
    exit /b
)
if exist ".venv\Scripts\python.exe" (
    ".venv\Scripts\python.exe" -m scripts.pico_debug
    exit /b
)
echo Missing .venv. Install requirements-pico-teleop.txt in a Python 3.12 virtual environment.
pause
