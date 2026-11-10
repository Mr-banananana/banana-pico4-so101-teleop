@echo off
setlocal
cd /d "%~dp0"
set "PICO_MOTION_PROFILE=agile_fast"
if exist ".venv-so101-hardware\Scripts\pythonw.exe" (
    start "" ".venv-so101-hardware\Scripts\pythonw.exe" -m scripts.pico_debug --real
    exit /b
)
echo Missing .venv-so101-hardware environment.
pause
