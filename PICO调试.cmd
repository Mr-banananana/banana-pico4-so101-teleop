@echo off
setlocal
cd /d "%~dp0"
call "%~dp0scripts\prepare_pico_env.cmd"
if errorlevel 1 (
  pause
  exit /b 1
)
if not exist "%~dp0.venv-so101-hardware\Scripts\pythonw.exe" (
  echo Python window launcher is missing. Reinstall Python 3.12 and try again.
  pause
  exit /b 1
)
start "" "%~dp0.venv-so101-hardware\Scripts\pythonw.exe" -m scripts.pico_debug
exit /b 0
