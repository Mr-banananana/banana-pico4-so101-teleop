@echo off
setlocal
cd /d "%~dp0.."
set "PICO_VENV=.venv-so101-hardware"
set "PICO_PYTHON=%PICO_VENV%\Scripts\python.exe"
set "PICO_REQUIREMENTS=requirements-pico-teleop.txt"
set "PICO_STAMP=%PICO_VENV%\.requirements-pico-teleop.txt"

if not exist "%PICO_PYTHON%" (
  where py >nul 2>&1
  if errorlevel 1 goto :python_missing
  echo Creating the project environment with Python 3.12...
  py -3.12 -m venv "%PICO_VENV%"
  if errorlevel 1 goto :python_missing
)

"%PICO_PYTHON%" -c "import sys; raise SystemExit(0 if sys.version_info[:2] == (3, 12) else 1)"
if errorlevel 1 (
  echo The existing project environment is not Python 3.12.
  echo Remove the .venv-so101-hardware folder and run this launcher again.
  exit /b 1
)

if exist "%PICO_STAMP%" (
  fc /b "%PICO_REQUIREMENTS%" "%PICO_STAMP%" >nul 2>&1
  if not errorlevel 1 goto :ready
)
echo Installing or updating teleoperation dependencies. This may take a few minutes...
"%PICO_PYTHON%" -m pip install --disable-pip-version-check -r "%PICO_REQUIREMENTS%"
if errorlevel 1 (
  echo Dependency installation failed. Check your internet connection and try again.
  exit /b 1
)
copy /y "%PICO_REQUIREMENTS%" "%PICO_STAMP%" >nul
if errorlevel 1 (
  echo Could not save the dependency version marker; the next launch will check packages again.
)

:ready
exit /b 0

:python_missing
echo Python 3.12 is required. Install Python 3.12 with the Python Launcher, then run this file again.
echo Download: https://www.python.org/downloads/
exit /b 1
