@echo off
REM run_logger.bat
REM Starts the Plantify data logger.
REM Place next to plantify_logger.py — double-click to run.

cd /d "%~dp0"

python --version >nul 2>&1
if errorlevel 1 (
    echo ERROR: Python not found. Install Python and add it to PATH.
    pause
    exit /b 1
)

if not exist plantify_logger.py (
    echo ERROR: plantify_logger.py not found in this folder.
    pause
    exit /b 1
)

echo Starting Plantify logger...
echo Press Ctrl+C to stop.
echo.
python plantify_logger.py
pause
