@echo off
chcp 65001 >nul
cd /d "%~dp0"
python -m a9tools self-test
pause
