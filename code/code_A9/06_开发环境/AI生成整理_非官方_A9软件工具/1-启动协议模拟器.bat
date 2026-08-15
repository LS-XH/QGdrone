@echo off
chcp 65001 >nul
cd /d "%~dp0"
python -m a9tools simulate --host 127.0.0.1 --port 14560 --rate 2
pause
