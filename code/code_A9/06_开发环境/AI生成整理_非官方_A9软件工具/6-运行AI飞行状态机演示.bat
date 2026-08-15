@echo off
chcp 65001 >nul
cd /d "%~dp0"
"D:\Flight\a9-project\python\.venv\Scripts\python.exe" -m a9tools scan-demo
pause
