@echo off
chcp 65001 >nul
cd /d "%~dp0"
python -m a9tools mission-check examples\mission_sample.json --html reports\mission_report.html
start "" "%~dp0reports\mission_report.html"
pause
