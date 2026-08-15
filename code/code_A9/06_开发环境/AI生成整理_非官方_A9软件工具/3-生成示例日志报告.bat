@echo off
chcp 65001 >nul
cd /d "%~dp0"
python -m a9tools log-report examples\flight_sample.csv --out reports\flight_sample
start "" "%~dp0reports\flight_sample\report.html"
pause
