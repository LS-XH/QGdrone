@echo off
chcp 65001 >nul
cd /d "%~dp0"
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "scripts\a9_flash.ps1" -Mode check
