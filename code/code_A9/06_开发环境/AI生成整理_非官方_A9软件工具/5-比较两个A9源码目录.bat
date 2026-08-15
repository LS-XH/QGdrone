@echo off
chcp 65001 >nul
cd /d "%~dp0"
if "%~1"=="" goto usage
if "%~2"=="" goto usage
python -m a9tools firmware-diff "%~1" "%~2" --html reports\firmware_diff.html
if exist "%~dp0reports\firmware_diff.html" start "" "%~dp0reports\firmware_diff.html"
pause
exit /b %ERRORLEVEL%

:usage
echo 用法：把两个已解压的 A9 源码目录依次拖到此批处理文件上，或在终端执行：
echo 5-比较两个A9源码目录.bat "源码目录A" "源码目录B"
pause
exit /b 2
