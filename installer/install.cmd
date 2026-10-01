@echo off
rem Installs the replacement SimDLL.dll. See README.txt; install.ps1 does the work.
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0install.ps1" %*
echo.
pause
