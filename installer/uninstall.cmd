@echo off
rem Puts the game's own SimDLL.dll back. See README.txt; install.ps1 does the work.
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0install.ps1" -Uninstall %*
echo.
pause
