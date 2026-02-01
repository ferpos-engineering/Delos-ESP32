@echo off
setlocal
set PY=%~dp0python\python.exe
"%PY%" -E -s -m esptool %*
endlocal
