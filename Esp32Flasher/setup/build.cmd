@echo off
setlocal

cls

echo ***
echo *** Esp32Flasher - setup generation
echo ***

call .\Helpers\_ensure_innosetup.bat
if ERRORLEVEL 1 goto :Error

REM ****************************************************************************
REM *** Setup for INNOSetup
REM *** 
REM *** Specify the sign tool used by InnoSetup. 
REM *** $p will be replaced by the parameters given in the directive SignTool of the ISS file.

REM ***************************************************************************
REM *** Sign tool
REM *** Works only in the CSM LAN and cannot be replaced by $SOMETHING_ELSE

%_SC% /DVARIANT=full /DMYVERSION=1.0.0.1 /DDEPENDENCIES .\InnoScripts\Iss.iss
if ERRORLEVEL 1 goto :Error
cd Helpers
call _postBuild
cd ..
goto :Done

:Error
ECHO BUILD FAILED!!!

:Done

endlocal
pause