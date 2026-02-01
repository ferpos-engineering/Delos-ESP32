@echo off
REM ***************************************************************************
REM *** Ensure availability of InnoSetup Compiler
REM ***************************************************************************

REM *   Resulting environment settings:
REM *
REM *   _SC     Path to InnoSetup Compiler
REM *
REM ****************************************************************************


REM *** Choose a InnoSetup installation folder
REM *** Try Innosetup6 first ...
set _SC="%LOCALAPPDATA%\Programs\Inno Setup 6\ISCC.EXE"
if exist %_SC% goto :DoIt
set _SC="%ProgramFiles%\Inno Setup 6\ISCC.EXE"
if exist %_SC% goto :DoIt
set _SC="%ProgramFiles(x86)%\Inno Setup 6\ISCC.EXE"
if exist %_SC% goto :DoIt
set _SC="Z:\Applikationen\TOOLS\InnoSetup\V6.0\ISCC.EXE"
if exist %_SC% goto :DoIt


REM *** Then try InnoSetup5
set _SC="%ProgramFiles%\Inno Setup 5\ISCC.EXE"
if exist %_SC% goto :DoIt
set _SC="%ProgramFiles(x86)%\Inno Setup 5\ISCC.EXE"
if exist %_SC% goto :DoIt
set _SC="Z:\Applikationen\TOOLS\InnoSetup\V5.0\ISCC.EXE"
if exist %_SC% goto :DoIt

REM *** Nothing found, can't continue in this case
Echo *** InnoSetup Compiler not found ! ***
exit /B 1


:DoIt
Echo *** InnoSetup: %_SC%
REM ****************************************************************************

