@echo off

set SERVICE_NAME="PantexApp Service"

sc.exe stop %SERVICE_NAME%

sc.exe delete %SERVICE_NAME%