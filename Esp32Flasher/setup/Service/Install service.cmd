@echo off

echo %1

set SERVICE_NAME="PantexApp Service"
set SERVICE_PATH=%1

sc.exe create %SERVICE_NAME% binpath=%SERVICE_PATH%

sc.exe description %SERVICE_NAME% "Integrates WorkMed services in PantexApp."

sc.exe config %SERVICE_NAME% start=auto

sc.exe start %SERVICE_NAME%