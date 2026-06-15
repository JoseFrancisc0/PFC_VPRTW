@echo off
setlocal enabledelayedexpansion

echo ========================================================
echo GENERACION DE DATASET MASIVO PARA DQN (OFFLINE RL)
echo ========================================================

:: Crear las carpetas necesarias si no existen
if not exist "..\Results\CLASSIC\metrics" mkdir "..\Results\CLASSIC\metrics"
if not exist "..\Results\CLASSIC\routes" mkdir "..\Results\CLASSIC\routes"
if not exist "experiences" mkdir "experiences"

:: Hiperparametros
set ITERATIONS=25000
set RUNS_PER_INSTANCE=3

echo Generando %RUNS_PER_INSTANCE% ejecuciones por cada instancia...

for /r "..\solomon-100" %%F in (*.txt) do (
    echo Procesando instancia: %%~nxF
    
    for /L %%R in (1, 1, %RUNS_PER_INSTANCE%) do (
        echo   -^> Ejecucion %%R de %RUNS_PER_INSTANCE%
        ..\build\Release\ALNS_VRPTW.exe "%%F" CLASSIC %ITERATIONS% %%R
    )
    
    echo Terminado: %%~nxF
    echo --------------------------------------------------------
)

echo.
echo ========================================================
echo GENERACION DE DATASET MULTI-RUN COMPLETADA
echo Todas las experiencias han sido guardadas en DQN_Pipeline\experiences\
echo ========================================================
pause
