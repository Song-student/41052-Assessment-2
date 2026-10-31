@echo off
setlocal
cd /d "%~dp0"

set "EXPERIMENT=%~1"
set "GRID_COUNT=%~2"
set "INTERACTIVE=0"

if "%EXPERIMENT%"=="" goto menu
goto start

:menu
set "INTERACTIVE=1"
echo.
echo Select an experiment:
echo 1. General
echo 2. Weight Scale and Distribution
echo 3. Mean and Maximum Edge Weights
echo 4. Sample Size (50 grids)
echo 5. Sample Size (100 grids)
echo 6. Fixed Mean Distribution
echo 7. Bucket Scanning
echo 0. Exit
echo.
choice /C 12345670 /N /M "Enter selection: "
set "SELECTED=%errorlevel%"
if "%SELECTED%"=="1" set "EXPERIMENT=general"
if "%SELECTED%"=="2" set "EXPERIMENT=weight-scale"
if "%SELECTED%"=="3" set "EXPERIMENT=mean-max"
if "%SELECTED%"=="4" (
    set "EXPERIMENT=sample-size"
    set "GRID_COUNT=50"
)
if "%SELECTED%"=="5" (
    set "EXPERIMENT=sample-size"
    set "GRID_COUNT=100"
)
if "%SELECTED%"=="6" set "EXPERIMENT=fixed-mean"
if "%SELECTED%"=="7" set "EXPERIMENT=bucket"
if "%SELECTED%"=="8" exit /b 0
if "%EXPERIMENT%"=="" goto menu

:start
if not defined SKIP_BUILD (
    call "%~dp0build.bat"
    if errorlevel 1 goto failed
)

if not exist "results" mkdir "results"

if /I "%EXPERIMENT%"=="sample-size" if "%GRID_COUNT%"=="" set "GRID_COUNT=100"

echo Running: %EXPERIMENT% %GRID_COUNT%

if /I "%EXPERIMENT%"=="sample-size" (
    set "OUTPUT=results\sample-size_%GRID_COUNT%.txt"
) else (
    set "OUTPUT=results\%EXPERIMENT%.txt"
)

powershell.exe -NoProfile -ExecutionPolicy Bypass -Command ^
    "$arguments = @($env:EXPERIMENT); if ($env:EXPERIMENT -eq 'sample-size') { $arguments += $env:GRID_COUNT }; & '.\build\benchmark.exe' @arguments 2>&1 | Tee-Object -FilePath $env:OUTPUT; exit $LASTEXITCODE"

if errorlevel 1 goto failed

echo Completed: %OUTPUT%
if "%INTERACTIVE%"=="1" pause
exit /b 0

:failed
echo.
echo Build or experiment failed. Check errors above and the results folder.
if "%INTERACTIVE%"=="1" pause
exit /b 1
