@echo off
setlocal EnableExtensions
cd /d "%~dp0"

set "SOURCE=A2SearchAlgorithmsComparison.cpp"
if not exist "%SOURCE%" (
    if exist "A2 Search Algorithms Comparison.cpp" set "SOURCE=A2 Search Algorithms Comparison.cpp"
)
if not exist "%SOURCE%" (
    echo ERROR: Main C++ file not found in %CD%
    echo Expected A2SearchAlgorithmsComparison.cpp
    exit /b 1
)

if not exist "build" mkdir "build"

where cl.exe >nul 2>&1
if not errorlevel 1 goto msvc

where g++.exe >nul 2>&1
if not errorlevel 1 goto gcc

set "VSWHERE=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe"
if not exist "%VSWHERE%" set "VSWHERE=%ProgramFiles%\Microsoft Visual Studio\Installer\vswhere.exe"
if exist "%VSWHERE%" (
    for /f "usebackq delims=" %%I in (`"%VSWHERE%" -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath`) do set "VSROOT=%%I"
)

if defined VSROOT (
    if exist "%VSROOT%\Common7\Tools\VsDevCmd.bat" (
        call "%VSROOT%\Common7\Tools\VsDevCmd.bat" -arch=amd64 -host_arch=amd64 >nul
    )
)

where cl.exe >nul 2>&1
if not errorlevel 1 goto msvc

where g++.exe >nul 2>&1
if not errorlevel 1 goto gcc

echo.
echo ERROR: No C++ compiler was found.
echo Open Visual Studio Installer, choose Modify, and install "Desktop development with C++".
echo Then run run_experiments.bat again.
exit /b 1

:msvc
echo Building with MSVC...
cl /nologo /std:c++20 /EHsc /O2 /Fo:build\benchmark.obj /Fe:build\benchmark.exe "%SOURCE%"
exit /b %errorlevel%

:gcc
echo Building with g++...
g++ -std=c++20 -O2 "%SOURCE%" -o "build\benchmark.exe"
exit /b %errorlevel%
