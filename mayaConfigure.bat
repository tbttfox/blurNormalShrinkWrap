setlocal
REM "A quick batch file with variable options to kick off a cmake build on Windows"

SET MAYA_VERSION=2024
SET BUILD=mayabuild_%MAYA_VERSION%
SET COMPILER=Visual Studio 17 2022

SET PFX=%~dp0
cd %PFX%
REM rmdir %BUILD% /s /q
mkdir %BUILD%
cd %BUILD%

cmake ^
    -DMAYA_VERSION=%MAYA_VERSION% ^
    -G "%COMPILER%" ..\

cmake --build . --config RelWithDebInfo
pause

