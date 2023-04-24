setlocal
REM "A quick batch file with variable options to kick off a cmake build on Windows"

SET MAYA_VERSION=2023
SET BUILD=mayabuild_%MAYA_VERSION%
SET COMPILER=Visual Studio 16 2019

SET PFX=%~dp0
cd %PFX%
REM rmdir %BUILD% /s /q
REM mkdir %BUILD%
cd %BUILD%

cmake ^
    -DMAYA_VERSION=%MAYA_VERSION% ^
    -G "%COMPILER%" ..\

REM cmake --build . --config Release --target INSTALL
pause

