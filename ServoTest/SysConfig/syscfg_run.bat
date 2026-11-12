@echo off
rem Common SysConfig launcher. Usage: syscfg_run.bat build | gui
rem build = CLI, regenerate User\ti_msp_dl_config.c/.h (Keil Before Build)
rem gui   = open SysConfig GUI for User\config.syscfg (Keil Tools menu)

set "MODE=%~1"
if /I "%MODE%"=="gui" goto mode_ok
if /I "%MODE%"=="build" goto mode_ok
echo [SYSCFG] ERROR: usage: syscfg_run.bat build ^| gui
exit /b 1
:mode_ok

set "SYSCFG_DIR=%~dp0"
if "%SYSCFG_DIR:~-1%"=="\" set "SYSCFG_DIR=%SYSCFG_DIR:~0,-1%"
set "PROJ_ROOT=%SYSCFG_DIR%\.."
if "%PROJ_ROOT:~-1%"=="\" set "PROJ_ROOT=%PROJ_ROOT:~0,-1%"
set "USER_DIR=%PROJ_ROOT%\User"
set "PRODUCT_JSON=%SYSCFG_DIR%\product.json"
set "SYSCFG_PATH=%USER_DIR%\config.syscfg"

if not defined SYSCFG_ROOT if exist "%SYSCFG_DIR%\syscfg_root.txt" (
    set /p SYSCFG_ROOT=<"%SYSCFG_DIR%\syscfg_root.txt"
)
set "SYSCFG_ROOT=%SYSCFG_ROOT:"=%"
if "%SYSCFG_ROOT:~-1%"=="\" set "SYSCFG_ROOT=%SYSCFG_ROOT:~0,-1%"

if not defined SYSCFG_ROOT (
    echo [SYSCFG] ERROR: SysConfig path not set.
    echo [SYSCFG] Copy SysConfig\syscfg_root.txt.example to SysConfig\syscfg_root.txt
    exit /b 1
)
if "%SYSCFG_ROOT%"=="" (
    echo [SYSCFG] ERROR: SysConfig path is empty in syscfg_root.txt
    exit /b 1
)

if not exist "%PRODUCT_JSON%" (
    echo [SYSCFG] ERROR: product.json not found: %PRODUCT_JSON%
    exit /b 1
)
if not exist "%SYSCFG_PATH%" (
    echo [SYSCFG] ERROR: config not found: %SYSCFG_PATH%
    exit /b 1
)

if /I "%MODE%"=="build" goto do_build
if /I "%MODE%"=="gui" goto do_gui
exit /b 1

:do_build
set "SYSCFG_CLI=%SYSCFG_ROOT%\sysconfig_cli.bat"
if not exist "%SYSCFG_CLI%" (
    echo [SYSCFG] ERROR: CLI not found: %SYSCFG_CLI%
    exit /b 1
)
echo [SYSCFG] mode=build CLI=%SYSCFG_CLI%
echo [SYSCFG] product=%PRODUCT_JSON%
echo [SYSCFG] input=%SYSCFG_PATH%
echo [SYSCFG] output=%USER_DIR%
call "%SYSCFG_CLI%" -o "%USER_DIR%" -s "%PRODUCT_JSON%" --compiler keil "%SYSCFG_PATH%"
set "RC=%ERRORLEVEL%"
if not "%RC%"=="0" (
    echo [SYSCFG] ERROR: generation failed, code %RC%
    exit /b %RC%
)
echo [SYSCFG] OK: ti_msp_dl_config.c / .h updated
exit /b 0

:do_gui
if not exist "%SYSCFG_ROOT%\sysconfig_gui.bat" (
    echo [SYSCFG] ERROR: sysconfig_gui.bat not found under %SYSCFG_ROOT%
    exit /b 1
)
echo [SYSCFG] mode=gui HOME=%SYSCFG_ROOT%
call "%SYSCFG_ROOT%\sysconfig_gui.bat" --compiler keil -s "%PRODUCT_JSON%" "%SYSCFG_PATH%"
exit /b %ERRORLEVEL%
