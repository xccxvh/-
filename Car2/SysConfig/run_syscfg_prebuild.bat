@echo off
rem Keil Before Build wrapper. Keil: cmd.exe /C "..\SysConfig\run_syscfg_prebuild.bat"
call "%~dp0syscfg_run.bat" build %*
exit /b %ERRORLEVEL%
