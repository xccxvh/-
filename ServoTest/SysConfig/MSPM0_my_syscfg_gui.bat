@echo off
rem Keil Tools wrapper. Command=..\SysConfig\MSPM0_my_syscfg_gui.bat , Arguments empty
call "%~dp0syscfg_run.bat" gui %*
exit /b %ERRORLEVEL%
