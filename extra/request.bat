@echo off
cd /D %~dp0
cls
py .\request.py --ip %1 %2
echo.
