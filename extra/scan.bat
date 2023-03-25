@echo off
cd /D %~dp0
:repeat
cls
py .\scan.py --server %1
set key=s
set /p key=Vuoi ripetere la scansione [S/n]?: 
if /i %key%==s (
    goto repeat
)
