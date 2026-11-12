@echo off
title HRD FT-991A bridge - close this window to stop it
echo ============================================================
echo  HRD / FT-991A OmniRig bridge
echo  1. Make sure OmniRig shows On-line.
echo  2. Wait for the line "Listening on COM13" and about 15 seconds.
echo  3. Then start HRD (Yaesu FT-950 on COM12).
echo  To stop: close HRD first, then close this window.
echo ============================================================
echo.
python "%~dp0omnirig_bridge.py" COM13 --baud 38400
echo.
echo The bridge has stopped.
pause
