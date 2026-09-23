@echo off
setlocal
chcp 65001 >nul
cd /d "%~dp0"

set "PY37=C:\Users\LENOVO\AppData\Local\Programs\Python\Python37\python.exe"
set "PY38=D:\Miniconda3\envs\yolov8\python.exe"
set "MODEL=%~dp0yolo11x_sprite_coke_v1.pt"
set "PORT=8768"
set "CENTER_LOG=%~dp0robomaster_first_left_coke_drop_test_log.csv"

if not exist "%PY37%" goto :missing_file
if not exist "%PY38%" goto :missing_file
if not exist "%MODEL%" goto :missing_file

"%PY37%" -c "import robomaster, cv2" || goto :dependency_error
"%PY38%" -c "import torch, ultralytics, cv2" || goto :dependency_error

echo ============================================================
echo Fixed sequence: front-left Coke, then Sprite, then remaining cans
echo 1. Connect this computer to the RoboMaster EP Wi-Fi.
echo 2. Put one Coke in the front-row far-left cell.
echo 3. Start on the grid centerline and face straight ahead.
echo 4. Clear the path behind the robot and on its left side.
echo 5. Coke: left 90, forward 0.30 m, release, back 0.30 m,
echo    right 100, back 0.08 m, then continue recognition.
echo 6. Sprite: right 90, forward 0.42 m, release, back 0.42 m,
echo    left 100, back 0.08 m, then continue recognition.
echo 7. If no can is visible, move forward 0.10 m, stop, then recognize again.
echo 8. Press S once to start. X is emergency stop. Q/Esc exits.
echo ============================================================
pause

ping -n 1 -w 1500 192.168.2.1 >nul
if errorlevel 1 (
  echo [WARNING] 192.168.2.1 did not answer ping.
  echo Check the RoboMaster Wi-Fi connection.
  pause
)

start "YOLO Fixed Six Grid Sequence" /D "%~dp0" "%PY38%" "%~dp0yolo_six_grid_sort_server_py38.py" --model "%MODEL%" --device 0 --labels coke_can,sprite_can --target-sequence coke_can,sprite_can,coke_can --confidence 0.30 --target-policy front-left --target-x 0.50 --align-deadband 0.025 --stable-frames 2 --lost-target-frames 5 --max-auto-pulses 20 --pid-kp 2.2 --pid-ki 0.05 --pid-kd 0.02 --pid-min-pulse 0.08 --pid-max-pulse 0.35 --approach-stop-center-y 0.530 --approach-hard-stop-center-y 0.540 --near-field-center-y 0.46 --near-align-tolerance 0.05 --final-grasp-zone-center-y 0.48 --final-grasp-tolerance 0.07 --fallback-grasp-center-y 0.515 --fallback-grasp-tolerance 0.085 --fallback-grasp-min-height 0.54 --stall-grasp-center-y 0.515 --stall-grasp-min-height 0.54 --stall-progress-epsilon 0.004 --stall-forward-pulses 3 --push-plateau-frames 5 --near-lateral-pulse 0.10 --near-lateral-rpm 20 --max-near-align-pulses 3 --absolute-stop-center-y 0.560 --approach-min-height 0.08 --distance-filter-frames 5 --approach-kp 1.0 --approach-min-pulse 0.08 --approach-max-pulse 0.18 --max-forward-pulses 30 --steer-kp 100 --steer-ki 2 --steer-kd 0.2 --steer-max-rpm 10 --steer-recovery-error 0.12 --max-sort-items 6 --empty-finish-frames 100000 --cycle-rearm-frames 3 --cycle-rearm-max-center-y 0.35 --search-forward-after-frames 10 --search-forward-distance 0.10 --max-search-forward-steps 10 --log-center-y-every-frame --center-y-log "%CENTER_LOG%" --control-mode auto-approach --port %PORT%
timeout /t 2 /nobreak >nul

"%PY37%" "%~dp0robomaster_six_grid_sort_client_py37.py" --conn-type ap --robot-ip 192.168.2.1 --max-missing-camera-frames 3 --max-identical-camera-frames 8 --camera-freeze-timeout 2.0 --safety-log "%~dp0robomaster_client_safety.log" --control-mode auto-approach --wheel-rpm 22 --forward-wheel-rpm 22 --max-steer-rpm 10 --yaw-trim-rpm 0 --pulse-duration 0.35 --soft-stop-rpm 0 --soft-stop-duration 0 --max-auto-pulses 20 --max-forward-pulses 30 --search-forward-distance 0.10 --search-move-speed 0.6 --max-search-forward-steps 10 --heading-tolerance-deg 1.0 --heading-max-correction-deg 3.0 --heading-correction-speed 10 --heading-settle-time 0.10 --grasp-after-approach --sort-mission --first-left-drop-test --fixed-left-turn-deg 90 --fixed-return-turn-deg 100 --fixed-drop-forward-distance 0.30 --fixed-drop-retreat-distance 0.30 --sprite-drop-forward-distance 0.42 --sprite-drop-retreat-distance 0.42 --post-turn-retreat-distance 0.08 --drop-lateral-distance 0.30 --sort-move-speed 0.6 --sort-action-timeout 12 --arm-x-mm 180 --arm-ready-y-mm 40 --arm-lower-y-mm 0 --arm-lift-y-mm 80 --arm-action-timeout 15 --gripper-wait 1.5 --pre-grasp-settle 0.5 --port %PORT%
set "RESULT=%ERRORLEVEL%"

echo.
echo Fixed Coke/Sprite sequence exited with code %RESULT%.
pause
exit /b %RESULT%

:missing_file
echo [ERROR] A configured Python executable or model file is missing.
pause
exit /b 2

:dependency_error
echo [ERROR] A required Python package is missing.
pause
exit /b 2
