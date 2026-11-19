# EOG-controlled robotic arm - Gazebo simulation

Simulates the project pipeline from the Phase-1 presentation:
EOG acquisition -> filtering / drift handling -> command detection (L/R/U/D/Blink) -> 4-DOF arm pick & place.

## Run (Windows 11 + WSL2 Ubuntu 24.04 with ROS 2 Jazzy + Gazebo Harmonic)
In PowerShell:

    wsl -d Ubuntu-24.04 -- bash -c 'cd "/mnt/c/Users/Pranjali Desai/OneDrive/Desktop/final_year/eog_arm_gazebo" && ./run_demo.sh'

Options: `--mode naive` (fixed threshold, no validation - shows why drift/false-command handling is needed),
`--speed 2` (faster), `--seed N`, `--clean` (no natural blinks/glances), `--headless --no-dashboard`.
Gazebo opens on the left, the live EOG dashboard docks on the right.

## Files
- `arm_config.py`   arm geometry, joint limits, IK (gripper kept vertical)
- `generate_world.py` -> `eog_arm_world.sdf`  Gazebo world (arm, table, cube, goal pad, joint controllers)
- `eog_arm_sim.py`  synthetic EOG + pipeline + arm controller + ROS 2 publisher + dashboard
- `run_demo.sh`     starts Gazebo, ros_gz_bridge and the simulation

## What is simulated / simplified
- EOG is synthetic (no recorded data): 250 Hz, 2 channels, 50 Hz hum, noise, slow baseline drift,
  natural blinks and involuntary glances.
- Detection is rule-based (adaptive baseline + dwell + amplitude + refractory), NOT the trained RF/SVM
  from the results slides.
- Commands: LEFT/RIGHT = shoulder pan, UP/DOWN = reach/lower along a path, BLINK = toggle gripper.
- Joints: pan, shoulder, elbow, wrist (auto-levelled to keep gripper down) + gripper; arm gravity is off
  (ideal servos).
