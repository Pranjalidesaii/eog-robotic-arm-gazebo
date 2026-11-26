# EOG-controlled robotic arm - Gazebo simulation

Simulation of the project *Controlling Robotic Arm Using EOG Signals*:
EOG acquisition -> filtering / drift handling -> command detection (Left / Right / Up / Down / Blink) -> arm pick & place in Gazebo.

A scripted "user" looks left, down, blinks, looks up, right, down, blinks... and the arm turns to a red cube,
grabs it and places it on a green pad. A live dashboard shows the EOG signals, the detected commands and an event log.

## What you need

| Requirement | Notes |
|---|---|
| **Ubuntu 24.04** | Native install, a VM, or **WSL2 on Windows 11** (see below). |
| **ROS 2 Jazzy** | Provides `rclpy` and `ros2`. |
| **Gazebo Harmonic** (gz-sim 8) + **ros_gz_bridge** | Installed together by `ros-jazzy-ros-gz`. |
| Python 3.12 packages | `numpy`, `scipy`, `matplotlib`, `tkinter` (`python3-tk`) |
| A display | Windows 11 WSLg works out of the box. Headless mode needs none. |

**Not supported:** macOS and native Windows (Gazebo Harmonic + ROS 2 Jazzy target Ubuntu 24.04). Use WSL2 or a VM.

## Setup

### Windows 11 (WSL2)
1. Open **PowerShell as Administrator** and run: `wsl --install -d Ubuntu-24.04`, then restart and create the Ubuntu user.
   (Windows 11 shows Linux GUI windows automatically through WSLg; update with `wsl --update` if windows don't appear.)
2. Open the **Ubuntu 24.04** app and follow the Ubuntu steps below.

### Ubuntu 24.04
```bash
git clone https://github.com/Pranjalidesaii/eog-robotic-arm-gazebo.git
cd eog-robotic-arm-gazebo
chmod +x *.sh
./setup_ubuntu.sh        # one-time: installs ROS 2 Jazzy, Gazebo Harmonic, ros_gz and the Python packages
```
`setup_ubuntu.sh` runs `apt` with `sudo` (needs your password) and takes several minutes the first time.
If you already have ROS 2 Jazzy, you only need:
`sudo apt install ros-jazzy-ros-gz python3-numpy python3-scipy python3-matplotlib python3-tk`

> Tip for WSL2 users: clone into the Linux home folder (`~`), not `/mnt/c/...` - it is faster and avoids line-ending problems.

## Run
```bash
./run_demo.sh
```
Wait ~15-20 s. A **Gazebo** window (3D arm) and the **EOG dashboard** (graphs) open, then the arm moves by itself for
roughly 40-90 s. At the end the terminal prints a line such as:

    scenario complete - cube is 3.8 cm from goal centre: SUCCESS | rejected involuntary events: 19 | mean command latency 570 ms

Options (append to `./run_demo.sh`):

| Option | Meaning |
|---|---|
| `--mode naive` | Fixed threshold, no validation - shows why drift / false-command handling is needed (false blink, drift moves). |
| `--speed 2` | Run the scenario faster. |
| `--seed N` | Different random noise / natural blinks. |
| `--clean` | No natural blinks or involuntary glances. |
| `--headless --no-dashboard` | No windows at all (servers / CI). Only the text log and result line. |
| `--headless` | No Gazebo window, dashboard only. |

Stop with **Ctrl+C** or by closing the dashboard window.

## Troubleshooting
- **`ERROR: ... not found` right at start** - the pre-flight check in `run_demo.sh` tells you which package is missing; run `./setup_ubuntu.sh`.
- **No window appears (WSL2)** - run `wsl --update`, then `wsl --shutdown` in PowerShell and start again.
- **Arm closes the gripper but misses the cube / behaves strangely on a second run** - an earlier run is still alive
  and fights the new one. Run `./stop_demo.sh` (or `wsl --shutdown` on Windows) and start again.
  `run_demo.sh` also tries to clean up leftovers automatically.
- **Gazebo window is blank for a while** - first start compiles/loads the world; wait ~20 s. Software rendering in a VM
  can be slow; the result line still appears.
- **`MISSED` in the result line** - run once more (`./stop_demo.sh` first). The simulation is physics-based, so the final
  position varies by a few cm between runs; a heavily loaded machine can cause a miss.

## Files
- `arm_config.py` - arm geometry, joint limits, inverse kinematics (gripper kept vertical)
- `generate_world.py` -> `eog_arm_world.sdf` - Gazebo world (arm, table, cube, goal pad, joint controllers)
- `eog_arm_sim.py` - synthetic EOG + detection pipeline + arm controller + ROS 2 publisher + dashboard
- `gui.config` - Gazebo window layout
- `run_demo.sh` - checks dependencies, starts Gazebo, the ros_gz bridge and the simulation
- `stop_demo.sh` - kills leftover simulation processes
- `setup_ubuntu.sh`, `requirements.txt` - dependency installation

## What is simulated / simplified
- EOG is **synthetic** (no recorded data): 250 Hz, 2 channels, 50 Hz hum, noise, slow baseline drift,
  natural blinks and involuntary glances.
- Detection is **rule-based** (50 Hz notch, moving average, adaptive baseline, dwell time, amplitude threshold,
  refractory period), not the trained RF/SVM models from the project results.
- Commands: LEFT/RIGHT = shoulder pan, UP/DOWN = reach/lower along a path, BLINK = toggle gripper.
- Joints: pan, shoulder, elbow, wrist (auto-levelled to keep the gripper pointing down) + gripper;
  arm gravity is off (ideal servos).
- Command latency is about 0.5 s (dwell time + decision window).
