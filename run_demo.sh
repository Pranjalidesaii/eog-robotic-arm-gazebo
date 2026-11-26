#!/usr/bin/env bash
# Launches: Gazebo (GUI) + ROS<->Gazebo bridge + EOG simulation/dashboard.
# Usage:  ./run_demo.sh [--mode robust|naive] [--speed 1.0] [--seed 7] [--headless]
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$HERE"

# ---- pre-flight checks: fail early with a clear message instead of a cryptic error ----------
die() { echo "ERROR: $1" >&2; echo "       See README.md (Setup) or run ./setup_ubuntu.sh" >&2; exit 1; }

[[ -f /opt/ros/jazzy/setup.bash ]] || die "ROS 2 Jazzy not found at /opt/ros/jazzy (this project needs Ubuntu 24.04 + ROS 2 Jazzy)."
source /opt/ros/jazzy/setup.bash
command -v gz   >/dev/null || die "'gz' (Gazebo Harmonic) not found. Install it with: sudo apt install ros-jazzy-ros-gz"
command -v ros2 >/dev/null || die "'ros2' not found after sourcing ROS."
PLUGDIR=/opt/ros/jazzy/opt/gz_sim_vendor/lib/gz-sim-8/plugins
[[ -d "$PLUGDIR" ]] || die "Gazebo plugin folder missing ($PLUGDIR). Install: sudo apt install ros-jazzy-gz-sim-vendor ros-jazzy-ros-gz"
ros2 pkg prefix ros_gz_bridge >/dev/null 2>&1 || die "ros_gz_bridge missing. Install: sudo apt install ros-jazzy-ros-gz"
python3 - <<'PY' || die "A Python package is missing (numpy / scipy / matplotlib / tkinter / rclpy). Install: sudo apt install python3-numpy python3-scipy python3-matplotlib python3-tk"
import numpy, scipy, matplotlib, rclpy
PY

HEADLESS=0; NODASH=0; ARGS=()
for a in "$@"; do
  [[ "$a" == "--headless" ]] && HEADLESS=1 || ARGS+=("$a")
  [[ "$a" == "--no-dashboard" ]] && NODASH=1
done
if [[ $HEADLESS == 0 || $NODASH == 0 ]]; then
  [[ -n "$DISPLAY$WAYLAND_DISPLAY" ]] || die "No display found (DISPLAY is empty). On Windows use Windows 11 WSL2 (WSLg); on a server run: ./run_demo.sh --headless --no-dashboard"
  python3 -c "import tkinter" 2>/dev/null || die "tkinter missing (needed for the dashboard). Install: sudo apt install python3-tk"
fi

export GZ_SIM_SYSTEM_PLUGIN_PATH=$PLUGDIR:$GZ_SIM_SYSTEM_PLUGIN_PATH
export GZ_SIM_RESOURCE_PATH="$HERE:$GZ_SIM_RESOURCE_PATH"

python3 generate_world.py >/dev/null || exit 1

# Stop leftovers from an earlier run - a stale simulation keeps publishing to the arm and
# makes the new run miss the cube.
pkill -f "gz sim" 2>/dev/null; pkill -f parameter_bridge 2>/dev/null; pkill -f eog_arm_sim.py 2>/dev/null; sleep 1

PIDS=()
cleanup() { kill "${PIDS[@]}" 2>/dev/null; pkill -f "gz sim" 2>/dev/null; }
trap cleanup EXIT INT TERM

if [[ $HEADLESS == 1 ]]; then
  gz sim -s -r eog_arm_world.sdf > /tmp/gz_sim.log 2>&1 &
else
  gz sim -r --gui-config "$HERE/gui.config" eog_arm_world.sdf > /tmp/gz_sim.log 2>&1 &
fi
PIDS+=($!)

echo "Waiting for Gazebo to load the world..."
for i in $(seq 1 90); do
  gz topic -l 2>/dev/null | grep -q "/eog_arm/elbow/cmd_pos" && break
  sleep 1
done

BR=()
for j in shoulder_pan shoulder_lift elbow wrist finger_left finger_right; do
  BR+=("/eog_arm/$j/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double")
done
BR+=("/world/eog_world/model/eog_arm/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model")
ros2 run ros_gz_bridge parameter_bridge "${BR[@]}" > /tmp/gz_bridge.log 2>&1 &
PIDS+=($!)
sleep 3
echo "Starting EOG simulation (close the dashboard window or press Ctrl+C to stop)"
python3 eog_arm_sim.py "${ARGS[@]}"
