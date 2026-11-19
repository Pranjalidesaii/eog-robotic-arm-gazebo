#!/usr/bin/env bash
# Launches: Gazebo (GUI) + ROS<->Gazebo bridge + EOG simulation/dashboard.
# Usage:  ./run_demo.sh [--mode robust|naive] [--speed 1.0] [--seed 7] [--headless]
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$HERE"
source /opt/ros/jazzy/setup.bash
export GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/ros/jazzy/opt/gz_sim_vendor/lib/gz-sim-8/plugins:$GZ_SIM_SYSTEM_PLUGIN_PATH
export GZ_SIM_RESOURCE_PATH="$HERE:$GZ_SIM_RESOURCE_PATH"

HEADLESS=0; ARGS=()
for a in "$@"; do [[ "$a" == "--headless" ]] && HEADLESS=1 || ARGS+=("$a"); done

python3 generate_world.py >/dev/null || exit 1
pkill -f "gz sim" 2>/dev/null; pkill -f parameter_bridge 2>/dev/null; sleep 1

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
