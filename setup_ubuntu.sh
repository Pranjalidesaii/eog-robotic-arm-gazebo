#!/usr/bin/env bash
# One-time setup for the EOG robotic-arm Gazebo simulation.
# Target: Ubuntu 24.04 (native, or WSL2 "Ubuntu-24.04" on Windows 11).
# Installs: ROS 2 Jazzy, Gazebo Harmonic + ros_gz bridge, and the Python packages.
# Usage:  ./setup_ubuntu.sh        (asks for your sudo password)
set -e

if ! grep -q "Ubuntu 24.04" /etc/os-release 2>/dev/null; then
  echo "This script is for Ubuntu 24.04 (Noble). Detected:"; grep PRETTY_NAME /etc/os-release || true
  echo "ROS 2 Jazzy + Gazebo Harmonic officially support Ubuntu 24.04 only. Aborting."
  exit 1
fi

echo "==> Enabling the 'universe' repository and base tools"
sudo apt-get update
sudo apt-get install -y software-properties-common curl gnupg lsb-release locales
sudo add-apt-repository -y universe

echo "==> Adding the ROS 2 apt repository (if not already present)"
if [ ! -f /etc/apt/sources.list.d/ros2.list ]; then
  sudo mkdir -p /usr/share/keyrings
  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
       -o /usr/share/keyrings/ros-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
fi
sudo apt-get update

echo "==> Installing ROS 2 Jazzy, Gazebo Harmonic (via ros_gz) and Python dependencies"
# ros-jazzy-ros-gz pulls in Gazebo Harmonic (gz-sim 8) and ros_gz_bridge.
sudo apt-get install -y \
  ros-jazzy-ros-base \
  ros-jazzy-ros-gz \
  python3-numpy python3-scipy python3-matplotlib python3-tk

echo
echo "==> Done. Verifying..."
source /opt/ros/jazzy/setup.bash
command -v gz   >/dev/null && echo "  gz sim : $(gz sim --versions 2>/dev/null | head -1)" || echo "  gz NOT found"
command -v ros2 >/dev/null && echo "  ros2   : found" || echo "  ros2 NOT found"
python3 -c "import numpy, scipy, matplotlib, tkinter; print('  python : numpy', numpy.__version__, '| scipy', scipy.__version__, '| matplotlib', matplotlib.__version__)"
echo
echo "Now run:  ./run_demo.sh"
