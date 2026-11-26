#!/usr/bin/env bash
# Stops every leftover piece of the simulation (Gazebo, ROS bridge, EOG sim).
# Run this if a previous run was closed badly and the arm "misses" the cube on the next run.
pkill -f "gz sim" 2>/dev/null
pkill -f parameter_bridge 2>/dev/null
pkill -f eog_arm_sim.py 2>/dev/null
echo "Simulation processes stopped."
