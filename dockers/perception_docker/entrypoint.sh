#!/bin/bash
set -e

# Source ROS2 and workspace
source /opt/ros/humble/setup.bash
source /root/humble_ws/install/setup.bash

# Set FastDDS config
export FASTRTPS_DEFAULT_PROFILES_FILE=/root/.ros/fastdds.xml

echo "================================================"
echo "Manipulation Scenario Container Started"
echo "================================================"
echo "Starting Isaac Sim + MoveIt2 integration..."
echo "================================================"

# Start Isaac MoveIt integration with Simple Grasping
ros2 launch isaac_moveit isaac_moveit_perception.launch.py ros2_control_hardware_type:=isaac

# Keep container running
wait
