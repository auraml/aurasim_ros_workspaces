#!/bin/bash
set -e

# Source ROS2 and workspace
source /opt/ros/humble/setup.bash
source /root/humble_ws/install/setup.bash

# Set FastDDS config
export FASTRTPS_DEFAULT_PROFILES_FILE=/root/.ros/fastdds.xml

echo "================================================"
echo "Nav2 Scenario Container Started"
echo "================================================"
echo "Starting TiaGo Isaac control..."
echo "================================================"

# Start control in background
ros2 launch tiago_pro_bringup tiago_isaac_bringup.launch.py &
CONTROL_PID=$!

# Wait for controller_manager to be ready
echo "Waiting for controller_manager to initialize..."
TIMEOUT=60
ELAPSED=0
while ! ros2 service list | grep -q "/controller_manager/list_controllers"; do
  if [ $ELAPSED -ge $TIMEOUT ]; then
    echo "ERROR: controller_manager service not available after ${TIMEOUT}s"
    exit 1
  fi
  echo "Waiting for controller_manager service... (${ELAPSED}s/${TIMEOUT}s)"
  sleep 1
  ELAPSED=$((ELAPSED + 1))
done
echo "controller_manager is ready!"

echo "================================================"
echo "Starting navigation..."
echo "================================================"

# Start navigation in background
ros2 launch robot_navigation tiago_navigation.launch.py &
NAV_PID=$!

echo "================================================"
echo "All services started!"
echo "Control PID: $CONTROL_PID"
echo "Navigation PID: $NAV_PID"
echo "Press Ctrl+C to stop all services"
echo "================================================"

# Wait for all background processes
wait
