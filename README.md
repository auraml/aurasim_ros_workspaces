# aurasim_ros_workspaces
AuraSIM ROS Workspace. Contains ROS tutorials

## Quick Start Commands

### Install Dependencies
```bash
# Install all dependencies
cd humble_ws
rosdep install --from-paths src --ignore-src -r -y
```

### Build Workspace
```bash
# Build all packages
cd humble_ws
colcon build

# Build specific package
colcon build --packages-select robot_navigation

# Build with symlink install (for development)
colcon build --symlink-install
```

### Source Workspace
```bash
# Source the workspace (run after each build)
cd humble_ws
source install/setup.bash
```

### Launch Commands

#### Robot Navigation
```bash
# Launch navigation system
ros2 launch robot_navigation navigation.launch.py

# Launch with specific map
ros2 launch robot_navigation navigation.launch.py map:=/path/to/your/map.yaml

# Launch in simulation mode
ros2 launch robot_navigation navigation.launch.py use_sim_time:=true
```

#### RViz Visualization
```bash
# Launch RViz with navigation config
rviz2 -d src/robot_navigation/rviz2/robot_navigation.rviz
```

### Useful Development Commands
```bash
# Clean build
cd humble_ws
rm -rf build install log
colcon build

# Check package dependencies
rosdep check --from-paths src --ignore-src

# List available launch files
ros2 launch robot_navigation --show-args navigation.launch.py
```
