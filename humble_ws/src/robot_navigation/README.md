# robot_navigation

The `robot_navigation` package provides navigation capabilities for robots using ROS 2. It integrates with the Navigation2 stack and includes configurations for maps, parameters, and RViz visualization.

## Features
- Launch files for setting up navigation with Navigation2.
- Configurable parameters for robot navigation.
- Support for simulation time and namespace configuration.
- Integration with RViz for visualization.

## Files
- **CMakeLists.txt**: Defines the build system for the package.
- **package.xml**: Specifies package dependencies and metadata.
- **launch/navigation.launch.py**: Launch file to set up the Navigation2 stack, including map loading, parameter configuration, and RViz visualization.

## Dependencies
This package depends on several ROS 2 packages, including:
- `nav2_bringup`
- `nav2_amcl`
- `nav2_planner`
- `rviz2`
- `pointcloud_to_laserscan`
- And more (see `package.xml` for the full list).

## Usage
To launch the navigation stack, use the following command:
```bash
ros2 launch robot_navigation navigation.launch.py use_namespace:=true namespace:=robot1
```

You can customize the launch by providing arguments such as:
- `map`: Path to the map file.
- `params_file`: Path to the parameter file.
- `use_sim_time`: Whether to use simulation time.
- `namespace`: Namespace for the nodes.
- `use_namespace`: Whether to apply the namespace to launched nodes.

## License
This package is licensed under the Apache-2.0 License. See the `package.xml` for details.