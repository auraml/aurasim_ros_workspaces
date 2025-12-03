# Nav2 Navigation Scenario (Tiago Pro)

Docker container for running Nav2 navigation stack with Tiago Pro robot connected to Isaac Sim.

## Table of Contents

- [Quick Start](#quick-start)
- [Controllers](#controllers)
- [Nav2 Launch](#nav2-launch)
- [Simple Commander Usage](#simple-commander-usage)

## Quick Start

```bash
cd dockers/navigation_docker
./run_navigation_example.sh --build
```

Container starts two launch files sequentially:
1. `tiago_isaac_bringup.launch.py` — robot controllers
2. `tiago_navigation.launch.py` — Nav2 stack

## Controllers

`ros2 launch tiago_pro_bringup tiago_isaac_bringup.launch.py`

Brings up ros2_control stack for Tiago Pro robot. Launch file performs following:

1. Starts `controller_manager` node with hardware interface configuration from `tiago_pro_description/ros2_control/gazebo_controller_manager_cfg.yaml`. This node manages all ros2_control controllers and communicates with Isaac Sim through topic-based hardware interface.

2. Spawns default controllers via `tiago_pro_controller_configuration/default_controllers.launch.py`. Controllers handle joint trajectories for arms, base velocity, head and gripper movements.

3. Launches `robot_state_publisher` from `tiago_pro_description` package. Publishes robot URDF to `/robot_description` topic and broadcasts TF transforms based on joint states.

4. Starts `twist_mux` node for velocity command multiplexing. Handles priority between different cmd_vel sources (navigation, joystick, etc.) based on configuration in `tiago_pro_bringup/config/twist_mux/`.

5. Launches `play_motion2` for predefined motion execution. Allows triggering named motions like "home", "wave", etc.

6. Starts `gripper_grasper` wrapper node for simplified gripper control interface.

## Nav2 Launch

`ros2 launch robot_navigation tiago_navigation.launch.py`

Wraps standard `nav2_bringup/bringup_launch.py` with Tiago-specific configuration. Launch arguments:

- `map` — path to map yaml file (default: `robot_navigation/maps/warehouse.yaml`)
- `params_file` — navigation parameters (default: `robot_navigation/params/tiago_navigation_params.yaml`)
- `use_sim_time` — use simulation clock (default: `true`)
- `use_rviz` — launch RViz visualization (default: `false`)

Navigation stack components started:

1. **Map Server** — loads occupancy grid from warehouse map yaml/png files. Map has 0.05m resolution.

2. **AMCL** — Adaptive Monte Carlo Localization using `nav2_amcl::OmniMotionModel` for omnidirectional base. Uses front lidar `/TiagoPro/TIM781_front/scan` for particle filter updates. Configured with 500-2000 particles, global frame `world`, odom frame `odom`.

3. **Planner Server** — global path planning using `nav2_navfn_planner/NavfnPlanner` (Dijkstra algorithm). Tolerance 0.5m, allows planning through unknown space.

4. **Controller Server** — local trajectory tracking using `nav2_mppi_controller::MPPIController`. Configured for omni motion model with velocity limits vx: [-0.35, 1.0], vy: [-0.35, 1.0], wz: 1.9 rad/s. Uses 2000 trajectory samples with 56 time steps at 0.1s intervals.

5. **Behavior Server** — recovery behaviors: spin, backup, wait, drive_on_heading. Activated when robot gets stuck or path is blocked.

6. **Smoother Server** — path smoothing using `nav2_smoother::SimpleSmoother`.

7. **Waypoint Follower** — multi-goal navigation with `WaitAtWaypoint` task executor (200ms pause at each waypoint).

8. **Velocity Smoother** — smooths velocity commands to prevent jerky motion. Max velocity [1.0, 0.0, 1.9], max acceleration [2.0, 0.0, 3.0].

9. **BT Navigator** — behavior tree based navigation using default `navigate_to_pose_w_replanning_and_recovery.xml`. Global frame `world`, robot frame `base_link`, uses odometry from `/TiagoPro/odom`.

10. **Lifecycle Manager** — manages node lifecycle transitions (configure, activate, deactivate, cleanup).

Costmap configuration:

- **Local Costmap** — 3x3m rolling window, 0.05m resolution. Uses voxel layers for both front (`/TiagoPro/TIM781_front/scan`) and rear (`/TiagoPro/TIM781_rear/scan`) lidars. Inflation radius 0.05m.

- **Global Costmap** — static map with obstacle layers for front and rear lidars. Inflation radius 0.1m. Robot footprint `[0.35, 0.24]` meters.

## Simple Commander Usage

Test script demonstrates nav2_simple_commander Python API:

```bash
ros2 run nav2_apps nav2_tiago_test
```

Implementation in `nav2_apps/nav2_apps/nav2_tiago_test.py` shows:

1. **Initialization** — create `BasicNavigator` instance, call `waitUntilNav2Active()` to wait for all Nav2 servers to become active.

2. **Initial Pose** — set robot starting position using `setInitialPose()`. Creates `PoseStamped` message with frame_id `world` and publishes to `/initialpose` topic for AMCL.

3. **Navigation** — call `goToPose()` with goal `PoseStamped`. Method sends goal to `/navigate_to_pose` action server.

4. **Feedback Loop** — poll `isTaskComplete()` and `getFeedback()` to monitor navigation progress. Feedback contains `distance_remaining` to goal.

5. **Result Handling** — check `getResult()` for `TaskResult.SUCCEEDED`, `CANCELED`, or `FAILED`.

Test executes square route through 4 waypoints: `(0,0) → (2,0) → (2,2) → (0,2) → (0,0)`

Available BasicNavigator methods:

| Method | Action Server | Description |
|--------|---------------|-------------|
| `goToPose(pose)` | `/navigate_to_pose` | Navigate to single goal |
| `followWaypoints(poses)` | `/follow_waypoints` | Sequential waypoint navigation |
| `goThroughPoses(poses)` | `/navigate_through_poses` | Navigate through intermediate poses |
| `getPath(start, goal)` | `/compute_path_to_pose` | Compute path without executing |
| `smoothPath(path)` | `/smooth_path` | Smooth computed path |
