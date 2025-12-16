# MoveIt2 Manipulation Scenario (Panda Robot)

Docker container for running MoveIt2 manipulation stack with Franka Emika Panda robot connected to Isaac Sim.

```bash
cd dockers/manipulation_docker
./run_manipulation_example.sh --build
```

Container starts the launch file:
- `isaac_moveit.launch.py` — MoveIt2 stack with robot controllers

---

## Running Pick and Place Demo

Inside the running container:

```bash
ros2 run moveit_apps pick_and_place
```

This executes a complete pick-and-place sequence:
1. Move to pre-grasp position above the cube
2. Open gripper
3. Lower to grasp position (Cartesian path)
4. Close gripper
5. Lift to pre-grasp height (Cartesian path)
6. Lift to safe travel height
7. Move to drop position above basket
8. Open gripper to release cube
9. Return to home position

---

For detailed documentation, see:

[MoveIt2 Manipulation Tutorial](https://github.com/auraml/docs/blob/main/docs/tutorials/ros/moveit2_manipulation.md)
