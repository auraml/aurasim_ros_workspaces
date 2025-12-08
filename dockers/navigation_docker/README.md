# Nav2 Navigation Scenario (Tiago Pro)

Docker container for running Nav2 navigation stack with Tiago Pro robot connected to Isaac Sim.

```bash
cd dockers/navigation_docker
./run_navigation_example.sh --build
```

Container starts two launch files sequentially:
1. `tiago_isaac_bringup.launch.py` — robot controllers
2. `tiago_navigation.launch.py` — Nav2 stack

---

For detailed documentation, see:

[Nav2 Navigation Tutorial](https://github.com/auraml/docs/blob/main/docs/tutorials/ros/nav2_cicd.md)
