# Isaac Sim MVP

This directory contains the first config-driven Isaac Sim integration. It builds
a simple facade, modular grid, and robot placeholder from `configs/*.yaml`, then
publishes:

- `/clock`
- `/tf`
- `/grid/imu/raw`
- `/robot/imu/raw`
- `/grid/lidar/points` unless LiDAR is disabled

The generated stage is saved to `outputs/isaac/mvp_scene.usda`.

The scene includes a ground plane, gravity, collision geometry, a high-mass
kinematic facade, a finite-mass dynamic grid, and a hollow grid frame. The robot
body is mounted on the facade-facing side of the grid. A depth-camera placeholder
faces the facade from the robot, and the Isaac Sim HESAI XT32 SD10 RTX LiDAR
asset is mounted on the grid module above the robot.

## Run

Close Conda and source ROS 2 before launching:

```bash
conda deactivate 2>/dev/null || true
source /opt/ros/jazzy/setup.bash
./isaac/run_mvp.sh
```

For a bounded headless smoke test without RTX LiDAR:

```bash
source /opt/ros/jazzy/setup.bash
./isaac/run_mvp.sh --headless --test --no-lidar
```

Use a second ROS 2 terminal to inspect the bridge:

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic list
ros2 topic echo /robot/imu/raw --once
ros2 topic echo /grid/imu/raw --once
ros2 topic hz /grid/lidar/points
```

For a bounded headless ROS validation, keep the simulation near wall-clock time:

```bash
./isaac/run_mvp.sh --headless --no-lidar --realtime --frames 2400
```

Set `ISAAC_SIM_DIR` if the release build is installed elsewhere:

```bash
ISAAC_SIM_DIR=/path/to/isaacsim ./isaac/run_mvp.sh
```
