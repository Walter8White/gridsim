# Gridsim

`Gridsim` is the simulation foundation for a deployable modular grid positioned
approximately 1-1.5 m in front of a building facade. A mobile robot travels on
the grid while passive sensors support localization, calibration, and structural
state estimation.

This repository is intentionally a small first foundation. Isaac Sim will provide
the scene and sensor simulation, ROS 2 Jazzy will provide runtime communication,
and the Python modules here define testable models that can also run without
either platform installed.

## MVP scope

The first MVP focuses on:

- facade-to-grid calibration;
- robot odometry on the grid;
- grid stability estimation;
- passive sensing simulation;
- sensor noise and sim-to-real robustness.

The planned sensor set includes a 3D LiDAR, grid and robot IMUs, motor encoders,
linear rail encoders, module joint encoders, distributed 1D distance sensors,
homing switches, module state sensors, and a virtual tool frame.

Drilling physics, force/contact sensing, safety hardware, embedded compute, power
electronics, and industrial deployment details are explicitly out of scope for
this MVP.

## Requirements

- Ubuntu 24.04 on x86_64/amd64
- NVIDIA GPU with a driver supported by the selected Isaac Sim release
- Python 3.12 recommended for ROS 2 Jazzy development
- ROS 2 Jazzy
- Isaac Sim, installed separately using NVIDIA's current installation guidance

ROS 2 and Isaac Sim are optional for the standalone model tests and placeholder
example. Isaac Sim ships a managed Python environment; do not install Isaac
packages into this repository's virtual environment.

### Verified workstation

The current development workstation has been verified with:

- Ubuntu 24.04
- NVIDIA GeForce RTX 5080 Laptop GPU
- NVIDIA open driver 580.159.03
- ROS 2 Jazzy installed at `/opt/ros/jazzy`
- Isaac Sim 6.0.0-rc.59 source build at `~/isaacsim`
- Isaac Sim ROS 2 bridge, RTX LiDAR, and IMU components

The exact detected state can be checked at any time with:

```bash
./scripts/check_system.sh
```

## Install

Check the host first:

```bash
./scripts/check_system.sh
```

Create the local environment and install the Python package:

```bash
./scripts/setup_env.sh
source .venv/bin/activate
```

The script uses the selected `python3` interpreter. On a standard Ubuntu 24.04
ROS workstation, that should be Python 3.12. Set `PYTHON_BIN=/path/to/python`
before running the script to choose another interpreter.

### ROS 2 Jazzy

If `ros2` is not found, install ROS 2 Jazzy from the official Ubuntu deb package
instructions at <https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html>.
Use the `ros-jazzy-ros-base` variant for a minimal headless system or
`ros-jazzy-desktop` for standard development tools. Then source it:

```bash
source /opt/ros/jazzy/setup.bash
```

Build the ROS package after the standalone environment is ready:

```bash
conda deactivate 2>/dev/null || true
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --base-paths src/gridsim_ros
source install/setup.bash
ros2 launch gridsim_ros mvp_sim.launch.py
```

The `pytest-repeat` warning sometimes printed by Ubuntu's system `setuptools`
during `colcon build` is harmless when the package finishes successfully.

## Run

Run the deterministic placeholder simulation:

```bash
python examples/run_mvp_sim.py
```

Run the tests:

```bash
pytest
```

## ROS 2 and Isaac Sim

Isaac Sim is installed separately from this repository. Keep Isaac-specific
extensions in its managed Python environment and use ROS 2 topics as the
boundary to the platform-independent `gridsim` packages.

Do not activate Conda while running ROS 2 or Isaac Sim. For the verified local
source build, launch Isaac Sim in one terminal:

```bash
conda deactivate 2>/dev/null || true
source /opt/ros/jazzy/setup.bash
~/isaacsim/_build/linux-x86_64/release/isaac-sim.sh
```

Launch the project ROS nodes in another terminal:

```bash
cd ~/deploya/gridsim
conda deactivate 2>/dev/null || true
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch gridsim_ros mvp_sim.launch.py
```

Both processes must use the same `ROS_DOMAIN_ID`. The initial integration will
connect Isaac Sim RTX LiDAR and IMU publishers to `gridsim_ros`, followed by
encoder and distance-sensor topics.

## Architecture

- `src/gridsim_core`: coordinate transforms and physical state containers
- `src/gridsim_sensors`: deterministic, seeded sensor noise models
- `src/gridsim_estimation`: odometry, calibration, and stability placeholders
- `src/gridsim_ros`: ROS 2 package, nodes, and launch description
- `configs`: scenario, geometry, and sensor parameters
- `assets`: future USD and source assets
- `examples`: platform-independent runnable examples
- `docs`: design decisions and MVP boundaries

See [docs/architecture.md](docs/architecture.md),
[docs/mvp_scope.md](docs/mvp_scope.md), and
[docs/sensor_modeling.md](docs/sensor_modeling.md) for more detail.

## Next steps

1. Create simplified facade, grid, and robot USD assets.
2. Implement and validate the `world -> facade -> grid -> robot_base -> tool`
   frame tree.
3. Map configuration values to Isaac Sim sensor APIs and ROS 2 topics.
4. Connect RTX LiDAR, grid IMU, and robot IMU data through the ROS 2 bridge.
5. Add recorded-data replay and calibration fixtures.
6. Replace placeholder estimators with validated algorithms and uncertainty.
