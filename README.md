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
conda deactivate 2>/dev/null || true
./scripts/setup_env.sh
source .venv/bin/activate
python --version
```

On Ubuntu, the script defaults to `/usr/bin/python3` so an active Conda
installation cannot silently create an incompatible environment. It also
recreates `.venv` automatically if that directory was built with another Python
interpreter. Set `PYTHON_BIN=/path/to/python` before running the script to make
an intentional override. Creating the environment requires the Ubuntu
`python3-venv` package or the `uv` command; install the former with
`sudo apt install python3-venv` if neither is available.

Keep the standalone Python environment separate from the ROS 2 environment. If
possible, use a clean shell for the standalone tests. The project configuration
blocks ROS launch-testing plugins from the unit-test run, so this sequence also
works after ROS has been sourced:

```bash
deactivate 2>/dev/null || true
conda deactivate 2>/dev/null || true
./scripts/setup_env.sh
source .venv/bin/activate
pytest
```

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

## ROS 2

Isaac Sim is installed separately from this repository. Keep Isaac-specific
extensions in its managed Python environment and use ROS 2 topics as the
boundary to the platform-independent `gridsim` packages.

Do not activate Conda while running ROS 2. Launch the project ROS nodes with:

```bash
cd ~/deploya/gridsim
conda deactivate 2>/dev/null || true
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch gridsim_ros mvp_sim.launch.py
```

The Isaac utilities that remain in this repository are limited to facade asset
visualization. See [`isaac/`](isaac/README.md).

### IFC facade import

`src/gridsim_ifc` extracts walls, windows, doors, levels, materials, and units from an IFC
building model via IfcOpenShell, and can convert the same geometry to a visual USD asset for
Isaac. This spans two of the environments above plus the standalone `.venv`:

- **Extraction only** (`gridsim_ifc.extract_ifc`, no `pxr` needed) runs in the standalone
  `.venv` — `ifcopenshell` is in `requirements.txt`, so `pytest tests/test_ifc_extract.py` works
  out of the box.
- **USD conversion** (`gridsim_ifc.usd_export.build_ifc_usd`) additionally needs `pxr`
  (`usd-core`), so per-asset `convert_*.py` scripts (e.g.
  `assets/facades/fzk_haus/convert_fzk_haus.py`) run in the conda `base` environment, same as
  the CAD sensor converters under `assets/cad/sensors/*/convert_*.py`:

  ```bash
  conda run -n base python assets/facades/fzk_haus/convert_fzk_haus.py
  ```

- **Visualizing in Isaac**: `./isaac/view_ifc_facade.sh` references the converted `.usd` (see
  `assets/facades/fzk_haus/README.md` for the full walkthrough on the bundled FZK-Haus sample).

## Architecture

- `src/gridsim_core`: coordinate transforms and physical state containers
- `src/gridsim_sensors`: deterministic, seeded sensor noise models
- `src/gridsim_estimation`: odometry, calibration, and stability placeholders
- `src/gridsim_ifc`: IFC building model extraction (IfcOpenShell) and USD conversion
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
