# Isaac Sim Utilities

This directory currently contains utility entry points for visualizing imported
facade assets in Isaac Sim.

## IFC Facade Viewer

Close Conda and source ROS 2 before launching:

```bash
conda deactivate 2>/dev/null || true
source /opt/ros/jazzy/setup.bash
./isaac/view_ifc_facade.sh
```

Set `ISAAC_SIM_DIR` if the release build is installed elsewhere:

```bash
ISAAC_SIM_DIR=/path/to/isaacsim ./isaac/view_ifc_facade.sh
```
