# Architecture

The project separates simulation-platform integration from reusable models.
`gridsim_core` owns geometry and state, `gridsim_sensors` owns measurement noise,
and `gridsim_estimation` consumes those measurements. None of these packages
depends on ROS 2 or Isaac Sim.

`gridsim_ros` is the runtime adapter. Its initial nodes publish placeholder state,
apply configurable sensor noise, and expose a calibration trigger. Isaac Sim will
eventually own scene physics and raw synthetic measurements, communicating
through the ROS 2 bridge.

## Frame convention

Transforms are homogeneous 4x4 matrices using right-handed coordinates. The
intended frame tree is:

```text
world -> facade -> grid -> robot_base -> tool
```

Sensor extrinsics attach beneath either `grid` or `robot_base`. Configuration
files use SI units: metres, seconds, radians, metres per second, and SI-derived
units unless explicitly stated.

## Data flow

1. Isaac Sim advances scene state and emits ideal sensor measurements.
2. Sensor models add configurable noise, bias, quantization, and latency.
3. ROS 2 nodes publish measurements with timestamps and frame identifiers.
4. Estimators update odometry, facade-grid calibration, and stability metrics.
5. Tests validate platform-independent behavior before simulator integration.
