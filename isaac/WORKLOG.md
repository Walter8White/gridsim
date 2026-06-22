# MVP Isaac Integration Worklog

## Capability mapping

- Scene authoring: local USD geometry with `pxr.UsdGeom`
- Physics: kinematic grid and robot rigid bodies with a PhysX scene
- Sensors: Isaac Sim 6 experimental physics IMUs and RTX LiDAR
- ROS 2: OmniGraph clock, transform, IMU, and PointCloud2 publishers
- Validation: standalone config tests, headless stage generation, ROS topic checks

## Integration order

1. Load and validate repository YAML configuration.
2. Build facade, grid modules, robot placeholder, and frame hierarchy.
3. Add grid and robot IMU prims.
4. Add ROS 2 clock, transform, and IMU action graph.
5. Add optional RTX LiDAR and PointCloud2 writer.
6. Save the composed stage and run a bounded headless validation.

## Validation

- The generated USD stage opens successfully and contains the facade, grid,
  robot, sensors, and ROS graph.
- ROS 2 publishes `/clock`, both IMU topics, and the complete transform chain:
  `world -> facade -> grid -> robot_base -> tool`.
- The RTX LiDAR publishes `sensor_msgs/msg/PointCloud2` on
  `/grid/lidar/points` with frame `grid_lidar`.
- The scene now exports a ground collider, explicit gravity, facade/grid mass
  properties, a dynamic hollow frame-style grid, and a facade-facing robot
  nose/tool.
- The robot is mounted on the facade-facing side of the grid, with a
  facade-facing depth-camera placeholder on the robot and the HESAI XT32 SD10
  LiDAR asset mounted on the grid module above it.
- Keep the `LidarSensor` Python object alive for the full simulation loop.
  Releasing it also releases its render product and ROS writer.
- The first RTX launch may spend about a minute compiling shaders. Later
  launches use the local cache and start substantially faster.
