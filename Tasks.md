# Tasks

## Product Simulation Roadmap

This roadmap tracks the path from the current Isaac Sim MVP toward a realistic
simulation stack for the grid-and-robot product.

## 1. Product Physics Foundation

- Import a representative CAD model for one grid module.
- Convert the module into a reusable USD asset.
- Add mass, center of mass, inertia, and collision geometry.
- Model the real module actuation, including motor type, limits, torque, speed,
  friction, and backlash.
- Replace the placeholder hollow grid with assembled module assets.
- Validate that the grid moves under gravity and actuator commands.

## 2. Robot On Grid

- Import or model the real robot chassis.
- Add mass, inertia, collision geometry, and contact surfaces.
- Add the robot drive or rail mechanism used on the grid.
- Define the tool frame and interchangeable tool payloads.
- Validate robot motion on the dynamic grid.

## 3. Sensors

- Select the target LiDAR model and reproduce its key properties in Isaac Sim.
- Define IMU, encoders, distance sensors, cameras, and optional force/contact
  sensors.
- Add realistic sensor frames, extrinsics, update rates, latency, and ROS 2
  topic names.
- Add configurable noise, bias, dropout, and calibration error.
- Validate all sensors in RViz and with ROS 2 topic checks.

## 4. Client Facade Import

- Support client BIM/CAD inputs such as IFC, STEP, OBJ, or USD.
- Extract facade dimensions, planes, openings, forbidden zones, anchor points,
  and target work areas.
- Convert facade geometry into an Isaac Sim scene asset.
- Generate a task map from the imported facade.

## 5. Mission Planning

- Convert facade task maps into an ordered sequence of robot/grid actions.
- Add tool selection based on the operation to perform.
- Add trajectory generation for grid deployment and robot motion.
- Add checks for forbidden zones, reachability, and collision risk.

## 6. Control And Estimation

- Add controller interfaces for module actuators and robot motion.
- Add state estimation with IMU, encoders, LiDAR, and facade measurements.
- Prototype EKF-based localization and grid deformation estimation.
- Add closed-loop behavior that adapts to measured facade errors.

## 7. Environment Realism

- Add wind loads and gusts.
- Add gravity-consistent dynamics for all non-fixed bodies.
- Add compliance, flexion, and vibration for the grid.
- Add contact/friction effects between robot, grid, tools, and facade.
- Add scenario randomization for regression testing.

## 8. Validation

- Add headless Isaac Sim smoke tests for physics and ROS topics.
- Add golden scenarios for imported facade tasks.
- Add automated checks for topic availability, TF tree consistency, and sensor
  rates.
- Track simulation assumptions and known gaps in the worklog.
