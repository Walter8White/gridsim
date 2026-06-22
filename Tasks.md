# Tasks

## Product Simulation Roadmap

This roadmap tracks the path from the current Isaac Sim MVP toward a realistic
simulation stack for the deployable cross-grid and tool-robot product.

The current priority is not to simulate the full final machine immediately.
The first objective is to validate the robot sensor head: local wall distance
estimation, wall angle estimation, vibration detection, and ROS 2 integration.
The same ROS 2 interfaces should later be used both in simulation and on the
real prototype hardware.

## 1. Sensor Head MVP

* Create a simplified robot tool sensor head in Isaac Lab.
* Add three forward-facing distance sensors using raycasts:

  * left distance sensor
  * center distance sensor
  * right distance sensor
* Model these sensors as TF-Luna-like low-cost distance sensors.
* Add a simulated IMU on the robot tool body, representing a BNO085-like IMU.
* Define the sensor frames:

  * `tool_distance_left_frame`
  * `tool_distance_center_frame`
  * `tool_distance_right_frame`
  * `tool_imu_frame`
* Publish the simulated sensor outputs to ROS 2 topics:

  * `/tool/distance_left`
  * `/tool/distance_center`
  * `/tool/distance_right`
  * `/tool/imu`
* Add configurable sensor properties:

  * update rate
  * Gaussian noise
  * bias
  * latency
  * dropout probability
  * maximum range
  * minimum range
* Validate that the simulated sensors behave correctly in front of a simple wall.

## 2. Wall Distance And Angle Estimation

* Create a ROS 2 node that subscribes to the three distance sensor topics.
* Estimate the local average distance between the robot tool and the wall.
* Estimate the local wall angle from the left, center, and right distance
  measurements.
* Detect when the robot tool is not parallel to the wall.
* Publish processed outputs:

  * `/tool/wall_distance`
  * `/tool/wall_angle`
  * `/tool/sensor_confidence`
* Add simple sanity checks:

  * distance within valid range
  * distance measurements are consistent
  * no excessive sensor dropout
  * no excessive disagreement between left, center, and right sensors
* Validate the estimator in simulation with:

  * a flat wall
  * an angled sensor head
  * a wall with local bumps
  * noisy sensor measurements

## 3. Vibration And Safety State

* Use the simulated IMU to estimate robot tool vibration.
* Compute a simple vibration metric from acceleration and angular velocity.
* Publish:

  * `/tool/vibration_level`
  * `/tool/safety_state`
* Define a first safety condition:

  * wall distance is within tolerance
  * wall angle is within tolerance
  * vibration is below threshold
  * sensor confidence is high enough
* Make the safety state return false when:

  * the robot is too close to the wall
  * the robot is too far from the wall
  * the robot is too tilted relative to the wall
  * vibration is too high
  * sensor readings are missing or inconsistent
* Validate the safety logic in simulation using controlled test scenarios.

## 4. Simulation-To-Real Sensor Interface

* Define a common ROS 2 interface for both simulation and real hardware.
* The simulated Isaac sensor publisher and the real hardware serial reader must
  publish the same topics, units, frame IDs, and message types.
* Create two interchangeable launch modes:

  * `sensor_head_sim.launch.py`
  * `sensor_head_real.launch.py`
* The downstream estimator must not depend on whether the data comes from Isaac
  or from the real prototype.
* Keep the ROS 2 topic structure stable:

  * `/tool/distance_left`
  * `/tool/distance_center`
  * `/tool/distance_right`
  * `/tool/imu`
  * `/tool/wall_distance`
  * `/tool/wall_angle`
  * `/tool/vibration_level`
  * `/tool/safety_state`
* Document all sensor units, frame conventions, and expected update rates.

## 5. Real Sensor Head Prototype

* Build a first physical sensor head using:

  * 3× TF-Luna or equivalent low-cost distance sensors
  * 1× BNO085 or equivalent IMU
  * 1× Teensy 4.1 or similar microcontroller
  * 1× Raspberry Pi 5 or small Ubuntu machine running ROS 2
  * 1× clean 5 V power supply
* Mount the three distance sensors rigidly on a small plate:

  * left
  * center
  * right
* Mount the IMU close to the expected tool body frame.
* Use the microcontroller to read the distance sensors and IMU.
* Send all raw sensor data to the Raspberry Pi over USB serial.
* Create a ROS 2 `serial_sensor_head_publisher` node that publishes the same
  topics as the simulation.
* Validate the real sensor head against a flat wall at known distances:

  * 0.5 m
  * 1.0 m
  * 1.5 m
  * 2.0 m
* Compare real measurements with simulation assumptions.
* Tune the simulated noise, bias, dropout, and latency based on real data.

## 6. Cross-Grid Product Model

* Replace the previous bar-based description with a cross-based grid structure.
* Model the product as repeated cross-shaped units assembled together.
* Each cross unit contains:

  * vertical actuated structure
  * horizontal mechanical connection
  * integrated rails
* Cross units are connected vertically through actuated/motorized elements.
* Cross units are mechanically fixed horizontally.
* Rails are present across the structure and are used to move the mobile beam.
* The grid must remain reconfigurable for different facade sizes.
* Avoid assuming a fixed grid size in the simulation architecture.
* Add configuration parameters for:

  * number of cross units horizontally
  * number of cross units vertically
  * cross unit size
  * rail spacing
  * facade distance

## 7. Mobile Beam And Tool Motion

* Model a mobile beam moving on the grid rails.
* Model the tool robot moving along the mobile beam.
* Define the two main motion axes:

  * beam motion on the grid rails
  * tool motion along the beam
* Add simplified rail constraints for the first simulation version.
* Later, add more realistic contact, friction, backlash, and rail imperfections.
* Define the tool frame as the reference frame for wall distance estimation.
* Ensure the sensor head is attached to the tool frame.
* Validate that the tool can reach positions in the facade plane.

## 8. Rail Position Measurement

* Model magnetic strips as passive position references on the rails.
* Model magnetic read heads on the moving elements:

  * read heads on the mobile beam for beam position
  * read head on the tool robot for tool position on the beam
* For the mobile beam, support two read heads when the beam is guided by two
  rails.
* Estimate beam skew using:

  * `x_top`
  * `x_bottom`
  * `skew = x_top - x_bottom`
* Publish simulated rail measurement topics:

  * `/beam/x_top`
  * `/beam/x_bottom`
  * `/beam/skew`
  * `/tool/y_position`
* Add configurable measurement noise, bias, quantization, latency, and dropout.
* Keep this separate from motor encoder feedback.

## 9. Motor Encoders And Motion Feedback

* Add motor encoder feedback for all actuated axes:

  * beam drive motors
  * tool translation motor
  * cross-grid deployment motors
  * tool-specific actuators if needed
* Use motor encoders for low-level control feedback.
* Use magnetic rail measurements for real position correction.
* Keep a clear distinction:

  * motor encoder = what the motor did
  * magnetic strip = where the moving element actually is
  * distance sensors = where the facade is relative to the tool
  * IMU = how the tool or structure is moving dynamically
* Add consistency checks between motor encoder estimates and rail measurements.
* Detect possible mechanical issues:

  * slipping
  * backlash
  * blocked axis
  * rail misalignment
  * excessive skew

## 10. BIM-To-Task Workflow

* Support future BIM or CAD facade input.
* Extract useful renovation information:

  * facade dimensions
  * openings
  * forbidden zones
  * target work areas
  * material zones
  * expected facade plane
* Generate an initial theoretical sequence of robot actions from the BIM data.
* Treat BIM as the theoretical plan, not as the final truth.
* Use the robot scan to correct the BIM-derived task map.
* Later, support facade import formats such as IFC, STEP, OBJ, or USD.

## 11. Pre-Work Facade Scan

* Simulate the workflow where the grid is deployed in front of the facade.
* Move the robot tool across the facade plane.
* Use the distance sensors, and later optional LiDAR or camera sensors, to scan
  the real facade geometry.
* Compare the measured facade with the theoretical BIM facade.
* Detect:

  * facade offset
  * facade tilt
  * local bumps
  * local recesses
  * unexpected obstacles
  * regions where sensor confidence is low
* Recalibrate the action sequence before executing renovation tasks.

## 12. Control And Estimation

* Start with simple deterministic estimation:

  * average wall distance
  * wall angle
  * vibration threshold
  * safety state
* Later add state estimation using:

  * motor encoders
  * magnetic rail measurements
  * distance sensors
  * IMU
* Prototype an EKF or similar estimator only after the basic sensor pipeline is
  validated.
* Use the estimator to track:

  * beam position
  * tool position
  * beam skew
  * tool-to-wall distance
  * tool-to-wall angle
  * vibration state
* Keep low-level safety deterministic and threshold-based.
* Do not rely on AI or learned control for critical safety decisions.

## 13. Environment Realism

* Add facade geometry variations:

  * flat wall
  * tilted wall
  * wall with bumps
  * wall with recesses
  * obstacles
* Add grid and tool perturbations:

  * vibration
  * wind-like oscillations
  * small structural movement
  * rail measurement noise
  * sensor dropout
* Add progressive realism only after the basic sensor and ROS 2 pipeline works.
* Keep simple scenarios available for fast regression testing.

## 14. Validation

* Add headless Isaac Sim smoke tests for:

  * distance sensor outputs
  * IMU outputs
  * ROS 2 topic availability
  * topic frequency
  * TF tree consistency
  * safety state behavior
* Add golden scenarios:

  * flat wall, parallel tool
  * flat wall, angled tool
  * wall with local bump
  * vibration above threshold
  * sensor dropout
  * beam skew
* Add automated checks for:

  * expected topic names
  * expected frame IDs
  * valid numeric ranges
  * no NaN values
  * stable update rates
* Track simulation assumptions and known gaps in the worklog.

## 15. Immediate Next Tasks

* Implement the simulated sensor head with three raycast distance sensors.
* Add the simulated IMU to the tool body.
* Publish raw simulated sensor data to ROS 2.
* Implement the ROS 2 wall distance and wall angle estimator.
* Implement the first safety state node.
* Create a simple wall scene with flat, tilted, and bumped wall variants.
* Prepare the real sensor head hardware list:

  * 3× TF-Luna
  * 1× BNO085
  * 1× Teensy 4.1
  * 1× Raspberry Pi 5
  * 1× clean 5 V power supply
* Ensure the real hardware publisher will use the same ROS 2 interface as the
  simulation publisher.
