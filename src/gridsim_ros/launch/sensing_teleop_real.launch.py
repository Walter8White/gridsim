"""Launch real-hardware sensing and teleop nodes (placeholder).

Nodes here connect to real TF-Luna sensors and a BNO085 IMU via the
Teensy 4.1 serial bridge.  The wall estimator is identical to simulation.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            # Keyboard teleop — identical to sim; robot pose drives the bridge
            Node(
                package="gridsim_ros",
                executable="teleop_robot",
                name="teleop_robot_node",
                output="screen",
                prefix="xterm -e",
            ),
            # TODO: replace distance_sensor with real Teensy serial bridge node
            Node(
                package="gridsim_ros",
                executable="distance_sensor",
                name="distance_sensor_node",
                output="screen",
            ),
            # TODO: replace imu_sim with real BNO085 driver node
            Node(
                package="gridsim_ros",
                executable="imu_sim",
                name="imu_sim_node",
                output="screen",
            ),
            # Wall estimator is hardware-independent
            Node(
                package="gridsim_ros",
                executable="wall_estimator",
                name="wall_estimator_node",
                output="screen",
            ),
        ]
    )
