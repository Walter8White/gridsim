"""Launch simulated sensing and teleop nodes.

Usage:
  ros2 launch gridsim_ros sensing_teleop_sim.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            # Simulated TF-Luna sensors (geometric raycasts)
            Node(
                package="gridsim_ros",
                executable="distance_sensor",
                name="distance_sensor_node",
                output="screen",
            ),
            # Simulated BNO085 IMU
            Node(
                package="gridsim_ros",
                executable="imu_sim",
                name="imu_sim_node",
                output="screen",
            ),
            # Wall distance / angle estimator
            Node(
                package="gridsim_ros",
                executable="wall_estimator",
                name="wall_estimator_node",
                output="screen",
            ),
        ]
    )
