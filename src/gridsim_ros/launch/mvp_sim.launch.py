"""Launch the platform-independent ROS 2 MVP placeholder nodes."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="gridsim_ros",
                executable="sim_state_publisher",
                name="sim_state_publisher",
                output="screen",
            ),
            Node(
                package="gridsim_ros",
                executable="sensor_noise_node",
                name="sensor_noise_node",
                output="screen",
            ),
            Node(
                package="gridsim_ros",
                executable="calibration_node",
                name="calibration_node",
                output="screen",
            ),
        ]
    )
