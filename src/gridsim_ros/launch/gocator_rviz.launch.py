"""Launch the simulated Gocator point cloud publisher and RViz."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("gridsim_ros"), "rviz", "gocator_pointcloud.rviz"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("points_per_profile", default_value="800"),
            DeclareLaunchArgument("profile_spacing_m", default_value="0.01"),
            DeclareLaunchArgument("scan_speed_m_s", default_value="0.25"),
            DeclareLaunchArgument("publish_rate_hz", default_value="10.0"),
            DeclareLaunchArgument("max_points", default_value="250000"),
            Node(
                package="gridsim_ros",
                executable="gocator_pointcloud",
                name="gocator_pointcloud",
                output="screen",
                parameters=[
                    {
                        "points_per_profile": ParameterValue(
                            LaunchConfiguration("points_per_profile"), value_type=int
                        ),
                        "profile_spacing_m": ParameterValue(
                            LaunchConfiguration("profile_spacing_m"), value_type=float
                        ),
                        "scan_speed_m_s": ParameterValue(
                            LaunchConfiguration("scan_speed_m_s"), value_type=float
                        ),
                        "publish_rate_hz": ParameterValue(
                            LaunchConfiguration("publish_rate_hz"), value_type=float
                        ),
                        "max_points": ParameterValue(LaunchConfiguration("max_points"), value_type=int),
                    }
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
                condition=IfCondition(LaunchConfiguration("rviz")),
            ),
        ]
    )
