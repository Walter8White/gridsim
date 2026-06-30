"""Launch Isaac animated scanner, ROS point cloud publisher, and RViz."""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("gridsim_ros"), "rviz", "gocator_pointcloud.rviz"]
    )
    workspace = LaunchConfiguration("workspace")
    scan_speed = LaunchConfiguration("scan_speed_m_s")

    return LaunchDescription(
        [
            DeclareLaunchArgument("workspace", default_value=str(Path.cwd())),
            DeclareLaunchArgument("isaac", default_value="true"),
            DeclareLaunchArgument("isaac_ros_bridge", default_value="true"),
            DeclareLaunchArgument("standalone_publisher", default_value="false"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("points_per_profile", default_value="3700"),
            DeclareLaunchArgument("profile_spacing_m", default_value="0.0005"),
            DeclareLaunchArgument("nominal_profile_rate_hz", default_value="40.0"),
            DeclareLaunchArgument("scan_speed_m_s", default_value="0.0"),
            DeclareLaunchArgument("publish_rate_hz", default_value="10.0"),
            DeclareLaunchArgument("max_points", default_value="500000"),
            DeclareLaunchArgument("publish_current_profile", default_value="true"),
            ExecuteProcess(
                cmd=[
                    PathJoinSubstitution([workspace, "isaac", "run_mvp.sh"]),
                    "--scan-speed",
                    scan_speed,
                    "--ros-bridge",
                    "--ros-publish-rate",
                    LaunchConfiguration("publish_rate_hz"),
                    "--ros-max-points",
                    LaunchConfiguration("max_points"),
                ],
                cwd=workspace,
                output="screen",
                condition=IfCondition(
                    PythonExpression([
                        "'", LaunchConfiguration("isaac"), "' == 'true' and '",
                        LaunchConfiguration("isaac_ros_bridge"), "' == 'true'",
                    ])
                ),
            ),
            ExecuteProcess(
                cmd=[
                    PathJoinSubstitution([workspace, "isaac", "run_mvp.sh"]),
                    "--scan-speed",
                    scan_speed,
                ],
                cwd=workspace,
                output="screen",
                condition=IfCondition(
                    PythonExpression([
                        "'", LaunchConfiguration("isaac"), "' == 'true' and '",
                        LaunchConfiguration("isaac_ros_bridge"), "' != 'true'",
                    ])
                ),
            ),
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
                        "nominal_profile_rate_hz": ParameterValue(
                            LaunchConfiguration("nominal_profile_rate_hz"), value_type=float
                        ),
                        "scan_speed_m_s": ParameterValue(scan_speed, value_type=float),
                        "publish_rate_hz": ParameterValue(
                            LaunchConfiguration("publish_rate_hz"), value_type=float
                        ),
                        "max_points": ParameterValue(LaunchConfiguration("max_points"), value_type=int),
                        "publish_current_profile": ParameterValue(
                            LaunchConfiguration("publish_current_profile"), value_type=bool
                        ),
                    }
                ],
                condition=IfCondition(LaunchConfiguration("standalone_publisher")),
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
