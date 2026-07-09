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
    mode = LaunchConfiguration("mode")

    return LaunchDescription(
        [
            DeclareLaunchArgument("workspace", default_value=str(Path.cwd())),
            DeclareLaunchArgument(
                "mode",
                default_value="visual",
                description="visual: Isaac GUI only, fast preview motion. acquisition: headless Isaac + full-resolution export + RViz point cloud.",
            ),
            DeclareLaunchArgument("isaac", default_value="true"),
            DeclareLaunchArgument("isaac_ros_bridge", default_value="true"),
            DeclareLaunchArgument("standalone_publisher", default_value="false"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("output_dir", default_value="outputs/gocator_scan"),
            DeclareLaunchArgument("points_per_profile", default_value="3700"),
            DeclareLaunchArgument("profile_spacing_m", default_value="0.0005"),
            DeclareLaunchArgument("nominal_profile_rate_hz", default_value="2000.0"),
            DeclareLaunchArgument("scan_speed_m_s", default_value="0.0"),
            DeclareLaunchArgument("visual_scan_speed_m_s", default_value="0.25"),
            DeclareLaunchArgument("publish_rate_hz", default_value="5.0"),
            DeclareLaunchArgument("max_points", default_value="50000"),
            # Stride for the RViz display accumulator in acquisition mode:
            # 1 profile out of N is kept for display (reduces display memory).
            DeclareLaunchArgument("display_profile_stride", default_value="20"),
            DeclareLaunchArgument("display_point_stride", default_value="1"),
            DeclareLaunchArgument("publish_current_profile", default_value="true"),
            # acquisition mode: headless Isaac, streaming full-res export, RViz display
            ExecuteProcess(
                cmd=[
                    PathJoinSubstitution([workspace, "isaac", "run_mvp.sh"]),
                    "--headless",
                    "--scan-speed",
                    scan_speed,
                    "--sim-dt",
                    "0.25",
                    "--ros-bridge",
                    "--ros-publish-rate",
                    LaunchConfiguration("publish_rate_hz"),
                    "--ros-max-points",
                    LaunchConfiguration("max_points"),
                    "--ros-profile-stride",
                    LaunchConfiguration("display_profile_stride"),
                    "--ros-profile-point-stride",
                    LaunchConfiguration("display_point_stride"),
                    "--record-full-res",
                    "--record-output-dir",
                    LaunchConfiguration("output_dir"),
                    "--stop-after-scan",
                ],
                cwd=workspace,
                output="screen",
                condition=IfCondition(
                    PythonExpression([
                        "'", LaunchConfiguration("isaac"), "' == 'true' and '",
                        LaunchConfiguration("isaac_ros_bridge"), "' == 'true' and '",
                        mode, "' == 'acquisition'",
                    ])
                ),
            ),
            # visual mode: Isaac GUI only, fast scan speed, no ROS
            ExecuteProcess(
                cmd=[
                    PathJoinSubstitution([workspace, "isaac", "run_mvp.sh"]),
                    "--scan-speed",
                    LaunchConfiguration("visual_scan_speed_m_s"),
                ],
                cwd=workspace,
                output="screen",
                condition=IfCondition(
                    PythonExpression([
                        "'", LaunchConfiguration("isaac"), "' == 'true' and '",
                        mode, "' == 'visual'",
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
                condition=IfCondition(
                    PythonExpression([
                        "'", LaunchConfiguration("rviz"), "' == 'true' and '",
                        mode, "' == 'acquisition'",
                    ])
                ),
            ),
        ]
    )
