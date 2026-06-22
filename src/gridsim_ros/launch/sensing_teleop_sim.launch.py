"""Launch simulated sensing and teleop nodes, optionally with Isaac Sim.

Usage:
  ros2 launch gridsim_ros sensing_teleop_sim.launch.py
  ros2 launch gridsim_ros sensing_teleop_sim.launch.py use_isaac:=true
"""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def _find_project_root() -> Path:
    """Walk up from __file__ until we find the isaac/run_mvp.sh marker."""
    p = Path(__file__).resolve()
    for parent in p.parents:
        if (parent / "isaac" / "run_mvp.sh").exists():
            return parent
    return p.parents[5]  # fallback: colcon layout depth

_PROJECT_ROOT = str(_find_project_root())
_ISAAC_SCRIPT = str(Path(_PROJECT_ROOT) / "isaac" / "run_mvp.sh")


def generate_launch_description() -> LaunchDescription:
    use_isaac_arg = DeclareLaunchArgument(
        "use_isaac",
        default_value="false",
        description="Launch Isaac Sim alongside the ROS 2 nodes",
    )

    isaac_process = ExecuteProcess(
        cmd=[_ISAAC_SCRIPT],
        cwd=_PROJECT_ROOT,
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_isaac")),
    )

    return LaunchDescription(
        [
            use_isaac_arg,
            isaac_process,
            # Keyboard teleop — opens in gnome-terminal so stdin is available
            ExecuteProcess(
                cmd=[
                    "gnome-terminal",
                    "--",
                    "bash", "-c",
                    (
                        "set +u; "
                        "source /opt/ros/jazzy/setup.bash; "
                        f"source {_PROJECT_ROOT}/install/setup.bash; "
                        "echo 'Teleop ready — use W/S/A/D/Q/E keys'; "
                        "ros2 run gridsim_ros teleop_robot; "
                        "read -p 'Press Enter to close...'"
                    ),
                ],
                output="screen",
            ),
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
