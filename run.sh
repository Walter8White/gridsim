#!/usr/bin/env bash
# Wrapper: sources ROS 2 + colcon overlay, then runs the given ros2 launch command.
# Usage:
#   ./run.sh sensing_teleop_sim.launch.py [args...]
#   ./run.sh sensing_teleop_sim.launch.py use_isaac:=true
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ROS setup scripts reference unset vars; disable -u around them
set +u
source /opt/ros/jazzy/setup.bash
source "${SCRIPT_DIR}/install/setup.bash"
set -u

LAUNCH_FILE="${1:-sensing_teleop_sim.launch.py}"
shift 2>/dev/null || true

exec ros2 launch gridsim_ros "${LAUNCH_FILE}" "$@"
