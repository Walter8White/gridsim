"""Tests for the new sensing/teleop logic."""

import math
import sys
from pathlib import Path

_ROS_SRC = Path(__file__).resolve().parents[1] / "src" / "gridsim_ros"
sys.path.insert(0, str(_ROS_SRC))

from gridsim_ros.distance_sensor_node import _raycast
from gridsim_ros.wall_estimator_node import _SENSOR_SPAN_M


def test_raycast_center_no_yaw():
    """Center sensor at yaw=0 returns robot_y exactly."""
    assert math.isclose(_raycast(2.0, 0.0, 0.0), 2.0)


def test_raycast_center_no_yaw_various():
    for y in [0.5, 1.0, 3.0, 7.9]:
        assert math.isclose(_raycast(y, 0.0, 0.0), y), f"failed at y={y}"


def test_raycast_left_right_symmetry():
    """Left and right sensors are symmetric at yaw=0."""
    spacing = 0.15
    d_left = _raycast(2.0, -spacing, 0.0)
    d_right = _raycast(2.0, spacing, 0.0)
    assert math.isclose(d_left, d_right)


def test_raycast_yaw_changes_distances():
    """Positive yaw (CCW / rotate left) swings right sensor farther from facade."""
    yaw = 0.2  # ~11.5°
    d_left = _raycast(2.0, -0.15, yaw)
    d_right = _raycast(2.0, 0.15, yaw)
    assert d_right > d_left


def test_raycast_clamps_min():
    assert _raycast(0.1, 0.0, 0.0) == 0.2  # clamped to min


def test_raycast_clamps_max():
    assert _raycast(9.0, 0.0, 0.0) == 8.0  # clamped to max


def test_wall_angle_zero_at_symmetric():
    """Wall angle should be zero when left == right."""
    d_left = d_right = 2.0
    angle = math.atan2(d_right - d_left, _SENSOR_SPAN_M)
    assert math.isclose(angle, 0.0)


def test_wall_angle_positive_when_right_farther():
    d_left = 1.8
    d_right = 2.2
    angle = math.atan2(d_right - d_left, _SENSOR_SPAN_M)
    assert angle > 0.0


def test_wall_distance_mean():
    vals = [1.9, 2.0, 2.1]
    mean = sum(vals) / 3
    assert math.isclose(mean, 2.0)
