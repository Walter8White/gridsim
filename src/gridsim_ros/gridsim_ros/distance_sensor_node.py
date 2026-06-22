"""Simulate 3x TF-Luna LiDAR sensors via geometric raycasting against the facade."""

from __future__ import annotations

import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import Range

# TF-Luna specs
_MIN_RANGE_M = 0.2
_MAX_RANGE_M = 8.0
_NOISE_STD_M = 0.015  # 1.5 cm std (1–3 cm range)
_RATE_HZ = 30.0

# Sensor offsets in robot frame (along robot right axis)
_SENSOR_SPACING_M = 0.15  # left at -0.15, center at 0, right at +0.15

# Facade is at world Y = 0; robot Y = distance from facade
_FACADE_Y = 0.0


def _yaw_from_pose(pose: PoseStamped) -> float:
    q = pose.pose.orientation
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _raycast(robot_y: float, dx_robot: float, yaw: float) -> float:
    """Distance from a sensor at robot-frame offset dx_robot to the facade plane."""
    cos_yaw = math.cos(yaw)
    if abs(cos_yaw) < 1e-6:
        return float("nan")
    sensor_y = robot_y + dx_robot * math.sin(yaw)
    dist = sensor_y / cos_yaw
    return max(_MIN_RANGE_M, min(_MAX_RANGE_M, dist))


class DistanceSensorNode(Node):
    def __init__(self) -> None:
        super().__init__("distance_sensor_node")
        self._rng = np.random.default_rng()

        self._sub = self.create_subscription(
            PoseStamped, "/robot/pose", self._pose_cb, 10
        )
        self._pub_left = self.create_publisher(Range, "/tool/distance_left", 10)
        self._pub_center = self.create_publisher(Range, "/tool/distance_center", 10)
        self._pub_right = self.create_publisher(Range, "/tool/distance_right", 10)

        self._robot_y = 2.0
        self._yaw = 0.0
        self._robot_x = 0.0

        self.create_timer(1.0 / _RATE_HZ, self._publish)

    def _pose_cb(self, msg: PoseStamped) -> None:
        self._robot_x = msg.pose.position.x
        self._robot_y = msg.pose.position.y
        self._yaw = _yaw_from_pose(msg)

    def _make_range(self, distance: float, frame_id: str) -> Range:
        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.radiation_type = Range.INFRARED
        msg.field_of_view = 0.03  # ~2° beam (TF-Luna)
        msg.min_range = _MIN_RANGE_M
        msg.max_range = _MAX_RANGE_M
        noisy = distance + float(self._rng.normal(0.0, _NOISE_STD_M))
        msg.range = float(max(_MIN_RANGE_M, min(_MAX_RANGE_M, noisy)))
        return msg

    def _publish(self) -> None:
        d_left = _raycast(self._robot_y, -_SENSOR_SPACING_M, self._yaw)
        d_center = _raycast(self._robot_y, 0.0, self._yaw)
        d_right = _raycast(self._robot_y, _SENSOR_SPACING_M, self._yaw)

        self._pub_left.publish(self._make_range(d_left, "distance_left"))
        self._pub_center.publish(self._make_range(d_center, "distance_center"))
        self._pub_right.publish(self._make_range(d_right, "distance_right"))


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = DistanceSensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
