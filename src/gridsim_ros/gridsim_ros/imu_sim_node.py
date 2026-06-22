"""Simulate BNO085 IMU mounted on robot body."""

from __future__ import annotations

import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import Imu

# BNO085 noise approximation
_ACCEL_NOISE_STD = 0.05   # m/s²
_GYRO_NOISE_STD = 0.003   # rad/s
_RATE_HZ = 50.0
_DT = 1.0 / _RATE_HZ


def _yaw_from_pose(pose: PoseStamped) -> float:
    q = pose.pose.orientation
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class ImuSimNode(Node):
    def __init__(self) -> None:
        super().__init__("imu_sim_node")
        self._rng = np.random.default_rng()

        self._sub = self.create_subscription(
            PoseStamped, "/robot/pose", self._pose_cb, 10
        )
        self._pub = self.create_publisher(Imu, "/tool/imu", 10)

        self._x = 0.0
        self._y = 2.0
        self._yaw = 0.0
        self._vx = 0.0
        self._vy = 0.0
        self._vyaw = 0.0
        self._last_stamp: float | None = None

        self.create_timer(_DT, self._publish)

    def _pose_cb(self, msg: PoseStamped) -> None:
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        x = msg.pose.position.x
        y = msg.pose.position.y
        yaw = _yaw_from_pose(msg)

        if self._last_stamp is not None and t > self._last_stamp:
            dt = t - self._last_stamp
            self._vx = (x - self._x) / dt
            self._vy = (y - self._y) / dt
            self._vyaw = (yaw - self._yaw) / dt

        self._x = x
        self._y = y
        self._yaw = yaw
        self._last_stamp = t

    def _publish(self) -> None:
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "robot_imu"

        # Orientation from yaw
        half = self._yaw * 0.5
        msg.orientation.w = math.cos(half)
        msg.orientation.z = math.sin(half)
        msg.orientation_covariance[0] = -1.0  # not estimated (diagonal placeholder)

        # Angular velocity (yaw rate + noise)
        msg.angular_velocity.z = self._vyaw + float(self._rng.normal(0.0, _GYRO_NOISE_STD))

        # Linear acceleration: gravity-subtracted, noise only (no real dynamics model)
        msg.linear_acceleration.x = float(self._rng.normal(0.0, _ACCEL_NOISE_STD))
        msg.linear_acceleration.y = float(self._rng.normal(0.0, _ACCEL_NOISE_STD))
        msg.linear_acceleration.z = 9.81 + float(self._rng.normal(0.0, _ACCEL_NOISE_STD))

        self._pub.publish(msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ImuSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
