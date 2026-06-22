"""Subscribe to the three distance sensors and estimate wall distance and angle."""

from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Float32

# Full left-to-right sensor span (left at -0.15m, right at +0.15m)
_SENSOR_SPAN_M = 0.30


class WallEstimatorNode(Node):
    def __init__(self) -> None:
        super().__init__("wall_estimator_node")

        self._d_left: float | None = None
        self._d_center: float | None = None
        self._d_right: float | None = None

        self.create_subscription(Range, "/tool/distance_left", self._cb_left, 10)
        self.create_subscription(Range, "/tool/distance_center", self._cb_center, 10)
        self.create_subscription(Range, "/tool/distance_right", self._cb_right, 10)

        self._pub_dist = self.create_publisher(Float32, "/tool/wall_distance", 10)
        self._pub_angle = self.create_publisher(Float32, "/tool/wall_angle", 10)

        self.create_timer(0.05, self._publish)  # 20 Hz output

    def _cb_left(self, msg: Range) -> None:
        self._d_left = msg.range

    def _cb_center(self, msg: Range) -> None:
        self._d_center = msg.range

    def _cb_right(self, msg: Range) -> None:
        self._d_right = msg.range

    def _publish(self) -> None:
        if None in (self._d_left, self._d_center, self._d_right):
            return

        wall_dist = (self._d_left + self._d_center + self._d_right) / 3.0
        wall_angle = math.atan2(self._d_right - self._d_left, _SENSOR_SPAN_M)

        self._pub_dist.publish(Float32(data=float(wall_dist)))
        self._pub_angle.publish(Float32(data=float(wall_angle)))


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = WallEstimatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
