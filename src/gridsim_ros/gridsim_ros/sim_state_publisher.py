"""Publish a placeholder robot pose until Isaac Sim becomes the state source."""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node


class SimStatePublisher(Node):
    """Publish an identity robot pose at a configurable rate."""

    def __init__(self) -> None:
        super().__init__("sim_state_publisher")
        self.declare_parameter("publish_rate_hz", 20.0)
        rate_hz = float(self.get_parameter("publish_rate_hz").value)
        if rate_hz <= 0:
            raise ValueError("publish_rate_hz must be positive")
        self.publisher = self.create_publisher(PoseStamped, "robot/pose", 10)
        self.timer = self.create_timer(1.0 / rate_hz, self.publish_pose)

    def publish_pose(self) -> None:
        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "grid"
        message.pose.orientation.w = 1.0
        self.publisher.publish(message)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SimStatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
