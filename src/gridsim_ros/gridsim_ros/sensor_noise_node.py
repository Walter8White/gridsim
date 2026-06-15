"""Apply simple Gaussian noise to incoming IMU measurements."""

from __future__ import annotations

import random

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class SensorNoiseNode(Node):
    """Relay raw IMU data with configurable independent axis noise."""

    def __init__(self) -> None:
        super().__init__("sensor_noise_node")
        self.declare_parameter("accelerometer_noise_std_mps2", 0.02)
        self.declare_parameter("gyroscope_noise_std_radps", 0.002)
        self.declare_parameter("random_seed", 42)
        self.acceleration_std = float(
            self.get_parameter("accelerometer_noise_std_mps2").value
        )
        self.angular_std = float(
            self.get_parameter("gyroscope_noise_std_radps").value
        )
        self.rng = random.Random(int(self.get_parameter("random_seed").value))
        self.publisher = self.create_publisher(Imu, "imu/noisy", 10)
        self.subscription = self.create_subscription(
            Imu, "imu/raw", self.handle_imu, 10
        )

    def handle_imu(self, message: Imu) -> None:
        for field in ("x", "y", "z"):
            value = getattr(message.linear_acceleration, field)
            setattr(
                message.linear_acceleration,
                field,
                value + self.rng.gauss(0.0, self.acceleration_std),
            )
            value = getattr(message.angular_velocity, field)
            setattr(
                message.angular_velocity,
                field,
                value + self.rng.gauss(0.0, self.angular_std),
            )
        self.publisher.publish(message)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SensorNoiseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
