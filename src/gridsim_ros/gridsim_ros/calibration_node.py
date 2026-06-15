"""Expose the facade-grid calibration placeholder through a ROS service."""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class CalibrationNode(Node):
    """Report calibration readiness without claiming a solved transform."""

    def __init__(self) -> None:
        super().__init__("calibration_node")
        self.service = self.create_service(
            Trigger, "calibrate_facade_grid", self.handle_calibration
        )

    def handle_calibration(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        del request
        response.success = False
        response.message = (
            "Calibration input is not connected; estimator scaffold is ready."
        )
        return response


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = CalibrationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
