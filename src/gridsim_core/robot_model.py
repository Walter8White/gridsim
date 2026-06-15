"""Robot state used by placeholder estimators and examples."""

from __future__ import annotations

from dataclasses import dataclass
from math import cos, sin


@dataclass
class RobotState:
    """Planar robot pose and wheel state in the grid frame."""

    x_m: float = 0.0
    y_m: float = 0.0
    yaw_rad: float = 0.0
    left_wheel_position_rad: float = 0.0
    right_wheel_position_rad: float = 0.0
    linear_velocity_mps: float = 0.0
    angular_velocity_radps: float = 0.0

    def integrate_body_velocity(
        self, linear_velocity_mps: float, angular_velocity_radps: float, dt_s: float
    ) -> None:
        """Advance a planar pose using a midpoint heading approximation."""
        if dt_s < 0:
            raise ValueError("dt_s must be non-negative")
        midpoint_yaw = self.yaw_rad + 0.5 * angular_velocity_radps * dt_s
        self.x_m += linear_velocity_mps * cos(midpoint_yaw) * dt_s
        self.y_m += linear_velocity_mps * sin(midpoint_yaw) * dt_s
        self.yaw_rad += angular_velocity_radps * dt_s
        self.linear_velocity_mps = linear_velocity_mps
        self.angular_velocity_radps = angular_velocity_radps
