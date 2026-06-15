"""Wheel-encoder odometry for the mobile grid robot."""

from __future__ import annotations

from dataclasses import dataclass, field

from gridsim_core.robot_model import RobotState


@dataclass
class DifferentialDriveOdometry:
    """Integrate differential-drive wheel positions into a planar pose."""

    wheel_radius_m: float
    wheel_separation_m: float
    state: RobotState = field(default_factory=RobotState)
    _previous_left_rad: float | None = field(default=None, init=False, repr=False)
    _previous_right_rad: float | None = field(default=None, init=False, repr=False)

    def __post_init__(self) -> None:
        if self.wheel_radius_m <= 0 or self.wheel_separation_m <= 0:
            raise ValueError("wheel dimensions must be positive")

    def update(
        self, left_position_rad: float, right_position_rad: float, dt_s: float
    ) -> RobotState:
        """Update pose from absolute wheel positions."""
        if dt_s <= 0:
            raise ValueError("dt_s must be positive")
        if self._previous_left_rad is None or self._previous_right_rad is None:
            self._previous_left_rad = left_position_rad
            self._previous_right_rad = right_position_rad
            self.state.left_wheel_position_rad = left_position_rad
            self.state.right_wheel_position_rad = right_position_rad
            return self.state

        left_delta = (left_position_rad - self._previous_left_rad) * self.wheel_radius_m
        right_delta = (
            right_position_rad - self._previous_right_rad
        ) * self.wheel_radius_m
        linear_delta = 0.5 * (left_delta + right_delta)
        angular_delta = (right_delta - left_delta) / self.wheel_separation_m

        self.state.integrate_body_velocity(
            linear_delta / dt_s, angular_delta / dt_s, dt_s
        )
        self.state.left_wheel_position_rad = left_position_rad
        self.state.right_wheel_position_rad = right_position_rad
        self._previous_left_rad = left_position_rad
        self._previous_right_rad = right_position_rad
        return self.state
