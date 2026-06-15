"""Accelerometer and gyroscope noise with bias random walk."""

from __future__ import annotations

import numpy as np
from numpy.typing import ArrayLike, NDArray


class ImuModel:
    """Simulate noisy 3-axis acceleration and angular velocity."""

    def __init__(
        self,
        accelerometer_noise_std_mps2: float = 0.02,
        gyroscope_noise_std_radps: float = 0.002,
        accelerometer_bias: ArrayLike = (0.0, 0.0, 0.0),
        gyroscope_bias: ArrayLike = (0.0, 0.0, 0.0),
        bias_random_walk: float = 0.0,
        seed: int | None = None,
    ) -> None:
        if min(
            accelerometer_noise_std_mps2,
            gyroscope_noise_std_radps,
            bias_random_walk,
        ) < 0:
            raise ValueError("noise and random walk values must be non-negative")
        self.accelerometer_noise_std_mps2 = accelerometer_noise_std_mps2
        self.gyroscope_noise_std_radps = gyroscope_noise_std_radps
        self.accelerometer_bias = self._vector(accelerometer_bias)
        self.gyroscope_bias = self._vector(gyroscope_bias)
        self.bias_random_walk = bias_random_walk
        self.rng = np.random.default_rng(seed)

    @staticmethod
    def _vector(value: ArrayLike) -> NDArray[np.float64]:
        vector = np.asarray(value, dtype=float)
        if vector.shape != (3,):
            raise ValueError("IMU values must contain exactly three axes")
        return vector.copy()

    def measure(
        self,
        acceleration_mps2: ArrayLike,
        angular_velocity_radps: ArrayLike,
        dt_s: float = 0.0,
    ) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
        if dt_s < 0:
            raise ValueError("dt_s must be non-negative")
        acceleration = self._vector(acceleration_mps2)
        angular_velocity = self._vector(angular_velocity_radps)

        walk_std = self.bias_random_walk * np.sqrt(dt_s)
        self.accelerometer_bias += self.rng.normal(0.0, walk_std, size=3)
        self.gyroscope_bias += self.rng.normal(0.0, walk_std, size=3)

        acceleration += self.accelerometer_bias + self.rng.normal(
            0.0, self.accelerometer_noise_std_mps2, size=3
        )
        angular_velocity += self.gyroscope_bias + self.rng.normal(
            0.0, self.gyroscope_noise_std_radps, size=3
        )
        return acceleration, angular_velocity
