"""Minimal Cartesian LiDAR noise model."""

from __future__ import annotations

import numpy as np
from numpy.typing import ArrayLike, NDArray


class LidarModel:
    """Add independent Gaussian noise to XYZ point measurements."""

    def __init__(self, noise_std_m: float = 0.01, seed: int | None = None) -> None:
        if noise_std_m < 0:
            raise ValueError("noise_std_m must be non-negative")
        self.noise_std_m = noise_std_m
        self.rng = np.random.default_rng(seed)

    def measure(self, points_xyz: ArrayLike) -> NDArray[np.float64]:
        points = np.asarray(points_xyz, dtype=float)
        if points.ndim < 1 or points.shape[-1] != 3:
            raise ValueError("LiDAR points must have a final dimension of size three")
        noise = self.rng.normal(0.0, self.noise_std_m, size=points.shape)
        return points + noise
