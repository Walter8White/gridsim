"""Range-limited 1D distance sensor model."""

from __future__ import annotations

import numpy as np
from numpy.typing import ArrayLike, NDArray


class DistanceSensorModel:
    """Add noise to scalar ranges and clip valid values to sensor limits."""

    def __init__(
        self,
        min_range_m: float,
        max_range_m: float,
        noise_std_m: float = 0.0,
        seed: int | None = None,
    ) -> None:
        if min_range_m < 0 or max_range_m <= min_range_m:
            raise ValueError("distance sensor range is invalid")
        if noise_std_m < 0:
            raise ValueError("noise_std_m must be non-negative")
        self.min_range_m = min_range_m
        self.max_range_m = max_range_m
        self.noise_std_m = noise_std_m
        self.rng = np.random.default_rng(seed)

    def measure(self, distance_m: ArrayLike) -> NDArray[np.float64]:
        distances = np.asarray(distance_m, dtype=float)
        noisy = distances + self.rng.normal(0.0, self.noise_std_m, distances.shape)
        valid = np.isfinite(noisy)
        return np.where(
            valid, np.clip(noisy, self.min_range_m, self.max_range_m), noisy
        )
