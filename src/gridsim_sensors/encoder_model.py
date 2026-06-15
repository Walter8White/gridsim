"""Generic quantized encoder model."""

from __future__ import annotations

import numpy as np
from numpy.typing import ArrayLike, NDArray


class EncoderModel:
    """Add Gaussian noise and quantize a position measurement."""

    def __init__(
        self, resolution: float, noise_std: float = 0.0, seed: int | None = None
    ) -> None:
        if resolution <= 0:
            raise ValueError("resolution must be positive")
        if noise_std < 0:
            raise ValueError("noise_std must be non-negative")
        self.resolution = resolution
        self.noise_std = noise_std
        self.rng = np.random.default_rng(seed)

    def measure(self, position: ArrayLike) -> NDArray[np.float64]:
        values = np.asarray(position, dtype=float)
        noisy = values + self.rng.normal(0.0, self.noise_std, size=values.shape)
        return np.asarray(np.round(noisy / self.resolution) * self.resolution)
