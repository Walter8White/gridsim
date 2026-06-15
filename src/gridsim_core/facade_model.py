"""Simple planar facade geometry."""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from numpy.typing import ArrayLike, NDArray


@dataclass
class FacadeModel:
    """Finite facade plane and its calibration landmarks."""

    width_m: float
    height_m: float
    normal: ArrayLike = (0.0, 0.0, 1.0)
    offset_m: float = 0.0
    anchor_points: dict[str, ArrayLike] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if self.width_m <= 0 or self.height_m <= 0:
            raise ValueError("facade dimensions must be positive")
        normal = np.asarray(self.normal, dtype=float)
        if normal.shape != (3,) or np.linalg.norm(normal) == 0:
            raise ValueError("normal must be a non-zero 3D vector")
        self.normal = normal / np.linalg.norm(normal)
        self.anchor_points = {
            name: self._point(value) for name, value in self.anchor_points.items()
        }

    @staticmethod
    def _point(value: ArrayLike) -> NDArray[np.float64]:
        point = np.asarray(value, dtype=float)
        if point.shape != (3,):
            raise ValueError("facade points must contain three values")
        return point

    def signed_distance(self, point: ArrayLike) -> float:
        """Return signed point-to-plane distance in metres."""
        return float(np.dot(self.normal, self._point(point)) + self.offset_m)

    def project_to_plane(self, point: ArrayLike) -> NDArray[np.float64]:
        """Project a 3D point onto the facade plane."""
        point_array = self._point(point)
        return point_array - self.signed_distance(point_array) * self.normal
