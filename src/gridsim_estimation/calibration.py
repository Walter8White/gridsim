"""Initial facade-grid calibration interface."""

from __future__ import annotations

import numpy as np
from numpy.typing import ArrayLike, NDArray

from gridsim_core.transforms import make_transform


def estimate_facade_grid_transform(
    grid_points: ArrayLike, facade_points: ArrayLike
) -> NDArray[np.float64]:
    """Estimate translation from paired points, assuming aligned orientations.

    Rotation estimation and robust outlier rejection are intentionally deferred.
    """
    grid = np.asarray(grid_points, dtype=float)
    facade = np.asarray(facade_points, dtype=float)
    if grid.shape != facade.shape or grid.ndim != 2 or grid.shape[1] != 3:
        raise ValueError("point sets must have matching shapes of (N, 3)")
    if len(grid) == 0:
        raise ValueError("at least one point correspondence is required")
    translation = np.mean(facade - grid, axis=0)
    return make_transform(translation)
