"""Normalized placeholder metric for grid stability."""

from __future__ import annotations

import numpy as np
from numpy.typing import ArrayLike


def stability_score(
    tilt_rad: ArrayLike,
    flexion_m: ArrayLike,
    max_tilt_rad: float,
    max_flexion_m: float,
) -> float:
    """Return a score from zero (outside limits) to one (nominal)."""
    if max_tilt_rad <= 0 or max_flexion_m <= 0:
        raise ValueError("stability limits must be positive")
    tilt_ratio = np.max(np.abs(np.asarray(tilt_rad, dtype=float))) / max_tilt_rad
    flexion_ratio = (
        np.max(np.abs(np.asarray(flexion_m, dtype=float))) / max_flexion_m
    )
    penalty = 0.5 * float(tilt_ratio) + 0.5 * float(flexion_ratio)
    return float(np.clip(1.0 - penalty, 0.0, 1.0))
