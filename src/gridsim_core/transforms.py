"""Small helpers for rigid transforms represented as homogeneous matrices."""

from __future__ import annotations

import numpy as np
from numpy.typing import ArrayLike, NDArray


def make_transform(
    translation: ArrayLike = (0.0, 0.0, 0.0),
    rotation: ArrayLike | None = None,
) -> NDArray[np.float64]:
    """Create a 4x4 rigid transform from a translation and 3x3 rotation."""
    translation_array = np.asarray(translation, dtype=float)
    if translation_array.shape != (3,):
        raise ValueError("translation must contain exactly three values")

    rotation_array = (
        np.eye(3, dtype=float)
        if rotation is None
        else np.asarray(rotation, dtype=float)
    )
    if rotation_array.shape != (3, 3):
        raise ValueError("rotation must be a 3x3 matrix")

    transform = np.eye(4, dtype=float)
    transform[:3, :3] = rotation_array
    transform[:3, 3] = translation_array
    return transform


def compose_transforms(*transforms: ArrayLike) -> NDArray[np.float64]:
    """Compose transforms in parent-to-child order."""
    result = np.eye(4, dtype=float)
    for transform in transforms:
        matrix = np.asarray(transform, dtype=float)
        if matrix.shape != (4, 4):
            raise ValueError("each transform must be a 4x4 matrix")
        result = result @ matrix
    return result


def transform_points(transform: ArrayLike, points: ArrayLike) -> NDArray[np.float64]:
    """Apply a homogeneous transform to one point or an array of points."""
    matrix = np.asarray(transform, dtype=float)
    point_array = np.asarray(points, dtype=float)
    if matrix.shape != (4, 4):
        raise ValueError("transform must be a 4x4 matrix")
    if point_array.shape[-1:] != (3,):
        raise ValueError("points must have a final dimension of size three")

    flat_points = point_array.reshape(-1, 3)
    homogeneous = np.column_stack((flat_points, np.ones(len(flat_points))))
    transformed = (matrix @ homogeneous.T).T[:, :3]
    return transformed.reshape(point_array.shape)
