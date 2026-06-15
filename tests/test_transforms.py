import numpy as np

from gridsim_core.transforms import (
    compose_transforms,
    make_transform,
    transform_points,
)


def test_transform_composition_applies_both_translations() -> None:
    world_from_grid = make_transform((1.0, 2.0, 0.0))
    grid_from_robot = make_transform((0.5, 0.0, 0.2))

    world_from_robot = compose_transforms(world_from_grid, grid_from_robot)

    np.testing.assert_allclose(world_from_robot[:3, 3], [1.5, 2.0, 0.2])
    np.testing.assert_allclose(
        transform_points(world_from_robot, [0.0, 0.0, 0.0]),
        [1.5, 2.0, 0.2],
    )
