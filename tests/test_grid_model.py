import pytest

from gridsim_core.grid_model import GridModel, ModuleState


def test_grid_module_indexing_is_row_major() -> None:
    grid = GridModel(rows=3, columns=4, module_width_m=1.0, module_height_m=1.0)

    assert grid.module_index(0, 0) == 0
    assert grid.module_index(2, 3) == 11
    assert grid.module_coordinates(6) == (1, 2)


def test_grid_module_state_and_bounds() -> None:
    grid = GridModel(rows=2, columns=2, module_width_m=1.0, module_height_m=1.0)
    grid.set_module_state(1, 0, ModuleState.FAULT)

    assert grid.get_module_state(1, 0) is ModuleState.FAULT
    with pytest.raises(IndexError):
        grid.module_index(2, 0)
