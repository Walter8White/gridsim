"""State container for a rectangular modular grid."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum


class ModuleState(str, Enum):
    """Discrete deployment state for one grid module."""

    UNKNOWN = "unknown"
    FOLDED = "folded"
    DEPLOYING = "deploying"
    LOCKED = "locked"
    FAULT = "fault"


@dataclass
class GridModel:
    """Rectangular grid with row-major module indexing."""

    rows: int
    columns: int
    module_width_m: float
    module_height_m: float
    initial_state: ModuleState = ModuleState.LOCKED
    _states: list[ModuleState] = field(init=False, repr=False)

    def __post_init__(self) -> None:
        if self.rows <= 0 or self.columns <= 0:
            raise ValueError("rows and columns must be positive")
        if self.module_width_m <= 0 or self.module_height_m <= 0:
            raise ValueError("module dimensions must be positive")
        self._states = [self.initial_state] * self.module_count

    @property
    def module_count(self) -> int:
        return self.rows * self.columns

    def module_index(self, row: int, column: int) -> int:
        """Return the row-major index for a module."""
        if not 0 <= row < self.rows or not 0 <= column < self.columns:
            raise IndexError("module coordinates are outside the grid")
        return row * self.columns + column

    def module_coordinates(self, index: int) -> tuple[int, int]:
        """Return row and column for a row-major module index."""
        if not 0 <= index < self.module_count:
            raise IndexError("module index is outside the grid")
        return divmod(index, self.columns)

    def module_center(self, row: int, column: int) -> tuple[float, float, float]:
        """Return the module center in the grid frame."""
        self.module_index(row, column)
        return (
            (column + 0.5) * self.module_width_m,
            (row + 0.5) * self.module_height_m,
            0.0,
        )

    def set_module_state(
        self, row: int, column: int, state: ModuleState | str
    ) -> None:
        self._states[self.module_index(row, column)] = ModuleState(state)

    def get_module_state(self, row: int, column: int) -> ModuleState:
        return self._states[self.module_index(row, column)]
