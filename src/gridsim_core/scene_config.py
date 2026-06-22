"""Load and validate the geometry needed by the Isaac Sim scene."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import yaml


@dataclass(frozen=True)
class MvpSceneConfig:
    facade_width_m: float
    facade_height_m: float
    facade_mass_kg: float
    grid_rows: int
    grid_columns: int
    grid_mass_kg: float
    module_width_m: float
    module_height_m: float
    module_depth_m: float
    facade_standoff_m: float
    simulation_frequency_hz: float

    @property
    def grid_width_m(self) -> float:
        return self.grid_columns * self.module_width_m

    @property
    def grid_height_m(self) -> float:
        return self.grid_rows * self.module_height_m

    @classmethod
    def from_directory(cls, config_dir: str | Path) -> "MvpSceneConfig":
        directory = Path(config_dir)
        grid = _load_yaml(directory / "grid.yaml")
        facade = _load_yaml(directory / "facade.yaml")
        simulation = _load_yaml(directory / "simulation.yaml")

        config = cls(
            facade_width_m=float(facade["dimensions_m"]["width"]),
            facade_height_m=float(facade["dimensions_m"]["height"]),
            facade_mass_kg=float(facade.get("mass_kg", 1_000_000_000.0)),
            grid_rows=int(grid["rows"]),
            grid_columns=int(grid["columns"]),
            grid_mass_kg=float(grid.get("mass_kg", 180.0)),
            module_width_m=float(grid["module"]["width_m"]),
            module_height_m=float(grid["module"]["height_m"]),
            module_depth_m=float(grid["module"]["depth_m"]),
            facade_standoff_m=float(grid.get("facade_standoff_m", 1.25)),
            simulation_frequency_hz=float(simulation["simulation_frequency_hz"]),
        )
        config.validate()
        return config

    def validate(self) -> None:
        positive = {
            "facade_width_m": self.facade_width_m,
            "facade_height_m": self.facade_height_m,
            "facade_mass_kg": self.facade_mass_kg,
            "grid_rows": self.grid_rows,
            "grid_columns": self.grid_columns,
            "grid_mass_kg": self.grid_mass_kg,
            "module_width_m": self.module_width_m,
            "module_height_m": self.module_height_m,
            "module_depth_m": self.module_depth_m,
            "facade_standoff_m": self.facade_standoff_m,
            "simulation_frequency_hz": self.simulation_frequency_hz,
        }
        invalid = [k for k, v in positive.items() if v <= 0]
        if invalid:
            raise ValueError(f"scene values must be positive: {', '.join(invalid)}")


def _load_yaml(path: Path) -> dict:
    if not path.is_file():
        raise FileNotFoundError(f"missing configuration file: {path}")
    with path.open(encoding="utf-8") as f:
        data = yaml.safe_load(f)
    if not isinstance(data, dict):
        raise ValueError(f"configuration must contain a mapping: {path}")
    return data
