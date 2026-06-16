"""Load and validate the geometry needed by the Isaac Sim MVP scene."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import yaml


@dataclass(frozen=True)
class MvpSceneConfig:
    """Compact, simulator-independent description of the first MVP scene."""

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
    grid_imu_rate_hz: float
    robot_imu_rate_hz: float
    lidar_rate_hz: float
    lidar_translation_m: tuple[float, float, float]
    robot_imu_translation_m: tuple[float, float, float]
    grid_imu_translation_m: tuple[float, float, float]

    @property
    def grid_width_m(self) -> float:
        return self.grid_columns * self.module_width_m

    @property
    def grid_height_m(self) -> float:
        return self.grid_rows * self.module_height_m

    @classmethod
    def from_directory(cls, config_dir: str | Path) -> "MvpSceneConfig":
        """Load the MVP scene parameters from the repository YAML files."""
        directory = Path(config_dir)
        grid = _load_yaml(directory / "grid.yaml")
        facade = _load_yaml(directory / "facade.yaml")
        sensors = _load_yaml(directory / "sensors.yaml")
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
            grid_imu_rate_hz=float(simulation["publishing_rates_hz"]["grid_imu"]),
            robot_imu_rate_hz=float(
                simulation["publishing_rates_hz"]["robot_imu"]
            ),
            lidar_rate_hz=float(simulation["publishing_rates_hz"]["lidar"]),
            lidar_translation_m=_translation(sensors["lidar"]),
            robot_imu_translation_m=_translation(sensors["robot_imu"]),
            grid_imu_translation_m=_translation(sensors["grid_imu"]),
        )
        config.validate()
        return config

    def validate(self) -> None:
        """Reject invalid dimensions and rates before simulator startup."""
        positive_values = {
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
            "grid_imu_rate_hz": self.grid_imu_rate_hz,
            "robot_imu_rate_hz": self.robot_imu_rate_hz,
            "lidar_rate_hz": self.lidar_rate_hz,
        }
        invalid = [name for name, value in positive_values.items() if value <= 0]
        if invalid:
            raise ValueError(f"scene values must be positive: {', '.join(invalid)}")


def _load_yaml(path: Path) -> dict:
    if not path.is_file():
        raise FileNotFoundError(f"missing configuration file: {path}")
    with path.open(encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict):
        raise ValueError(f"configuration must contain a mapping: {path}")
    return data


def _translation(sensor: dict) -> tuple[float, float, float]:
    values = sensor["extrinsics"]["translation_m"]
    if not isinstance(values, list) or len(values) != 3:
        raise ValueError("sensor translation must contain exactly three values")
    return tuple(float(value) for value in values)
