#!/usr/bin/env python3
"""Run a deterministic, simulator-free demonstration of the MVP models."""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from gridsim_core import FacadeModel, GridModel
from gridsim_estimation import DifferentialDriveOdometry, stability_score
from gridsim_sensors import DistanceSensorModel, EncoderModel


def load_json(path: Path) -> dict:
    with path.open(encoding="utf-8") as stream:
        return json.load(stream)


def main() -> None:
    example_dir = Path(__file__).resolve().parent
    grid_config = load_json(example_dir / "simple_grid.json")
    facade_config = load_json(example_dir / "simple_facade.json")

    grid = GridModel(
        rows=grid_config["rows"],
        columns=grid_config["columns"],
        module_width_m=grid_config["module_width_m"],
        module_height_m=grid_config["module_height_m"],
    )
    facade = FacadeModel(
        width_m=facade_config["width_m"],
        height_m=facade_config["height_m"],
        normal=facade_config["normal"],
        offset_m=facade_config["offset_m"],
        anchor_points=facade_config["anchor_points"],
    )

    encoder = EncoderModel(resolution=0.001, noise_std=0.0002, seed=42)
    distance_sensor = DistanceSensorModel(0.05, 3.0, noise_std_m=0.003, seed=42)
    odometry = DifferentialDriveOdometry(
        wheel_radius_m=grid_config["wheel_radius_m"],
        wheel_separation_m=grid_config["wheel_separation_m"],
    )

    odometry.update(0.0, 0.0, dt_s=0.1)
    for wheel_position in np.linspace(0.2, 2.0, 10):
        measured = float(encoder.measure(wheel_position))
        odometry.update(measured, measured, dt_s=0.1)

    measured_standoff = float(
        distance_sensor.measure(grid_config["facade_standoff_m"])
    )
    score = stability_score(
        tilt_rad=[0.003, -0.002],
        flexion_m=[0.001, 0.002],
        max_tilt_rad=np.deg2rad(2.0),
        max_flexion_m=0.02,
    )

    print(f"Scenario: {grid.rows}x{grid.columns} grid, {grid.module_count} modules")
    print(f"Facade: {facade.width_m:.1f} m x {facade.height_m:.1f} m")
    print(f"Measured facade standoff: {measured_standoff:.3f} m")
    print(f"Estimated robot pose: x={odometry.state.x_m:.3f} m, "
          f"y={odometry.state.y_m:.3f} m, yaw={odometry.state.yaw_rad:.3f} rad")
    print(f"Placeholder stability score: {score:.3f}")


if __name__ == "__main__":
    main()
