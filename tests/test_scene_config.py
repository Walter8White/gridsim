from pathlib import Path

import pytest

from gridsim_core.scene_config import MvpSceneConfig

PROJECT_ROOT = Path(__file__).resolve().parents[1]


def test_mvp_scene_config_loads_repository_yaml() -> None:
    config = MvpSceneConfig.from_directory(PROJECT_ROOT / "configs")

    assert config.grid_rows == 4
    assert config.grid_columns == 6
    assert config.grid_width_m == pytest.approx(6.0)
    assert config.grid_height_m == pytest.approx(4.0)
    assert config.facade_standoff_m == pytest.approx(1.25)
    assert config.facade_mass_kg == pytest.approx(1_000_000_000.0)
    assert config.grid_mass_kg == pytest.approx(180.0)
    assert config.simulation_frequency_hz == pytest.approx(240.0)
