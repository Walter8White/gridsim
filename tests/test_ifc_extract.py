from pathlib import Path

import pytest

pytest.importorskip("ifcopenshell")

from gridsim_ifc import extract_ifc

FZK_HAUS_PATH = Path(__file__).resolve().parents[1] / "assets/facades/fzk_haus/AC20-FZK-Haus.ifc"


@pytest.fixture(scope="module")
def fzk_haus():
    if not FZK_HAUS_PATH.exists():
        pytest.skip(f"sample IFC not present at {FZK_HAUS_PATH}")
    return extract_ifc(FZK_HAUS_PATH)


def test_units_are_metres(fzk_haus) -> None:
    assert fzk_haus.units.length_unit_name == "METRE"
    assert fzk_haus.units.scale_to_m == pytest.approx(1.0)


def test_finds_levels(fzk_haus) -> None:
    assert len(fzk_haus.levels) >= 2
    names = {level.name for level in fzk_haus.levels}
    assert "Erdgeschoss" in names


def test_finds_walls_with_positive_bbox_extent(fzk_haus) -> None:
    assert len(fzk_haus.walls) > 0
    for wall in fzk_haus.walls:
        extent = [hi - lo for lo, hi in zip(wall.bbox_m.min_m, wall.bbox_m.max_m)]
        assert all(e > 0.0 for e in extent), f"wall {wall.guid} has a degenerate bbox"


def test_finds_at_least_one_exterior_wall(fzk_haus) -> None:
    assert any(wall.is_external for wall in fzk_haus.walls)


def test_finds_windows_and_doors(fzk_haus) -> None:
    assert len(fzk_haus.windows) > 0
    assert len(fzk_haus.doors) > 0


def test_openings_resolve_a_host_wall(fzk_haus) -> None:
    wall_guids = {wall.guid for wall in fzk_haus.walls}
    resolved = [w for w in fzk_haus.windows + fzk_haus.doors if w.host_wall_guid in wall_guids]
    assert resolved, "no window/door resolved a host wall guid"


def test_finds_roof_slabs(fzk_haus) -> None:
    assert len(fzk_haus.roofs) > 0
    for roof in fzk_haus.roofs:
        extent = [hi - lo for lo, hi in zip(roof.bbox_m.min_m, roof.bbox_m.max_m)]
        assert all(e > 0.0 for e in extent), f"roof {roof.guid} has a degenerate bbox"
