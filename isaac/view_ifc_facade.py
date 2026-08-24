#!/usr/bin/env python3
"""Minimal Isaac Sim viewer for a converted IFC facade USD asset.

This utility only loads the converted facade scene for visual inspection, with
no sensor or ROS wiring.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--test", action="store_true")
    parser.add_argument(
        "--ifc-usd",
        type=Path,
        default=Path("assets/facades/fzk_haus/fzk_haus_visual.usd"),
        help="Converted IFC USD asset to reference (see assets/facades/*/convert_*.py).",
    )
    parser.add_argument("--frames", type=int, default=0, help="Exit after N frames (0 = run until closed).")
    parser.add_argument("--output", type=Path)
    args, kit_args = parser.parse_known_args()
    sys.argv = [sys.argv[0], *kit_args]
    return args


ARGS = parse_args()

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": ARGS.headless or ARGS.test, "renderer": "RayTracedLighting"})

import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import omni.usd
from pxr import Gf, Usd, UsdGeom, UsdLux, UsdPhysics

PROJECT_ROOT = Path(__file__).resolve().parents[1]
WORLD_PATH = "/World"
IFC_FACADE_PATH = "/World/ifc_facade"
GROUND_PATH = "/World/ground"
GROUND_MARGIN_M = 5.0
GROUND_THICKNESS_M = 0.1


def _project_path(path_value: Path) -> Path:
    return path_value if path_value.is_absolute() else PROJECT_ROOT / path_value


def _create_box(stage, path: str, size, translation, color) -> UsdGeom.Cube:
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    cube.AddTranslateOp().Set(Gf.Vec3d(*translation))
    cube.AddScaleOp().Set(Gf.Vec3f(*size))
    cube.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    return cube


def _add_ground_under(stage, reference_prim) -> None:
    """Size/position a collidable ground plane from the referenced asset's own bbox, so it
    fits whichever IFC building was loaded rather than assuming FZK-Haus's footprint."""
    bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    bound = bbox_cache.ComputeWorldBound(reference_prim).ComputeAlignedRange()
    lo, hi = bound.GetMin(), bound.GetMax()
    center_x = (lo[0] + hi[0]) / 2.0
    center_y = (lo[1] + hi[1]) / 2.0
    size_x = (hi[0] - lo[0]) + GROUND_MARGIN_M * 2.0
    size_y = (hi[1] - lo[1]) + GROUND_MARGIN_M * 2.0
    ground_top_z = lo[2]

    ground = _create_box(
        stage,
        GROUND_PATH,
        (size_x, size_y, GROUND_THICKNESS_M),
        (center_x, center_y, ground_top_z - GROUND_THICKNESS_M / 2.0),
        (0.22, 0.24, 0.25),
    )
    UsdPhysics.CollisionAPI.Apply(ground.GetPrim())


def build_scene(ifc_usd_path: Path):
    stage_utils.create_new_stage()
    stage_utils.set_stage_units(meters_per_unit=1.0)
    stage = omni.usd.get_context().get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    stage.DefinePrim(WORLD_PATH, "Xform")
    stage.SetDefaultPrim(stage.GetPrimAtPath(WORLD_PATH))

    physics = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    physics.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
    physics.CreateGravityMagnitudeAttr().Set(9.81)

    UsdLux.DomeLight.Define(stage, "/World/DomeLight").CreateIntensityAttr(500.0)
    sun = UsdLux.DistantLight.Define(stage, "/World/Sun")
    sun.CreateIntensityAttr(1200.0)
    sun.CreateAngleAttr(1.0)

    ifc_facade = stage.DefinePrim(IFC_FACADE_PATH, "Xform")
    if not ifc_usd_path.exists():
        raise FileNotFoundError(
            f"IFC USD asset not found: {ifc_usd_path} "
            "(run the matching assets/facades/<name>/convert_*.py first)"
        )
    ifc_facade.GetReferences().AddReference(str(ifc_usd_path))
    _add_ground_under(stage, ifc_facade)

    return stage


def main() -> int:
    ifc_usd_path = _project_path(ARGS.ifc_usd)
    stage = build_scene(ifc_usd_path)

    out = (ARGS.output or PROJECT_ROOT / "outputs/isaac/ifc_facade_scene.usda").resolve()
    out.parent.mkdir(parents=True, exist_ok=True)
    stage.GetRootLayer().Export(str(out))
    print(f"Saved scene: {out}", flush=True)
    print(f"Referencing: {ifc_usd_path}", flush=True)

    app_utils.play()
    simulation_app.update()

    frame_limit = ARGS.frames
    if ARGS.test and frame_limit <= 0:
        frame_limit = 10

    frame_count = 0
    try:
        while simulation_app.is_running():
            simulation_app.update()
            frame_count += 1
            if frame_limit > 0 and frame_count >= frame_limit:
                print(f"Completed {frame_count} simulation frames", flush=True)
                break
    finally:
        simulation_app.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
