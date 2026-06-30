#!/usr/bin/env python3
"""Generate and inspect the Gocator 2690 visual sensor scene."""

from __future__ import annotations

import argparse
import subprocess
from pathlib import Path

from pxr import Usd, UsdGeom, UsdPhysics

PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = PROJECT_ROOT / "outputs/isaac/gocator2690_verify.usda"
SENSOR_PATH = "/World/Robot/scanner_mount_link/Gocator2690"
SCANNER_FRAME_PATH = f"{SENSOR_PATH}/scanner_frame"
SCAN_VOLUME_PATH = f"{SENSOR_PATH}/scan_volume"


def _format_vec(values) -> str:
    return "(" + ", ".join(f"{float(v):.6f}" for v in values) + ")"


def _generate_scene(output: Path) -> None:
    subprocess.run(
        [
            str(PROJECT_ROOT / "isaac/run_mvp.sh"),
            "--headless",
            "--test",
            "--no-lidar",
            "--no-ros",
            "--output",
            str(output),
        ],
        cwd=PROJECT_ROOT,
        check=True,
    )


def _inspect_scene(output: Path) -> int:
    stage = Usd.Stage.Open(str(output))
    if stage is None:
        raise RuntimeError(f"failed to open {output}")

    sensor = stage.GetPrimAtPath(SENSOR_PATH)
    frame = stage.GetPrimAtPath(SCANNER_FRAME_PATH)
    scan_volume = stage.GetPrimAtPath(SCAN_VOLUME_PATH)
    if not sensor:
        raise RuntimeError(f"missing sensor prim: {SENSOR_PATH}")
    if not frame:
        raise RuntimeError(f"missing scanner frame: {SCANNER_FRAME_PATH}")
    if not scan_volume:
        raise RuntimeError(f"missing scan volume: {SCAN_VOLUME_PATH}")

    cache = UsdGeom.XformCache()
    frame_matrix = cache.GetLocalToWorldTransform(frame)
    translation = frame_matrix.ExtractTranslation()
    x_axis = frame_matrix.TransformDir((1.0, 0.0, 0.0)).GetNormalized()
    y_axis = frame_matrix.TransformDir((0.0, 1.0, 0.0)).GetNormalized()
    z_axis = frame_matrix.TransformDir((0.0, 0.0, 1.0)).GetNormalized()

    collision_prims = []
    visual_collision_prims = []
    for prim in Usd.PrimRange(sensor):
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            path = str(prim.GetPath())
            collision_prims.append(path)
            if f"{SENSOR_PATH}/visual" in path:
                visual_collision_prims.append(path)

    print(f"scene: {output}")
    print(f"sensor: {SENSOR_PATH}")
    print(f"scanner_frame world translation m: {_format_vec(translation)}")
    print(f"scanner_frame world X axis: {_format_vec(x_axis)}")
    print(f"scanner_frame world Y axis: {_format_vec(y_axis)}")
    print(f"scanner_frame world Z axis: {_format_vec(z_axis)}")
    print("sensor metadata:")
    for name in (
        "model",
        "x_fov_near_mm",
        "x_fov_far_mm",
        "z_measurement_range_mm",
        "clearance_distance_mm",
        "nominal_standoff_mm",
    ):
        attr = sensor.GetAttribute(name)
        print(f"  {name}: {attr.Get() if attr else '<missing>'}")
    print("collision prims:")
    for path in collision_prims:
        print(f"  {path}")
    print("scan volume:")
    for name in (
        "clearance_distance_mm",
        "measurement_range_mm",
        "x_fov_near_mm",
        "x_fov_far_mm",
    ):
        attr = scan_volume.GetAttribute(name)
        print(f"  {name}: {attr.Get() if attr else '<missing>'}")

    if visual_collision_prims:
        print("ERROR: detailed visual CAD has PhysicsCollisionAPI:")
        for path in visual_collision_prims:
            print(f"  {path}")
        return 1
    if collision_prims != [f"{SENSOR_PATH}/collision/body"]:
        print("ERROR: expected exactly one simplified collision body")
        return 1

    print("OK: only simplified Gocator2690 collision participates in PhysX")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--skip-generate", action="store_true")
    args = parser.parse_args()

    output = args.output.resolve()
    if not args.skip_generate:
        _generate_scene(output)
    return _inspect_scene(output)


if __name__ == "__main__":
    raise SystemExit(main())
