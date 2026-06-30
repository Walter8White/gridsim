#!/usr/bin/env python3
"""Generate and inspect the first functional Gocator 2690 profile simulation."""

from __future__ import annotations

import argparse
import subprocess
from pathlib import Path

import numpy as np
from pxr import Usd, UsdGeom

PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SCENE = PROJECT_ROOT / "outputs/isaac/gocator2690_profile_scene.usda"
DEFAULT_CLOUD = PROJECT_ROOT / "outputs/isaac/gocator2690_point_cloud.csv"
SCANNER_FRAME_PATH = "/World/Robot/scanner_mount_link/Gocator2690/scanner_frame"

import sys

sys.path.insert(0, str(PROJECT_ROOT / "src"))

from gridsim_sensors import (  # noqa: E402
    Gocator2690LineProfiler,
    Gocator2690Spec,
    GocatorPointCloudAccumulator,
    ScannerFramePose,
)


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


def _scanner_pose_from_stage(stage: Usd.Stage) -> ScannerFramePose:
    frame = stage.GetPrimAtPath(SCANNER_FRAME_PATH)
    if not frame:
        raise RuntimeError(f"missing scanner frame: {SCANNER_FRAME_PATH}")

    matrix = UsdGeom.XformCache().GetLocalToWorldTransform(frame)
    origin = np.array(matrix.ExtractTranslation(), dtype=np.float64)
    x_axis = np.array(matrix.TransformDir((1.0, 0.0, 0.0)), dtype=np.float64)
    y_axis = np.array(matrix.TransformDir((0.0, 1.0, 0.0)), dtype=np.float64)
    z_axis = np.array(matrix.TransformDir((0.0, 0.0, 1.0)), dtype=np.float64)
    return ScannerFramePose.from_arrays(origin, x_axis, y_axis, z_axis)


def _write_cloud(path: Path, cloud: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as fp:
        fp.write("x_m,y_m,z_m\n")
        for point in cloud:
            fp.write(f"{point[0]:.6f},{point[1]:.6f},{point[2]:.6f}\n")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scene", type=Path, default=DEFAULT_SCENE)
    parser.add_argument("--cloud", type=Path, default=DEFAULT_CLOUD)
    parser.add_argument("--profiles", type=int, default=8)
    parser.add_argument("--skip-generate", action="store_true")
    args = parser.parse_args()

    scene_path = args.scene.resolve()
    if not args.skip_generate:
        _generate_scene(scene_path)

    stage = Usd.Stage.Open(str(scene_path))
    if stage is None:
        raise RuntimeError(f"failed to open {scene_path}")

    pose = _scanner_pose_from_stage(stage)
    spec = Gocator2690Spec()
    profiler = Gocator2690LineProfiler(spec)
    profile = profiler.sample_facade_plane(pose)

    accumulator = GocatorPointCloudAccumulator(spec.profile_spacing_m)
    for index in range(args.profiles):
        moved_pose = ScannerFramePose.from_arrays(
            pose.origin_m + pose.y_axis * spec.profile_spacing_m * index,
            pose.x_axis,
            pose.y_axis,
            pose.z_axis,
        )
        accumulator.add_profile(
            profiler.sample_facade_plane(
                moved_pose,
                timestamp_s=index / spec.nominal_profile_rate_hz,
            )
        )

    cloud = accumulator.point_cloud()
    _write_cloud(args.cloud.resolve(), cloud)

    print(f"scene: {scene_path}")
    print(f"scanner_frame: {SCANNER_FRAME_PATH}")
    print(f"points_per_profile: {spec.points_per_profile}")
    print(f"nominal_standoff_m: {spec.nominal_standoff_m:.3f}")
    print(f"nominal_profile_width_m: {profiler.nominal_width_m:.6f}")
    print(f"measurement_range_m: {spec.min_measurement_distance_m:.3f}..{spec.max_measurement_distance_m:.3f}")
    print(f"single_profile_valid_hits: {int(profile.valid.sum())}/{profile.valid.size}")
    if profile.valid.any():
        valid_ranges = profile.ranges_m[profile.valid]
        print(f"single_profile_range_m: {valid_ranges.min():.6f}..{valid_ranges.max():.6f}")
    print(f"accumulated_profiles: {args.profiles}")
    print(f"point_cloud_points: {len(cloud)}")
    print(f"height_map_cells: {len(accumulator.height_map())}")
    print(f"point_cloud_csv: {args.cloud.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
