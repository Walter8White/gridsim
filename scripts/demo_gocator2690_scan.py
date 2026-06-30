#!/usr/bin/env python3
"""Demo Gocator 2690 encoder-triggered scan on a synthetic facade panel."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "src"))

from gridsim_sensors import (  # noqa: E402
    Gocator2690LineProfiler,
    Gocator2690Spec,
    GocatorEncoderTriggeredAcquisition,
    GocatorPointCloudAccumulator,
    ScannerFramePose,
    export_height_map_npy,
    export_point_cloud_ply,
    export_profiles_csv,
    export_profiles_npz,
)


def synthetic_facade_y(x_m: np.ndarray, z_m: np.ndarray) -> np.ndarray:
    """2 m x 2 m facade: mm bowing, one vertical joint, localized defects."""
    x = np.asarray(x_m, dtype=np.float64)
    z = np.asarray(z_m, dtype=np.float64)
    bow = 0.006 * (1.0 - (x / 1.0) ** 2) * (1.0 - ((z - 1.0) / 1.0) ** 2)
    joint = np.where(np.abs(x - 0.18) < 0.012, -0.004, 0.0)
    dent = -0.007 * np.exp(-(((x + 0.35) / 0.08) ** 2 + ((z - 1.25) / 0.10) ** 2))
    bump = 0.005 * np.exp(-(((x - 0.45) / 0.06) ** 2 + ((z - 0.55) / 0.07) ** 2))
    patch_offset = np.where((x > -0.75) & (x < -0.55) & (z > 0.35) & (z < 0.75), 0.003, 0.0)
    return bow + joint + dent + bump + patch_offset


def _scanner_pose(z_m: float) -> ScannerFramePose:
    return ScannerFramePose.from_arrays(
        [0.0, -1.0, z_m],
        [1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0],
        [0.0, 1.0, 0.0],
    )


def _add_mesh(stage: Usd.Stage, path: str, points, counts, indices, color) -> None:
    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr([Gf.Vec3f(*p) for p in points])
    mesh.CreateFaceVertexCountsAttr(counts)
    mesh.CreateFaceVertexIndicesAttr(indices)
    mesh.CreateSubdivisionSchemeAttr("none")
    mesh.CreateDisplayColorAttr([Gf.Vec3f(*color)])


def _add_facade_debug(stage: Usd.Stage, path: str, resolution: int = 80) -> None:
    xs = np.linspace(-1.0, 1.0, resolution)
    zs = np.linspace(0.0, 2.0, resolution)
    points = []
    for z in zs:
        for x in xs:
            points.append((x, float(synthetic_facade_y(x, z)), z))
    counts = []
    indices = []
    for row in range(resolution - 1):
        for col in range(resolution - 1):
            i = row * resolution + col
            counts.append(4)
            indices.extend([i, i + 1, i + resolution + 1, i + resolution])
    _add_mesh(stage, path, points, counts, indices, (0.55, 0.58, 0.62))


def _add_axes(stage: Usd.Stage, pose: ScannerFramePose, root: str) -> None:
    for name, axis, color in (
        ("x", pose.x_axis, (1.0, 0.05, 0.05)),
        ("y", pose.y_axis, (0.05, 1.0, 0.05)),
        ("z", pose.z_axis, (0.05, 0.2, 1.0)),
    ):
        curve = UsdGeom.BasisCurves.Define(stage, f"{root}/{name}_axis")
        curve.CreateTypeAttr("linear")
        curve.CreateCurveVertexCountsAttr([2])
        curve.CreatePointsAttr([
            Gf.Vec3f(*pose.origin_m),
            Gf.Vec3f(*(pose.origin_m + axis * 0.18)),
        ])
        curve.CreateWidthsAttr([0.01, 0.01])
        curve.CreateDisplayColorAttr([Gf.Vec3f(*color)])


def _add_debug_rays(stage: Usd.Stage, profile, root: str, stride: int) -> None:
    valid_indices = np.flatnonzero(profile.valid_mask)
    sampled = valid_indices[:: max(1, stride)]
    points = []
    counts = []
    for index in sampled:
        points.extend([profile.scanner_pose_world.origin_m, profile.points_world[index]])
        counts.append(2)
    if not counts:
        return
    curve = UsdGeom.BasisCurves.Define(stage, f"{root}/sample_rays")
    curve.CreateTypeAttr("linear")
    curve.CreateCurveVertexCountsAttr(counts)
    curve.CreatePointsAttr([Gf.Vec3f(*p) for p in points])
    curve.CreateWidthsAttr([0.002] * len(points))
    curve.CreateDisplayColorAttr([Gf.Vec3f(0.1, 0.45, 1.0)])


def _add_points(stage: Usd.Stage, path: str, points: np.ndarray, color, width: float) -> None:
    prim = UsdGeom.Points.Define(stage, path)
    prim.CreatePointsAttr([Gf.Vec3f(*p) for p in points])
    prim.CreateWidthsAttr([width] * len(points))
    prim.CreateDisplayColorAttr([Gf.Vec3f(*color)])


def _write_debug_usd(path: Path, profiles, cloud: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    stage = Usd.Stage.CreateNew(str(path))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    stage.DefinePrim("/World", "Xform")
    stage.SetDefaultPrim(stage.GetPrimAtPath("/World"))
    UsdLux.DomeLight.Define(stage, "/World/DomeLight").CreateIntensityAttr(600.0)
    _add_facade_debug(stage, "/World/demo_facade")
    if profiles:
        _add_axes(stage, profiles[0].scanner_pose_world, "/World/scanner_frame_axes")
        _add_debug_rays(stage, profiles[0], "/World/profile_debug", stride=max(1, profiles[0].x_m.size // 64))
        _add_points(stage, "/World/profile_debug/profile_hits", profiles[0].valid_points_m, (1.0, 0.55, 0.05), 0.01)
    if len(cloud):
        _add_points(stage, "/World/accumulated_point_cloud", cloud[:: max(1, len(cloud) // 15000)], (0.05, 0.9, 0.95), 0.004)
    stage.GetRootLayer().Save()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", type=Path, default=PROJECT_ROOT / "outputs/gocator_demo")
    parser.add_argument("--points-per-profile", type=int, default=3700)
    parser.add_argument("--profile-spacing", type=float, default=0.0005)
    parser.add_argument("--strip-height", type=float, default=2.0)
    parser.add_argument("--scan-speed", type=float, default=0.04)
    parser.add_argument("--sim-dt", type=float, default=0.01)
    parser.add_argument("--max-profiles", type=int, default=240, help="Runtime guard for the quick demo; set <=0 for full strip.")
    parser.add_argument("--csv", action="store_true", help="Also export per-point CSV; can be large.")
    args = parser.parse_args()

    spec = Gocator2690Spec(
        points_per_profile=args.points_per_profile,
        profile_spacing_m=args.profile_spacing,
        nominal_standoff_m=1.0,
    )
    profiler = Gocator2690LineProfiler(spec)

    def sampler(pose: ScannerFramePose, timestamp_s: float, profile_index: int, encoder_position_m: float):
        return profiler.sample_surface(
            pose,
            synthetic_facade_y,
            timestamp_s=timestamp_s,
            profile_index=profile_index,
            encoder_position_m=encoder_position_m,
        )

    acquisition = GocatorEncoderTriggeredAcquisition(profiler, sampler)
    accumulator = GocatorPointCloudAccumulator(spec.profile_spacing_m)
    profiles = []

    z0 = 0.0
    t = 0.0
    while True:
        z = z0 + args.scan_speed * t
        pose = _scanner_pose(z)
        for profile in acquisition.update(pose, t):
            profiles.append(profile)
            accumulator.add_profile(profile)
        if z >= args.strip_height:
            break
        if args.max_profiles > 0 and len(profiles) >= args.max_profiles:
            break
        t += args.sim_dt

    cloud = accumulator.point_cloud()
    height_map = accumulator.height_map_array()
    out = args.output_dir.resolve()
    out.mkdir(parents=True, exist_ok=True)
    export_profiles_npz(out / "gocator_profiles.npz", profiles)
    if args.csv:
        export_profiles_csv(out / "gocator_profiles.csv", profiles)
    export_point_cloud_ply(out / "gocator_point_cloud.ply", cloud)
    export_height_map_npy(out / "gocator_height_map.npy", height_map)
    _write_debug_usd(out / "gocator_demo_debug.usda", profiles, cloud)

    print(f"profiles: {len(profiles)}")
    print(f"points_per_profile: {spec.points_per_profile}")
    print(f"valid_points: {len(cloud)}")
    print(f"encoder_span_m: {profiles[-1].encoder_position_m if profiles else 0.0:.6f}")
    print(f"npz: {out / 'gocator_profiles.npz'}")
    print(f"ply: {out / 'gocator_point_cloud.ply'}")
    print(f"height_map: {out / 'gocator_height_map.npy'}")
    print(f"debug_usd: {out / 'gocator_demo_debug.usda'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
