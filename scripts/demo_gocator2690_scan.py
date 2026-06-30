#!/usr/bin/env python3
"""Demo Gocator 2690 boustrophedon scan on the sensor-integration facade."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

import numpy as np
from pxr import Gf, Usd, UsdGeom, UsdLux

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

FACADE_WIDTH_M = 10.0
FACADE_HEIGHT_M = 10.0
SENSOR_STANDOFF_M = 1.0


def synthetic_facade_y(x_m: np.ndarray, z_m: np.ndarray) -> np.ndarray:
    """Same signed height-field convention as isaac/run_mvp.py.

    Sensor is at negative Y and looks toward +Y. Negative Y offsets protrude
    toward the sensor, positive Y offsets are recesses/craters.
    """
    x = np.asarray(x_m, dtype=np.float64)
    z = np.asarray(z_m, dtype=np.float64)
    bow = -0.045 * (1.0 - (x / 5.0) ** 2) * (1.0 - ((z - 5.0) / 5.0) ** 2)
    waves = (
        -0.006 * np.sin(1.5 * x + 0.4) * np.sin(1.2 * z)
        -0.0025 * np.sin(8.5 * x + 1.7) * np.sin(7.0 * z)
    )

    defects = np.zeros_like(np.broadcast_arrays(x, z)[0], dtype=np.float64)
    for cx, cz, sx, sz, amp in (
        (-2.2, 2.8, 0.35, 0.42, 0.045),
        (1.9, 6.8, 0.55, 0.36, 0.036),
        (3.6, 3.2, 0.42, 0.46, 0.030),
        (-3.5, 7.4, 0.45, 0.34, 0.040),
        (0.1, 4.8, 0.22, 0.20, 0.020),
        (-4.4, 5.9, 0.18, 0.25, 0.018),
        (-1.1, 5.7, 0.48, 0.38, -0.030),
        (2.8, 1.6, 0.35, 0.36, -0.026),
        (-4.0, 4.2, 0.30, 0.48, -0.020),
        (0.4, 8.4, 0.55, 0.32, -0.028),
        (4.1, 8.0, 0.26, 0.22, -0.016),
        (-0.9, 1.5, 0.055, 0.060, 0.006),
        (-0.1, 2.9, 0.040, 0.055, 0.005),
        (1.4, 4.2, 0.050, 0.045, 0.006),
        (2.7, 5.4, 0.065, 0.055, 0.007),
        (-3.9, 6.1, 0.045, 0.070, 0.006),
        (-2.6, 8.7, 0.055, 0.055, 0.005),
        (3.8, 7.2, 0.050, 0.050, 0.006),
        (4.5, 3.9, 0.040, 0.045, 0.004),
        (-1.8, 1.9, 0.050, 0.050, -0.004),
        (0.7, 3.7, 0.060, 0.045, -0.005),
        (2.2, 8.8, 0.045, 0.060, -0.004),
        (-4.4, 2.9, 0.055, 0.040, -0.004),
    ):
        defects += amp * np.exp(-(((x - cx) / sx) ** 2 + ((z - cz) / sz) ** 2))

    joints = np.zeros_like(defects)
    for joint_x, phase in ((-3.2, 0.0), (-1.1, 0.8), (1.2, 1.9), (3.4, 2.7)):
        center = joint_x + 0.035 * np.sin(1.7 * z + phase) + 0.012 * np.sin(6.5 * z + phase)
        width = 0.018 + 0.010 * (0.5 + 0.5 * np.sin(3.1 * z + phase))
        joints = np.where(np.abs(x - center) < width, joints + 0.009, joints)
    for joint_z, phase in ((2.2, 0.4), (5.0, 1.5), (7.8, 2.1)):
        center = joint_z + 0.030 * np.sin(1.4 * x + phase) + 0.010 * np.sin(5.5 * x)
        width = 0.016 + 0.008 * (0.5 + 0.5 * np.sin(2.5 * x + phase))
        joints = np.where(np.abs(z - center) < width, joints + 0.006, joints)

    patches = np.zeros_like(defects)
    for x0, x1, z0, z1, offset in (
        (-4.4, -3.5, 0.9, 1.7, 0.006),
        (-0.7, 0.1, 3.2, 4.1, 0.005),
        (2.3, 3.5, 7.0, 8.0, 0.007),
        (-2.8, -1.7, 8.2, 9.1, 0.005),
    ):
        patches = np.where((x > x0) & (x < x1) & (z > z0) & (z < z1), patches + offset, patches)
    return bow + waves + defects + joints + patches


def _scanner_pose(x_m: float, z_m: float) -> ScannerFramePose:
    return ScannerFramePose.from_arrays(
        [x_m, -SENSOR_STANDOFF_M, z_m],
        [1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0],
        [0.0, 1.0, 0.0],
    )


def _wall_valid_mask(points_world: np.ndarray) -> np.ndarray:
    return (
        (points_world[:, 0] >= -FACADE_WIDTH_M / 2.0)
        & (points_world[:, 0] <= FACADE_WIDTH_M / 2.0)
        & (points_world[:, 2] >= 0.0)
        & (points_world[:, 2] <= FACADE_HEIGHT_M)
    )


def _add_mesh(stage: Usd.Stage, path: str, points, counts, indices, color) -> None:
    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr([Gf.Vec3f(*p) for p in points])
    mesh.CreateFaceVertexCountsAttr(counts)
    mesh.CreateFaceVertexIndicesAttr(indices)
    mesh.CreateSubdivisionSchemeAttr("none")
    mesh.CreateDisplayColorAttr([Gf.Vec3f(*color)])


def _add_facade_debug(stage: Usd.Stage, path: str, resolution: int = 120) -> None:
    xs = np.linspace(-FACADE_WIDTH_M / 2.0, FACADE_WIDTH_M / 2.0, resolution)
    zs = np.linspace(0.0, FACADE_HEIGHT_M, resolution)
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
            Gf.Vec3f(*(pose.origin_m + axis * 0.35)),
        ])
        curve.CreateWidthsAttr([0.012, 0.012])
        curve.CreateDisplayColorAttr([Gf.Vec3f(*color)])


def _add_polyline(stage: Usd.Stage, path: str, points: np.ndarray, color, width: float) -> None:
    if len(points) < 2:
        return
    curve = UsdGeom.BasisCurves.Define(stage, path)
    curve.CreateTypeAttr("linear")
    curve.CreateCurveVertexCountsAttr([len(points)])
    curve.CreatePointsAttr([Gf.Vec3f(*p) for p in points])
    curve.CreateWidthsAttr([width] * len(points))
    curve.CreateDisplayColorAttr([Gf.Vec3f(*color)])


def _add_laser_profile_lines(stage: Usd.Stage, profiles, root: str, max_profiles: int = 80) -> None:
    if not profiles:
        return
    step = max(1, len(profiles) // max_profiles)
    for debug_index, profile in enumerate(profiles[::step]):
        valid_points = profile.valid_points_m
        if len(valid_points) < 2:
            continue
        sampled = valid_points[:: max(1, len(valid_points) // 400)]
        _add_polyline(
            stage,
            f"{root}/laser_contact_{debug_index:03d}",
            sampled,
            (1.0, 0.0, 0.0),
            0.012,
        )


def _add_points(stage: Usd.Stage, path: str, points: np.ndarray, color, width: float) -> None:
    if len(points) == 0:
        return
    prim = UsdGeom.Points.Define(stage, path)
    prim.CreatePointsAttr([Gf.Vec3f(*p) for p in points])
    prim.CreateWidthsAttr([width] * len(points))
    prim.CreateDisplayColorAttr([Gf.Vec3f(*color)])


def _write_debug_usd(path: Path, profiles, cloud: np.ndarray, scan_path: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    stage = Usd.Stage.CreateNew(str(path))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    stage.DefinePrim("/World", "Xform")
    stage.SetDefaultPrim(stage.GetPrimAtPath("/World"))
    UsdLux.DomeLight.Define(stage, "/World/DomeLight").CreateIntensityAttr(600.0)
    _add_facade_debug(stage, "/World/demo_facade")
    _add_polyline(stage, "/World/scanner_path", scan_path, (1.0, 0.85, 0.05), 0.018)
    if profiles:
        _add_axes(stage, profiles[0].scanner_pose_world, "/World/scanner_frame_axes_start")
        _add_axes(stage, profiles[-1].scanner_pose_world, "/World/scanner_frame_axes_end")
        _add_laser_profile_lines(stage, profiles, "/World/laser_contact_lines")
    if len(cloud):
        display_cloud = cloud[:: max(1, len(cloud) // 60000)]
        _add_points(stage, "/World/accumulated_point_cloud", display_cloud, (0.05, 0.9, 0.95), 0.006)
    stage.GetRootLayer().Save()


def _build_pass_centers(scan_width_m: float, horizontal_overlap: float) -> list[float]:
    step_m = scan_width_m * (1.0 - horizontal_overlap)
    step_m = max(step_m, 1e-6)
    first = -FACADE_WIDTH_M / 2.0 + scan_width_m / 2.0
    last = FACADE_WIDTH_M / 2.0 - scan_width_m / 2.0
    centers = []
    x = first
    while x <= last + 1e-9:
        centers.append(float(x))
        x += step_m
    if not centers or centers[-1] < last - 1e-6:
        centers.append(float(last))
    return centers


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", type=Path, default=PROJECT_ROOT / "outputs/gocator_demo")
    parser.add_argument("--points-per-profile", type=int, default=3700)
    parser.add_argument("--profile-spacing", type=float, default=0.0005)
    parser.add_argument("--scan-speed", type=float, default=0.08)
    parser.add_argument("--sim-dt", type=float, default=0.01)
    parser.add_argument("--horizontal-overlap", type=float, default=0.0)
    parser.add_argument("--max-profiles", type=int, default=600, help="Runtime guard; set <=0 for full wall scan.")
    parser.add_argument("--csv", action="store_true", help="Also export per-point CSV; can be large.")
    args = parser.parse_args()

    spec = Gocator2690Spec(
        points_per_profile=args.points_per_profile,
        profile_spacing_m=args.profile_spacing,
        nominal_standoff_m=SENSOR_STANDOFF_M,
    )
    profiler = Gocator2690LineProfiler(spec)
    scan_width_m = float(spec.width_at_distance(SENSOR_STANDOFF_M))
    pass_centers = _build_pass_centers(scan_width_m, args.horizontal_overlap)

    def sampler(pose: ScannerFramePose, timestamp_s: float, profile_index: int, encoder_position_m: float):
        profile = profiler.sample_surface(
            pose,
            synthetic_facade_y,
            timestamp_s=timestamp_s,
            profile_index=profile_index,
            encoder_position_m=encoder_position_m,
        )
        profile.valid_mask &= _wall_valid_mask(profile.points_world)
        return profile

    acquisition = GocatorEncoderTriggeredAcquisition(profiler, sampler)
    accumulator = GocatorPointCloudAccumulator(spec.profile_spacing_m)
    profiles = []
    scan_path = []
    t = 0.0
    stop = False

    for pass_index, x_m in enumerate(pass_centers):
        direction = 1.0 if pass_index % 2 == 0 else -1.0
        z_m = 0.0 if direction > 0 else FACADE_HEIGHT_M + 0.001
        target_z_m = FACADE_HEIGHT_M + 0.001 if direction > 0 else 0.0

        while True:
            pose = _scanner_pose(x_m, z_m)
            scan_path.append(pose.origin_m.copy())
            for profile in acquisition.update(pose, t):
                profiles.append(profile)
                accumulator.add_profile(profile)
            if (direction > 0 and z_m >= target_z_m) or (direction < 0 and z_m <= target_z_m):
                break
            if args.max_profiles > 0 and len(profiles) >= args.max_profiles:
                stop = True
                break
            z_m += direction * args.scan_speed * args.sim_dt
            z_m = min(z_m, target_z_m) if direction > 0 else max(z_m, target_z_m)
            t += args.sim_dt
        if stop:
            break

    cloud = accumulator.point_cloud()
    height_map = accumulator.height_map_array()
    out = args.output_dir.resolve()
    out.mkdir(parents=True, exist_ok=True)
    export_profiles_npz(out / "gocator_profiles.npz", profiles)
    if args.csv:
        export_profiles_csv(out / "gocator_profiles.csv", profiles)
    export_point_cloud_ply(out / "gocator_point_cloud.ply", cloud)
    export_height_map_npy(out / "gocator_height_map.npy", height_map)
    _write_debug_usd(out / "gocator_demo_debug.usda", profiles, cloud, np.asarray(scan_path))

    valid_ratio = float(len(cloud) / (len(profiles) * spec.points_per_profile)) if profiles else 0.0
    metrics = {
        "wall_width_m": FACADE_WIDTH_M,
        "wall_height_m": FACADE_HEIGHT_M,
        "scan_width_at_1m_m": scan_width_m,
        "horizontal_step_m": scan_width_m * (1.0 - args.horizontal_overlap),
        "pass_centers_x_m": pass_centers,
        "completed_passes_or_partial": int(len({round(float(p.scanner_pose_world.origin_m[0]), 6) for p in profiles})),
        "profiles": len(profiles),
        "points_per_profile": spec.points_per_profile,
        "valid_points": int(len(cloud)),
        "valid_ratio": valid_ratio,
        "profile_spacing_m": spec.profile_spacing_m,
        "scan_speed_m_s": args.scan_speed,
    }
    with (out / "scan_metrics.json").open("w", encoding="utf-8") as fp:
        json.dump(metrics, fp, indent=2)

    print(f"profiles: {len(profiles)}")
    print(f"points_per_profile: {spec.points_per_profile}")
    print(f"valid_points: {len(cloud)}")
    print(f"valid_ratio: {valid_ratio:.3f}")
    print(f"scan_width_at_1m_m: {scan_width_m:.6f}")
    print(f"passes_planned: {len(pass_centers)}")
    print(f"npz: {out / 'gocator_profiles.npz'}")
    print(f"ply: {out / 'gocator_point_cloud.ply'}")
    print(f"height_map: {out / 'gocator_height_map.npy'}")
    print(f"metrics: {out / 'scan_metrics.json'}")
    print(f"debug_usd: {out / 'gocator_demo_debug.usda'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
