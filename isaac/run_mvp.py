#!/usr/bin/env python3
"""Sensor-integration Isaac Sim scene.

This MVP intentionally removes the grid, rails, carriage, and deployment
mechanisms. The scene focuses on a Gocator-like scanner facing a facade with
visible defects so we can inspect scan data and point clouds next.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--test", action="store_true")
    parser.add_argument("--no-lidar", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--no-ros", action="store_true")
    parser.add_argument("--ros-bridge", action="store_true", help="Publish Isaac Gocator point clouds to ROS 2.")
    parser.add_argument("--ros-frame-id", default="world")
    parser.add_argument("--ros-publish-rate", type=float, default=10.0)
    parser.add_argument("--ros-max-points", type=int, default=500000)
    parser.add_argument("--realtime", action="store_true")
    parser.add_argument("--frames", type=int, default=0)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--scan-speed", type=float, default=0.0, help="Live scanner path speed in m/s. <=0 uses Gocator profile_rate * profile_spacing.")
    # Kept as no-op compatibility flags for older scripts/commands.
    parser.add_argument("--motor-rotate-x", type=float, default=90.0, help=argparse.SUPPRESS)
    parser.add_argument("--motor-rotate-y", type=float, default=90.0, help=argparse.SUPPRESS)
    parser.add_argument("--motor-rotate-z", type=float, default=180.0, help=argparse.SUPPRESS)
    parser.add_argument("--carriage-log", type=Path, help=argparse.SUPPRESS)
    parser.add_argument("--carriage-control", action="store_true", help=argparse.SUPPRESS)
    args, kit_args = parser.parse_known_args()
    sys.argv = [sys.argv[0], *kit_args]
    return args


ARGS = parse_args()

from isaacsim import SimulationApp

simulation_app = SimulationApp(
    {"headless": ARGS.headless or ARGS.test, "renderer": "RayTracedLighting"}
)

import omni.usd
import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import numpy as np
import yaml
from isaacsim.core.simulation_manager import SimulationManager
from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdPhysics, Vt

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "src"))

from gridsim_sensors import (  # noqa: E402
    Gocator2690LineProfiler,
    Gocator2690Spec,
    GocatorEncoderTriggeredAcquisition,
    GocatorPointCloudAccumulator,
    GocatorProfile,
    ScannerFramePose,
)

WORLD_PATH = "/World"
FACADE_PATH = "/World/facade"
GROUND_PATH = "/World/ground"
ROBOT_ROOT_PATH = "/World/Robot"
SENSOR_NAME = "Gocator2690"
GOCATOR2690_METADATA_PATH = PROJECT_ROOT / "assets/cad/sensors/gocator2690/gocator2690.json"

FACADE_WIDTH_M = 10.0
FACADE_HEIGHT_M = 10.0


def _load_sensor_config() -> dict:
    path = PROJECT_ROOT / "configs/sensors.yaml"
    if not path.exists():
        return {}
    with path.open("r", encoding="utf-8") as fp:
        data = yaml.safe_load(fp)
    return data if isinstance(data, dict) else {}


def _load_gocator2690_metadata() -> dict:
    if not GOCATOR2690_METADATA_PATH.exists():
        return {}
    with GOCATOR2690_METADATA_PATH.open("r", encoding="utf-8") as fp:
        return json.load(fp)


_SENSOR_CONFIG = _load_sensor_config()
_GOCATOR2690_METADATA = _load_gocator2690_metadata()


def _gocator2690_metadata_defaults() -> dict:
    return {
        "model": "Gocator 2690",
        "generated_usd": "assets/cad/sensors/gocator2690/gocator2690_visual.usd",
        "datasheet_values": {
            "x_fov_near_mm": 385,
            "x_fov_far_mm": 2000,
            "clearance_distance_mm": 325,
            "z_measurement_range_mm": 1550,
            "nominal_standoff_mm": 1000,
            "points_per_profile": 3700,
            "nominal_profile_rate_hz": 40,
            "profile_spacing_m": 0.0005,
        },
        "housing_collision_envelope_m": {"x": 0.055, "y": 0.105, "z": 0.291},
        **_GOCATOR2690_METADATA,
    }


def _gocator2690_datasheet(config: dict, metadata: dict) -> dict:
    datasheet = {**metadata.get("datasheet_values", {})}
    for attr_name in (
        "x_fov_near_mm",
        "x_fov_far_mm",
        "z_measurement_range_mm",
        "clearance_distance_mm",
        "nominal_standoff_mm",
        "points_per_profile",
        "nominal_profile_rate_hz",
        "profile_spacing_m",
        "wavelength_nm",
        "ip_rating",
    ):
        if attr_name in config:
            datasheet[attr_name] = config[attr_name]
    return datasheet


def _active_gocator2690_datasheet() -> dict:
    return _gocator2690_datasheet(_SENSOR_CONFIG.get("gocator2690", {}), _gocator2690_metadata_defaults())


def _create_box(stage, path: str, size, translation, color):
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    cube.AddTranslateOp().Set(Gf.Vec3d(*translation))
    cube.AddScaleOp().Set(Gf.Vec3f(*size))
    cube.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    return cube


def _add_collision(prim) -> None:
    UsdPhysics.CollisionAPI.Apply(prim)


def _make_rigid(prim, mass_kg=None, *, kinematic=False) -> None:
    rb = UsdPhysics.RigidBodyAPI.Apply(prim)
    rb.CreateKinematicEnabledAttr(kinematic)
    if mass_kg is not None:
        UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(float(mass_kg))


def _add_custom_attr(prim, name: str, value) -> None:
    if isinstance(value, bool):
        attr_type = Sdf.ValueTypeNames.Bool
    elif isinstance(value, int):
        attr_type = Sdf.ValueTypeNames.Int
    elif isinstance(value, float):
        attr_type = Sdf.ValueTypeNames.Double
    else:
        attr_type = Sdf.ValueTypeNames.String
        value = str(value)
    prim.CreateAttribute(name, attr_type, custom=True).Set(value)


def _vec3_from_config(values, default: tuple[float, float, float]) -> tuple[float, float, float]:
    if not isinstance(values, (list, tuple)) or len(values) != 3:
        return default
    return (float(values[0]), float(values[1]), float(values[2]))


def _project_path(path_value: str | None, default: Path) -> Path:
    if not path_value:
        return default
    path = Path(path_value)
    if path.is_absolute():
        return path
    return PROJECT_ROOT / path


def _add_xyz_rotation_ops(xform: UsdGeom.Xform, rotation_deg: tuple[float, float, float]) -> None:
    rx, ry, rz = rotation_deg
    xform.AddRotateXOp().Set(float(rx))
    xform.AddRotateYOp().Set(float(ry))
    xform.AddRotateZOp().Set(float(rz))


def _gocator_standoff_m(datasheet: dict) -> float:
    return float(datasheet.get("nominal_standoff_mm", 1000)) * 0.001


def _gocator_scan_speed_m_s(datasheet: dict) -> float:
    profile_rate_hz = float(datasheet.get("nominal_profile_rate_hz", 40))
    profile_spacing_m = float(datasheet.get("profile_spacing_m", 0.0005))
    return profile_rate_hz * profile_spacing_m


def _gocator_profile_width_m(datasheet: dict, distance_m: float | None = None) -> float:
    distance_m = _gocator_standoff_m(datasheet) if distance_m is None else distance_m
    clearance_m = float(datasheet.get("clearance_distance_mm", 325)) * 0.001
    range_m = float(datasheet.get("z_measurement_range_mm", 1550)) * 0.001
    near_fov_m = float(datasheet.get("x_fov_near_mm", 385)) * 0.001
    far_fov_m = float(datasheet.get("x_fov_far_mm", 2000)) * 0.001
    alpha = (distance_m - clearance_m) / range_m
    return near_fov_m + alpha * (far_fov_m - near_fov_m)


def _scan_pass_centers(datasheet: dict) -> list[float]:
    profile_width_m = _gocator_profile_width_m(datasheet)
    first = -FACADE_WIDTH_M / 2.0 + profile_width_m / 2.0
    last = FACADE_WIDTH_M / 2.0 - profile_width_m / 2.0
    centers = []
    x = first
    while x <= last + 1e-9:
        centers.append(float(x))
        x += profile_width_m
    if not centers or centers[-1] < last - 1e-6:
        centers.append(float(last))
    return centers


def _scan_path_points(datasheet: dict) -> list[Gf.Vec3d]:
    points: list[Gf.Vec3d] = []
    top_z = FACADE_HEIGHT_M + 0.001
    centers = _scan_pass_centers(datasheet)
    standoff_m = _gocator_standoff_m(datasheet)
    for index, x_m in enumerate(centers):
        z_start = 0.0 if index % 2 == 0 else top_z
        z_end = top_z if index % 2 == 0 else 0.0
        start = Gf.Vec3d(x_m, -standoff_m, z_start)
        end = Gf.Vec3d(x_m, -standoff_m, z_end)
        if not points:
            points.append(start)
        elif points[-1] != start:
            points.append(start)
        points.append(end)
        if index + 1 < len(centers):
            points.append(Gf.Vec3d(centers[index + 1], -standoff_m, z_end))
    return points


def _scan_pose_at_time(datasheet: dict, elapsed_s: float, speed_m_s: float) -> Gf.Vec3d:
    path = _scan_path_points(datasheet)
    if speed_m_s <= 0.0:
        speed_m_s = _gocator_scan_speed_m_s(datasheet)
    if len(path) < 2 or speed_m_s <= 0.0:
        return path[0]
    segment_lengths = []
    for a, b in zip(path[:-1], path[1:]):
        segment_lengths.append((b - a).GetLength())
    total_length = sum(segment_lengths)
    if total_length <= 0.0:
        return path[0]
    distance = (elapsed_s * speed_m_s) % total_length
    for a, b, length in zip(path[:-1], path[1:], segment_lengths):
        if distance <= length or length <= 0.0:
            alpha = 0.0 if length <= 0.0 else distance / length
            return a + (b - a) * alpha
        distance -= length
    return path[-1]


def _facade_y_offset(x_m, z_m):
    # Sensor sits at negative Y and looks toward +Y, so visible protrusions
    # toward the sensor are negative Y. Craters/recesses are positive Y.
    bow = -0.045 * (1.0 - (x_m / 5.0) ** 2) * (1.0 - ((z_m - 5.0) / 5.0) ** 2)
    waves = (
        -0.006 * math.sin(1.5 * x_m + 0.4) * math.sin(1.2 * z_m)
        -0.0025 * math.sin(8.5 * x_m + 1.7) * math.sin(7.0 * z_m)
    )
    dents = [
        (-2.2, 2.8, 0.35, 0.42, 0.045),
        (1.9, 6.8, 0.55, 0.36, 0.036),
        (3.6, 3.2, 0.42, 0.46, 0.030),
        (-3.5, 7.4, 0.45, 0.34, 0.040),
        (0.1, 4.8, 0.22, 0.20, 0.020),
        (-4.4, 5.9, 0.18, 0.25, 0.018),
    ]
    bumps = [
        (-1.1, 5.7, 0.48, 0.38, -0.030),
        (2.8, 1.6, 0.35, 0.36, -0.026),
        (-4.0, 4.2, 0.30, 0.48, -0.020),
        (0.4, 8.4, 0.55, 0.32, -0.028),
        (4.1, 8.0, 0.26, 0.22, -0.016),
    ]
    fine_pits = [
        (-0.9, 1.5, 0.055, 0.060, 0.006),
        (-0.1, 2.9, 0.040, 0.055, 0.005),
        (1.4, 4.2, 0.050, 0.045, 0.006),
        (2.7, 5.4, 0.065, 0.055, 0.007),
        (-3.9, 6.1, 0.045, 0.070, 0.006),
        (-2.6, 8.7, 0.055, 0.055, 0.005),
        (3.8, 7.2, 0.050, 0.050, 0.006),
        (4.5, 3.9, 0.040, 0.045, 0.004),
    ]
    fine_bumps = [
        (-1.8, 1.9, 0.050, 0.050, -0.004),
        (0.7, 3.7, 0.060, 0.045, -0.005),
        (2.2, 8.8, 0.045, 0.060, -0.004),
        (-4.4, 2.9, 0.055, 0.040, -0.004),
    ]
    defects = 0.0
    for cx, cz, sx, sz, amp in dents + bumps + fine_pits + fine_bumps:
        defects += amp * math.exp(-(((x_m - cx) / sx) ** 2 + ((z_m - cz) / sz) ** 2))
    joints = 0.0
    for joint_x, phase in ((-3.2, 0.0), (-1.1, 0.8), (1.2, 1.9), (3.4, 2.7)):
        center = joint_x + 0.035 * math.sin(1.7 * z_m + phase) + 0.012 * math.sin(6.5 * z_m + phase)
        width = 0.018 + 0.010 * (0.5 + 0.5 * math.sin(3.1 * z_m + phase))
        if abs(x_m - center) < width:
            joints += 0.009
    for joint_z, phase in ((2.2, 0.4), (5.0, 1.5), (7.8, 2.1)):
        center = joint_z + 0.030 * math.sin(1.4 * x_m + phase) + 0.010 * math.sin(5.5 * x_m)
        width = 0.016 + 0.008 * (0.5 + 0.5 * math.sin(2.5 * x_m + phase))
        if abs(z_m - center) < width:
            joints += 0.006
    patches = 0.0
    for x0, x1, z0, z1, offset in (
        (-4.4, -3.5, 0.9, 1.7, 0.006),
        (-0.7, 0.1, 3.2, 4.1, 0.005),
        (2.3, 3.5, 7.0, 8.0, 0.007),
        (-2.8, -1.7, 8.2, 9.1, 0.005),
    ):
        if x0 < x_m < x1 and z0 < z_m < z1:
            patches += offset
    return bow + waves + defects + joints + patches


def _facade_y_offset_array(x_m: np.ndarray, z_m: np.ndarray) -> np.ndarray:
    vectorized = np.vectorize(_facade_y_offset, otypes=[np.float64])
    return vectorized(x_m, z_m)


def _wall_valid_mask(points_world: np.ndarray) -> np.ndarray:
    return (
        (points_world[:, 0] >= -FACADE_WIDTH_M / 2.0)
        & (points_world[:, 0] <= FACADE_WIDTH_M / 2.0)
        & (points_world[:, 2] >= 0.0)
        & (points_world[:, 2] <= FACADE_HEIGHT_M)
    )


def _surface_front_y(x_m: float, z_m: float, depth_m: float, clearance_m: float = 0.006) -> float:
    """Place a flat visual marker just in front of the local facade surface."""
    return _facade_y_offset(x_m, z_m) - depth_m / 2.0 - clearance_m


def _create_defect_facade(stage) -> None:
    facade = UsdGeom.Xform.Define(stage, FACADE_PATH)
    _make_rigid(facade.GetPrim(), 1.0e9, kinematic=True)

    resolution = 180
    xs = [
        -FACADE_WIDTH_M / 2.0 + FACADE_WIDTH_M * i / (resolution - 1)
        for i in range(resolution)
    ]
    zs = [FACADE_HEIGHT_M * i / (resolution - 1) for i in range(resolution)]
    points = []
    for z in zs:
        for x in xs:
            points.append(Gf.Vec3f(x, _facade_y_offset(x, z), z))

    counts = []
    indices = []
    for row in range(resolution - 1):
        for col in range(resolution - 1):
            i = row * resolution + col
            counts.append(4)
            indices.extend([i, i + 1, i + resolution + 1, i + resolution])

    surface = UsdGeom.Mesh.Define(stage, f"{FACADE_PATH}/surface")
    surface.CreatePointsAttr(Vt.Vec3fArray(points))
    surface.CreateFaceVertexCountsAttr(Vt.IntArray(counts))
    surface.CreateFaceVertexIndicesAttr(Vt.IntArray(indices))
    surface.CreateSubdivisionSchemeAttr("none")
    surface.CreateDisplayColorAttr(Vt.Vec3fArray([Gf.Vec3f(0.56, 0.58, 0.60)]))
    surface.CreateDoubleSidedAttr(True)
    _add_collision(surface.GetPrim())

    # Visible windows: recessed dark panels with light frames.
    window_specs = [
        ("window_a", (-3.7, 2.0), (0.80, 0.014, 0.95)),
        ("window_b", (0.9, 2.7), (0.95, 0.014, 1.20)),
        ("window_c", (3.3, 6.1), (0.75, 0.014, 1.00)),
        ("window_d", (-1.9, 7.2), (1.10, 0.014, 1.25)),
    ]
    for name, xz, size in window_specs:
        x, z = xz
        sx, sy, sz = size
        y = _surface_front_y(x, z, sy, clearance_m=0.020)
        window = _create_box(stage, f"{FACADE_PATH}/{name}_glass", size, (x, y, z), (0.04, 0.08, 0.12))
        _add_collision(window.GetPrim())
        for suffix, frame_size, frame_xz in (
            ("top", (sx + 0.12, 0.018, 0.04), (x, z + sz / 2.0 + 0.045)),
            ("bottom", (sx + 0.12, 0.018, 0.04), (x, z - sz / 2.0 - 0.045)),
            ("left", (0.04, 0.018, sz + 0.12), (x - sx / 2.0 - 0.045, z)),
            ("right", (0.04, 0.018, sz + 0.12), (x + sx / 2.0 + 0.045, z)),
        ):
            fx, fz = frame_xz
            fy = _surface_front_y(fx, fz, frame_size[1], clearance_m=0.004)
            _create_box(stage, f"{FACADE_PATH}/{name}_{suffix}", frame_size, (fx, fy, fz), (0.88, 0.86, 0.80))

    # High-contrast markers for defects/repair patches.
    for idx, size, pos, color in (
        (0, (0.90, 0.014, 0.75), (-3.95, 1.30), (0.70, 0.68, 0.60)),
        (1, (0.80, 0.014, 0.90), (-0.30, 3.65), (0.78, 0.74, 0.65)),
        (2, (1.20, 0.014, 0.95), (2.90, 7.50), (0.65, 0.62, 0.56)),
        (3, (1.10, 0.014, 0.70), (-2.25, 8.65), (0.46, 0.45, 0.44)),
    ):
        x, z = pos
        y = _surface_front_y(x, z, size[1], clearance_m=0.004)
        _create_box(stage, f"{FACADE_PATH}/repair_patch_{idx}", size, (x, y, z), color)

    for j, (base_x, phase) in enumerate(((-3.2, 0.0), (-1.1, 0.8), (1.2, 1.9), (3.4, 2.7))):
        for seg in range(14):
            z = 0.45 + seg * 0.68
            x = base_x + 0.035 * math.sin(1.7 * z + phase) + 0.012 * math.sin(6.5 * z + phase)
            height = 0.42 + 0.10 * math.sin(2.3 * seg + phase)
            width = 0.018 + 0.006 * ((seg + j) % 3)
            _create_box(
                stage,
                f"{FACADE_PATH}/vertical_joint_marker_{j}_{seg}",
                (width, 0.016, height),
                (x, _surface_front_y(x, z, 0.016, clearance_m=0.003), z),
                (0.28, 0.28, 0.30),
            )
    for j, (base_z, phase) in enumerate(((2.2, 0.4), (5.0, 1.5), (7.8, 2.1))):
        for seg in range(13):
            x = -4.5 + seg * 0.75
            z = base_z + 0.030 * math.sin(1.4 * x + phase) + 0.010 * math.sin(5.5 * x)
            length = 0.45 + 0.15 * math.sin(1.7 * seg + phase)
            height = 0.018 + 0.006 * ((seg + j) % 2)
            _create_box(
                stage,
                f"{FACADE_PATH}/horizontal_joint_marker_{j}_{seg}",
                (length, 0.016, height),
                (x, _surface_front_y(x, z, 0.016, clearance_m=0.003), z),
                (0.34, 0.34, 0.36),
            )

    # Small visible chips/cracks.
    for idx, x, z, sx, sz in (
        (0, -2.2, 2.8, 0.18, 0.07),
        (1, 1.9, 6.8, 0.24, 0.08),
        (2, 3.6, 3.2, 0.16, 0.06),
        (3, -3.5, 7.4, 0.22, 0.06),
        (4, 0.4, 8.4, 0.28, 0.06),
    ):
        _create_box(
            stage,
            f"{FACADE_PATH}/chip_marker_{idx}",
            (sx, 0.016, sz),
            (x, _surface_front_y(x, z, 0.016, clearance_m=0.003), z),
            (0.18, 0.18, 0.19),
        )

    # Fine pitting markers visible in close inspection.
    for idx, x, z in (
        (0, -0.9, 1.5),
        (1, -0.1, 2.9),
        (2, 1.4, 4.2),
        (3, 2.7, 5.4),
        (4, -3.9, 6.1),
        (5, -2.6, 8.7),
        (6, 3.8, 7.2),
        (7, 4.5, 3.9),
    ):
        _create_box(
            stage,
            f"{FACADE_PATH}/fine_pit_marker_{idx}",
            (0.055, 0.012, 0.055),
            (x, _surface_front_y(x, z, 0.012, clearance_m=0.002), z),
            (0.12, 0.12, 0.13),
        )


def _add_scanner_frame_axes(stage, frame_path: str) -> None:
    axis_specs = [
        ("x_axis", (0.10, 0.004, 0.004), (0.05, 0.0, 0.0), (1.0, 0.05, 0.05)),
        ("y_axis", (0.004, 0.10, 0.004), (0.0, 0.05, 0.0), (0.05, 1.0, 0.05)),
        ("z_axis", (0.004, 0.004, 0.10), (0.0, 0.0, 0.05), (0.05, 0.2, 1.0)),
    ]
    for name, size, translation, color in axis_specs:
        _create_box(stage, f"{frame_path}/{name}", size, translation, color)


def _add_profile_scan_volume(stage, sensor_path: str, datasheet: dict) -> None:
    cd_m = float(datasheet.get("clearance_distance_mm", 325)) * 0.001
    mr_m = float(datasheet.get("z_measurement_range_mm", 1550)) * 0.001
    near_fov_m = float(datasheet.get("x_fov_near_mm", 385)) * 0.001
    far_fov_m = float(datasheet.get("x_fov_far_mm", 2000)) * 0.001

    scan = UsdGeom.Xform.Define(stage, f"{sensor_path}/scan_volume")
    scan_prim = scan.GetPrim()
    _add_custom_attr(scan_prim, "clearance_distance_mm", int(round(cd_m * 1000.0)))
    _add_custom_attr(scan_prim, "measurement_range_mm", int(round(mr_m * 1000.0)))
    _add_custom_attr(scan_prim, "x_fov_near_mm", int(round(near_fov_m * 1000.0)))
    _add_custom_attr(scan_prim, "x_fov_far_mm", int(round(far_fov_m * 1000.0)))
    _add_custom_attr(scan_prim, "visible_volume", False)
    _add_custom_attr(scan_prim, "debug_note", "Hidden by default; red laser_contact_line shows the facade intersection.")


def _laser_contact_points(datasheet: dict, center_x_m: float, center_z_m: float) -> Vt.Vec3fArray:
    profile_width_m = _gocator_profile_width_m(datasheet)
    x_min = max(-FACADE_WIDTH_M / 2.0, center_x_m - profile_width_m / 2.0)
    x_max = min(FACADE_WIDTH_M / 2.0, center_x_m + profile_width_m / 2.0)
    sample_count = 160
    points = []
    for index in range(sample_count):
        alpha = index / (sample_count - 1)
        x = x_min + (x_max - x_min) * alpha
        z = min(max(center_z_m, 0.0), FACADE_HEIGHT_M)
        y = _surface_front_y(x, z, 0.012, clearance_m=0.0015)
        points.append(Gf.Vec3f(x, y, z))
    return Vt.Vec3fArray(points)


def _add_laser_contact_line(stage, datasheet: dict, center_x_m: float, center_z_m: float) -> None:
    curve = UsdGeom.BasisCurves.Define(stage, f"{FACADE_PATH}/laser_contact_line")
    curve.CreateTypeAttr("linear")
    points = _laser_contact_points(datasheet, center_x_m, center_z_m)
    curve.CreateCurveVertexCountsAttr(Vt.IntArray([len(points)]))
    curve.CreatePointsAttr(points)
    curve.CreateWidthsAttr(Vt.FloatArray([0.018] * len(points)))
    curve.CreateDisplayColorAttr(Vt.Vec3fArray([Gf.Vec3f(1.0, 0.02, 0.01)]))


def _add_scan_path_preview(stage, datasheet: dict) -> None:
    path = _scan_path_points(datasheet)
    curve = UsdGeom.BasisCurves.Define(stage, "/World/scanner_path_preview")
    curve.CreateTypeAttr("linear")
    curve.CreateCurveVertexCountsAttr(Vt.IntArray([len(path)]))
    curve.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(point[0], point[1], point[2]) for point in path]))
    curve.CreateWidthsAttr(Vt.FloatArray([0.012] * len(path)))
    curve.CreateDisplayColorAttr(Vt.Vec3fArray([Gf.Vec3f(1.0, 0.82, 0.05)]))


class IsaacGocatorRosBridge:
    """Publish Gocator data generated from the Isaac scanner state to ROS 2."""

    def __init__(self, datasheet: dict) -> None:
        import rclpy
        from sensor_msgs.msg import PointCloud2, PointField
        from std_msgs.msg import Header

        self.rclpy = rclpy
        self.PointCloud2 = PointCloud2
        self.PointField = PointField
        self.Header = Header
        if not rclpy.ok():
            rclpy.init(args=None)
        self.node = rclpy.create_node("isaac_gocator_bridge")
        self.profile_pub = self.node.create_publisher(PointCloud2, "gocator/profile_points", 2)
        self.cloud_pub = self.node.create_publisher(PointCloud2, "gocator/points", 2)
        self.frame_id = ARGS.ros_frame_id
        self.publish_period_s = 1.0 / max(ARGS.ros_publish_rate, 1e-6)
        self.last_publish_s = -1.0
        self.max_points = ARGS.ros_max_points

        spec = Gocator2690Spec(
            points_per_profile=int(datasheet.get("points_per_profile", 3700)),
            profile_spacing_m=float(datasheet.get("profile_spacing_m", 0.0005)),
            nominal_standoff_m=_gocator_standoff_m(datasheet),
            nominal_profile_rate_hz=float(datasheet.get("nominal_profile_rate_hz", 40.0)),
        )
        self.profiler = Gocator2690LineProfiler(spec)
        self.acquisition = GocatorEncoderTriggeredAcquisition(self.profiler, self._sample_profile)
        self.accumulator = GocatorPointCloudAccumulator(spec.profile_spacing_m)
        self.node.get_logger().info(
            f"Isaac ROS bridge publishing /gocator/profile_points and /gocator/points "
            f"({spec.points_per_profile} pts/profile, {spec.nominal_profile_rate_hz:.1f} Hz, "
            f"spacing={spec.profile_spacing_m:.6f} m)"
        )

    def update(self, scanner_pose: Gf.Vec3d, timestamp_s: float) -> None:
        pose = ScannerFramePose.from_arrays(
            [scanner_pose[0], scanner_pose[1], scanner_pose[2]],
            [1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0],
            [0.0, 1.0, 0.0],
        )
        stamp = self.node.get_clock().now().to_msg()
        for profile in self.acquisition.update(pose, timestamp_s):
            self.accumulator.add_profile(profile)
            self.profile_pub.publish(self._pointcloud2(profile.valid_points_m, stamp))

        if self.last_publish_s < 0.0 or timestamp_s - self.last_publish_s >= self.publish_period_s:
            self.last_publish_s = timestamp_s
            self.cloud_pub.publish(self._pointcloud2(self._preview_cloud(), stamp))
        self.rclpy.spin_once(self.node, timeout_sec=0.0)

    def shutdown(self) -> None:
        self.node.destroy_node()
        if self.rclpy.ok():
            self.rclpy.shutdown()

    def _sample_profile(
        self,
        pose: ScannerFramePose,
        timestamp_s: float,
        profile_index: int,
        encoder_position_m: float,
    ):
        profile = self.profiler.sample_surface(
            pose,
            _facade_y_offset_array,
            timestamp_s=timestamp_s,
            profile_index=profile_index,
            encoder_position_m=encoder_position_m,
        )
        profile.valid_mask &= _wall_valid_mask(profile.points_world)
        return profile

    def _preview_cloud(self) -> np.ndarray:
        profiles = self.accumulator.profiles
        if not profiles:
            return np.empty((0, 3), dtype=np.float64)
        total_valid = sum(int(profile.valid_mask.sum()) for profile in profiles)
        if self.max_points <= 0 or total_valid <= self.max_points:
            return self.accumulator.point_cloud()
        per_profile_budget = max(2, self.max_points // len(profiles))
        sampled_profiles = [
            _sample_profile_points(profile, per_profile_budget)
            for profile in profiles
        ]
        sampled_profiles = [points for points in sampled_profiles if len(points)]
        if not sampled_profiles:
            return np.empty((0, 3), dtype=np.float64)
        return np.vstack(sampled_profiles)

    def _pointcloud2(self, points: np.ndarray, stamp) -> object:
        points32 = np.asarray(points, dtype=np.float32)
        message = self.PointCloud2()
        message.header = self.Header(stamp=stamp, frame_id=self.frame_id)
        message.height = 1
        message.width = int(len(points32))
        message.fields = [
            self.PointField(name="x", offset=0, datatype=self.PointField.FLOAT32, count=1),
            self.PointField(name="y", offset=4, datatype=self.PointField.FLOAT32, count=1),
            self.PointField(name="z", offset=8, datatype=self.PointField.FLOAT32, count=1),
        ]
        message.is_bigendian = False
        message.point_step = 12
        message.row_step = message.point_step * message.width
        message.is_dense = True
        message.data = points32.tobytes()
        return message


def _sample_profile_points(profile: GocatorProfile, max_points: int) -> np.ndarray:
    points = profile.valid_points_m
    if len(points) <= max_points:
        return points
    indices = np.linspace(0, len(points) - 1, max_points, dtype=np.int64)
    return points[indices]


def _create_gocator_sensor(stage) -> str | None:
    config = _SENSOR_CONFIG.get("gocator2690", {})
    metadata = _gocator2690_metadata_defaults()
    datasheet = _active_gocator2690_datasheet()

    asset_path = _project_path(config.get("usd_asset_path"), PROJECT_ROOT / metadata["generated_usd"])
    mount_path = f"{ROBOT_ROOT_PATH}/scanner_mount_link"
    sensor_path = f"{mount_path}/{SENSOR_NAME}"
    standoff_m = _gocator_standoff_m(datasheet)
    profile_width_m = _gocator_profile_width_m(datasheet, standoff_m)
    start_x_m = -FACADE_WIDTH_M / 2.0 + profile_width_m / 2.0
    start_z_m = 0.0

    robot = UsdGeom.Xform.Define(stage, ROBOT_ROOT_PATH)
    robot.AddTranslateOp().Set(Gf.Vec3d(start_x_m, -standoff_m, start_z_m))
    mount = UsdGeom.Xform.Define(stage, mount_path)
    mount.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.0))

    sensor = UsdGeom.Xform.Define(stage, sensor_path)
    # Optical frame: X profile width, Y scanner travel, Z measurement toward facade.
    sensor.AddOrientOp().Set(Gf.Quatf(0.0, Gf.Vec3f(0.0, 0.70710678, 0.70710678)))
    sensor_prim = sensor.GetPrim()
    _add_custom_attr(sensor_prim, "model", metadata["model"])
    for attr_name, value in datasheet.items():
        _add_custom_attr(sensor_prim, attr_name, value)

    sensor_pose = config.get("sensor_pose_on_mount", {})
    sensor_rpy = _vec3_from_config(sensor_pose.get("rpy_deg", [-90.0, 0.0, 180.0]), (-90.0, 0.0, 180.0))
    visual = UsdGeom.Xform.Define(stage, f"{sensor_path}/visual")
    _add_xyz_rotation_ops(visual, sensor_rpy)
    visual_asset = UsdGeom.Xform.Define(stage, f"{sensor_path}/visual/asset")
    if asset_path.exists():
        visual_asset.GetPrim().GetReferences().AddReference(str(asset_path))
    else:
        _create_box(stage, f"{sensor_path}/visual/fallback_body", (0.055, 0.105, 0.291), (0.0, 0.0, 0.0), (0.05, 0.07, 0.08))
        print(f"[gocator2690] visual asset missing, using fallback: {asset_path}", flush=True)

    envelope = metadata.get("housing_collision_envelope_m", {})
    collision = UsdGeom.Xform.Define(stage, f"{sensor_path}/collision")
    _add_xyz_rotation_ops(collision, sensor_rpy)
    body = _create_box(
        stage,
        f"{sensor_path}/collision/body",
        (
            float(envelope.get("x", 0.055)),
            float(envelope.get("y", 0.105)),
            float(envelope.get("z", 0.291)),
        ),
        (0.0, 0.0, 0.0),
        (0.8, 0.25, 0.05),
    )
    _add_collision(body.GetPrim())
    UsdGeom.Imageable(collision.GetPrim()).MakeInvisible()

    scanner_frame = UsdGeom.Xform.Define(stage, f"{sensor_path}/scanner_frame")
    scanner_frame.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.0))
    _add_scanner_frame_axes(stage, f"{sensor_path}/scanner_frame")
    _add_profile_scan_volume(stage, sensor_path, datasheet)
    _add_laser_contact_line(stage, datasheet, start_x_m, start_z_m)

    print(
        f"[gocator2690] mounted at bottom-left scan start x={start_x_m:.3f} m, z={start_z_m:.3f} m; "
        f"profile_width_at_standoff={profile_width_m:.3f} m, scan_speed={_gocator_scan_speed_m_s(datasheet):.4f} m/s",
        flush=True,
    )
    return f"{sensor_path}/scanner_frame"


def build_scene():
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

    ground = _create_box(stage, GROUND_PATH, (12.0, 6.0, 0.05), (0.0, -1.0, -0.025), (0.22, 0.24, 0.25))
    _add_collision(ground.GetPrim())

    _create_defect_facade(stage)
    _create_gocator_sensor(stage)
    _add_scan_path_preview(stage, _active_gocator2690_datasheet())
    return stage


def main() -> int:
    stage = build_scene()
    datasheet = _active_gocator2690_datasheet()

    out = (ARGS.output or PROJECT_ROOT / "outputs/isaac/sensor_integration_scene.usda").resolve()
    out.parent.mkdir(parents=True, exist_ok=True)
    stage.GetRootLayer().Export(str(out))
    print(f"Saved scene: {out}", flush=True)

    SimulationManager.setup_simulation(dt=1.0 / 60.0, device="cpu")
    app_utils.play()
    simulation_app.update()

    frame_limit = ARGS.frames
    if ARGS.test and frame_limit <= 0:
        frame_limit = 10

    robot_translate_attr = stage.GetPrimAtPath(ROBOT_ROOT_PATH).GetAttribute("xformOp:translate")
    laser_points_attr = stage.GetPrimAtPath(f"{FACADE_PATH}/laser_contact_line").GetAttribute("points")
    ros_bridge = None
    if ARGS.ros_bridge and not ARGS.no_ros:
        try:
            ros_bridge = IsaacGocatorRosBridge(datasheet)
        except Exception as exc:
            print(f"[ros_bridge] disabled: {exc}", flush=True)

    sim_dt = 1.0 / 60.0
    frame_count = 0
    try:
        while simulation_app.is_running():
            elapsed_s = frame_count * sim_dt
            scanner_pose = _scan_pose_at_time(datasheet, elapsed_s, ARGS.scan_speed)
            robot_translate_attr.Set(scanner_pose)
            laser_points_attr.Set(_laser_contact_points(datasheet, scanner_pose[0], scanner_pose[2]))
            if ros_bridge is not None:
                ros_bridge.update(scanner_pose, elapsed_s)
            simulation_app.update()
            frame_count += 1
            if frame_limit > 0 and frame_count >= frame_limit:
                print(f"Completed {frame_count} simulation frames", flush=True)
                break
    finally:
        if ros_bridge is not None:
            ros_bridge.shutdown()
        simulation_app.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
