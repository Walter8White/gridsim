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
from typing import Callable


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
    parser.add_argument("--ros-profile-stride", type=int, default=1, help="Add one profile to the RViz display accumulator every N generated profiles.")
    parser.add_argument("--ros-profile-point-stride", type=int, default=1, help="Publish one point every N points when sending the current profile to RViz.")
    parser.add_argument(
        "--raycast-mode",
        choices=["analytic", "mesh"],
        default="analytic",
        help="analytic: fast height-field bisection (default). mesh: real ray-triangle "
        "raycast against the built facade mesh, for validation (requires trimesh).",
    )
    parser.add_argument("--record-full-res", action="store_true", help="Export full-resolution Gocator profiles on shutdown.")
    parser.add_argument("--record-csv", action="store_true", help="Also export full-resolution profiles as CSV. This can be large.")
    parser.add_argument("--record-output-dir", type=Path, default=Path("outputs/gocator_scan"))
    parser.add_argument("--stop-after-scan", action="store_true", help="Exit once one complete facade scan path has been traversed.")
    parser.add_argument("--realtime", action="store_true")
    parser.add_argument("--sim-dt", type=float, default=1.0 / 60.0, help="Isaac simulation step in seconds. Larger values are useful for headless acquisition.")
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
    GocatorProfile,
    ScannerFramePose,
    export_height_map_npy,
    export_point_cloud_ply,
)

WORLD_PATH = "/World"
FACADE_PATH = "/World/facade"
GROUND_PATH = "/World/ground"
ROBOT_ROOT_PATH = "/World/Robot"
SENSOR_NAME = "Gocator2690"
GOCATOR2690_METADATA_PATH = PROJECT_ROOT / "assets/cad/sensors/gocator2690/gocator2690.json"

FACADE_WIDTH_M = 3.0
FACADE_HEIGHT_M = 3.0


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
            "nominal_profile_rate_hz": 2000,
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
    profile_rate_hz = float(datasheet.get("nominal_profile_rate_hz", 2000))
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


def _scan_path_length_m(datasheet: dict) -> float:
    path = _scan_path_points(datasheet)
    return float(sum((b - a).GetLength() for a, b in zip(path[:-1], path[1:])))


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
    half_width = FACADE_WIDTH_M * 0.5
    half_height = FACADE_HEIGHT_M * 0.5
    z_center = FACADE_HEIGHT_M * 0.5
    x_norm = x_m / max(half_width, 1e-6)
    z_norm = (z_m - z_center) / max(half_height, 1e-6)
    bow = -0.020 * (1.0 - x_norm**2) * (1.0 - z_norm**2)
    waves = (
        -0.004 * math.sin(3.0 * x_m + 0.4) * math.sin(2.6 * z_m)
        -0.0020 * math.sin(12.0 * x_m + 1.7) * math.sin(10.0 * z_m)
    )
    dents = [
        (-0.72, 0.78, 0.18, 0.20, 0.030),
        (0.65, 2.15, 0.26, 0.18, 0.025),
        (1.05, 1.15, 0.16, 0.20, 0.018),
        (-1.10, 2.42, 0.18, 0.16, 0.024),
        (0.05, 1.55, 0.12, 0.10, 0.014),
    ]
    bumps = [
        (-0.35, 1.85, 0.22, 0.20, -0.024),
        (0.90, 0.52, 0.18, 0.18, -0.020),
        (-1.22, 1.35, 0.14, 0.22, -0.016),
        (0.20, 2.58, 0.24, 0.16, -0.020),
    ]
    fine_pits = [
        (-0.55, 0.48, 0.045, 0.050, 0.006),
        (-0.08, 0.92, 0.040, 0.045, 0.005),
        (0.42, 1.35, 0.045, 0.045, 0.006),
        (0.80, 1.78, 0.050, 0.050, 0.006),
        (-1.18, 1.95, 0.045, 0.055, 0.005),
        (-0.78, 2.60, 0.050, 0.050, 0.005),
        (1.12, 2.35, 0.045, 0.045, 0.006),
    ]
    fine_bumps = [
        (-0.95, 0.65, 0.045, 0.045, -0.004),
        (0.22, 1.20, 0.055, 0.045, -0.005),
        (0.70, 2.70, 0.045, 0.055, -0.004),
        (-1.25, 0.95, 0.050, 0.040, -0.004),
    ]
    defects = 0.0
    for cx, cz, sx, sz, amp in dents + bumps + fine_pits + fine_bumps:
        defects += amp * math.exp(-(((x_m - cx) / sx) ** 2 + ((z_m - cz) / sz) ** 2))
    joints = 0.0
    for joint_x, phase in ((-0.95, 0.0), (0.15, 0.8), (1.05, 1.9)):
        center = joint_x + 0.020 * math.sin(3.0 * z_m + phase) + 0.008 * math.sin(10.0 * z_m + phase)
        width = 0.012 + 0.006 * (0.5 + 0.5 * math.sin(4.5 * z_m + phase))
        if abs(x_m - center) < width:
            joints += 0.006
    for joint_z, phase in ((0.75, 0.4), (1.55, 1.5), (2.35, 2.1)):
        center = joint_z + 0.020 * math.sin(2.8 * x_m + phase) + 0.008 * math.sin(8.0 * x_m)
        width = 0.012 + 0.006 * (0.5 + 0.5 * math.sin(4.0 * x_m + phase))
        if abs(z_m - center) < width:
            joints += 0.005
    patches = 0.0
    for x0, x1, z0, z1, offset in (
        (-1.35, -0.95, 0.35, 0.75, 0.005),
        (-0.25, 0.15, 1.00, 1.45, 0.004),
        (0.65, 1.20, 2.10, 2.58, 0.006),
    ):
        if x0 < x_m < x1 and z0 < z_m < z1:
            patches += offset
    return bow + waves + defects + joints + patches


def _facade_y_offset_vectorized(x_m: np.ndarray, z_m: np.ndarray) -> np.ndarray:
    """Fully numpy-native implementation of _facade_y_offset for batch use."""
    x = np.asarray(x_m, dtype=np.float64)
    z = np.asarray(z_m, dtype=np.float64)
    half_width = FACADE_WIDTH_M * 0.5
    half_height = FACADE_HEIGHT_M * 0.5
    z_center = FACADE_HEIGHT_M * 0.5
    x_norm = x / max(half_width, 1e-6)
    z_norm = (z - z_center) / max(half_height, 1e-6)
    bow = -0.020 * (1.0 - x_norm ** 2) * (1.0 - z_norm ** 2)
    waves = (
        -0.004 * np.sin(3.0 * x + 0.4) * np.sin(2.6 * z)
        - 0.0020 * np.sin(12.0 * x + 1.7) * np.sin(10.0 * z)
    )
    dents = [
        (-0.72, 0.78, 0.18, 0.20, 0.030),
        (0.65, 2.15, 0.26, 0.18, 0.025),
        (1.05, 1.15, 0.16, 0.20, 0.018),
        (-1.10, 2.42, 0.18, 0.16, 0.024),
        (0.05, 1.55, 0.12, 0.10, 0.014),
    ]
    bumps = [
        (-0.35, 1.85, 0.22, 0.20, -0.024),
        (0.90, 0.52, 0.18, 0.18, -0.020),
        (-1.22, 1.35, 0.14, 0.22, -0.016),
        (0.20, 2.58, 0.24, 0.16, -0.020),
    ]
    fine_pits = [
        (-0.55, 0.48, 0.045, 0.050, 0.006),
        (-0.08, 0.92, 0.040, 0.045, 0.005),
        (0.42, 1.35, 0.045, 0.045, 0.006),
        (0.80, 1.78, 0.050, 0.050, 0.006),
        (-1.18, 1.95, 0.045, 0.055, 0.005),
        (-0.78, 2.60, 0.050, 0.050, 0.005),
        (1.12, 2.35, 0.045, 0.045, 0.006),
    ]
    fine_bumps = [
        (-0.95, 0.65, 0.045, 0.045, -0.004),
        (0.22, 1.20, 0.055, 0.045, -0.005),
        (0.70, 2.70, 0.045, 0.055, -0.004),
        (-1.25, 0.95, 0.050, 0.040, -0.004),
    ]
    defects = np.zeros_like(x)
    for cx, cz, sx, sz, amp in dents + bumps + fine_pits + fine_bumps:
        defects += amp * np.exp(-(((x - cx) / sx) ** 2 + ((z - cz) / sz) ** 2))
    joints = np.zeros_like(x)
    for joint_x, phase in ((-0.95, 0.0), (0.15, 0.8), (1.05, 1.9)):
        center = joint_x + 0.020 * np.sin(3.0 * z + phase) + 0.008 * np.sin(10.0 * z + phase)
        width = 0.012 + 0.006 * (0.5 + 0.5 * np.sin(4.5 * z + phase))
        joints += np.where(np.abs(x - center) < width, 0.006, 0.0)
    for joint_z, phase in ((0.75, 0.4), (1.55, 1.5), (2.35, 2.1)):
        center = joint_z + 0.020 * np.sin(2.8 * x + phase) + 0.008 * np.sin(8.0 * x)
        width = 0.012 + 0.006 * (0.5 + 0.5 * np.sin(4.0 * x + phase))
        joints += np.where(np.abs(z - center) < width, 0.005, 0.0)
    patches = np.zeros_like(x)
    for x0, x1, z0, z1, offset in (
        (-1.35, -0.95, 0.35, 0.75, 0.005),
        (-0.25, 0.15, 1.00, 1.45, 0.004),
        (0.65, 1.20, 2.10, 2.58, 0.006),
    ):
        patches += np.where((x > x0) & (x < x1) & (z > z0) & (z < z1), offset, 0.0)
    return bow + waves + defects + joints + patches


def _facade_scan_boxes() -> list[tuple[str, tuple[float, float, float], tuple[float, float], tuple[float, float, float]]]:
    boxes: list[tuple[str, tuple[float, float, float], tuple[float, float], tuple[float, float, float]]] = []
    window_specs = [
        ("window_a", (-0.82, 0.90), (0.45, 0.014, 0.58)),
        ("window_b", (0.72, 1.85), (0.55, 0.014, 0.70)),
    ]
    for name, xz, size in window_specs:
        x, z = xz
        sx, sy, sz = size
        boxes.append((f"{name}_glass", size, xz, (0.04, 0.08, 0.12)))
        for suffix, frame_size, frame_xz in (
            ("top", (sx + 0.12, 0.018, 0.04), (x, z + sz / 2.0 + 0.045)),
            ("bottom", (sx + 0.12, 0.018, 0.04), (x, z - sz / 2.0 - 0.045)),
            ("left", (0.04, 0.018, sz + 0.12), (x - sx / 2.0 - 0.045, z)),
            ("right", (0.04, 0.018, sz + 0.12), (x + sx / 2.0 + 0.045, z)),
        ):
            boxes.append((f"{name}_{suffix}", frame_size, frame_xz, (0.88, 0.86, 0.80)))

    boxes.extend(
        [
            ("repair_patch_0", (0.42, 0.014, 0.35), (-1.10, 0.48), (0.70, 0.68, 0.60)),
            ("repair_patch_1", (0.44, 0.014, 0.45), (-0.12, 1.25), (0.78, 0.74, 0.65)),
            ("repair_patch_2", (0.62, 0.014, 0.48), (0.92, 2.35), (0.65, 0.62, 0.56)),
        ]
    )

    for j, (base_x, phase) in enumerate(((-0.95, 0.0), (0.15, 0.8), (1.05, 1.9))):
        for seg in range(5):
            z = 0.35 + seg * 0.55
            x = base_x + 0.020 * math.sin(3.0 * z + phase) + 0.008 * math.sin(10.0 * z + phase)
            height = 0.30 + 0.06 * math.sin(2.3 * seg + phase)
            width = 0.018 + 0.006 * ((seg + j) % 3)
            boxes.append((f"vertical_joint_marker_{j}_{seg}", (width, 0.016, height), (x, z), (0.28, 0.28, 0.30)))
    for j, (base_z, phase) in enumerate(((0.75, 0.4), (1.55, 1.5), (2.35, 2.1))):
        for seg in range(5):
            x = -1.20 + seg * 0.55
            z = base_z + 0.020 * math.sin(2.8 * x + phase) + 0.008 * math.sin(8.0 * x)
            length = 0.30 + 0.10 * math.sin(1.7 * seg + phase)
            height = 0.018 + 0.006 * ((seg + j) % 2)
            boxes.append((f"horizontal_joint_marker_{j}_{seg}", (length, 0.016, height), (x, z), (0.34, 0.34, 0.36)))

    for idx, x, z, sx, sz in (
        (0, -0.72, 0.78, 0.16, 0.06),
        (1, 0.65, 2.15, 0.22, 0.07),
        (2, 1.05, 1.15, 0.14, 0.05),
        (3, -1.10, 2.42, 0.20, 0.06),
    ):
        boxes.append((f"chip_marker_{idx}", (sx, 0.016, sz), (x, z), (0.18, 0.18, 0.19)))

    for idx, x, z in (
        (0, -0.55, 0.48),
        (1, -0.08, 0.92),
        (2, 0.42, 1.35),
        (3, 0.80, 1.78),
        (4, -1.18, 1.95),
        (5, -0.78, 2.60),
        (6, 1.12, 2.35),
    ):
        boxes.append((f"fine_pit_marker_{idx}", (0.055, 0.012, 0.055), (x, z), (0.12, 0.12, 0.13)))
    return boxes


def _box_center_y_for_scan_box(name: str, size: tuple[float, float, float], xz: tuple[float, float]) -> float:
    _, sy, _ = size
    x, z = xz
    if name.endswith("_glass"):
        clearance = 0.020
    elif name.startswith("fine_pit"):
        clearance = 0.002
    elif name.startswith(("vertical_joint", "horizontal_joint", "chip")):
        clearance = 0.003
    else:
        clearance = 0.004
    return _surface_front_y(x, z, sy, clearance_m=clearance)


def _scene_surface_y_array(x_m: np.ndarray, z_m: np.ndarray) -> np.ndarray:
    x = np.asarray(x_m, dtype=np.float64)
    z = np.asarray(z_m, dtype=np.float64)
    surface = _facade_y_offset_vectorized(x, z)
    for name, size, xz, _color in _facade_scan_boxes():
        sx, sy, sz = size
        cx, cz = xz
        mask = (np.abs(x - cx) <= sx * 0.5) & (np.abs(z - cz) <= sz * 0.5)
        if np.any(mask):
            front_y = _box_center_y_for_scan_box(name, size, xz) - sy * 0.5
            surface = np.where(mask, np.minimum(surface, front_y), surface)
    return surface


def _build_scene_surface_raster(res_m: float = 0.001) -> tuple[np.ndarray, float, float, float]:
    """Precompute the full scene surface (smooth + boxes) into a dense XZ raster.

    Called once at startup. Lookup at profile time is a simple integer index —
    ~1000× faster than calling _scene_surface_y_array per bisection iteration.
    """
    x_min, x_max = -1.65, 1.65
    z_min, z_max = -0.05, 3.20
    xs = np.arange(x_min, x_max + res_m, res_m)
    zs = np.arange(z_min, z_max + res_m, res_m)
    XX, ZZ = np.meshgrid(xs, zs, indexing="ij")
    surface = _facade_y_offset_vectorized(XX.ravel(), ZZ.ravel()).reshape(XX.shape)
    for name, size, xz, _color in _facade_scan_boxes():
        sx, sy, sz = size
        cx, cz = xz
        xi = np.where((xs >= cx - sx * 0.5) & (xs <= cx + sx * 0.5))[0]
        zi = np.where((zs >= cz - sz * 0.5) & (zs <= cz + sz * 0.5))[0]
        if xi.size and zi.size:
            front_y = _box_center_y_for_scan_box(name, size, xz) - sy * 0.5
            surface[np.ix_(xi, zi)] = np.minimum(surface[np.ix_(xi, zi)], front_y)
    return surface, float(x_min), float(z_min), float(res_m)


def _make_raster_sampler(raster: np.ndarray, x0: float, z0: float, res_m: float):
    """Return a fast surface sampler backed by nearest-neighbour raster lookup."""
    nx, nz = raster.shape

    def sampler(x_m: np.ndarray, z_m: np.ndarray) -> np.ndarray:
        xi = np.clip(np.round((np.asarray(x_m, dtype=np.float64) - x0) / res_m).astype(np.int32), 0, nx - 1)
        zi = np.clip(np.round((np.asarray(z_m, dtype=np.float64) - z0) / res_m).astype(np.int32), 0, nz - 1)
        return raster[xi, zi]

    return sampler


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


def _facade_grid_points(resolution: int = 90) -> tuple[list[Gf.Vec3f], list[int], list[int]]:
    """Regular XZ grid over the facade, height-offset per _facade_y_offset, as quad faces."""
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
    return points, counts, indices


def _box_triangles(size: tuple[float, float, float], center: tuple[float, float, float]) -> tuple[np.ndarray, np.ndarray]:
    """Axis-aligned box as (8 vertices, 12 triangle faces) for raycast meshes."""
    sx, sy, sz = size
    cx, cy, cz = center
    hx, hy, hz = sx * 0.5, sy * 0.5, sz * 0.5
    vertices = np.array(
        [
            [cx - hx, cy - hy, cz - hz],
            [cx + hx, cy - hy, cz - hz],
            [cx + hx, cy + hy, cz - hz],
            [cx - hx, cy + hy, cz - hz],
            [cx - hx, cy - hy, cz + hz],
            [cx + hx, cy - hy, cz + hz],
            [cx + hx, cy + hy, cz + hz],
            [cx - hx, cy + hy, cz + hz],
        ],
        dtype=np.float64,
    )
    faces = np.array(
        [
            [0, 1, 2], [0, 2, 3],  # -Z
            [4, 6, 5], [4, 7, 6],  # +Z
            [0, 4, 5], [0, 5, 1],  # -Y
            [3, 2, 6], [3, 6, 7],  # +Y
            [0, 3, 7], [0, 7, 4],  # -X
            [1, 5, 6], [1, 6, 2],  # +X
        ],
        dtype=np.int64,
    )
    return vertices, faces


def _facade_mesh_for_raycast(resolution: int = 90) -> tuple[np.ndarray, np.ndarray]:
    """Facade surface + defect boxes as one triangle mesh, for real ray-triangle raycasting.

    Reuses the same grid points and box placements as _create_defect_facade / the analytic
    _scene_surface_y_array, so the raycast validates against the same geometry that's rendered.
    """
    points, _counts, _indices = _facade_grid_points(resolution)
    vertices = np.array([[p[0], p[1], p[2]] for p in points], dtype=np.float64)
    faces = []
    for row in range(resolution - 1):
        for col in range(resolution - 1):
            i = row * resolution + col
            faces.append((i, i + 1, i + resolution + 1))
            faces.append((i, i + resolution + 1, i + resolution))
    faces = np.array(faces, dtype=np.int64)

    for name, size, xz, _color in _facade_scan_boxes():
        x, z = xz
        y = _box_center_y_for_scan_box(name, size, xz)
        box_vertices, box_faces = _box_triangles(size, (x, y, z))
        offset = len(vertices)
        vertices = np.vstack([vertices, box_vertices])
        faces = np.vstack([faces, box_faces + offset])
    return vertices, faces


def _build_facade_ray_intersector() -> Callable[[np.ndarray, np.ndarray], tuple[np.ndarray, np.ndarray]]:
    """Build a ray_intersect_fn (see Gocator2690LineProfiler.sample_mesh) via real ray-triangle
    intersection against the facade mesh, using trimesh. Requires `trimesh` in Isaac's Python env
    (isaac/README.md documents the one-time install)."""
    import trimesh

    vertices, faces = _facade_mesh_for_raycast()
    mesh = trimesh.Trimesh(vertices=vertices, faces=faces, process=False)
    intersector = trimesh.ray.ray_triangle.RayMeshIntersector(mesh)

    def intersect_fn(origins: np.ndarray, directions: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        n = len(origins)
        t = np.full(n, np.nan, dtype=np.float64)
        normals = np.zeros((n, 3), dtype=np.float64)
        locations, index_ray, index_tri = intersector.intersects_location(
            origins, directions, multiple_hits=False
        )
        if len(index_ray):
            t[index_ray] = np.einsum(
                "ij,ij->i", locations - origins[index_ray], directions[index_ray]
            )
            normals[index_ray] = mesh.face_normals[index_tri]
        return t, normals

    return intersect_fn


def _create_defect_facade(stage) -> None:
    facade = UsdGeom.Xform.Define(stage, FACADE_PATH)
    _make_rigid(facade.GetPrim(), 1.0e9, kinematic=True)

    points, counts, indices = _facade_grid_points(resolution=90)

    surface = UsdGeom.Mesh.Define(stage, f"{FACADE_PATH}/surface")
    surface.CreatePointsAttr(Vt.Vec3fArray(points))
    surface.CreateFaceVertexCountsAttr(Vt.IntArray(counts))
    surface.CreateFaceVertexIndicesAttr(Vt.IntArray(indices))
    surface.CreateSubdivisionSchemeAttr("none")
    surface.CreateDisplayColorAttr(Vt.Vec3fArray([Gf.Vec3f(0.56, 0.58, 0.60)]))
    surface.CreateDoubleSidedAttr(True)
    _add_collision(surface.GetPrim())

    for name, size, xz, color in _facade_scan_boxes():
        x, z = xz
        y = _box_center_y_for_scan_box(name, size, xz)
        box = _create_box(stage, f"{FACADE_PATH}/{name}", size, (x, y, z), color)
        _add_collision(box.GetPrim())


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
    xs = np.linspace(x_min, x_max, sample_count, dtype=np.float64)
    zs = np.full(sample_count, min(max(center_z_m, 0.0), FACADE_HEIGHT_M), dtype=np.float64)
    # Draw just in front of the scanned hit surface to avoid z-fighting.
    ys = _scene_surface_y_array(xs, zs) - 0.002
    points = [Gf.Vec3f(float(x), float(y), float(z)) for x, y, z in zip(xs, ys, zs)]
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


class GocatorStreamingExporter:
    """Flush Gocator valid points to disk in batches; finalize to PLY + heightmap.

    Only xyz (float32) of valid points is kept per batch so peak RAM during
    acquisition stays low (~70 MB per batch at BATCH_SIZE=2000 profiles).
    At finalize() all batch files are loaded once to write the final outputs.
    """

    BATCH_SIZE = 2000

    def __init__(self, output_dir: Path, profile_spacing_m: float = 0.0005) -> None:
        self.output_dir = output_dir
        self.profile_spacing_m = profile_spacing_m
        output_dir.mkdir(parents=True, exist_ok=True)
        self._xyz_buffer: list[np.ndarray] = []
        self._batch_index = 0
        self.total_profiles = 0
        self.total_valid_points = 0

    def add_profile(self, profile: GocatorProfile) -> None:
        valid_pts = profile.valid_points_m
        if valid_pts.size:
            self._xyz_buffer.append(valid_pts.astype(np.float32))
            self.total_valid_points += len(valid_pts)
        self.total_profiles += 1
        if self.total_profiles % self.BATCH_SIZE == 0:
            self._flush()

    def _flush(self) -> None:
        if not self._xyz_buffer:
            return
        batch = np.vstack(self._xyz_buffer)
        np.save(self.output_dir / f"_pts_batch_{self._batch_index:05d}.npy", batch)
        self._batch_index += 1
        self._xyz_buffer.clear()

    def finalize(self) -> np.ndarray:
        """Flush remaining buffer, write PLY + heightmap, delete batch files. Returns cloud."""
        self._flush()
        batch_paths = sorted(self.output_dir.glob("_pts_batch_*.npy"))
        if not batch_paths:
            cloud = np.empty((0, 3), dtype=np.float32)
        else:
            cloud = np.vstack([np.load(p) for p in batch_paths])
            for p in batch_paths:
                p.unlink()

        export_point_cloud_ply(self.output_dir / "gocator_point_cloud.ply", cloud)
        height_map = _cloud_to_height_map_array(cloud, self.profile_spacing_m)
        export_height_map_npy(self.output_dir / "gocator_height_map.npy", height_map)
        return cloud


class LightweightDisplayAccumulator:
    """Accumulate valid xyz (float32) with a stride for RViz preview.

    Stores 1 profile out of every `display_stride` to keep memory bounded.
    Old chunks are evicted when the total exceeds `max_points * 1.5`.
    """

    def __init__(self, max_points: int, display_stride: int) -> None:
        self.max_points = max_points
        self.display_stride = max(1, display_stride)
        self._chunks: list[np.ndarray] = []
        self._total_points = 0
        self._profile_count = 0

    def add_profile(self, profile: GocatorProfile) -> None:
        self._profile_count += 1
        if self._profile_count % self.display_stride != 0:
            return
        pts = profile.valid_points_m
        if pts.size == 0:
            return
        chunk = pts.astype(np.float32)
        self._chunks.append(chunk)
        self._total_points += len(chunk)
        # Evict oldest chunks when well over budget
        while self._total_points > self.max_points * 1.5 and len(self._chunks) > 1:
            self._total_points -= len(self._chunks.pop(0))

    def get_display_points(self) -> np.ndarray:
        if not self._chunks:
            return np.empty((0, 3), dtype=np.float32)
        cloud = np.vstack(self._chunks)
        if len(cloud) <= self.max_points:
            return cloud
        idx = np.linspace(0, len(cloud) - 1, self.max_points, dtype=np.int64)
        return cloud[idx]


def _cloud_to_height_map_array(
    cloud: np.ndarray,
    profile_spacing_m: float,
    x_resolution_m: float = 0.005,
    fill_value: float = np.nan,
) -> np.ndarray:
    """Build a 2-D height map from an (N, 3) xyz point cloud."""
    if cloud.size == 0:
        return np.empty((0, 0), dtype=np.float32)
    x_res = x_resolution_m
    y_res = profile_spacing_m
    x_bins = np.floor((cloud[:, 0] - cloud[:, 0].min()) / x_res).astype(np.int32)
    y_bins = np.floor((cloud[:, 2] - cloud[:, 2].min()) / y_res).astype(np.int32)
    image = np.full((y_bins.max() + 1, x_bins.max() + 1), fill_value, dtype=np.float32)
    image[y_bins, x_bins] = cloud[:, 1]
    return image


class IsaacGocatorRosBridge:
    """Publish Gocator data generated from the Isaac scanner state to ROS 2."""

    def __init__(
        self,
        datasheet: dict,
        surface_sampler=None,
        *,
        raycast_mode: str = "analytic",
        ray_intersect_fn: Callable[[np.ndarray, np.ndarray], tuple[np.ndarray, np.ndarray]] | None = None,
    ) -> None:
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
        self.display_stride = max(1, ARGS.ros_profile_stride)
        self.display_point_stride = max(1, ARGS.ros_profile_point_stride)

        spec = Gocator2690Spec(
            points_per_profile=int(datasheet.get("points_per_profile", 3700)),
            profile_spacing_m=float(datasheet.get("profile_spacing_m", 0.0005)),
            nominal_standoff_m=_gocator_standoff_m(datasheet),
            nominal_profile_rate_hz=float(datasheet.get("nominal_profile_rate_hz", 2000.0)),
        )
        self.datasheet = datasheet
        self.scan_speed_m_s = ARGS.scan_speed if ARGS.scan_speed > 0.0 else _gocator_scan_speed_m_s(datasheet)
        self.profile_period_s = 1.0 / max(spec.nominal_profile_rate_hz, 1e-6)
        self.next_profile_time_s = 0.0
        self.profile_index = 0
        self.profiler = Gocator2690LineProfiler(spec)

        # Display accumulator: lightweight xyz float32 with stride for RViz.
        self.display_accumulator = LightweightDisplayAccumulator(
            max_points=self.max_points,
            display_stride=self.display_stride,
        )

        self._surface_sampler = surface_sampler if surface_sampler is not None else _scene_surface_y_array
        self.raycast_mode = raycast_mode
        self._ray_intersect_fn = ray_intersect_fn

        # Streaming exporter: full-res valid points flushed to disk per batch.
        self.exporter: GocatorStreamingExporter | None = None
        if ARGS.record_full_res:
            output_dir = ARGS.record_output_dir
            if not output_dir.is_absolute():
                output_dir = PROJECT_ROOT / output_dir
            self.exporter = GocatorStreamingExporter(output_dir, spec.profile_spacing_m)

        self.node.get_logger().info(
            f"Isaac ROS bridge: {spec.points_per_profile} pts/profile, "
            f"{spec.nominal_profile_rate_hz:.0f} Hz, spacing={spec.profile_spacing_m:.4f} m, "
            f"scan_speed={self.scan_speed_m_s:.3f} m/s | "
            f"display_stride={self.display_stride}, max_display_pts={self.max_points}, "
            f"export={'streaming' if self.exporter else 'off'}"
        )

    def update(self, timestamp_s: float) -> None:
        stamp = self.node.get_clock().now().to_msg()
        while self.next_profile_time_s <= timestamp_s + 1e-12:
            scanner_pose = _scan_pose_at_time(
                self.datasheet,
                self.next_profile_time_s,
                self.scan_speed_m_s,
            )
            profile = self._sample_profile_at_pose(scanner_pose, self.next_profile_time_s)

            # Full-res streaming export: every profile.
            if self.exporter is not None:
                self.exporter.add_profile(profile)

            # Display: strided, lightweight, for RViz.
            self.display_accumulator.add_profile(profile)
            if self.profile_index % self.display_stride == 0:
                pts = profile.valid_points_m
                if self.display_point_stride > 1:
                    pts = pts[:: self.display_point_stride]
                self.profile_pub.publish(self._pointcloud2(pts, stamp))

            self.profile_index += 1
            self.next_profile_time_s += self.profile_period_s

        if self.last_publish_s < 0.0 or timestamp_s - self.last_publish_s >= self.publish_period_s:
            self.last_publish_s = timestamp_s
            self.cloud_pub.publish(self._pointcloud2(self.display_accumulator.get_display_points(), stamp))
        self.rclpy.spin_once(self.node, timeout_sec=0.0)

    def shutdown(self) -> None:
        if self.exporter is not None:
            self._finalize_export()
        self.node.get_logger().info(f"Isaac ROS bridge: {self.profile_index} profiles processed")
        self.node.destroy_node()
        if self.rclpy.ok():
            self.rclpy.shutdown()

    def _finalize_export(self) -> None:
        output_dir = self.exporter.output_dir
        cloud = self.exporter.finalize()
        summary = {
            "profiles": self.exporter.total_profiles,
            "valid_points": int(len(cloud)),
            "points_per_profile": int(self.profiler.spec.points_per_profile),
            "profile_rate_hz": float(self.profiler.spec.nominal_profile_rate_hz),
            "profile_spacing_m": float(self.profiler.spec.profile_spacing_m),
            "scan_speed_m_s": float(self.scan_speed_m_s),
            "files": {
                "point_cloud_ply": "gocator_point_cloud.ply",
                "height_map_npy": "gocator_height_map.npy",
            },
        }
        with (output_dir / "scan_summary.json").open("w", encoding="utf-8") as fp:
            json.dump(summary, fp, indent=2)
        self.node.get_logger().info(
            f"Exported scan to {output_dir} "
            f"({self.exporter.total_profiles} profiles, {len(cloud)} valid points)"
        )

    def _sample_profile_at_pose(self, scanner_pose: Gf.Vec3d, timestamp_s: float):
        pose = ScannerFramePose.from_arrays(
            [scanner_pose[0], scanner_pose[1], scanner_pose[2]],
            [1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0],
            [0.0, 1.0, 0.0],
        )
        if self.raycast_mode == "mesh":
            profile = self.profiler.sample_mesh(
                pose,
                self._ray_intersect_fn,
                timestamp_s=timestamp_s,
                profile_index=self.profile_index,
                encoder_position_m=timestamp_s * self.scan_speed_m_s,
            )
        else:
            profile = self.profiler.sample_surface(
                pose,
                self._surface_sampler,
                timestamp_s=timestamp_s,
                profile_index=self.profile_index,
                encoder_position_m=timestamp_s * self.scan_speed_m_s,
            )
        profile.valid_mask &= _wall_valid_mask(profile.points_world)
        return profile

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

    ground = _create_box(
        stage,
        GROUND_PATH,
        (FACADE_WIDTH_M + 2.0, 4.0, 0.05),
        (0.0, -1.0, -0.025),
        (0.22, 0.24, 0.25),
    )
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

    SimulationManager.setup_simulation(dt=ARGS.sim_dt, device="cpu")
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
            if ARGS.raycast_mode == "mesh":
                print("[surface] Building facade raycast mesh (trimesh)...", flush=True)
                intersect_fn = _build_facade_ray_intersector()
                print("[surface] Raycast mesh ready.", flush=True)
                ros_bridge = IsaacGocatorRosBridge(
                    datasheet, raycast_mode="mesh", ray_intersect_fn=intersect_fn
                )
            else:
                print("[surface] Precomputing facade surface raster (1 mm)...", flush=True)
                _raster, _rx0, _rz0, _rres = _build_scene_surface_raster(res_m=0.001)
                _fast_surface = _make_raster_sampler(_raster, _rx0, _rz0, _rres)
                print("[surface] Raster ready.", flush=True)
                ros_bridge = IsaacGocatorRosBridge(datasheet, surface_sampler=_fast_surface)
        except Exception as exc:
            print(f"[ros_bridge] disabled: {exc}", flush=True)

    sim_dt = ARGS.sim_dt
    scan_speed_m_s = ARGS.scan_speed if ARGS.scan_speed > 0.0 else _gocator_scan_speed_m_s(datasheet)
    scan_duration_s = _scan_path_length_m(datasheet) / scan_speed_m_s if scan_speed_m_s > 0.0 else 0.0
    if ARGS.stop_after_scan:
        print(f"[scan] stop-after-scan enabled, estimated duration={scan_duration_s:.2f}s", flush=True)

    frame_count = 0
    try:
        while simulation_app.is_running():
            elapsed_s = frame_count * sim_dt
            scanner_pose = _scan_pose_at_time(datasheet, elapsed_s, ARGS.scan_speed)
            robot_translate_attr.Set(scanner_pose)
            laser_points_attr.Set(_laser_contact_points(datasheet, scanner_pose[0], scanner_pose[2]))
            if ros_bridge is not None:
                ros_bridge.update(elapsed_s)
            simulation_app.update()
            frame_count += 1
            if frame_limit > 0 and frame_count >= frame_limit:
                print(f"Completed {frame_count} simulation frames", flush=True)
                break
            if ARGS.stop_after_scan and scan_duration_s > 0.0 and elapsed_s >= scan_duration_s:
                print(f"Completed one full scan path in {elapsed_s:.2f}s simulated time", flush=True)
                break
    finally:
        if ros_bridge is not None:
            ros_bridge.shutdown()
        simulation_app.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
