#!/usr/bin/env python3
"""Sensing/teleop gridsim scene in Isaac Sim 6.

Robot at /World/robot follows /robot/pose via a background rclpy thread.
3× TF-Luna sensor boxes are mounted on the robot front face.
"""

from __future__ import annotations

import argparse
import json
import math
import queue
import sys
import threading
import time
from pathlib import Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--test", action="store_true")
    parser.add_argument("--no-lidar", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--no-ros", action="store_true")
    parser.add_argument("--realtime", action="store_true")
    parser.add_argument("--frames", type=int, default=0)
    parser.add_argument("--output", type=Path)
    args, kit_args = parser.parse_known_args()
    sys.argv = [sys.argv[0], *kit_args]
    return args


ARGS = parse_args()

from isaacsim import SimulationApp

simulation_app = SimulationApp(
    {"headless": ARGS.headless or ARGS.test, "renderer": "RayTracedLighting"}
)

import omni.graph.core as og
import omni.usd
import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import usdrt
from isaacsim.core.simulation_manager import SimulationManager
from pxr import Gf, UsdGeom, UsdLux, UsdPhysics

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "src"))

from gridsim_core.scene_config import MvpSceneConfig

FACADE_PATH = "/World/facade"
GROUND_PATH = "/World/ground"
GRID_PATH = "/World/grid"
ROBOT_PATH = "/World/robot"

GRID_BAR_ASSET_PATH = PROJECT_ROOT / "assets/cad/grid/vertical_grid_bar.usd"
GRID_BAR_METADATA_PATH = PROJECT_ROOT / "assets/cad/grid/vertical_grid_bar.json"
HORIZONTAL_GRID_BAR_ASSET_PATH = PROJECT_ROOT / "assets/cad/grid/horizontal_rail.usd"
HORIZONTAL_GRID_BAR_METADATA_PATH = PROJECT_ROOT / "assets/cad/grid/horizontal_rail.json"
CARRIAGE_ASSET_PATH = PROJECT_ROOT / "assets/cad/grid/carriage.usd"
CARRIAGE_METADATA_PATH = PROJECT_ROOT / "assets/cad/grid/carriage.json"

_SENSOR_SPACING_M = 0.15          # left=-0.15, center=0, right=+0.15
_ROBOT_BODY_W, _ROBOT_BODY_D, _ROBOT_BODY_H = 0.55, 0.42, 0.30

_DEFAULT_GRID_BAR_METADATA = {
    "scene_unit_scale": 0.001,
    "length_asset_units": 2000.0,
    "width_m": 0.11725,
    "depth_m": 0.142,
}


def _load_grid_bar_metadata() -> dict[str, float]:
    if not GRID_BAR_METADATA_PATH.exists():
        return _DEFAULT_GRID_BAR_METADATA
    with GRID_BAR_METADATA_PATH.open("r", encoding="utf-8") as fp:
        metadata = json.load(fp)
    return {**_DEFAULT_GRID_BAR_METADATA, **metadata}


def _load_horizontal_grid_bar_metadata() -> dict[str, float]:
    if not HORIZONTAL_GRID_BAR_METADATA_PATH.exists():
        return _load_grid_bar_metadata()
    with HORIZONTAL_GRID_BAR_METADATA_PATH.open("r", encoding="utf-8") as fp:
        metadata = json.load(fp)
    return {**_DEFAULT_GRID_BAR_METADATA, **metadata}


def _load_carriage_metadata() -> dict[str, float]:
    if not CARRIAGE_METADATA_PATH.exists():
        return {"scene_unit_scale": 0.001}
    with CARRIAGE_METADATA_PATH.open("r", encoding="utf-8") as fp:
        metadata = json.load(fp)
    return {"scene_unit_scale": 0.001, **metadata}


_GRID_BAR_METADATA = _load_grid_bar_metadata()
_GRID_BAR_UNIT_SCALE = float(_GRID_BAR_METADATA["scene_unit_scale"])
_GRID_BAR_LENGTH_ASSET_UNITS = float(_GRID_BAR_METADATA["length_asset_units"])
_GRID_BAR_LENGTH_M = _GRID_BAR_UNIT_SCALE * _GRID_BAR_LENGTH_ASSET_UNITS
_GRID_BAR_WIDTH_M = float(_GRID_BAR_METADATA["width_m"])
_GRID_BAR_DEPTH_M = float(_GRID_BAR_METADATA["depth_m"])

_HORIZONTAL_GRID_BAR_METADATA = _load_horizontal_grid_bar_metadata()
_HORIZONTAL_GRID_BAR_UNIT_SCALE = float(_HORIZONTAL_GRID_BAR_METADATA["scene_unit_scale"])
_HORIZONTAL_GRID_BAR_LENGTH_ASSET_UNITS = float(_HORIZONTAL_GRID_BAR_METADATA["length_asset_units"])
_HORIZONTAL_GRID_BAR_LENGTH_M = (
    _HORIZONTAL_GRID_BAR_UNIT_SCALE * _HORIZONTAL_GRID_BAR_LENGTH_ASSET_UNITS
)

_CARRIAGE_METADATA = _load_carriage_metadata()
_CARRIAGE_UNIT_SCALE = float(_CARRIAGE_METADATA["scene_unit_scale"])

# Populated by build_scene(), consumed in the main loop
_robot_translate_op = None
_robot_yaw_op = None
_robot_z = None

# Latest pose from ROS2 teleop
_pose_q: queue.Queue[tuple[float, float, float]] = queue.Queue(maxsize=1)


def _start_pose_listener() -> None:
    """Background thread: subscribe to /robot/pose, push (x, y, yaw) into _pose_q."""
    try:
        import rclpy
        from geometry_msgs.msg import PoseStamped

        ctx = rclpy.Context()
        rclpy.init(context=ctx)
        node = rclpy.create_node("isaac_pose_follower", context=ctx)
        print("[pose_listener] started, waiting for /robot/pose ...", flush=True)

        _first = [True]

        def _cb(msg: PoseStamped) -> None:
            x = msg.pose.position.x
            y = msg.pose.position.y
            q = msg.pose.orientation
            yaw = 2.0 * math.atan2(q.z, q.w)
            if _first[0]:
                print(f"[pose_listener] first pose received  x={x:.3f}  y={y:.3f}  yaw={math.degrees(yaw):.1f}°", flush=True)
                _first[0] = False
            try:
                _pose_q.get_nowait()
            except queue.Empty:
                pass
            _pose_q.put_nowait((x, y, yaw))

        node.create_subscription(PoseStamped, "/robot/pose", _cb, 10)
        exec_ = rclpy.executors.SingleThreadedExecutor(context=ctx)
        exec_.add_node(node)
        exec_.spin()
    except Exception as exc:
        print(f"[pose_listener] FAILED: {exc}", flush=True)


def _create_xform(stage, path: str, translation=(0.0, 0.0, 0.0)):
    xform = UsdGeom.Xform.Define(stage, path)
    xform.AddTranslateOp().Set(Gf.Vec3d(*translation))
    return xform


def _create_box(stage, path, size, translation, color):
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    cube.AddTranslateOp().Set(Gf.Vec3d(*translation))
    cube.AddScaleOp().Set(Gf.Vec3f(*size))
    cube.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    return cube


def _add_collision(prim):
    UsdPhysics.CollisionAPI.Apply(prim)


def _make_rigid(prim, mass_kg=None, *, kinematic=False):
    rb = UsdPhysics.RigidBodyAPI.Apply(prim)
    rb.CreateKinematicEnabledAttr(kinematic)
    if mass_kg is not None:
        UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(float(mass_kg))


def _segment_count(length_m: float, asset_length_m: float) -> int:
    return max(1, round(length_m / asset_length_m))


def _create_grid_bar(stage, path: str, length_m: float, translation, color, *, horizontal=False):
    """Create one CAD visual grid bar with a simple invisible box collider."""
    bar = UsdGeom.Xform.Define(stage, path)
    bar.AddTranslateOp().Set(Gf.Vec3d(*translation))
    if horizontal:
        bar.AddRotateZOp().Set(90.0)

    asset_path = HORIZONTAL_GRID_BAR_ASSET_PATH if horizontal and HORIZONTAL_GRID_BAR_ASSET_PATH.exists() else GRID_BAR_ASSET_PATH
    asset_unit_scale = _HORIZONTAL_GRID_BAR_UNIT_SCALE if horizontal and HORIZONTAL_GRID_BAR_ASSET_PATH.exists() else _GRID_BAR_UNIT_SCALE
    asset_length_m = _HORIZONTAL_GRID_BAR_LENGTH_M if horizontal and HORIZONTAL_GRID_BAR_ASSET_PATH.exists() else _GRID_BAR_LENGTH_M
    has_cad_asset = asset_path.exists()
    if has_cad_asset:
        segment_count = _segment_count(length_m, asset_length_m)
        first_segment_y = -0.5 * (segment_count - 1) * asset_length_m
        for index in range(segment_count):
            visual = UsdGeom.Xform.Define(stage, f"{path}/visual_{index}")
            visual.AddTranslateOp().Set(
                Gf.Vec3d(0.0, first_segment_y + index * asset_length_m, 0.0)
            )
            visual.AddScaleOp().Set(
                Gf.Vec3f(asset_unit_scale, asset_unit_scale, asset_unit_scale)
            )
            oriented = UsdGeom.Xform.Define(stage, f"{path}/visual_{index}/oriented")
            oriented.AddRotateYOp().Set(-90.0)
            asset = UsdGeom.Xform.Define(stage, f"{path}/visual_{index}/oriented/asset")
            asset.GetPrim().GetReferences().AddReference(str(asset_path))

    collider = _create_box(
        stage,
        f"{path}/collider",
        (_GRID_BAR_WIDTH_M, length_m, _GRID_BAR_DEPTH_M),
        (0.0, 0.0, 0.0),
        color,
    )
    _add_collision(collider.GetPrim())
    if has_cad_asset:
        UsdGeom.Imageable(collider.GetPrim()).MakeInvisible()
    return bar


def _create_carriage_preview(stage, path: str, translation):
    if not CARRIAGE_ASSET_PATH.exists():
        return None
    carriage = UsdGeom.Xform.Define(stage, path)
    carriage.AddTranslateOp().Set(Gf.Vec3d(*translation))
    carriage.AddScaleOp().Set(
        Gf.Vec3f(_CARRIAGE_UNIT_SCALE, _CARRIAGE_UNIT_SCALE, _CARRIAGE_UNIT_SCALE)
    )
    asset = UsdGeom.Xform.Define(stage, f"{path}/asset")
    asset.GetPrim().GetReferences().AddReference(str(CARRIAGE_ASSET_PATH))
    return carriage


def build_scene(config: MvpSceneConfig):
    global _robot_translate_op, _robot_yaw_op, _robot_z

    stage_utils.create_new_stage()
    stage_utils.set_stage_units(meters_per_unit=1.0)
    stage = omni.usd.get_context().get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    stage.DefinePrim("/World", "Xform")
    stage.SetDefaultPrim(stage.GetPrimAtPath("/World"))

    physics = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    physics.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
    physics.CreateGravityMagnitudeAttr().Set(9.81)

    UsdLux.DomeLight.Define(stage, "/World/DomeLight").CreateIntensityAttr(500.0)
    sun = UsdLux.DistantLight.Define(stage, "/World/Sun")
    sun.CreateIntensityAttr(1200.0)
    sun.CreateAngleAttr(1.0)

    ground = _create_box(stage, GROUND_PATH, (12.0, 8.0, 0.05), (0.0, 0.0, -0.025), (0.22, 0.24, 0.25))
    _add_collision(ground.GetPrim())

    # ── Facade ──────────────────────────────────────────────────────────────
    facade = UsdGeom.Xform.Define(stage, FACADE_PATH)
    facade.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, config.facade_height_m / 2.0))
    facade.AddRotateXOp().Set(90.0)
    _make_rigid(facade.GetPrim(), config.facade_mass_kg, kinematic=True)
    surf = _create_box(
        stage, f"{FACADE_PATH}/surface",
        (config.facade_width_m, config.facade_height_m, 0.1),
        (0.0, 0.0, -0.05), (0.55, 0.58, 0.62),
    )
    _add_collision(surf.GetPrim())

    # ── Grid frame ───────────────────────────────────────────────────────────
    vertical_visual_height_m = (
        _segment_count(config.grid_height_m + _GRID_BAR_WIDTH_M, _GRID_BAR_LENGTH_M)
        * _GRID_BAR_LENGTH_M
    )
    grid = _create_xform(stage, GRID_PATH,
                         (0.0, -config.facade_standoff_m, vertical_visual_height_m / 2.0))
    grid.AddRotateXOp().Set(90.0)
    _make_rigid(grid.GetPrim(), config.grid_mass_kg)
    bw = _GRID_BAR_WIDTH_M
    ox, oy = -0.5 * config.grid_width_m, -0.5 * config.grid_height_m
    for c in range(config.grid_columns + 1):
        if c % 2 == 1:
            continue
        _create_grid_bar(
            stage, f"{GRID_PATH}/frame/v{c}",
            config.grid_height_m + bw,
            (ox + c * config.module_width_m, 0.0, 0.0),
            (0.12, 0.32, 0.58),
        )
    for r in range(config.grid_rows + 1):
        if r in (0, config.grid_rows) or r == config.grid_rows // 2:
            continue
        _create_grid_bar(
            stage, f"{GRID_PATH}/frame/h{r}",
            config.grid_width_m + bw,
            (0.0, oy + r * config.module_height_m, 0.0),
            (0.10, 0.28, 0.52),
            horizontal=True,
        )
    _create_carriage_preview(stage, f"{GRID_PATH}/carriage_preview", (0.0, 0.0, 0.0))

    return stage


def create_ros_graph() -> None:
    keys = og.Controller.Keys
    tf_edges = [
        ("Facade", "", FACADE_PATH),
        ("Grid",   "", GRID_PATH),
    ]
    tf_nodes, tf_connections, tf_values = [], [], []
    for name, parent_path, child_path in tf_edges:
        c, p = f"Compute{name}TF", f"Publish{name}TF"
        tf_nodes += [
            (c, "isaacsim.core.nodes.IsaacComputeTransformTree"),
            (p, "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
        ]
        tf_connections += [
            ("OnPlaybackTick.outputs:tick",              f"{c}.inputs:execIn"),
            (f"{c}.outputs:execOut",                     f"{p}.inputs:execIn"),
            (f"{c}.outputs:parentFrames",                f"{p}.inputs:parentFrames"),
            (f"{c}.outputs:childFrames",                 f"{p}.inputs:childFrames"),
            (f"{c}.outputs:translations",                f"{p}.inputs:translations"),
            (f"{c}.outputs:orientations",                f"{p}.inputs:orientations"),
            ("ReadSimTime.outputs:simulationTime",       f"{p}.inputs:timeStamp"),
        ]
        tf_values += [
            (f"{c}.inputs:targetPrims", [usdrt.Sdf.Path(child_path)]),
            (f"{p}.inputs:topicName", "/tf"),
        ]
        if parent_path:
            tf_values.append((f"{c}.inputs:parentPrim", [usdrt.Sdf.Path(parent_path)]))

    og.Controller.edit(
        {"graph_path": "/World/ROSGraph", "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("ReadSimTime",    "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("PublishClock",   "isaacsim.ros2.bridge.ROS2PublishClock"),
            ] + tf_nodes,
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick",        "PublishClock.inputs:execIn"),
                ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
            ] + tf_connections,
            keys.SET_VALUES: [
                ("PublishClock.inputs:topicName", "/clock"),
            ] + tf_values,
        },
    )


def main() -> int:
    config = MvpSceneConfig.from_directory(PROJECT_ROOT / "configs")
    stage = build_scene(config)

    if not ARGS.test and not ARGS.no_ros:
        app_utils.enable_extension("isaacsim.ros2.bridge")
        simulation_app.update()
        create_ros_graph()

    out = (ARGS.output or PROJECT_ROOT / "outputs/isaac/teleop_scene.usda").resolve()
    out.parent.mkdir(parents=True, exist_ok=True)
    stage.GetRootLayer().Export(str(out))
    print(f"Saved scene: {out}", flush=True)

    SimulationManager.setup_simulation(
        dt=1.0 / config.simulation_frequency_hz, device="cpu"
    )
    app_utils.play()
    simulation_app.update()

    frame_limit = ARGS.frames
    if ARGS.test and frame_limit <= 0:
        frame_limit = 10

    frame_count = 0
    frame_period_s = 1.0 / config.simulation_frequency_hz

    while simulation_app.is_running():
        simulation_app.update()
        frame_count += 1
        if ARGS.realtime:
            time.sleep(frame_period_s)
        if frame_limit > 0 and frame_count >= frame_limit:
            break

    app_utils.stop()
    print(f"Completed {frame_count} simulation frames", flush=True)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    finally:
        simulation_app.close()
