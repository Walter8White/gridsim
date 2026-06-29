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
    parser.add_argument("--motor-rotate-x", type=float, default=90.0)
    parser.add_argument("--motor-rotate-y", type=float, default=90.0)
    parser.add_argument("--motor-rotate-z", type=float, default=180.0)
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
CARRIAGE_PATH = "/World/carriage"

RAIL_ASSET_PATH = PROJECT_ROOT / "assets/cad/grid/horizontal_rail.usd"
RAIL_METADATA_PATH = PROJECT_ROOT / "assets/cad/grid/horizontal_rail.json"
CARRIAGE_ASSET_PATH = PROJECT_ROOT / "assets/cad/grid/carriage.usd"
CARRIAGE_METADATA_PATH = PROJECT_ROOT / "assets/cad/grid/carriage.json"
DEPLOY_MOTOR_ASSET_PATH = PROJECT_ROOT / "assets/cad/grid/deployment_motor.usd"
DEPLOY_MOTOR_METADATA_PATH = PROJECT_ROOT / "assets/cad/grid/deployment_motor.json"

_SENSOR_SPACING_M = 0.15          # left=-0.15, center=0, right=+0.15
_ROBOT_BODY_W, _ROBOT_BODY_D, _ROBOT_BODY_H = 0.55, 0.42, 0.30

_DEFAULT_RAIL_METADATA = {
    "scene_unit_scale": 0.001,
    "length_asset_units": 2000.0,
    "width_m": 0.082,
    "depth_m": 0.07,
}


def _load_rail_metadata() -> dict[str, float]:
    if not RAIL_METADATA_PATH.exists():
        return _DEFAULT_RAIL_METADATA
    with RAIL_METADATA_PATH.open("r", encoding="utf-8") as fp:
        metadata = json.load(fp)
    return {**_DEFAULT_RAIL_METADATA, **metadata}


def _load_carriage_metadata() -> dict[str, float]:
    if not CARRIAGE_METADATA_PATH.exists():
        return {"scene_unit_scale": 0.001}
    with CARRIAGE_METADATA_PATH.open("r", encoding="utf-8") as fp:
        metadata = json.load(fp)
    return {"scene_unit_scale": 0.001, **metadata}


def _load_deploy_motor_metadata() -> dict:
    if not DEPLOY_MOTOR_METADATA_PATH.exists():
        return {"scene_unit_scale": 0.001}
    with DEPLOY_MOTOR_METADATA_PATH.open("r", encoding="utf-8") as fp:
        metadata = json.load(fp)
    return {"scene_unit_scale": 0.001, **metadata}


_RAIL_METADATA = _load_rail_metadata()
_RAIL_UNIT_SCALE = float(_RAIL_METADATA["scene_unit_scale"])
_RAIL_LENGTH_M = _RAIL_UNIT_SCALE * float(_RAIL_METADATA["length_asset_units"])
_RAIL_WIDTH_M = float(_RAIL_METADATA["width_m"])
_RAIL_DEPTH_M = float(_RAIL_METADATA["depth_m"])

_CARRIAGE_METADATA = _load_carriage_metadata()
_CARRIAGE_UNIT_SCALE = float(_CARRIAGE_METADATA["scene_unit_scale"])
_CARRIAGE_WIDTH_M = float(_CARRIAGE_METADATA.get("width_m", 0.06))
_CARRIAGE_HEIGHT_M = float(_CARRIAGE_METADATA.get("height_m", 0.024))
_CARRIAGE_DEPTH_M = float(_CARRIAGE_METADATA.get("depth_m", 0.08))

_DEPLOY_MOTOR_METADATA = _load_deploy_motor_metadata()
_DEPLOY_MOTOR_UNIT_SCALE = float(_DEPLOY_MOTOR_METADATA["scene_unit_scale"])
_DEPLOY_MOTOR_LENGTH_M = float(_DEPLOY_MOTOR_METADATA.get("length_m", 0.2))
_DEPLOY_MOTOR_WIDTH_M = float(_DEPLOY_MOTOR_METADATA.get("width_m", 0.12))
_DEPLOY_MOTOR_DEPTH_M = float(_DEPLOY_MOTOR_METADATA.get("depth_m", 0.11))
_DEPLOY_MOTOR_ROTATION_DEG = (
    ARGS.motor_rotate_x,
    ARGS.motor_rotate_y,
    ARGS.motor_rotate_z,
)
_DEPLOY_MOTOR_Y_OFFSET_M = 0.01
_GRID_BASE_Z_M = 0.025

# Populated by build_scene(), consumed in the main loop
_robot_translate_op = None
_robot_yaw_op = None
_robot_z = None

# Deployment drives — populated by build_scene()
_deploy_drives: list = []
_DEPLOY_ANGLE_DEG: float = 90.0  # max fold angle for column deployment

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


def _add_xyz_rotation_ops(xform: UsdGeom.Xform, rotation_deg: tuple[float, float, float]) -> None:
    rx, ry, rz = rotation_deg
    xform.AddRotateXOp().Set(float(rx))
    xform.AddRotateYOp().Set(float(ry))
    xform.AddRotateZOp().Set(float(rz))


def _add_rail(
    stage, path: str, world_pos: tuple, *,
    vertical: bool = False, kinematic: bool = True,
    mass_kg: float = 5.0, enable_collision: bool = True,
    visual_rotate_y_deg: float = -90.0,
) -> UsdGeom.Xform:
    """One horizontal_rail.usd segment placed in world space.

    vertical=False: long axis → World X  (Rz −90°)
    vertical=True:  long axis → World Z  (Rx +90°)
    Collider box dims are the same (width × length × depth) in the prim's local frame.
    """
    prim = UsdGeom.Xform.Define(stage, path)
    prim.AddTranslateOp().Set(Gf.Vec3d(*world_pos))
    if vertical:
        prim.AddRotateXOp().Set(90.0)   # asset Y → World Z
    else:
        prim.AddRotateZOp().Set(-90.0)  # asset Y → World X

    if kinematic:
        _make_rigid(prim.GetPrim(), kinematic=True)
    else:
        _make_rigid(prim.GetPrim(), mass_kg=mass_kg)

    if RAIL_ASSET_PATH.exists():
        vis = UsdGeom.Xform.Define(stage, f"{path}/visual")
        vis.AddScaleOp().Set(Gf.Vec3f(_RAIL_UNIT_SCALE, _RAIL_UNIT_SCALE, _RAIL_UNIT_SCALE))
        vis.AddRotateYOp().Set(visual_rotate_y_deg)
        asset_ref = UsdGeom.Xform.Define(stage, f"{path}/visual/asset")
        asset_ref.GetPrim().GetReferences().AddReference(str(RAIL_ASSET_PATH))

    coll = _create_box(
        stage, f"{path}/collider",
        (_RAIL_WIDTH_M, _RAIL_LENGTH_M, _RAIL_DEPTH_M),
        (0.0, 0.0, 0.0), (0.10, 0.28, 0.52),
    )
    if enable_collision:
        _add_collision(coll.GetPrim())
    UsdGeom.Imageable(coll.GetPrim()).MakeInvisible()
    return prim


def _create_h_rows(stage, config: MvpSceneConfig, base_z: float) -> None:
    """Kinematic horizontal rail rows.

    Row 3 is created as USD children of the upper column halves so it follows the
    deployment fold. This function only creates fixed rows.
    """
    y = -config.facade_standoff_m
    segs_per_row = round(config.grid_width_m / _RAIL_LENGTH_M)
    ox = -0.5 * config.grid_width_m + 0.5 * _RAIL_LENGTH_M
    for r in (0, 1):
        if r > config.grid_rows:
            continue
        z = base_z + r * config.module_height_m
        for s in range(segs_per_row):
            _add_rail(
                stage,
                f"/World/hrow{r}_seg{s}",
                (ox + s * _RAIL_LENGTH_M, y, z),
                visual_rotate_y_deg=0.0,
            )


def _create_v_columns(stage, config: MvpSceneConfig, base_z: float) -> list:
    """For each even column create lower (kinematic, articulation base) + upper (dynamic).

    ArticulationRootAPI goes on the kinematic LOWER half (fixed base).  The joint is
    placed outside both body prims to avoid USD hierarchy ambiguity.  A top horizontal
    rail visual is added as a USD child of each upper half so it follows the fold for free.
    Returns a list of DriveAPI objects, one per column.
    """
    drives = []
    half = config.grid_height_m / 2.0
    y = -config.facade_standoff_m
    ox = -0.5 * config.grid_width_m

    for c in range(config.grid_columns + 1):
        if c % 2 != 0:
            continue
        x = ox + c * config.module_width_m

        lower_path = f"/World/col{c}_lower"
        upper_path = f"/World/col{c}_upper"

        # Lower half: kinematic fixed base of this column's articulation
        _add_rail(stage, lower_path, (x, y, base_z + half / 2.0), vertical=True)
        UsdPhysics.ArticulationRootAPI.Apply(stage.GetPrimAtPath(lower_path))

        # Upper half: dynamic leaf — NO ArticulationRootAPI here (root is on lower)
        # Collision disabled so the fold isn't blocked by kinematic horizontal rails
        _add_rail(stage, upper_path, (x, y, base_z + half + half / 2.0), vertical=True,
                  kinematic=False, mass_kg=4.0, enable_collision=False)

        # Row 3 horizontal rail — visual-only child of upper half.
        # Placed at upper-local Y=0, i.e. the middle of the upper vertical rail.
        # Rz(-90°) in upper-local makes asset Y → upper-local X = World X.
        # As the upper half folds, the rail follows the deployment kinematics.
        if config.grid_rows >= 3 and c < config.grid_columns:
            h = UsdGeom.Xform.Define(stage, f"{upper_path}/hrow3_seg{c // 2}")
            h.AddTranslateOp().Set(Gf.Vec3f(1.0, 0.0, 0.0))
            h.AddRotateZOp().Set(-90.0)
            if RAIL_ASSET_PATH.exists():
                vis = UsdGeom.Xform.Define(stage, f"{upper_path}/hrow3_seg{c // 2}/visual")
                vis.AddScaleOp().Set(Gf.Vec3f(_RAIL_UNIT_SCALE, _RAIL_UNIT_SCALE, _RAIL_UNIT_SCALE))
                vis.AddRotateYOp().Set(-90.0)
                aref = UsdGeom.Xform.Define(stage, f"{upper_path}/hrow3_seg{c // 2}/visual/asset")
                aref.GetPrim().GetReferences().AddReference(str(RAIL_ASSET_PATH))

        lower_prim = stage.GetPrimAtPath(lower_path)
        upper_prim = stage.GetPrimAtPath(upper_path)

        # Joint is a sibling of both bodies at /World level — avoids USD hierarchy issues
        joint = UsdPhysics.RevoluteJoint.Define(stage, f"/World/col{c}_fold_joint")
        joint.CreateBody0Rel().SetTargets([lower_prim.GetPath()])
        joint.CreateBody1Rel().SetTargets([upper_prim.GetPath()])
        joint.CreateAxisAttr().Set("X")
        # local Y = World Z for both bodies (Rx 90° applied), so half/2 in local Y = top/bottom
        joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, half / 2.0, 0.0))
        joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, -half / 2.0, 0.0))
        joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0))
        joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0))
        joint.CreateLowerLimitAttr().Set(0.0)
        joint.CreateUpperLimitAttr().Set(0.0)
        joint.CreateCollisionEnabledAttr().Set(False)

        drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "angular")
        drive.CreateTypeAttr().Set("force")
        drive.CreateTargetVelocityAttr().Set(0.0)
        drive.CreateStiffnessAttr().Set(0.0)
        drive.CreateDampingAttr().Set(50.0)
        drive.CreateMaxForceAttr().Set(300.0)

        drives.append(drive)

        # Deployment motor visual — CAD asset at the fold junction.
        # It is offset toward the viewer so it does not hide inside the rail mesh.
        motor_path = f"/World/col{c}_motor_vis"
        motor_xf = UsdGeom.Xform.Define(stage, motor_path)
        motor_y = y + _DEPLOY_MOTOR_Y_OFFSET_M
        motor_xf.AddTranslateOp().Set(Gf.Vec3d(x, motor_y, base_z + half))
        _add_xyz_rotation_ops(motor_xf, _DEPLOY_MOTOR_ROTATION_DEG)
        if DEPLOY_MOTOR_ASSET_PATH.exists():
            vis = UsdGeom.Xform.Define(stage, f"{motor_path}/visual")
            vis.AddScaleOp().Set(Gf.Vec3f(
                _DEPLOY_MOTOR_UNIT_SCALE,
                _DEPLOY_MOTOR_UNIT_SCALE,
                _DEPLOY_MOTOR_UNIT_SCALE,
            ))
            aref = UsdGeom.Xform.Define(stage, f"{motor_path}/visual/asset")
            aref.GetPrim().GetReferences().AddReference(str(DEPLOY_MOTOR_ASSET_PATH))
        else:
            motor_sz = _RAIL_WIDTH_M * 1.4
            _create_box(
                stage, f"{motor_path}/fallback",
                (0.22, motor_sz, motor_sz),
                (0.0, 0.0, 0.0),
                (0.95, 0.48, 0.02),
            )

    print(f"[deploy] {len(drives)} column fold joints created", flush=True)
    return drives


def _grid_local_to_world(config: MvpSceneConfig, grid_center_z: float, local):
    x, y, z = local
    return (x, -config.facade_standoff_m - z, grid_center_z + y)


def _quat_x(degrees: float) -> Gf.Quatf:
    half_angle = math.radians(degrees) * 0.5
    return Gf.Quatf(
        math.cos(half_angle),
        Gf.Vec3f(math.sin(half_angle), 0.0, 0.0),
    )


def _create_physical_carriage(stage, config: MvpSceneConfig, grid_center_z: float):
    if not CARRIAGE_ASSET_PATH.exists():
        return None
    rail_column = 2
    rail_x = -0.5 * config.grid_width_m + rail_column * config.module_width_m
    rail_local_z = 0.5 * (_RAIL_DEPTH_M + _CARRIAGE_DEPTH_M)
    rail_local = (rail_x, 0.0, rail_local_z)
    carriage_world = _grid_local_to_world(config, grid_center_z, rail_local)

    path = CARRIAGE_PATH
    carriage = UsdGeom.Xform.Define(stage, path)
    carriage.AddTranslateOp().Set(Gf.Vec3d(*carriage_world))
    _make_rigid(carriage.GetPrim(), 2.0)

    visual = UsdGeom.Xform.Define(stage, f"{path}/visual")
    visual.AddScaleOp().Set(
        Gf.Vec3f(_CARRIAGE_UNIT_SCALE, _CARRIAGE_UNIT_SCALE, _CARRIAGE_UNIT_SCALE)
    )
    asset = UsdGeom.Xform.Define(stage, f"{path}/visual/asset")
    asset.AddTranslateOp().Set(Gf.Vec3d(0.0, 119.0, 0.0))
    asset.AddRotateYOp().Set(90.0)
    asset.GetPrim().GetReferences().AddReference(str(CARRIAGE_ASSET_PATH))

    collider = _create_box(
        stage,
        f"{path}/collider",
        (_CARRIAGE_WIDTH_M, _CARRIAGE_HEIGHT_M, _CARRIAGE_DEPTH_M),
        (0.0, 0.0, 0.0),
        (0.8, 0.15, 0.15),
    )
    _add_collision(collider.GetPrim())
    UsdGeom.Imageable(collider.GetPrim()).MakeInvisible()

    joint = UsdPhysics.PrismaticJoint.Define(stage, f"{path}/vertical_slide_joint")
    joint.CreateBody0Rel().SetTargets([stage.GetPrimAtPath(GRID_PATH).GetPath()])
    joint.CreateBody1Rel().SetTargets([carriage.GetPrim().GetPath()])
    joint.CreateAxisAttr().Set("Z")
    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(*rail_local))
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    joint.CreateLocalRot0Attr().Set(_quat_x(-90.0))
    joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0))
    joint.CreateLowerLimitAttr().Set(-0.5 * config.grid_height_m)
    joint.CreateUpperLimitAttr().Set(0.5 * config.grid_height_m)
    joint.CreateCollisionEnabledAttr().Set(False)

    drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "linear")
    drive.CreateTypeAttr().Set("force")
    drive.CreateTargetVelocityAttr().Set(0.0)
    drive.CreateDampingAttr().Set(150.0)
    drive.CreateStiffnessAttr().Set(0.0)
    drive.CreateMaxForceAttr().Set(250.0)
    return carriage


def set_deploy_velocity(velocity_degs: float) -> None:
    """Command all column fold joints: +v folds outward (deg/s), -v folds back, 0 holds."""
    for drive in _deploy_drives:
        drive.GetTargetVelocityAttr().Set(float(velocity_degs))


def build_scene(config: MvpSceneConfig):
    global _robot_translate_op, _robot_yaw_op, _robot_z, _deploy_drives

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

    # ── Grid anchor (kinematic, no visual — used by carriage joint) ──────────
    grid_center_z = _GRID_BASE_Z_M + config.grid_height_m / 2.0
    grid = _create_xform(stage, GRID_PATH,
                         (0.0, -config.facade_standoff_m, grid_center_z))
    grid.AddRotateXOp().Set(90.0)
    _make_rigid(grid.GetPrim(), config.grid_mass_kg, kinematic=True)

    # ── Horizontal rail rows (kinematic surface rails) ────────────────────────
    _create_h_rows(stage, config, _GRID_BASE_Z_M)

    # ── Vertical columns (lower kinematic + upper dynamic with fold motor) ────
    _deploy_drives = _create_v_columns(stage, config, _GRID_BASE_Z_M)

    _create_physical_carriage(stage, config, grid_center_z)

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

        set_deploy_velocity(0.0)

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
