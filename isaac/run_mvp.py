#!/usr/bin/env python3
"""Build and run the first config-driven gridsim scene in Isaac Sim 6."""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--test", action="store_true")
    parser.add_argument("--no-lidar", action="store_true")
    parser.add_argument("--realtime", action="store_true")
    parser.add_argument("--report-grid-pose", action="store_true")
    parser.add_argument("--frames", type=int, default=0)
    parser.add_argument("--output", type=Path)
    args, _ = parser.parse_known_args()
    return args


ARGS = parse_args()

from isaacsim import SimulationApp


HEADLESS = ARGS.headless or ARGS.test
PHYSICS_ONLY = HEADLESS and ARGS.no_lidar
simulation_app = SimulationApp(
    {
        "headless": HEADLESS,
        "renderer": "MinimalRendering" if PHYSICS_ONLY else "RayTracedLighting",
        "minimal_shading_mode": 4 if PHYSICS_ONLY else 0,
        "disable_viewport_updates": PHYSICS_ONLY,
    }
)

import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import numpy as np
import omni.graph.core as og
import omni.usd
import usdrt
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.sensors.experimental.physics import IMU
from pxr import Gf, UsdGeom, UsdLux, UsdPhysics


PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "src"))

from gridsim_core.scene_config import MvpSceneConfig


FACADE_PATH = "/World/facade"
GROUND_PATH = "/World/ground"
GRID_PATH = "/World/grid"
ROBOT_PATH = f"{GRID_PATH}/robot_base"
TOOL_PATH = f"{ROBOT_PATH}/tool"
GRID_IMU_PATH = f"{GRID_PATH}/grid_imu"
ROBOT_IMU_PATH = f"{ROBOT_PATH}/robot_imu"
LIDAR_PATH = f"{ROBOT_PATH}/robot_lidar"


def create_xform(stage, path: str, translation=(0.0, 0.0, 0.0)):
    xform = UsdGeom.Xform.Define(stage, path)
    xform.AddTranslateOp().Set(Gf.Vec3d(*translation))
    return xform


def create_box(
    stage,
    path: str,
    size: tuple[float, float, float],
    translation: tuple[float, float, float],
    color: tuple[float, float, float],
):
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    cube.AddTranslateOp().Set(Gf.Vec3d(*translation))
    cube.AddScaleOp().Set(Gf.Vec3f(*size))
    cube.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    return cube


def add_collision(prim) -> None:
    UsdPhysics.CollisionAPI.Apply(prim)


def set_mass(prim, mass_kg: float) -> None:
    mass_api = UsdPhysics.MassAPI.Apply(prim)
    mass_api.CreateMassAttr(float(mass_kg))


def make_rigid_body(prim, mass_kg: float | None = None, *, kinematic: bool = False) -> None:
    rigid_body = UsdPhysics.RigidBodyAPI.Apply(prim)
    rigid_body.CreateKinematicEnabledAttr(kinematic)
    if mass_kg is not None:
        set_mass(prim, mass_kg)


def make_kinematic(prim, mass_kg: float | None = None) -> None:
    make_rigid_body(prim, mass_kg, kinematic=True)


def build_scene(config: MvpSceneConfig):
    stage_utils.create_new_stage()
    stage_utils.set_stage_units(meters_per_unit=1.0)
    stage = omni.usd.get_context().get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    stage.DefinePrim("/World", "Xform")
    stage.SetDefaultPrim(stage.GetPrimAtPath("/World"))

    physics_scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    physics_scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
    physics_scene.CreateGravityMagnitudeAttr().Set(9.81)

    dome = UsdLux.DomeLight.Define(stage, "/World/DomeLight")
    dome.CreateIntensityAttr(500.0)
    sun = UsdLux.DistantLight.Define(stage, "/World/Sun")
    sun.CreateIntensityAttr(1200.0)
    sun.CreateAngleAttr(1.0)

    ground = create_box(
        stage,
        GROUND_PATH,
        (12.0, 8.0, 0.05),
        (0.0, 0.0, -0.025),
        (0.22, 0.24, 0.25),
    )
    add_collision(ground.GetPrim())

    facade = UsdGeom.Xform.Define(stage, FACADE_PATH)
    facade.AddTranslateOp().Set(
        Gf.Vec3d(0.0, 0.0, config.facade_height_m / 2.0)
    )
    facade.AddRotateXOp().Set(90.0)
    make_kinematic(facade.GetPrim(), config.facade_mass_kg)
    facade_surface = create_box(
        stage,
        f"{FACADE_PATH}/surface",
        (config.facade_width_m, config.facade_height_m, 0.1),
        (0.0, 0.0, -0.05),
        (0.55, 0.58, 0.62),
    )
    add_collision(facade_surface.GetPrim())

    grid = create_xform(
        stage,
        GRID_PATH,
        (0.0, -config.facade_standoff_m, config.facade_height_m / 2.0),
    )
    grid.AddRotateXOp().Set(90.0)
    make_rigid_body(grid.GetPrim(), config.grid_mass_kg)
    module_origin_x = -0.5 * config.grid_width_m
    module_origin_y = -0.5 * config.grid_height_m
    bar_width = min(config.module_width_m, config.module_height_m) * 0.08
    for column in range(config.grid_columns + 1):
        center_x = module_origin_x + column * config.module_width_m
        bar = create_box(
            stage,
            f"{GRID_PATH}/frame/vertical_{column}",
            (bar_width, config.grid_height_m + bar_width, config.module_depth_m),
            (center_x, 0.0, 0.0),
            (0.12, 0.32, 0.58),
        )
        add_collision(bar.GetPrim())
    for row in range(config.grid_rows + 1):
        center_y = module_origin_y + row * config.module_height_m
        bar = create_box(
            stage,
            f"{GRID_PATH}/frame/horizontal_{row}",
            (config.grid_width_m + bar_width, bar_width, config.module_depth_m),
            (0.0, center_y, 0.0),
            (0.10, 0.28, 0.52),
        )
        add_collision(bar.GetPrim())

    robot = create_xform(stage, ROBOT_PATH, (0.0, 0.0, 0.15))
    body = create_box(
        stage,
        f"{ROBOT_PATH}/body",
        (0.55, 0.42, 0.30),
        (0.0, 0.0, 0.0),
        (0.9, 0.45, 0.08),
    )
    nose = create_box(
        stage,
        f"{ROBOT_PATH}/facade_facing_nose",
        (0.18, 0.14, 0.16),
        (0.0, 0.0, -0.22),
        (0.05, 0.05, 0.05),
    )
    create_xform(stage, TOOL_PATH, (0.0, 0.0, -0.34))

    IMU.create(
        GRID_IMU_PATH,
        translations=np.array([config.grid_imu_translation_m]),
    )
    IMU.create(
        ROBOT_IMU_PATH,
        translations=np.array([config.robot_imu_translation_m]),
    )
    return stage


def create_ros_graph() -> None:
    keys = og.Controller.Keys
    tf_edges = [
        ("Facade", "", FACADE_PATH),
        ("Grid", FACADE_PATH, GRID_PATH),
        ("Robot", GRID_PATH, ROBOT_PATH),
        ("Tool", ROBOT_PATH, TOOL_PATH),
    ]
    tf_nodes = []
    tf_connections = []
    tf_values = []
    for name, parent_path, child_path in tf_edges:
        compute_name = f"Compute{name}TF"
        publish_name = f"Publish{name}TF"
        tf_nodes.extend(
            [
                (compute_name, "isaacsim.core.nodes.IsaacComputeTransformTree"),
                (publish_name, "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
            ]
        )
        tf_connections.extend(
            [
                ("OnPlaybackTick.outputs:tick", f"{compute_name}.inputs:execIn"),
                (f"{compute_name}.outputs:execOut", f"{publish_name}.inputs:execIn"),
                (
                    f"{compute_name}.outputs:parentFrames",
                    f"{publish_name}.inputs:parentFrames",
                ),
                (
                    f"{compute_name}.outputs:childFrames",
                    f"{publish_name}.inputs:childFrames",
                ),
                (
                    f"{compute_name}.outputs:translations",
                    f"{publish_name}.inputs:translations",
                ),
                (
                    f"{compute_name}.outputs:orientations",
                    f"{publish_name}.inputs:orientations",
                ),
                (
                    "ReadSimTime.outputs:simulationTime",
                    f"{publish_name}.inputs:timeStamp",
                ),
            ]
        )
        tf_values.extend(
            [
                (f"{compute_name}.inputs:targetPrims", [usdrt.Sdf.Path(child_path)]),
                (f"{publish_name}.inputs:topicName", "/tf"),
            ]
        )
        if parent_path:
            tf_values.append(
                (
                    f"{compute_name}.inputs:parentPrim",
                    [usdrt.Sdf.Path(parent_path)],
                )
            )

    og.Controller.edit(
        {"graph_path": "/World/ROSGraph", "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
                ("ReadGridIMU", "isaacsim.sensors.physics.IsaacReadIMU"),
                ("PublishGridIMU", "isaacsim.ros2.bridge.ROS2PublishImu"),
                ("ReadRobotIMU", "isaacsim.sensors.physics.IsaacReadIMU"),
                ("PublishRobotIMU", "isaacsim.ros2.bridge.ROS2PublishImu"),
            ]
            + tf_nodes,
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
                ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
                ("OnPlaybackTick.outputs:tick", "ReadGridIMU.inputs:execIn"),
                ("ReadGridIMU.outputs:execOut", "PublishGridIMU.inputs:execIn"),
                ("ReadGridIMU.outputs:sensorTime", "PublishGridIMU.inputs:timeStamp"),
                ("ReadGridIMU.outputs:linAcc", "PublishGridIMU.inputs:linearAcceleration"),
                ("ReadGridIMU.outputs:angVel", "PublishGridIMU.inputs:angularVelocity"),
                ("ReadGridIMU.outputs:orientation", "PublishGridIMU.inputs:orientation"),
                ("OnPlaybackTick.outputs:tick", "ReadRobotIMU.inputs:execIn"),
                ("ReadRobotIMU.outputs:execOut", "PublishRobotIMU.inputs:execIn"),
                ("ReadRobotIMU.outputs:sensorTime", "PublishRobotIMU.inputs:timeStamp"),
                ("ReadRobotIMU.outputs:linAcc", "PublishRobotIMU.inputs:linearAcceleration"),
                ("ReadRobotIMU.outputs:angVel", "PublishRobotIMU.inputs:angularVelocity"),
                ("ReadRobotIMU.outputs:orientation", "PublishRobotIMU.inputs:orientation"),
            ]
            + tf_connections,
            keys.SET_VALUES: [
                ("PublishClock.inputs:topicName", "/clock"),
                ("PublishGridIMU.inputs:topicName", "/grid/imu/raw"),
                ("PublishGridIMU.inputs:frameId", "grid_imu"),
                ("PublishRobotIMU.inputs:topicName", "/robot/imu/raw"),
                ("PublishRobotIMU.inputs:frameId", "robot_imu"),
            ]
            + tf_values,
        },
    )
    og.Controller.set(
        og.Controller.attribute("/World/ROSGraph/ReadGridIMU.inputs:imuPrim"),
        [usdrt.Sdf.Path(GRID_IMU_PATH)],
    )
    og.Controller.set(
        og.Controller.attribute("/World/ROSGraph/ReadRobotIMU.inputs:imuPrim"),
        [usdrt.Sdf.Path(ROBOT_IMU_PATH)],
    )


def create_lidar(config: MvpSceneConfig):
    from isaacsim.sensors.experimental.rtx import Lidar, LidarSensor

    lidar = Lidar.create(
        path=LIDAR_PATH,
        config="Example_Rotary",
        tick_rate=config.lidar_rate_hz,
        translations=[config.lidar_translation_m],
    )
    sensor = LidarSensor(lidar, annotators=[])
    sensor.attach_writer(
        "RtxLidarROS2PublishPointCloud",
        topicName="/robot/lidar/points",
        frameId="robot_lidar",
    )
    return sensor


def main() -> int:
    config = MvpSceneConfig.from_directory(PROJECT_ROOT / "configs")
    app_utils.enable_extension("isaacsim.ros2.bridge")
    simulation_app.update()

    stage = build_scene(config)
    create_ros_graph()
    lidar_sensor = None if ARGS.no_lidar else create_lidar(config)

    output_path = ARGS.output or PROJECT_ROOT / "outputs/isaac/mvp_scene.usda"
    output_path = output_path.resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    stage.GetRootLayer().Export(str(output_path))
    print(f"Saved MVP stage: {output_path}")

    SimulationManager.setup_simulation(
        dt=1.0 / config.simulation_frequency_hz,
        device="cpu",
    )
    grid_prim = stage.GetPrimAtPath(GRID_PATH)
    start_grid_height_m = None
    if ARGS.report_grid_pose:
        cache = UsdGeom.XformCache()
        start_grid_height_m = cache.GetLocalToWorldTransform(grid_prim).ExtractTranslation()[2]

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
    if ARGS.report_grid_pose:
        cache = UsdGeom.XformCache()
        end_grid_height_m = cache.GetLocalToWorldTransform(grid_prim).ExtractTranslation()[2]
        print(
            "Grid world height: "
            f"{start_grid_height_m:.4f} m -> {end_grid_height_m:.4f} m"
        )
    del lidar_sensor
    print(f"Completed {frame_count} simulation frames")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    finally:
        simulation_app.close()
