"""Publish a live Gocator-like facade point cloud for RViz."""

from __future__ import annotations

from pathlib import Path
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import yaml


def _add_project_src_to_path() -> None:
    candidates = [
        Path.cwd() / "src",
        Path(__file__).resolve().parents[4] / "src",
    ]
    for candidate in candidates:
        if (candidate / "gridsim_sensors").exists() and str(candidate) not in sys.path:
            sys.path.insert(0, str(candidate))


_add_project_src_to_path()

from gridsim_sensors import (  # noqa: E402
    GocatorProfile,
    Gocator2690LineProfiler,
    Gocator2690Spec,
    GocatorEncoderTriggeredAcquisition,
    GocatorPointCloudAccumulator,
    ScannerFramePose,
)

FACADE_WIDTH_M = 10.0
FACADE_HEIGHT_M = 10.0


def _find_project_root() -> Path:
    for parent in (Path.cwd(), *Path(__file__).resolve().parents):
        if (parent / "configs" / "sensors.yaml").exists() and (parent / "src").exists():
            return parent
    return Path.cwd()


PROJECT_ROOT = _find_project_root()


def _load_gocator_config() -> dict:
    path = PROJECT_ROOT / "configs" / "sensors.yaml"
    if not path.exists():
        return {}
    with path.open("r", encoding="utf-8") as fp:
        data = yaml.safe_load(fp) or {}
    config = data.get("gocator2690", {})
    return config if isinstance(config, dict) else {}


def _synthetic_facade_y(x_m: np.ndarray, z_m: np.ndarray) -> np.ndarray:
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
    ):
        defects += amp * np.exp(-(((x - cx) / sx) ** 2 + ((z - cz) / sz) ** 2))

    joints = np.zeros_like(defects)
    for joint_x, phase in ((-3.2, 0.0), (-1.1, 0.8), (1.2, 1.9), (3.4, 2.7)):
        center = joint_x + 0.035 * np.sin(1.7 * z + phase) + 0.012 * np.sin(6.5 * z + phase)
        width = 0.018 + 0.010 * (0.5 + 0.5 * np.sin(3.1 * z + phase))
        joints = np.where(np.abs(x - center) < width, joints + 0.009, joints)
    return bow + waves + defects + joints


def _scanner_pose(x_m: float, z_m: float, standoff_m: float) -> ScannerFramePose:
    return ScannerFramePose.from_arrays(
        [x_m, -standoff_m, z_m],
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


def _pass_centers(scan_width_m: float) -> list[float]:
    first = -FACADE_WIDTH_M / 2.0 + scan_width_m / 2.0
    last = FACADE_WIDTH_M / 2.0 - scan_width_m / 2.0
    centers = []
    x = first
    while x <= last + 1e-9:
        centers.append(float(x))
        x += scan_width_m
    if not centers or centers[-1] < last - 1e-6:
        centers.append(float(last))
    return centers


def _pointcloud2(points: np.ndarray, frame_id: str, stamp) -> PointCloud2:
    points32 = np.asarray(points, dtype=np.float32)
    message = PointCloud2()
    message.header = Header(stamp=stamp, frame_id=frame_id)
    message.height = 1
    message.width = int(len(points32))
    message.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    message.is_bigendian = False
    message.point_step = 12
    message.row_step = message.point_step * message.width
    message.is_dense = True
    message.data = points32.tobytes()
    return message


class GocatorPointCloudNode(Node):
    """Simulate vertical Gocator passes and publish accumulated points."""

    def __init__(self) -> None:
        super().__init__("gocator_pointcloud_node")
        config = _load_gocator_config()
        default_spec = Gocator2690Spec()

        self.declare_parameter("frame_id", "world")
        self.declare_parameter("points_per_profile", int(config.get("points_per_profile", default_spec.points_per_profile)))
        self.declare_parameter(
            "profile_spacing_m",
            float(config.get("profile_spacing_m", default_spec.profile_spacing_m)),
        )
        self.declare_parameter(
            "nominal_profile_rate_hz",
            float(config.get("nominal_profile_rate_hz", default_spec.nominal_profile_rate_hz)),
        )
        self.declare_parameter(
            "nominal_standoff_m",
            float(config.get("nominal_standoff_mm", default_spec.nominal_standoff_m * 1000.0)) * 0.001,
        )
        self.declare_parameter("scan_speed_m_s", 0.0)
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("max_points", 500000)
        self.declare_parameter("publish_current_profile", True)
        self.declare_parameter("loop_scan", True)

        self.frame_id = str(self.get_parameter("frame_id").value)
        self.max_points = int(self.get_parameter("max_points").value)
        self.publish_current_profile = bool(self.get_parameter("publish_current_profile").value)
        self.loop_scan = bool(self.get_parameter("loop_scan").value)
        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        nominal_profile_rate_hz = float(self.get_parameter("nominal_profile_rate_hz").value)
        profile_spacing_m = float(self.get_parameter("profile_spacing_m").value)
        requested_scan_speed_m_s = float(self.get_parameter("scan_speed_m_s").value)
        self.nominal_standoff_m = float(self.get_parameter("nominal_standoff_m").value)
        self.scan_speed_m_s = (
            requested_scan_speed_m_s
            if requested_scan_speed_m_s > 0.0
            else nominal_profile_rate_hz * profile_spacing_m
        )

        spec = Gocator2690Spec(
            points_per_profile=int(self.get_parameter("points_per_profile").value),
            profile_spacing_m=profile_spacing_m,
            nominal_standoff_m=self.nominal_standoff_m,
            nominal_profile_rate_hz=nominal_profile_rate_hz,
        )
        self.profiler = Gocator2690LineProfiler(spec)
        self.scan_width_m = float(spec.width_at_distance(spec.nominal_standoff_m))
        self.centers = _pass_centers(self.scan_width_m)
        self.acquisition = GocatorEncoderTriggeredAcquisition(self.profiler, self._sample_profile)
        self.accumulator = GocatorPointCloudAccumulator(spec.profile_spacing_m)

        self.pass_index = 0
        self.z_m = 0.0
        self.direction = 1.0
        self.last_time_s = None
        self.profile_publisher = self.create_publisher(PointCloud2, "gocator/profile_points", 2)
        self.publisher = self.create_publisher(PointCloud2, "gocator/points", 2)
        self.acquisition_timer = self.create_timer(1.0 / nominal_profile_rate_hz, self._acquire_tick)
        self.publish_timer = self.create_timer(1.0 / publish_rate_hz, self._publish_tick)
        self.get_logger().info(
            f"Gocator specs: {spec.points_per_profile} pts/profile, "
            f"{nominal_profile_rate_hz:.1f} profiles/s, spacing={spec.profile_spacing_m:.6f} m, "
            f"scan_speed={self.scan_speed_m_s:.4f} m/s, width_at_standoff={self.scan_width_m:.3f} m"
        )

    def _sample_profile(self, pose: ScannerFramePose, timestamp_s: float, profile_index: int, encoder_position_m: float):
        profile = self.profiler.sample_surface(
            pose,
            _synthetic_facade_y,
            timestamp_s=timestamp_s,
            profile_index=profile_index,
            encoder_position_m=encoder_position_m,
        )
        profile.valid_mask &= _wall_valid_mask(profile.points_world)
        return profile

    def _advance_pose(self, dt_s: float) -> ScannerFramePose:
        top_z = FACADE_HEIGHT_M + 0.001
        self.z_m += self.direction * self.scan_speed_m_s * dt_s
        if self.direction > 0.0 and self.z_m >= top_z:
            self.z_m = top_z
            self._next_pass()
        elif self.direction < 0.0 and self.z_m <= 0.0:
            self.z_m = 0.0
            self._next_pass()
        return _scanner_pose(self.centers[self.pass_index], self.z_m, self.nominal_standoff_m)

    def _next_pass(self) -> None:
        if self.pass_index + 1 >= len(self.centers):
            if not self.loop_scan:
                return
            self.pass_index = 0
            self.z_m = 0.0
            self.direction = 1.0
            self.accumulator.profiles.clear()
            self.acquisition = GocatorEncoderTriggeredAcquisition(self.profiler, self._sample_profile)
            return
        self.pass_index += 1
        self.direction *= -1.0

    def _acquire_tick(self) -> None:
        now = self.get_clock().now()
        now_s = now.nanoseconds * 1e-9
        if self.last_time_s is None:
            self.last_time_s = now_s
            pose = _scanner_pose(self.centers[self.pass_index], self.z_m, self.nominal_standoff_m)
        else:
            dt_s = max(0.0, now_s - self.last_time_s)
            self.last_time_s = now_s
            pose = self._advance_pose(dt_s)

        for profile in self.acquisition.update(pose, now_s):
            self.accumulator.add_profile(profile)
            if self.publish_current_profile:
                self.profile_publisher.publish(
                    _pointcloud2(profile.valid_points_m, self.frame_id, now.to_msg())
                )

    def _publish_tick(self) -> None:
        now = self.get_clock().now()
        cloud = self._preview_cloud()
        self.publisher.publish(_pointcloud2(cloud, self.frame_id, now.to_msg()))

    def _preview_cloud(self) -> np.ndarray:
        profiles = self.accumulator.profiles
        if not profiles:
            return np.empty((0, 3), dtype=np.float64)
        total_valid = sum(int(profile.valid_mask.sum()) for profile in profiles)
        if self.max_points <= 0 or total_valid <= self.max_points:
            return self.accumulator.point_cloud()

        # Preserve every vertical profile to avoid fake horizontal gaps in RViz.
        # Downsample within each profile instead; the full-resolution profile is
        # still published separately on /gocator/profile_points.
        per_profile_budget = max(2, self.max_points // len(profiles))
        sampled_profiles = [
            _sample_profile_points(profile, per_profile_budget)
            for profile in profiles
        ]
        sampled_profiles = [points for points in sampled_profiles if len(points)]
        if not sampled_profiles:
            return np.empty((0, 3), dtype=np.float64)
        return np.vstack(sampled_profiles)


def _sample_profile_points(profile: GocatorProfile, max_points: int) -> np.ndarray:
    points = profile.valid_points_m
    if len(points) <= max_points:
        return points
    indices = np.linspace(0, len(points) - 1, max_points, dtype=np.int64)
    return points[indices]


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = GocatorPointCloudNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
