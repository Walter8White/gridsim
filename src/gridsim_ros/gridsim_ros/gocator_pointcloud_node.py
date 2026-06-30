"""Publish a live Gocator-like facade point cloud for RViz."""

from __future__ import annotations

from pathlib import Path
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


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
    Gocator2690LineProfiler,
    Gocator2690Spec,
    GocatorEncoderTriggeredAcquisition,
    GocatorPointCloudAccumulator,
    ScannerFramePose,
)

FACADE_WIDTH_M = 10.0
FACADE_HEIGHT_M = 10.0
SENSOR_STANDOFF_M = 1.0


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
        self.declare_parameter("frame_id", "world")
        self.declare_parameter("points_per_profile", 800)
        self.declare_parameter("profile_spacing_m", 0.01)
        self.declare_parameter("scan_speed_m_s", 0.25)
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("max_points", 250000)
        self.declare_parameter("loop_scan", True)

        self.frame_id = str(self.get_parameter("frame_id").value)
        self.scan_speed_m_s = float(self.get_parameter("scan_speed_m_s").value)
        self.max_points = int(self.get_parameter("max_points").value)
        self.loop_scan = bool(self.get_parameter("loop_scan").value)
        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)

        spec = Gocator2690Spec(
            points_per_profile=int(self.get_parameter("points_per_profile").value),
            profile_spacing_m=float(self.get_parameter("profile_spacing_m").value),
            nominal_standoff_m=SENSOR_STANDOFF_M,
        )
        self.profiler = Gocator2690LineProfiler(spec)
        self.scan_width_m = float(spec.width_at_distance(SENSOR_STANDOFF_M))
        self.centers = _pass_centers(self.scan_width_m)
        self.acquisition = GocatorEncoderTriggeredAcquisition(self.profiler, self._sample_profile)
        self.accumulator = GocatorPointCloudAccumulator(spec.profile_spacing_m)

        self.pass_index = 0
        self.z_m = 0.0
        self.direction = 1.0
        self.last_time_s = None
        self.publisher = self.create_publisher(PointCloud2, "gocator/points", 2)
        self.timer = self.create_timer(1.0 / publish_rate_hz, self._tick)
        self.get_logger().info(
            f"Publishing /gocator/points in frame '{self.frame_id}' with {self.scan_width_m:.3f} m scan width"
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
        return _scanner_pose(self.centers[self.pass_index], self.z_m)

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

    def _tick(self) -> None:
        now = self.get_clock().now()
        now_s = now.nanoseconds * 1e-9
        if self.last_time_s is None:
            self.last_time_s = now_s
            pose = _scanner_pose(self.centers[self.pass_index], self.z_m)
        else:
            dt_s = max(0.0, now_s - self.last_time_s)
            self.last_time_s = now_s
            pose = self._advance_pose(dt_s)

        for profile in self.acquisition.update(pose, now_s):
            self.accumulator.add_profile(profile)

        cloud = self.accumulator.point_cloud()
        if self.max_points > 0 and len(cloud) > self.max_points:
            stride = int(np.ceil(len(cloud) / self.max_points))
            cloud = cloud[::stride]
        self.publisher.publish(_pointcloud2(cloud, self.frame_id, now.to_msg()))


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
