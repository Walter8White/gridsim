"""Functional Gocator 2690 laser-line profiler model.

The model is intentionally geometry-first: it consumes the scanner frame pose
from the Isaac/USD scene and produces one horizontal profile by intersecting a
Gocator-like ray pattern with the facade. It does not use the detailed CAD mesh
for measurement simulation.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable

import numpy as np


@dataclass(frozen=True)
class Gocator2690Spec:
    points_per_profile: int = 3700
    clearance_distance_m: float = 0.325
    measurement_range_m: float = 1.550
    fov_near_m: float = 0.385
    fov_far_m: float = 2.000
    nominal_standoff_m: float = 1.000
    nominal_profile_rate_hz: float = 40.0
    profile_spacing_m: float = 0.0005

    @property
    def min_measurement_distance_m(self) -> float:
        return self.clearance_distance_m

    @property
    def max_measurement_distance_m(self) -> float:
        return self.clearance_distance_m + self.measurement_range_m

    def width_at_distance(self, distance_m: float | np.ndarray) -> float | np.ndarray:
        """Linear Gocator frustum width at a scanner-frame +Z distance."""
        distance = np.asarray(distance_m)
        alpha = (
            (distance - self.min_measurement_distance_m)
            / (self.max_measurement_distance_m - self.min_measurement_distance_m)
        )
        width = self.fov_near_m + alpha * (self.fov_far_m - self.fov_near_m)
        if np.isscalar(distance_m):
            return float(width)
        return width


@dataclass(frozen=True)
class ScannerFramePose:
    origin_m: np.ndarray
    x_axis: np.ndarray
    y_axis: np.ndarray
    z_axis: np.ndarray

    @classmethod
    def from_arrays(
        cls,
        origin_m,
        x_axis,
        y_axis,
        z_axis,
    ) -> "ScannerFramePose":
        return cls(
            origin_m=_as_unit_array(origin_m, normalize=False),
            x_axis=_as_unit_array(x_axis),
            y_axis=_as_unit_array(y_axis),
            z_axis=_as_unit_array(z_axis),
        )

    @property
    def rotation_world_from_scanner(self) -> np.ndarray:
        return np.column_stack((self.x_axis, self.y_axis, self.z_axis))

    def world_to_scanner(self, points_m: np.ndarray) -> np.ndarray:
        return (points_m - self.origin_m) @ self.rotation_world_from_scanner


@dataclass
class GocatorProfile:
    timestamp_s: float
    profile_index: int
    encoder_position_m: float
    scanner_pose_world: ScannerFramePose
    x_m: np.ndarray
    z_m: np.ndarray
    valid_mask: np.ndarray
    hit_normals_world: np.ndarray
    points_world: np.ndarray

    @property
    def valid_points_m(self) -> np.ndarray:
        return self.points_world[self.valid_mask]

    @property
    def local_x_m(self) -> np.ndarray:
        return self.x_m

    @property
    def hit_points_m(self) -> np.ndarray:
        return self.points_world

    @property
    def hit_normals_m(self) -> np.ndarray:
        return self.hit_normals_world

    @property
    def ranges_m(self) -> np.ndarray:
        return self.z_m

    @property
    def valid(self) -> np.ndarray:
        return self.valid_mask


@dataclass
class GocatorPointCloudAccumulator:
    """Accumulate profiles into a point cloud and coarse height map."""

    profile_spacing_m: float = 0.0005
    profiles: list[GocatorProfile] = field(default_factory=list)

    def add_profile(self, profile: GocatorProfile) -> None:
        self.profiles.append(profile)

    def point_cloud(self) -> np.ndarray:
        points = [profile.valid_points_m for profile in self.profiles if profile.valid_points_m.size]
        if not points:
            return np.empty((0, 3), dtype=np.float64)
        return np.vstack(points)

    def height_map(self, x_resolution_m: float = 0.005) -> dict[tuple[int, int], float]:
        """Return a sparse map keyed by quantized facade X/Z cells.

        For the current flat-facade MVP, the stored value is world Y depth. This
        gives us a simple height/depth map that can later be replaced by a
        richer raster once facade normals and non-planar meshes are present.
        """
        cloud = self.point_cloud()
        if cloud.size == 0:
            return {}
        x_bins = np.floor(cloud[:, 0] / x_resolution_m).astype(int)
        z_bins = np.floor(cloud[:, 2] / self.profile_spacing_m).astype(int)
        sparse: dict[tuple[int, int], float] = {}
        for key, y in zip(zip(x_bins, z_bins), cloud[:, 1]):
            sparse[key] = float(y)
        return sparse

    def height_map_array(
        self,
        *,
        x_resolution_m: float = 0.005,
        y_resolution_m: float | None = None,
        fill_value: float = np.nan,
    ) -> np.ndarray:
        cloud = self.point_cloud()
        if cloud.size == 0:
            return np.empty((0, 0), dtype=np.float64)
        y_resolution = y_resolution_m or self.profile_spacing_m
        x_bins = np.floor((cloud[:, 0] - cloud[:, 0].min()) / x_resolution_m).astype(int)
        y_bins = np.floor((cloud[:, 2] - cloud[:, 2].min()) / y_resolution).astype(int)
        image = np.full((y_bins.max() + 1, x_bins.max() + 1), fill_value, dtype=np.float64)
        image[y_bins, x_bins] = cloud[:, 1]
        return image


class Gocator2690LineProfiler:
    """First functional Gocator 2690 profile simulator."""

    def __init__(self, spec: Gocator2690Spec | None = None) -> None:
        self.spec = spec or Gocator2690Spec()
        self.nominal_width_m = self.spec.width_at_distance(self.spec.nominal_standoff_m)

    def local_x_pattern(self, standoff_m: float | None = None) -> np.ndarray:
        """Profile sample X coordinates for the chosen measurement distance."""
        width_m = self.spec.width_at_distance(standoff_m or self.spec.nominal_standoff_m)
        return np.linspace(
            -0.5 * width_m,
            0.5 * width_m,
            self.spec.points_per_profile,
            dtype=np.float64,
        )

    def ray_pattern(
        self,
        pose: ScannerFramePose,
        *,
        standoff_m: float | None = None,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Return world ray origins and directions for one horizontal profile."""
        local_x_m = self.local_x_pattern(standoff_m)
        target_standoff_m = standoff_m or self.spec.nominal_standoff_m
        local_targets = np.column_stack(
            (
                local_x_m,
                np.zeros(self.spec.points_per_profile, dtype=np.float64),
                np.full(
                    self.spec.points_per_profile,
                    target_standoff_m,
                    dtype=np.float64,
                ),
            )
        )
        directions = local_targets @ pose.rotation_world_from_scanner.T
        directions /= np.linalg.norm(directions, axis=1, keepdims=True)
        origins = np.repeat(pose.origin_m[None, :], self.spec.points_per_profile, axis=0)
        return origins, directions, local_x_m

    def sample_facade_plane(
        self,
        pose: ScannerFramePose,
        *,
        timestamp_s: float = 0.0,
        profile_index: int = 0,
        encoder_position_m: float = 0.0,
        plane_point_m=(0.0, 0.0, 0.0),
        plane_normal_m=(0.0, -1.0, 0.0),
    ) -> GocatorProfile:
        """Intersect the profile rays with a facade plane.

        Returns one profile with hit point, hit normal, measurement range along
        scanner-frame +Z, and validity for each of the 3700 rays.
        """
        plane_point = np.asarray(plane_point_m, dtype=np.float64)
        plane_normal = _as_unit_array(plane_normal_m)

        center_denom = float(pose.z_axis @ plane_normal)
        if abs(center_denom) > 1e-9:
            center_t = float(((plane_point - pose.origin_m) @ plane_normal) / center_denom)
            standoff_m = center_t if center_t > 0.0 else self.spec.nominal_standoff_m
        else:
            standoff_m = self.spec.nominal_standoff_m

        origins, directions, local_x_m = self.ray_pattern(pose, standoff_m=standoff_m)

        denom = directions @ plane_normal
        numer = (plane_point - origins) @ plane_normal
        t = np.full(self.spec.points_per_profile, np.nan, dtype=np.float64)
        non_parallel = np.abs(denom) > 1e-9
        t[non_parallel] = numer[non_parallel] / denom[non_parallel]

        hits = origins + directions * t[:, None]
        local_hits = pose.world_to_scanner(hits)
        ranges = local_hits[:, 2]
        frustum_width = self.spec.width_at_distance(ranges)

        valid = (
            non_parallel
            & np.isfinite(t)
            & (t >= 0.0)
            & (ranges >= self.spec.min_measurement_distance_m)
            & (ranges <= self.spec.max_measurement_distance_m)
            & (np.abs(local_hits[:, 0]) <= 0.5 * frustum_width)
        )

        normals = np.repeat(plane_normal[None, :], self.spec.points_per_profile, axis=0)
        return self._make_profile(
            timestamp_s=timestamp_s,
            profile_index=profile_index,
            encoder_position_m=encoder_position_m,
            pose=pose,
            points_world=hits,
            hit_normals_world=normals,
            ranges_m=ranges,
            local_x_m=local_x_m,
            valid_mask=valid,
        )

    def sample_surface(
        self,
        pose: ScannerFramePose,
        surface_y_m: Callable[[np.ndarray, np.ndarray], np.ndarray],
        *,
        timestamp_s: float = 0.0,
        profile_index: int = 0,
        encoder_position_m: float = 0.0,
        normal_epsilon_m: float = 0.001,
    ) -> GocatorProfile:
        """Intersect the profile rays with a facade height field y=f(x,z)."""
        origins, directions, local_x_m = self.ray_pattern(
            pose, standoff_m=self.spec.nominal_standoff_m
        )
        t_min = self.spec.min_measurement_distance_m
        t_max = self.spec.max_measurement_distance_m
        lo = np.full(self.spec.points_per_profile, t_min, dtype=np.float64)
        hi = np.full(self.spec.points_per_profile, t_max, dtype=np.float64)

        def residual(t_values: np.ndarray) -> np.ndarray:
            points = origins + directions * t_values[:, None]
            return points[:, 1] - surface_y_m(points[:, 0], points[:, 2])

        r_lo = residual(lo)
        r_hi = residual(hi)
        bracketed = np.signbit(r_lo) != np.signbit(r_hi)

        for _ in range(36):
            mid = 0.5 * (lo + hi)
            r_mid = residual(mid)
            same_as_lo = np.signbit(r_mid) == np.signbit(r_lo)
            lo = np.where(bracketed & same_as_lo, mid, lo)
            r_lo = np.where(bracketed & same_as_lo, r_mid, r_lo)
            hi = np.where(bracketed & ~same_as_lo, mid, hi)

        t = np.where(bracketed, 0.5 * (lo + hi), np.nan)
        hits = origins + directions * t[:, None]
        local_hits = pose.world_to_scanner(hits)
        ranges = local_hits[:, 2]
        frustum_width = self.spec.width_at_distance(ranges)
        valid = (
            bracketed
            & np.isfinite(t)
            & (ranges >= self.spec.min_measurement_distance_m)
            & (ranges <= self.spec.max_measurement_distance_m)
            & (np.abs(local_hits[:, 0]) <= 0.5 * frustum_width)
        )
        normals = _height_field_normals(surface_y_m, hits, normal_epsilon_m)
        return self._make_profile(
            timestamp_s=timestamp_s,
            profile_index=profile_index,
            encoder_position_m=encoder_position_m,
            pose=pose,
            points_world=hits,
            hit_normals_world=normals,
            ranges_m=ranges,
            local_x_m=local_x_m,
            valid_mask=valid,
        )

    def _make_profile(
        self,
        *,
        timestamp_s: float,
        profile_index: int,
        encoder_position_m: float,
        pose: ScannerFramePose,
        points_world: np.ndarray,
        hit_normals_world: np.ndarray,
        ranges_m: np.ndarray,
        local_x_m: np.ndarray,
        valid_mask: np.ndarray,
    ) -> GocatorProfile:
        return GocatorProfile(
            timestamp_s=timestamp_s,
            profile_index=profile_index,
            encoder_position_m=encoder_position_m,
            scanner_pose_world=pose,
            x_m=local_x_m,
            z_m=ranges_m,
            valid_mask=valid_mask,
            hit_normals_world=hit_normals_world,
            points_world=points_world,
        )

class GocatorEncoderTriggeredAcquisition:
    """Emit profiles from encoder-equivalent travel in scanner-frame Y."""

    def __init__(
        self,
        profiler: Gocator2690LineProfiler,
        sampler: Callable[[ScannerFramePose, float, int, float], GocatorProfile],
    ) -> None:
        self.profiler = profiler
        self.sampler = sampler
        self.profile_index = 0
        self._reference_pose: ScannerFramePose | None = None
        self._last_encoder_position_m: float | None = None
        self._accumulated_motion_m = 0.0

    def encoder_position(self, pose: ScannerFramePose) -> float:
        if self._reference_pose is None:
            self._reference_pose = pose
        return float((pose.origin_m - self._reference_pose.origin_m) @ self._reference_pose.y_axis)

    def update(self, pose: ScannerFramePose, timestamp_s: float) -> list[GocatorProfile]:
        encoder_position_m = self.encoder_position(pose)
        if self._last_encoder_position_m is None:
            self._last_encoder_position_m = encoder_position_m
            return []

        delta_m = encoder_position_m - self._last_encoder_position_m
        self._last_encoder_position_m = encoder_position_m
        self._accumulated_motion_m += abs(delta_m)

        emitted: list[GocatorProfile] = []
        while self._accumulated_motion_m >= self.profiler.spec.profile_spacing_m:
            profile = self.sampler(
                pose,
                timestamp_s,
                self.profile_index,
                encoder_position_m,
            )
            emitted.append(profile)
            self.profile_index += 1
            self._accumulated_motion_m -= self.profiler.spec.profile_spacing_m
        return emitted


def export_profiles_npz(path: str | Path, profiles: list[GocatorProfile]) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        path,
        timestamp_s=np.array([p.timestamp_s for p in profiles], dtype=np.float64),
        profile_index=np.array([p.profile_index for p in profiles], dtype=np.int64),
        encoder_position_m=np.array([p.encoder_position_m for p in profiles], dtype=np.float64),
        scanner_origin_world=np.array([p.scanner_pose_world.origin_m for p in profiles], dtype=np.float64),
        scanner_x_axis_world=np.array([p.scanner_pose_world.x_axis for p in profiles], dtype=np.float64),
        scanner_y_axis_world=np.array([p.scanner_pose_world.y_axis for p in profiles], dtype=np.float64),
        scanner_z_axis_world=np.array([p.scanner_pose_world.z_axis for p in profiles], dtype=np.float64),
        x_m=np.array([p.x_m for p in profiles], dtype=np.float64),
        z_m=np.array([p.z_m for p in profiles], dtype=np.float64),
        valid_mask=np.array([p.valid_mask for p in profiles], dtype=bool),
        hit_normals_world=np.array([p.hit_normals_world for p in profiles], dtype=np.float64),
        points_world=np.array([p.points_world for p in profiles], dtype=np.float64),
    )


def export_profiles_csv(path: str | Path, profiles: list[GocatorProfile]) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as fp:
        fp.write("timestamp_s,profile_index,encoder_position_m,point_index,x_sensor_m,z_sensor_m,valid,x_world_m,y_world_m,z_world_m,nx_world,ny_world,nz_world\n")
        for profile in profiles:
            for point_index in range(profile.x_m.size):
                point = profile.points_world[point_index]
                normal = profile.hit_normals_world[point_index]
                fp.write(
                    f"{profile.timestamp_s:.6f},{profile.profile_index},"
                    f"{profile.encoder_position_m:.6f},{point_index},"
                    f"{profile.x_m[point_index]:.6f},{profile.z_m[point_index]:.6f},"
                    f"{int(profile.valid_mask[point_index])},"
                    f"{point[0]:.6f},{point[1]:.6f},{point[2]:.6f},"
                    f"{normal[0]:.6f},{normal[1]:.6f},{normal[2]:.6f}\n"
                )


def export_point_cloud_ply(path: str | Path, points_world: np.ndarray) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as fp:
        fp.write("ply\nformat ascii 1.0\n")
        fp.write(f"element vertex {len(points_world)}\n")
        fp.write("property float x\nproperty float y\nproperty float z\n")
        fp.write("end_header\n")
        for point in points_world:
            fp.write(f"{point[0]:.6f} {point[1]:.6f} {point[2]:.6f}\n")


def export_height_map_npy(path: str | Path, height_map: np.ndarray) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    np.save(path, height_map)


def _as_unit_array(values, *, normalize: bool = True) -> np.ndarray:
    array = np.asarray(values, dtype=np.float64)
    if array.shape != (3,):
        raise ValueError(f"expected 3-vector, got shape {array.shape}")
    if not normalize:
        return array
    norm = np.linalg.norm(array)
    if norm <= 0.0:
        raise ValueError("cannot normalize zero vector")
    return array / norm


def _height_field_normals(
    surface_y_m: Callable[[np.ndarray, np.ndarray], np.ndarray],
    points: np.ndarray,
    epsilon_m: float,
) -> np.ndarray:
    x = points[:, 0]
    z = points[:, 2]
    dy_dx = (
        surface_y_m(x + epsilon_m, z) - surface_y_m(x - epsilon_m, z)
    ) / (2.0 * epsilon_m)
    dy_dz = (
        surface_y_m(x, z + epsilon_m) - surface_y_m(x, z - epsilon_m)
    ) / (2.0 * epsilon_m)
    normals = np.column_stack((-dy_dx, np.ones_like(dy_dx), -dy_dz))
    norms = np.linalg.norm(normals, axis=1, keepdims=True)
    return normals / np.maximum(norms, 1e-12)
