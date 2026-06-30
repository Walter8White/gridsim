import numpy as np
import pytest

from gridsim_sensors import (
    GocatorEncoderTriggeredAcquisition,
    Gocator2690LineProfiler,
    Gocator2690Spec,
    GocatorPointCloudAccumulator,
    ScannerFramePose,
)


def _pose_at_y(y_m: float) -> ScannerFramePose:
    return ScannerFramePose.from_arrays(
        [0.0, y_m, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0],
        [0.0, 1.0, 0.0],
    )


def test_gocator_width_interpolates_at_nominal_standoff() -> None:
    spec = Gocator2690Spec()
    expected = 0.385 + ((1.0 - 0.325) / 1.550) * (2.000 - 0.385)

    assert spec.width_at_distance(1.0) == pytest.approx(expected)


def test_gocator_ray_pattern_has_3700_rays_from_scanner_frame() -> None:
    profiler = Gocator2690LineProfiler()
    origins, directions, local_x = profiler.ray_pattern(_pose_at_y(-1.0))

    assert origins.shape == (3700, 3)
    assert directions.shape == (3700, 3)
    assert local_x.shape == (3700,)
    np.testing.assert_allclose(origins, np.array([[0.0, -1.0, 0.0]] * 3700))
    np.testing.assert_allclose(np.linalg.norm(directions, axis=1), 1.0)
    assert directions[:, 1].min() > 0.0


def test_gocator_profile_hits_facade_plane_at_valid_nominal_range() -> None:
    profiler = Gocator2690LineProfiler()
    profile = profiler.sample_facade_plane(_pose_at_y(-1.0))

    assert profile.profile_index == 0
    assert profile.encoder_position_m == pytest.approx(0.0)
    assert profile.scanner_pose_world.origin_m.shape == (3,)
    assert profile.x_m.shape == (3700,)
    assert profile.z_m.shape == (3700,)
    assert profile.valid.shape == (3700,)
    assert profile.valid.all()
    np.testing.assert_allclose(profile.hit_points_m[:, 1], 0.0, atol=1e-9)
    np.testing.assert_allclose(profile.ranges_m, 1.0, atol=1e-9)


def test_gocator_profile_rejects_hits_outside_measurement_range() -> None:
    profiler = Gocator2690LineProfiler()

    too_close = profiler.sample_facade_plane(_pose_at_y(-0.2))
    too_far = profiler.sample_facade_plane(_pose_at_y(-2.0))

    assert not too_close.valid.any()
    assert not too_far.valid.any()


def test_gocator_accumulator_builds_point_cloud() -> None:
    spec = Gocator2690Spec(points_per_profile=10)
    profiler = Gocator2690LineProfiler(spec)
    accumulator = GocatorPointCloudAccumulator(spec.profile_spacing_m)

    accumulator.add_profile(profiler.sample_facade_plane(_pose_at_y(-1.0)))
    accumulator.add_profile(profiler.sample_facade_plane(_pose_at_y(-1.0)))

    assert accumulator.point_cloud().shape == (20, 3)
    assert accumulator.height_map()


def test_gocator_encoder_trigger_emits_only_after_profile_spacing() -> None:
    spec = Gocator2690Spec(points_per_profile=10, profile_spacing_m=0.01)
    profiler = Gocator2690LineProfiler(spec)

    def sampler(pose, timestamp_s, profile_index, encoder_position_m):
        return profiler.sample_facade_plane(
            pose,
            timestamp_s=timestamp_s,
            profile_index=profile_index,
            encoder_position_m=encoder_position_m,
        )

    acquisition = GocatorEncoderTriggeredAcquisition(profiler, sampler)

    assert acquisition.update(_pose_at_y(-1.0), 0.0) == []
    assert acquisition.update(_pose_at_y(-1.0), 0.1) == []

    profiles = acquisition.update(
        ScannerFramePose.from_arrays(
            [0.0, -1.0, 0.011],
            [1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0],
            [0.0, 1.0, 0.0],
        ),
        0.2,
    )

    assert len(profiles) == 1
    assert profiles[0].profile_index == 0
    assert profiles[0].encoder_position_m == pytest.approx(0.011)
