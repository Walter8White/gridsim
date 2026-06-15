import numpy as np

from gridsim_sensors import (
    DistanceSensorModel,
    EncoderModel,
    ImuModel,
    LidarModel,
)


def test_lidar_noise_preserves_shape_and_float_type() -> None:
    points = np.zeros((8, 3), dtype=np.float32)
    measured = LidarModel(noise_std_m=0.01, seed=1).measure(points)

    assert measured.shape == points.shape
    assert np.issubdtype(measured.dtype, np.floating)


def test_imu_noise_returns_three_axis_arrays() -> None:
    acceleration, angular_velocity = ImuModel(seed=1).measure(
        [0.0, 0.0, 9.81], [0.0, 0.0, 0.0], dt_s=0.01
    )

    assert acceleration.shape == (3,)
    assert angular_velocity.shape == (3,)


def test_encoder_and_distance_models_preserve_array_shape() -> None:
    positions = np.array([0.1, 0.2, 0.3])
    ranges = np.array([0.01, 1.0, 5.0])

    encoded = EncoderModel(0.001, seed=1).measure(positions)
    measured_ranges = DistanceSensorModel(0.05, 3.0, seed=1).measure(ranges)

    assert encoded.shape == positions.shape
    assert measured_ranges.shape == ranges.shape
    np.testing.assert_allclose(measured_ranges, [0.05, 1.0, 3.0])
