import pytest

from gridsim_estimation.odometry import DifferentialDriveOdometry


def test_odometry_updates_forward_motion() -> None:
    odometry = DifferentialDriveOdometry(
        wheel_radius_m=0.1, wheel_separation_m=0.5
    )
    odometry.update(0.0, 0.0, dt_s=0.1)

    state = odometry.update(1.0, 1.0, dt_s=0.5)

    assert state.x_m == pytest.approx(0.1)
    assert state.y_m == pytest.approx(0.0)
    assert state.yaw_rad == pytest.approx(0.0)
    assert state.linear_velocity_mps == pytest.approx(0.2)
