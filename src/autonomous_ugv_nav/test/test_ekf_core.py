"""
Unit tests for EKFCore and SlipDetector.
"""

import numpy as np
import pytest

from autonomous_ugv_nav.estimator.ekf_core import EKFCore
from autonomous_ugv_nav.estimator.slip_detector import SlipDetector


def test_ekf_prediction_straight_line():
    ekf = EKFCore(initial_state=[0.0, 0.0, 0.0, 1.0, 0.0])  # v = 1.0 m/s heading 0
    dt = 0.1

    state = ekf.predict(a_imu=0.0, omega_gyro=0.0, dt=dt)
    # x should advance by v * dt = 0.1m, y should be 0, theta should be 0
    assert np.isclose(state[0], 0.1, atol=1e-3)
    assert np.isclose(state[1], 0.0, atol=1e-3)
    assert np.isclose(state[2], 0.0, atol=1e-3)


def test_ekf_prediction_turning():
    ekf = EKFCore(initial_state=[0.0, 0.0, 0.0, 0.0, 0.0])
    dt = 0.1
    omega = 0.5  # rad/s

    state = ekf.predict(a_imu=0.0, omega_gyro=omega, dt=dt)
    # Theta should advance by omega * dt = 0.05 rad
    assert np.isclose(state[2], 0.05, atol=1e-3)


def test_ekf_vio_correction():
    ekf = EKFCore(initial_state=[0.0, 0.0, 0.0, 0.0, 0.0])
    # Prior covariance is large
    init_cov_trace = np.trace(ekf.P)

    z_vio = np.array([5.0, 2.0, 0.5])
    updated_state = ekf.update_vio(z_vio)

    # State should move towards measurement
    assert updated_state[0] > 1.0
    assert updated_state[1] > 0.5
    # Posterior covariance must decrease after measurement update
    post_cov_trace = np.trace(ekf.P)
    assert post_cov_trace < init_cov_trace


def test_slip_detector_and_covariance_gating():
    detector = SlipDetector(slip_threshold=0.30, window_size=3)

    # Nominal traction: wheel_v = 1.0, vio_v = 0.98 -> slip < 0.3
    for _ in range(3):
        is_slip = detector.update(wheel_v=1.0, vio_v=0.98, gyro_omega=0.0, wheel_omega=0.0)
    assert not is_slip
    assert detector.get_encoder_covariance_scale() == 1.0

    # Sudden slip event: wheel spinning at 2.0 m/s while vehicle stuck (vio_v = 0.1 m/s)
    for _ in range(3):
        is_slip = detector.update(wheel_v=2.0, vio_v=0.1, gyro_omega=0.0, wheel_omega=0.0)
    assert is_slip
    assert detector.get_encoder_covariance_scale() == 1e6


def test_ekf_wheel_update_with_slip_rejection():
    # Robot at (0, 0) with velocity 0.0
    ekf = EKFCore(initial_state=[0.0, 0.0, 0.0, 0.0, 0.0])

    # Case A: High-slip encoder anomaly (false wheel spin at 5.0 m/s) with 1e6 covariance scaling
    false_wheel_z = np.array([5.0, 0.0])
    ekf.update_wheel_odometry(false_wheel_z, cov_scale=1e6)

    # State velocity should barely change because covariance was gated out
    assert abs(ekf.x[3]) < 0.01, "Slipping wheel measurement must be ignored by scaled covariance."
