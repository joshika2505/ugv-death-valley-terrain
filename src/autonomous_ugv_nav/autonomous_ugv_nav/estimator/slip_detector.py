"""
Wheel Slip Detector for Terramechanics-Aware Sensor Fusion.
Compares wheel encoder odometry against Visual Odometry and IMU gyroscopes.
"""

from collections import deque
import numpy as np


class SlipDetector:
    """
    Monitors velocity discrepancies between wheel encoders and Visual-Inertial Odometry
    to detect wheel slip on loose dirt, sand, or mud.
    """

    def __init__(
        self,
        slip_threshold: float = 0.30,
        yaw_slip_threshold: float = 0.40,
        window_size: int = 5,
        slip_covariance_scale: float = 1e6
    ):
        self.slip_thresh = slip_threshold
        self.yaw_thresh = yaw_slip_threshold
        self.window_size = window_size
        self.slip_cov_scale = slip_covariance_scale

        self.slip_history = deque(maxlen=window_size)
        self.current_slip_ratio = 0.0
        self.is_slipping = False

    def update(
        self,
        wheel_v: float,
        vio_v: float,
        gyro_omega: float,
        wheel_omega: float
    ) -> bool:
        """
        Updates slip estimation with latest sensor readings.

        Args:
            wheel_v: Linear velocity from wheel encoders (m/s).
            vio_v: Linear velocity from Visual Odometry (m/s).
            gyro_omega: Angular velocity from IMU gyroscope (rad/s).
            wheel_omega: Angular velocity from differential wheel encoders (rad/s).

        Returns:
            is_slipping: Boolean indicating if significant slip is detected.
        """
        # Linear velocity slip ratio
        denom_v = max(abs(wheel_v), abs(vio_v), 0.08)
        linear_slip = abs(wheel_v - vio_v) / denom_v

        # Angular yaw slip ratio
        denom_omega = max(abs(gyro_omega), abs(wheel_omega), 0.15)
        yaw_slip = abs(gyro_omega - wheel_omega) / denom_omega

        # Flag sample as slipping if either linear or angular slip exceeds threshold
        sample_slipping = (linear_slip > self.slip_thresh) or (yaw_slip > self.yaw_thresh)
        self.slip_history.append(sample_slipping)
        self.current_slip_ratio = float(linear_slip)

        # Require majority in window to declare sustained slip
        if len(self.slip_history) > 0:
            slip_count = sum(self.slip_history)
            self.is_slipping = (slip_count / len(self.slip_history)) >= 0.5
        else:
            self.is_slipping = False

        return self.is_slipping

    def get_encoder_covariance_scale(self) -> float:
        """
        Returns covariance multiplier for wheel encoder measurements.
        Returns 1.0 during nominal traction, 1e6 during slip.
        """
        return self.slip_cov_scale if self.is_slipping else 1.0
