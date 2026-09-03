"""
Pure NumPy 5-State Extended Kalman Filter (EKF) Core.
Fuses Visual Odometry (VIO), IMU, and Wheel Encoders with Slip Gating.
"""

from typing import Tuple
import numpy as np


class EKFCore:
    """
    State Vector: [x, y, theta, v, omega]^T
    - x, y: Position in odom frame (m)
    - theta: Heading angle in odom frame (rad)
    - v: Linear forward velocity (m/s)
    - omega: Angular yaw rate (rad/s)
    """

    def __init__(
        self,
        initial_state: np.ndarray = None,
        process_noise_diag: tuple = (0.05, 0.05, 0.02, 0.1, 0.05),
        initial_cov_diag: tuple = (0.5, 0.5, 0.2, 0.1, 0.1)
    ):
        if initial_state is None:
            self.x = np.zeros(5, dtype=np.float64)
        else:
            self.x = np.array(initial_state, dtype=np.float64)

        self.P = np.diag(initial_cov_diag).astype(np.float64)
        self.Q = np.diag(process_noise_diag).astype(np.float64)

    def predict(self, a_imu: float, omega_gyro: float, dt: float) -> np.ndarray:
        """
        Executes EKF prediction step using non-linear skid-steer motion model.

        Args:
            a_imu: Longitudinal forward acceleration from IMU (m/s^2).
            omega_gyro: Angular rate around Z-axis from IMU gyro (rad/s).
            dt: Time delta (s).

        Returns:
            predicted_state: Array of shape (5,).
        """
        px, py, theta, v, omega = self.x

        # Midpoint angle integration
        mid_theta = theta + 0.5 * omega * dt

        # State transition
        next_px = px + v * np.cos(mid_theta) * dt
        next_py = py + v * np.sin(mid_theta) * dt
        next_theta = (theta + omega_gyro * dt + np.pi) % (2.0 * np.pi) - np.pi
        next_v = v + a_imu * dt
        next_omega = omega_gyro

        self.x = np.array([next_px, next_py, next_theta, next_v, next_omega], dtype=np.float64)

        # Jacobian F = df/dx
        F = np.eye(5, dtype=np.float64)
        F[0, 2] = -v * np.sin(mid_theta) * dt
        F[0, 3] = np.cos(mid_theta) * dt
        F[1, 2] = v * np.cos(mid_theta) * dt
        F[1, 3] = np.sin(mid_theta) * dt
        F[2, 4] = dt

        # Covariance prediction
        self.P = F @ self.P @ F.T + self.Q * dt
        return self.x.copy()

    def update_vio(self, z_vio: np.ndarray, R_vio: np.ndarray = None) -> np.ndarray:
        """
        Measurement update from Visual-Inertial Odometry pose [x, y, theta].

        Args:
            z_vio: Array of shape (3,) [x_meas, y_meas, theta_meas].
            R_vio: 3x3 measurement covariance matrix.
        """
        if R_vio is None:
            R_vio = np.diag([0.04, 0.04, 0.02])  # ~20cm, ~8 deg std

        H = np.zeros((3, 5), dtype=np.float64)
        H[0, 0] = 1.0
        H[1, 1] = 1.0
        H[2, 2] = 1.0

        # Innovation
        y = z_vio - H @ self.x
        # Wrap heading innovation to [-pi, pi]
        y[2] = (y[2] + np.pi) % (2.0 * np.pi) - np.pi

        S = H @ self.P @ H.T + R_vio
        K = self.P @ H.T @ np.linalg.inv(S)

        self.x = self.x + K @ y
        self.x[2] = (self.x[2] + np.pi) % (2.0 * np.pi) - np.pi

        I_KH = np.eye(5) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R_vio @ K.T
        return self.x.copy()

    def update_wheel_odometry(
        self,
        z_wheel: np.ndarray,
        R_wheel: np.ndarray = None,
        cov_scale: float = 1.0
    ) -> np.ndarray:
        """
        Measurement update from wheel encoder velocities [v_wheel, omega_wheel].

        Args:
            z_wheel: Array of shape (2,) [v_meas, omega_meas].
            R_wheel: 2x2 measurement covariance matrix.
            cov_scale: Dynamic multiplier for wheel slip gating (e.g. 1e6 during slip).
        """
        if R_wheel is None:
            R_wheel = np.diag([0.02, 0.05])

        scaled_R = R_wheel * cov_scale

        H = np.zeros((2, 5), dtype=np.float64)
        H[0, 3] = 1.0
        H[1, 4] = 1.0

        y = z_wheel - H @ self.x
        S = H @ self.P @ H.T + scaled_R
        K = self.P @ H.T @ np.linalg.inv(S)

        self.x = self.x + K @ y
        self.x[2] = (self.x[2] + np.pi) % (2.0 * np.pi) - np.pi

        I_KH = np.eye(5) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ scaled_R @ K.T
        return self.x.copy()

    def get_state(self) -> Tuple[np.ndarray, np.ndarray]:
        """Returns (state_vector, covariance_matrix)."""
        return self.x.copy(), self.P.copy()
