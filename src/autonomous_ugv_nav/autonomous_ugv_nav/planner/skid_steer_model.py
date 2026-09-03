"""
Skid-Steer / Differential Drive Kinematic Dynamics Model.
Fully vectorized for batch trajectory rollout across K samples in MPPI.
"""

import numpy as np


class SkidSteerModel:
    """
    Non-holonomic differential-drive and skid-steer kinematics with
    velocity, acceleration, and yaw rate constraints.
    """

    def __init__(
        self,
        max_v: float = 1.2,
        min_v: float = -0.4,
        max_omega: float = 2.0,
        max_accel: float = 1.5,
        max_yaw_accel: float = 3.0,
        wheelbase: float = 0.50,
        track_width: float = 0.45
    ):
        self.max_v = max_v
        self.min_v = min_v
        self.max_omega = max_omega
        self.max_accel = max_accel
        self.max_yaw_accel = max_yaw_accel
        self.wheelbase = wheelbase
        self.track_width = track_width

    def step(self, state: np.ndarray, control: np.ndarray, dt: float) -> np.ndarray:
        """
        Rolls forward a batch of states [K, 3] or [3] by one time step dt
        using 2nd-order Runge-Kutta / midpoint integration.

        Args:
            state: Array of shape (..., 3) representing [x, y, theta].
            control: Array of shape (..., 2) representing [v, omega].
            dt: Integration time step in seconds.

        Returns:
            next_state: Array of same shape as state [..., 3].
        """
        x = state[..., 0]
        y = state[..., 1]
        theta = state[..., 2]

        v = control[..., 0]
        omega = control[..., 1]

        # Midpoint angle for arc integration
        mid_theta = theta + 0.5 * omega * dt

        next_x = x + v * np.cos(mid_theta) * dt
        next_y = y + v * np.sin(mid_theta) * dt
        next_theta = (theta + omega * dt + np.pi) % (2.0 * np.pi) - np.pi

        return np.stack([next_x, next_y, next_theta], axis=-1)

    def enforce_constraints(self, controls: np.ndarray, prev_controls: np.ndarray = None, dt: float = 0.1) -> np.ndarray:
        """
        Enforces maximum velocity and acceleration limits on control sequences.

        Args:
            controls: Control array of shape (K, T, 2) or (T, 2) [v, omega].
            prev_controls: Control applied in the previous step (shape 2,) or None.
            dt: Time step.

        Returns:
            clipped_controls: Array of same shape with valid bounds.
        """
        clipped = controls.copy()

        # 1. Velocity bounds
        clipped[..., 0] = np.clip(clipped[..., 0], self.min_v, self.max_v)
        clipped[..., 1] = np.clip(clipped[..., 1], -self.max_omega, self.max_omega)

        # 2. Rate of change (acceleration) bounds along the time dimension
        if prev_controls is not None and controls.ndim >= 2:
            # First time step constrained by previous actual command
            dv_max = self.max_accel * dt
            domega_max = self.max_yaw_accel * dt

            # Step 0 constraint against prev_controls
            clipped[..., 0, 0] = np.clip(
                clipped[..., 0, 0],
                prev_controls[0] - dv_max,
                prev_controls[0] + dv_max
            )
            clipped[..., 0, 1] = np.clip(
                clipped[..., 0, 1],
                prev_controls[1] - domega_max,
                prev_controls[1] + domega_max
            )

            # Subsequent steps constrained along horizon T
            for t in range(1, clipped.shape[-2]):
                clipped[..., t, 0] = np.clip(
                    clipped[..., t, 0],
                    clipped[..., t-1, 0] - dv_max,
                    clipped[..., t-1, 0] + dv_max
                )
                clipped[..., t, 1] = np.clip(
                    clipped[..., t, 1],
                    clipped[..., t-1, 1] - domega_max,
                    clipped[..., t-1, 1] + domega_max
                )

        return clipped
