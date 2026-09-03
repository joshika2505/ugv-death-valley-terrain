"""
Core MPPI (Model Predictive Path Integral) Controller.
Vectorized NumPy implementation for real-time trajectory optimization on embedded CPUs.
"""

from typing import List, Tuple, Optional
import numpy as np

from autonomous_ugv_nav.planner.skid_steer_model import SkidSteerModel
from autonomous_ugv_nav.planner.cost_critics import BaseCritic


class MPPIController:
    """
    Model Predictive Path Integral controller optimizing skid-steer control sequences
    over continuous costmaps and reference paths.
    """

    def __init__(
        self,
        dynamics_model: SkidSteerModel,
        critics: List[BaseCritic],
        num_samples: int = 300,
        time_horizon: int = 20,
        dt: float = 0.1,
        temperature: float = 1.0,
        noise_std_v: float = 0.35,
        noise_std_omega: float = 0.85,
    ):
        self.dynamics = dynamics_model
        self.critics = critics
        self.K = num_samples
        self.T = time_horizon
        self.dt = dt
        self.lambda_ = temperature
        self.noise_std = np.array([noise_std_v, noise_std_omega], dtype=np.float32)

        # Control sequence mean: shape (T, 2)
        self.U = np.zeros((self.T, 2), dtype=np.float32)
        self.prev_applied_control = np.zeros(2, dtype=np.float32)

    def reset(self):
        """Resets the control sequence to zero."""
        self.U = np.zeros((self.T, 2), dtype=np.float32)
        self.prev_applied_control = np.zeros(2, dtype=np.float32)

    def compute_control(
        self,
        current_state: np.ndarray,      # (3,) [x, y, theta]
        costmap: np.ndarray,            # 2D cost array
        costmap_origin: tuple,          # (origin_x, origin_y, resolution)
        goal: Optional[np.ndarray],     # (3,) [gx, gy, gtheta]
        global_path: Optional[np.ndarray]  # (N, 2) [px, py]
    ) -> Tuple[float, float, np.ndarray, np.ndarray]:
        """
        Executes one MPPI optimization iteration.

        Returns:
            cmd_v: Optimal forward velocity command (m/s)
            cmd_omega: Optimal yaw rate command (rad/s)
            best_trajectory: Array of shape (T+1, 3) for visualization
            sampled_trajectories: Array of shape (K, T+1, 3) for visualization
        """
        # 1. Warm-start: Shift previous control sequence forward by 1 time step
        self.U[:-1] = self.U[1:]
        self.U[-1] = np.array([0.0, 0.0], dtype=np.float32)

        # 2. Sample random perturbations: shape (K, T, 2)
        noise = np.random.normal(0.0, 1.0, size=(self.K, self.T, 2)).astype(np.float32)
        perturbations = noise * self.noise_std

        # Include an unperturbed nominal control sequence as sample 0
        perturbations[0] = 0.0

        # Candidate control sequences
        candidate_controls = self.U[np.newaxis, :, :] + perturbations

        # Enforce kinodynamic velocity and acceleration limits
        candidate_controls = self.dynamics.enforce_constraints(
            candidate_controls,
            prev_controls=self.prev_applied_control,
            dt=self.dt
        )

        # 3. Roll out trajectories across horizon T
        trajectories = self._rollout_batch(current_state, candidate_controls)

        # 4. Evaluate Critic Costs
        total_costs = np.zeros(self.K, dtype=np.float32)
        for critic in self.critics:
            c_cost = critic.score(
                trajectories=trajectories,
                controls=candidate_controls,
                costmap=costmap,
                costmap_origin=costmap_origin,
                goal=goal,
                global_path=global_path
            )
            total_costs += c_cost

        # 5. Add Control Perturbation Energy Term
        # lambda * u^T Sigma^-1 epsilon
        sigma_inv = 1.0 / (self.noise_std ** 2)
        control_penalty = self.lambda_ * np.sum(
            self.U[np.newaxis, :, :] * sigma_inv * candidate_controls,
            axis=(1, 2)
        )
        total_costs += control_penalty

        # 6. Softmax Weighting
        min_cost = np.min(total_costs)
        # Shift costs by min_cost for numerical stability
        exp_costs = np.exp(-(total_costs - min_cost) / max(1e-4, self.lambda_))
        sum_exp = np.sum(exp_costs)
        if sum_exp < 1e-8 or np.isnan(sum_exp):
            weights = np.ones(self.K, dtype=np.float32) / self.K
        else:
            weights = exp_costs / sum_exp

        # 7. Update Control Sequence Mean
        weighted_controls = np.sum(weights[:, np.newaxis, np.newaxis] * candidate_controls, axis=0)
        self.U = self.dynamics.enforce_constraints(
            weighted_controls[np.newaxis, :, :],
            prev_controls=self.prev_applied_control,
            dt=self.dt
        )[0]

        # Extract immediate command to apply
        cmd_v = float(self.U[0, 0])
        cmd_omega = float(self.U[0, 1])
        self.prev_applied_control = np.array([cmd_v, cmd_omega], dtype=np.float32)

        # Best trajectory corresponds to minimum cost sample
        best_idx = int(np.argmin(total_costs))
        best_trajectory = trajectories[best_idx]

        return cmd_v, cmd_omega, best_trajectory, trajectories

    def _rollout_batch(self, initial_state: np.ndarray, controls: np.ndarray) -> np.ndarray:
        """
        Rolls out K trajectories in parallel across T steps.

        Args:
            initial_state: Shape (3,) [x, y, theta]
            controls: Shape (K, T, 2)

        Returns:
            trajectories: Shape (K, T+1, 3)
        """
        K, T, _ = controls.shape
        trajectories = np.zeros((K, T + 1, 3), dtype=np.float32)

        # Broadcast initial state across all K samples
        trajectories[:, 0, :] = initial_state

        current_states = np.tile(initial_state, (K, 1))

        for t in range(T):
            ctrl_step = controls[:, t, :]
            current_states = self.dynamics.step(current_states, ctrl_step, self.dt)
            trajectories[:, t + 1, :] = current_states

        return trajectories
