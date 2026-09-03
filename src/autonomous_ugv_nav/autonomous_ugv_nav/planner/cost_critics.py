"""
Modular Cost Critics for MPPI Trajectory Scoring.
Inspired by Nav2 MPPI critics, optimized with vectorized NumPy operations.
"""

from abc import ABC, abstractmethod
import numpy as np


class BaseCritic(ABC):
    """Abstract base class for modular trajectory cost evaluators."""

    def __init__(self, weight: float = 1.0):
        self.weight = weight

    @abstractmethod
    def score(
        self,
        trajectories: np.ndarray,      # (K, T+1, 3) [x, y, theta]
        controls: np.ndarray,          # (K, T, 2) [v, omega]
        costmap: np.ndarray,           # 2D cost grid
        costmap_origin: tuple,         # (origin_x, origin_y, resolution)
        goal: np.ndarray,              # (3,) [gx, gy, gtheta]
        global_path: np.ndarray        # (N, 2) [px, py]
    ) -> np.ndarray:                   # Returns cost array of shape (K,)
        pass


class ObstacleCritic(BaseCritic):
    """
    Evaluates trajectory collision risk by sampling the 2.5D semantic costmap.
    Assigns massive penalties for lethal obstacles and smooth penalties for inflated zones.
    """

    def __init__(self, weight: float = 20.0, lethal_cost_thresh: float = 250.0, lethal_penalty: float = 1e5):
        super().__init__(weight)
        self.lethal_cost_thresh = lethal_cost_thresh
        self.lethal_penalty = lethal_penalty

    def score(
        self,
        trajectories: np.ndarray,
        controls: np.ndarray,
        costmap: np.ndarray,
        costmap_origin: tuple,
        goal: np.ndarray,
        global_path: np.ndarray
    ) -> np.ndarray:
        K, T_plus_1, _ = trajectories.shape
        if costmap is None or costmap.size == 0:
            return np.zeros(K, dtype=np.float32)

        ox, oy, res = costmap_origin
        h, w = costmap.shape

        # Extract (x, y) coordinates for all K trajectories across all time steps
        x_pts = trajectories[:, :, 0]
        y_pts = trajectories[:, :, 1]

        # Convert to cell indices
        col_idx = np.clip(((x_pts - ox) / res).astype(np.int32), 0, w - 1)
        row_idx = np.clip(((y_pts - oy) / res).astype(np.int32), 0, h - 1)

        # Lookup cell costs
        cell_costs = costmap[row_idx, col_idx].astype(np.float32)

        # Check for lethal collisions
        lethal_hits = np.any(cell_costs >= self.lethal_cost_thresh, axis=1)

        # Cumulative proximity cost with quadratic weighting
        cum_cost = np.sum((cell_costs / 254.0) ** 2, axis=1)

        # Total obstacle cost per trajectory
        total_costs = self.weight * cum_cost
        total_costs[lethal_hits] += self.lethal_penalty

        return total_costs


class PathFollowCritic(BaseCritic):
    """
    Penalizes lateral deviation from the reference global path polyline.
    """

    def __init__(self, weight: float = 6.0):
        super().__init__(weight)

    def score(
        self,
        trajectories: np.ndarray,
        controls: np.ndarray,
        costmap: np.ndarray,
        costmap_origin: tuple,
        goal: np.ndarray,
        global_path: np.ndarray
    ) -> np.ndarray:
        K, T_plus_1, _ = trajectories.shape
        if global_path is None or len(global_path) == 0:
            return np.zeros(K, dtype=np.float32)

        # Points along trajectories: (K, T+1, 2)
        traj_xy = trajectories[:, :, :2]
        path_xy = global_path[:, :2]  # (N, 2)

        # Compute pairwise distance from each trajectory point to closest point on path
        # Using vectorized broadcasting: (K, T+1, 1, 2) - (1, 1, N, 2) -> (K, T+1, N, 2)
        # To conserve memory, check distances against a localized subset of path points
        diff = traj_xy[:, :, np.newaxis, :] - path_xy[np.newaxis, np.newaxis, :, :]
        dist_sq = np.sum(diff**2, axis=-1)  # (K, T+1, N)
        min_dist_to_path = np.sqrt(np.min(dist_sq, axis=-1))  # (K, T+1)

        # Integral of cross-track error
        return self.weight * np.sum(min_dist_to_path, axis=1)


class PathAlignCritic(BaseCritic):
    """
    Penalizes heading misalignment with the tangent of the reference path.
    """

    def __init__(self, weight: float = 3.0):
        super().__init__(weight)

    def score(
        self,
        trajectories: np.ndarray,
        controls: np.ndarray,
        costmap: np.ndarray,
        costmap_origin: tuple,
        goal: np.ndarray,
        global_path: np.ndarray
    ) -> np.ndarray:
        K, T_plus_1, _ = trajectories.shape
        if global_path is None or len(global_path) < 2:
            return np.zeros(K, dtype=np.float32)

        # Approximate path tangents
        path_diffs = np.diff(global_path[:, :2], axis=0)
        path_headings = np.arctan2(path_diffs[:, 1], path_diffs[:, 0])

        # Trajectory headings
        traj_headings = trajectories[:, :, 2]  # (K, T+1)

        # Find closest path segment for each trajectory point
        traj_xy = trajectories[:, :, :2]
        path_mid = (global_path[:-1, :2] + global_path[1:, :2]) / 2.0
        diff = traj_xy[:, :, np.newaxis, :] - path_mid[np.newaxis, np.newaxis, :, :]
        dist_sq = np.sum(diff**2, axis=-1)
        closest_seg = np.argmin(dist_sq, axis=-1)  # (K, T+1)

        target_headings = path_headings[closest_seg]  # (K, T+1)

        # Angular error in [-pi, pi]
        heading_err = (traj_headings - target_headings + np.pi) % (2.0 * np.pi) - np.pi
        return self.weight * np.sum(np.abs(heading_err), axis=1)


class GoalCritic(BaseCritic):
    """
    Evaluates terminal Euclidean distance to the destination goal.
    """

    def __init__(self, weight: float = 10.0):
        super().__init__(weight)

    def score(
        self,
        trajectories: np.ndarray,
        controls: np.ndarray,
        costmap: np.ndarray,
        costmap_origin: tuple,
        goal: np.ndarray,
        global_path: np.ndarray
    ) -> np.ndarray:
        if goal is None:
            return np.zeros(trajectories.shape[0], dtype=np.float32)

        # Terminal state of each trajectory: (K, 2)
        end_xy = trajectories[:, -1, :2]
        goal_xy = goal[:2]

        dist_to_goal = np.linalg.norm(end_xy - goal_xy, axis=-1)
        return self.weight * dist_to_goal


class GoalAngleCritic(BaseCritic):
    """
    Evaluates terminal heading error relative to the goal orientation.
    """

    def __init__(self, weight: float = 4.0, trigger_dist: float = 1.0):
        super().__init__(weight)
        self.trigger_dist = trigger_dist

    def score(
        self,
        trajectories: np.ndarray,
        controls: np.ndarray,
        costmap: np.ndarray,
        costmap_origin: tuple,
        goal: np.ndarray,
        global_path: np.ndarray
    ) -> np.ndarray:
        if goal is None or len(goal) < 3:
            return np.zeros(trajectories.shape[0], dtype=np.float32)

        end_xy = trajectories[:, -1, :2]
        dist_to_goal = np.linalg.norm(end_xy - goal[:2], axis=-1)

        end_theta = trajectories[:, -1, 2]
        goal_theta = goal[2]

        angle_err = np.abs((end_theta - goal_theta + np.pi) % (2.0 * np.pi) - np.pi)

        # Only activate when close to goal
        active_mask = dist_to_goal <= self.trigger_dist
        costs = np.zeros_like(dist_to_goal)
        costs[active_mask] = self.weight * angle_err[active_mask]
        return costs


class SmoothnessCritic(BaseCritic):
    """
    Penalizes aggressive acceleration, deceleration, and high angular jerk.
    """

    def __init__(self, weight: float = 2.0):
        super().__init__(weight)

    def score(
        self,
        trajectories: np.ndarray,
        controls: np.ndarray,
        costmap: np.ndarray,
        costmap_origin: tuple,
        goal: np.ndarray,
        global_path: np.ndarray
    ) -> np.ndarray:
        # Control acceleration delta across time steps: (K, T-1, 2)
        ctrl_diffs = np.diff(controls, axis=1)
        jerk = np.sum(ctrl_diffs**2, axis=(1, 2))
        return self.weight * jerk


class ConstraintCritic(BaseCritic):
    """
    Hard constraint enforcer for invalid kinodynamic parameters.
    """

    def __init__(self, weight: float = 100.0, max_v: float = 1.2, max_omega: float = 2.0):
        super().__init__(weight)
        self.max_v = max_v
        self.max_omega = max_omega

    def score(
        self,
        trajectories: np.ndarray,
        controls: np.ndarray,
        costmap: np.ndarray,
        costmap_origin: tuple,
        goal: np.ndarray,
        global_path: np.ndarray
    ) -> np.ndarray:
        v_violations = np.maximum(0.0, np.abs(controls[:, :, 0]) - self.max_v)
        omega_violations = np.maximum(0.0, np.abs(controls[:, :, 1]) - self.max_omega)
        return self.weight * np.sum(v_violations + omega_violations, axis=1)


class SemanticSpeedCritic(BaseCritic):
    """
    Encourages speed reduction when traversing medium-cost terrain (e.g., grass/roughness).
    """

    def __init__(self, weight: float = 4.0, friction_cost_range: tuple = (40.0, 150.0)):
        super().__init__(weight)
        self.low_c, self.high_c = friction_cost_range

    def score(
        self,
        trajectories: np.ndarray,
        controls: np.ndarray,
        costmap: np.ndarray,
        costmap_origin: tuple,
        goal: np.ndarray,
        global_path: np.ndarray
    ) -> np.ndarray:
        if costmap is None or costmap.size == 0:
            return np.zeros(trajectories.shape[0], dtype=np.float32)

        ox, oy, res = costmap_origin
        h, w = costmap.shape

        col_idx = np.clip(((trajectories[:, :-1, 0] - ox) / res).astype(np.int32), 0, w - 1)
        row_idx = np.clip(((trajectories[:, :-1, 1] - oy) / res).astype(np.int32), 0, h - 1)

        costs = costmap[row_idx, col_idx].astype(np.float32)
        in_friction = (costs >= self.low_c) & (costs <= self.high_c)

        # High velocity in friction terrain receives penalty
        forward_v = controls[:, :, 0]
        speed_penalties = in_friction * (forward_v ** 2)

        return self.weight * np.sum(speed_penalties, axis=1)
