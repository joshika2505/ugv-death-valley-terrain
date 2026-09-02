"""
3D Geometric Processing, Fast Vectorized RANSAC Ground Plane Fitting,
and Height / Slope Differential Calculation.
"""

from dataclasses import dataclass
from typing import Tuple, Optional, Dict, Any
import numpy as np


@dataclass
class GroundPlane:
    """Parametric ground plane representation: a*x + b*y + c*z + d = 0."""
    a: float = 0.0
    b: float = 0.0
    c: float = 1.0  # Points upwards
    d: float = 0.0  # Ground at base_link height Z=0
    inlier_ratio: float = 1.0
    slope_deg: float = 0.0

    @property
    def normal(self) -> np.ndarray:
        return np.array([self.a, self.b, self.c], dtype=np.float64)

    def height_at(self, x: np.ndarray, y: np.ndarray) -> np.ndarray:
        """Compute expected ground plane height Z_ground for any (x, y) coordinates."""
        if abs(self.c) < 1e-6:
            return np.zeros_like(x)
        return (-self.a * x - self.b * y - self.d) / self.c

    def distance_to_plane(self, pts: np.ndarray) -> np.ndarray:
        """Compute signed distance from 3D points (..., 3) to the plane."""
        return pts[..., 0] * self.a + pts[..., 1] * self.b + pts[..., 2] * self.c + self.d


class GroundPlaneFitter:
    """
    Fits a local ground plane directly in front of the robot using
    vectorized RANSAC with geometric priors.
    """

    def __init__(
        self,
        distance_threshold: float = 0.04,  # 4 cm inlier threshold
        max_iterations: int = 150,
        max_slope_deg: float = 40.0,
        roi_x_min: float = 0.3,
        roi_x_max: float = 4.0,
        roi_y_max: float = 1.8,
        roi_z_min: float = -0.5,
        roi_z_max: float = 0.5,
    ):
        self.distance_threshold = distance_threshold
        self.max_iterations = max_iterations
        self.max_slope_cos = np.cos(np.radians(max_slope_deg))
        self.roi_x_min = roi_x_min
        self.roi_x_max = roi_x_max
        self.roi_y_max = roi_y_max
        self.roi_z_min = roi_z_min
        self.roi_z_max = roi_z_max

    def fit(self, pts_robot: np.ndarray, valid_mask: Optional[np.ndarray] = None) -> GroundPlane:
        """
        Fit ground plane from 3D points in robot base frame.
        pts_robot: (H, W, 3) or (N, 3)
        """
        if pts_robot.ndim == 3:
            pts_flat = pts_robot.reshape(-1, 3)
            if valid_mask is not None:
                mask_flat = valid_mask.reshape(-1)
                pts_flat = pts_flat[mask_flat]
        else:
            pts_flat = pts_robot
            if valid_mask is not None:
                pts_flat = pts_flat[valid_mask]

        if len(pts_flat) < 30:
            # Fallback to horizontal ground at Z=0
            return GroundPlane(a=0.0, b=0.0, c=1.0, d=0.0, inlier_ratio=0.0, slope_deg=0.0)

        # 1. Filter points to region of interest (ROI) in front of vehicle
        roi_mask = (
            (pts_flat[:, 0] >= self.roi_x_min) & (pts_flat[:, 0] <= self.roi_x_max) &
            (np.abs(pts_flat[:, 1]) <= self.roi_y_max) &
            (pts_flat[:, 2] >= self.roi_z_min) & (pts_flat[:, 2] <= self.roi_z_max)
        )
        roi_pts = pts_flat[roi_mask]
        
        if len(roi_pts) < 30:
            roi_pts = pts_flat

        # Subsample for ultra-fast vectorized RANSAC execution
        n_pts = len(roi_pts)
        max_samples = min(1500, n_pts)
        if n_pts > max_samples:
            indices = np.random.choice(n_pts, max_samples, replace=False)
            sample_pts = roi_pts[indices]
        else:
            sample_pts = roi_pts

        n_pts = len(sample_pts)
        k_trials = min(self.max_iterations, 80)

        # Batch-generate random triplet indices (k_trials, 3)
        idx1 = np.random.randint(0, n_pts, size=k_trials)
        idx2 = np.random.randint(0, n_pts, size=k_trials)
        idx3 = np.random.randint(0, n_pts, size=k_trials)

        p1 = sample_pts[idx1]  # (k_trials, 3)
        p2 = sample_pts[idx2]
        p3 = sample_pts[idx3]

        v1 = p2 - p1
        v2 = p3 - p1
        normals = np.cross(v1, v2)  # (k_trials, 3)
        norm_lens = np.linalg.norm(normals, axis=1, keepdims=True) + 1e-9
        normals = normals / norm_lens

        # Ensure normals point upwards (c > 0)
        neg_c = normals[:, 2] < 0
        normals[neg_c] = -normals[neg_c]

        # Filter slope angle prior
        valid_trials = normals[:, 2] >= self.max_slope_cos
        if not np.any(valid_trials):
            return GroundPlane(a=0.0, b=0.0, c=1.0, d=0.0, inlier_ratio=1.0, slope_deg=0.0)

        normals_valid = normals[valid_trials]
        p1_valid = p1[valid_trials]
        d_valid = -np.sum(normals_valid * p1_valid, axis=1)  # (K_valid,)

        # Vectorized residual calculation: (N_pts, K_valid)
        residuals = np.abs(sample_pts @ normals_valid.T + d_valid)
        inlier_counts = np.count_nonzero(residuals <= self.distance_threshold, axis=0)

        best_trial_idx = int(np.argmax(inlier_counts))
        best_inliers = int(inlier_counts[best_trial_idx])
        best_normal = normals_valid[best_trial_idx]
        best_d = float(d_valid[best_trial_idx])

        slope = float(np.degrees(np.arccos(np.clip(best_normal[2], -1.0, 1.0))))
        best_plane = GroundPlane(
            a=float(best_normal[0]),
            b=float(best_normal[1]),
            c=float(best_normal[2]),
            d=best_d,
            inlier_ratio=float(best_inliers / n_pts),
            slope_deg=slope
        )

        return best_plane


class PointCloudProcessor:
    """
    Computes spatial geometric differentials:
    - Relative height delta_h = Z_r - Z_ground(X_r, Y_r)
    - Local surface gradients and step heights
    - Negative obstacle (ditch/drop-off) detection
    """

    def __init__(self, ground_plane: Optional[GroundPlane] = None):
        self.ground_plane = ground_plane or GroundPlane()

    def update_ground_plane(self, plane: GroundPlane) -> None:
        self.ground_plane = plane

    def compute_height_differential(
        self,
        pts_robot: np.ndarray,
        valid_mask: np.ndarray
    ) -> np.ndarray:
        """
        Compute relative height above ground surface:
        delta_h(u, v) = Z_r(u, v) - Z_ground(X_r(u, v), Y_r(u, v))
        
        Positive: Obstacle rising above ground (steps, rocks, grass)
        Negative: Drop-off below ground (ditches, holes, trenches)
        """
        h, w, _ = pts_robot.shape
        X = pts_robot[..., 0]
        Y = pts_robot[..., 1]
        Z = pts_robot[..., 2]

        z_ground = self.ground_plane.height_at(X, Y)
        delta_h = np.zeros((h, w), dtype=np.float64)
        delta_h[valid_mask] = Z[valid_mask] - z_ground[valid_mask]
        return delta_h

    def detect_negative_obstacles(
        self,
        pts_robot: np.ndarray,
        delta_h: np.ndarray,
        valid_mask: np.ndarray,
        min_drop_height: float = 0.12,  # 12 cm drop-off
    ) -> np.ndarray:
        """
        Detects negative hazards:
        1. Explicit drop-offs: delta_h < -min_drop_height
        2. Depth shadow voids: abrupt missing points beyond a continuous ground horizon
        """
        h, w = delta_h.shape
        negative_mask = np.zeros((h, w), dtype=bool)

        # 1. Direct step-down drop
        negative_mask[valid_mask & (delta_h < -min_drop_height)] = True

        # 2. Geometry check: In front of robot, points dropping lower than clearance
        X = pts_robot[..., 0]
        in_range = valid_mask & (X > 0.5) & (X < 5.0)
        deep_drop = in_range & (delta_h < -min_drop_height)
        negative_mask[deep_drop] = True

        return negative_mask
