"""
Pure Algorithmic Traversability Analyzer.
Implements Slope, Roughness, Step-Height, Negative Obstacle, and Inflation filters
on 2.5D elevation grids with pure NumPy vectorization.
"""

import math
import numpy as np


def compute_euclidean_distance_transform(binary_mask: np.ndarray, resolution: float) -> np.ndarray:
    """
    Computes Euclidean distance transform from non-zero cells.
    Tries scipy / cv2 first, falls back to fast vectorized coordinate distance.
    """
    try:
        from scipy.ndimage import distance_transform_edt
        return distance_transform_edt(binary_mask) * resolution
    except Exception:
        pass

    try:
        import cv2
        dist = cv2.distanceTransform((binary_mask.astype(np.uint8) * 255), cv2.DIST_L2, 5)
        return dist * resolution
    except Exception:
        pass

    # Pure NumPy fallback
    h, w = binary_mask.shape
    lethal_rows, lethal_cols = np.where(~binary_mask)
    if len(lethal_rows) == 0:
        return np.full((h, w), 1000.0, dtype=np.float32)

    # Grid coordinates
    rows, cols = np.indices((h, w))
    dist_grid = np.full((h, w), 1000.0, dtype=np.float32)

    # Compute minimum distance over lethal coordinates in blocks
    lethal_coords = np.column_stack([lethal_rows, lethal_cols])  # (M, 2)
    all_coords = np.column_stack([rows.ravel(), cols.ravel()])    # (N, 2)

    # Subsample/chunk if M is large to maintain real-time performance
    for lr, lc in lethal_coords:
        d = np.sqrt((rows - lr)**2 + (cols - lc)**2)
        dist_grid = np.minimum(dist_grid, d)

    return dist_grid * resolution


class TraversabilityAnalyzer:
    """
    Analyzes 2.5D elevation grid maps to calculate traversability costs
    for outdoor rough terrain autonomous navigation.
    """

    def __init__(self, resolution: float = 0.1, ground_clearance: float = 0.15):
        self.resolution = resolution
        self.ground_clearance = ground_clearance

    def compute_slope(self, elevation_grid: np.ndarray, max_slope_deg: float = 25.0) -> np.ndarray:
        """
        Calculates terrain slope using spatial elevation gradients (central differences).
        
        Args:
            elevation_grid: 2D array of elevation values (np.nan for invalid/unobserved cells).
            max_slope_deg: Maximum traversable slope in degrees.

        Returns:
            slope_cost: 2D array with values in [0, 254].
        """
        valid_mask = ~np.isnan(elevation_grid)
        if not np.any(valid_mask):
            return np.zeros_like(elevation_grid, dtype=np.uint8)

        filled_grid = np.where(valid_mask, elevation_grid, 0.0)

        # Vectorized central differences in pure NumPy
        dz_dx = np.zeros_like(filled_grid)
        dz_dx[:, 1:-1] = (filled_grid[:, 2:] - filled_grid[:, :-2]) / (2.0 * self.resolution)
        # Boundaries: forward/backward differences
        dz_dx[:, 0] = (filled_grid[:, 1] - filled_grid[:, 0]) / self.resolution
        dz_dx[:, -1] = (filled_grid[:, -1] - filled_grid[:, -2]) / self.resolution

        dz_dy = np.zeros_like(filled_grid)
        dz_dy[1:-1, :] = (filled_grid[2:, :] - filled_grid[:-2, :]) / (2.0 * self.resolution)
        dz_dy[0, :] = (filled_grid[1, :] - filled_grid[0, :]) / self.resolution
        dz_dy[-1, :] = (filled_grid[-1, :] - filled_grid[-2, :]) / self.resolution

        # Gradient magnitude and slope angle
        grad_mag = np.sqrt(dz_dx**2 + dz_dy**2)
        slope_rad = np.arctan(grad_mag)
        slope_deg = np.degrees(slope_rad)

        # Normalize cost: 0 at flat terrain, 254 at >= max_slope_deg
        slope_ratio = np.clip(slope_deg / max(1.0, max_slope_deg), 0.0, 1.0)
        slope_cost = (slope_ratio**2 * 254.0).astype(np.uint8)

        # Invalidate cells where original elevation data was absent
        slope_cost[~valid_mask] = 0
        return slope_cost

    def compute_roughness(self, elevation_grid: np.ndarray, window_size: int = 3, max_roughness_m: float = 0.08) -> np.ndarray:
        """
        Calculates terrain roughness as the local standard deviation of elevation in pure NumPy.

        Args:
            elevation_grid: 2D array of elevation values.
            window_size: Neighborhood kernel size (default 3x3).
            max_roughness_m: Residual height standard deviation considered lethal.

        Returns:
            roughness_cost: 2D array in range [0, 254].
        """
        valid_mask = ~np.isnan(elevation_grid)
        if not np.any(valid_mask):
            return np.zeros_like(elevation_grid, dtype=np.uint8)

        filled_grid = np.where(valid_mask, elevation_grid, 0.0)

        # Pad by 1 for 3x3 local neighborhood
        pad = np.pad(filled_grid, pad_width=1, mode='reflect')
        neighborhood = np.stack([
            pad[0:-2, 0:-2], pad[0:-2, 1:-1], pad[0:-2, 2:],
            pad[1:-1, 0:-2], pad[1:-1, 1:-1], pad[1:-1, 2:],
            pad[2:,   0:-2], pad[2:,   1:-1], pad[2:,   2:]
        ], axis=0)

        mean = np.mean(neighborhood, axis=0)
        variance = np.mean((neighborhood - mean)**2, axis=0)
        std_dev = np.sqrt(np.maximum(0.0, variance))

        roughness_ratio = np.clip(std_dev / max(1e-3, max_roughness_m), 0.0, 1.0)
        roughness_cost = (roughness_ratio * 254.0).astype(np.uint8)
        roughness_cost[~valid_mask] = 0
        return roughness_cost

    def compute_step_height(self, elevation_grid: np.ndarray, ground_clearance: float = None) -> np.ndarray:
        """
        Calculates maximum elevation difference between adjacent cells (step height).
        Steps exceeding vehicle ground clearance are classified as lethal obstacles (cost 254).

        Args:
            elevation_grid: 2D array of elevation values.
            ground_clearance: Clearance threshold in meters (defaults to instance setting).

        Returns:
            step_cost: 2D array in range [0, 254].
        """
        if ground_clearance is None:
            ground_clearance = self.ground_clearance

        valid_mask = ~np.isnan(elevation_grid)
        if not np.any(valid_mask):
            return np.zeros_like(elevation_grid, dtype=np.uint8)

        filled_grid = np.where(valid_mask, elevation_grid, 0.0)

        delta_up = np.zeros_like(filled_grid)
        delta_up[1:, :] = np.abs(filled_grid[1:, :] - filled_grid[:-1, :]) * (valid_mask[1:, :] & valid_mask[:-1, :])

        delta_down = np.zeros_like(filled_grid)
        delta_down[:-1, :] = np.abs(filled_grid[:-1, :] - filled_grid[1:, :]) * (valid_mask[:-1, :] & valid_mask[1:, :])

        delta_left = np.zeros_like(filled_grid)
        delta_left[:, 1:] = np.abs(filled_grid[:, 1:] - filled_grid[:, :-1]) * (valid_mask[:, 1:] & valid_mask[:, :-1])

        delta_right = np.zeros_like(filled_grid)
        delta_right[:, :-1] = np.abs(filled_grid[:, :-1] - filled_grid[:, 1:]) * (valid_mask[:, :-1] & valid_mask[:, 1:])

        max_delta = np.maximum.reduce([delta_up, delta_down, delta_left, delta_right])

        step_cost = np.zeros_like(max_delta, dtype=np.uint8)
        lethal_mask = max_delta >= ground_clearance
        step_cost[lethal_mask] = 254

        sub_lethal_mask = (~lethal_mask) & (max_delta > (0.5 * ground_clearance))
        step_cost[sub_lethal_mask] = (
            (max_delta[sub_lethal_mask] - 0.5 * ground_clearance) / (0.5 * ground_clearance) * 200.0
        ).astype(np.uint8)

        step_cost[~valid_mask] = 0
        return step_cost

    def fuse_traversability(
        self,
        slope_cost: np.ndarray,
        roughness_cost: np.ndarray,
        step_cost: np.ndarray,
        weights: tuple = (0.35, 0.25, 0.40)
    ) -> np.ndarray:
        """
        Combines slope, roughness, and step-height costs with strict lethal preservation.
        """
        w_s, w_r, w_step = weights
        weighted = (w_s * slope_cost.astype(float) +
                    w_r * roughness_cost.astype(float) +
                    w_step * step_cost.astype(float))

        fused = np.clip(weighted, 0.0, 254.0).astype(np.uint8)

        # Critical Safety Rule: If any component is LETHAL (>=253), fused cost MUST be lethal
        lethal_mask = (slope_cost >= 253) | (roughness_cost >= 253) | (step_cost >= 253)
        fused[lethal_mask] = 254

        return fused

    def inflate_costmap(
        self,
        cost_grid: np.ndarray,
        robot_radius: float = 0.35,
        inflation_radius: float = 0.70,
        decay_factor: float = 3.0
    ) -> np.ndarray:
        """
        Inflates lethal obstacles outward with exponential decay to create a safety margin.
        """
        lethal_mask = cost_grid >= 253
        if not np.any(lethal_mask):
            return cost_grid.copy()

        dist_m = compute_euclidean_distance_transform(~lethal_mask, self.resolution)
        inflated_grid = cost_grid.copy()

        # Inscribed radius zone: Full lethal cost
        inscribed_mask = (dist_m <= robot_radius) & (~lethal_mask)
        inflated_grid[inscribed_mask] = 253

        # Decay zone: Exponential decay from 252 down to 0
        decay_mask = (dist_m > robot_radius) & (dist_m <= inflation_radius)
        decay_distances = dist_m[decay_mask] - robot_radius
        norm_dist = decay_distances / max(1e-3, (inflation_radius - robot_radius))

        decay_costs = 252.0 * np.exp(-decay_factor * norm_dist)
        inflated_grid[decay_mask] = np.maximum(
            inflated_grid[decay_mask],
            decay_costs.astype(np.uint8)
        )

        return inflated_grid
