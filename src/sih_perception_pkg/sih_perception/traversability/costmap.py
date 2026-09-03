"""
2.5D Local Grid Costmap Generator with Obstacle Inflation.
Projects 3D point traversability decisions into an ego-centric 2D metric grid
compatible with standard reactive planners (ROS2 Nav2 / custom A* / TEB).
"""

from dataclasses import dataclass
from typing import Tuple, Optional, Dict, Any
import numpy as np
import cv2

from .decision_engine import TraversabilityDecision, TraversabilityType
from .vehicle_profile import TrackedVehicleProfile


@dataclass
class CostmapConfig:
    """Grid specifications for 2.5D Local Costmap."""
    resolution: float = 0.05       # 5 cm per cell
    size_x: float = 8.0            # 8 meters along forward axis (X: -1.0m to +7.0m)
    size_y: float = 8.0            # 8 meters lateral (Y: -4.0m to +4.0m)
    origin_x: float = -1.0         # X minimum relative to robot base_link
    origin_y: float = -4.0         # Y minimum relative to robot base_link


class LocalCostmap:
    """
    Constructs and maintains a 2.5D local costmap around the UGV.
    """

    def __init__(
        self,
        config: Optional[CostmapConfig] = None,
        vehicle_profile: Optional[TrackedVehicleProfile] = None
    ):
        self.cfg = config or CostmapConfig()
        self.profile = vehicle_profile or TrackedVehicleProfile()

        # Grid dimensions in cells
        self.cells_x = int(np.round(self.cfg.size_x / self.cfg.resolution))
        self.cells_y = int(np.round(self.cfg.size_y / self.cfg.resolution))
        
        # Build circular inflation kernel based on robot footprint & inflation radius
        inflate_cells = int(np.ceil(self.profile.inflation_radius / self.cfg.resolution))
        self.inflation_kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE,
            (2 * inflate_cells + 1, 2 * inflate_cells + 1)
        )

    def world_to_grid(self, x: np.ndarray, y: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Convert robot frame metric coordinates (X, Y) to costmap grid cell indices (gx, gy).
        
        Returns:
            gx: cell index along X axis (0 <= gx < cells_x)
            gy: cell index along Y axis (0 <= gy < cells_y)
            in_bounds: boolean mask of points inside grid boundary
        """
        gx = np.floor((x - self.cfg.origin_x) / self.cfg.resolution).astype(np.int32)
        gy = np.floor((y - self.cfg.origin_y) / self.cfg.resolution).astype(np.int32)

        in_bounds = (gx >= 0) & (gx < self.cells_x) & (gy >= 0) & (gy < self.cells_y)
        return gx, gy, in_bounds

    def grid_to_world(self, gx: int, gy: int) -> Tuple[float, float]:
        """Convert cell index (gx, gy) to robot frame metric coordinates (center of cell)."""
        x = self.cfg.origin_x + (gx + 0.5) * self.cfg.resolution
        y = self.cfg.origin_y + (gy + 0.5) * self.cfg.resolution
        return float(x), float(y)

    def generate(
        self,
        pts_robot: np.ndarray,
        valid_mask: np.ndarray,
        decision: TraversabilityDecision
    ) -> np.ndarray:
        """
        Generate 2.5D grid costmap [0..254] from 3D points and traversability decisions.
        
        Returns:
            inflated_grid: (cells_x, cells_y) uint8 array of navigation costs
        """
        # Default initialization: 0 (free) for observed area, 255 for unseen
        raw_grid = np.zeros((self.cells_x, self.cells_y), dtype=np.uint8)
        max_height_grid = np.zeros((self.cells_x, self.cells_y), dtype=np.float32)

        X = pts_robot[..., 0][valid_mask]
        Y = pts_robot[..., 1][valid_mask]
        costs = decision.cost_map[valid_mask]
        heights = decision.delta_h[valid_mask]

        if len(X) == 0:
            return raw_grid

        gx, gy, in_bounds = self.world_to_grid(X, Y)
        gx_valid = gx[in_bounds]
        gy_valid = gy[in_bounds]
        costs_valid = costs[in_bounds]
        heights_valid = heights[in_bounds]

        # Vectorized maximum cost and height accumulation via NumPy ufunc
        flat_grid = raw_grid.reshape(-1)
        flat_height = max_height_grid.reshape(-1)
        flat_idx = gx_valid * self.cells_y + gy_valid

        np.maximum.at(flat_grid, flat_idx, costs_valid)
        np.maximum.at(flat_height, flat_idx, heights_valid)

        # Inflate lethal obstacles (cost >= 254) using morphological dilation
        lethal_binary = (raw_grid >= 254).astype(np.uint8)
        inflated_lethal = cv2.dilate(lethal_binary, self.inflation_kernel)

        # Build final composite costmap
        final_grid = raw_grid.copy()
        # Points inside inflation zone get elevated cost for safe path planning margin
        inflation_zone = (inflated_lethal > 0) & (raw_grid < 254)
        final_grid[inflation_zone] = np.maximum(final_grid[inflation_zone], 180)
        final_grid[raw_grid >= 254] = 254

        return final_grid

    def to_ros_occupancy_grid(self, costmap_grid: np.ndarray) -> np.ndarray:
        """
        Convert [0..254] costmap to ROS standard OccupancyGrid int8 array [-1..100].
        -1: Unknown
        0: Completely free
        1..99: Traversable / Inscribed cost
        100: Lethal obstacle
        """
        ros_grid = np.full(costmap_grid.shape, -1, dtype=np.int8)
        
        free_mask = (costmap_grid == 0)
        ros_grid[free_mask] = 0
        
        trav_mask = (costmap_grid > 0) & (costmap_grid < 254)
        # Scale 1..253 to 1..99
        ros_grid[trav_mask] = (1 + (costmap_grid[trav_mask].astype(np.float32) / 253.0) * 98).astype(np.int8)
        
        lethal_mask = (costmap_grid >= 254)
        ros_grid[lethal_mask] = 100
        
        return ros_grid
