"""
Hybrid Semantic + 3D Geometric Traversability Decision Engine.
Evaluates terrain against tracked vehicle physics envelope to classify every pixel
and 3D point as Traversable (Run-Over Allowed), Lethal Obstacle, or Negative Hazard.
"""

from enum import IntEnum
from dataclasses import dataclass
from typing import Optional, Dict, Tuple
import numpy as np

from ..segmentation.taxonomy import SemanticClass
from ..core.geometry import GroundPlane
from .vehicle_profile import TrackedVehicleProfile


class TraversabilityType(IntEnum):
    """Actionable traversability classification for reactive planning."""
    FREE = 0                    # Smooth flat ground, clear to drive (Cost: 0)
    RUN_OVER_TRAVERSABLE = 1    # Small debris, low step <= H_step, tall soft grass (Cost: 10-50)
    LETHAL_OBSTACLE = 2         # Rigid obstacles > H_step (rocks, tree trunks, walls) (Cost: 254)
    NEGATIVE_HAZARD = 3         # Ditches, drop-offs, trenches (Cost: 255)
    INCLINE_HAZARD = 4          # Slopes steeper than tracked vehicle capability (Cost: 254)
    UNKNOWN = 5                 # Occluded, invalid depth, or out of range


TRAVERSABILITY_NAMES: Dict[int, str] = {
    TraversabilityType.FREE: "Free Drivable",
    TraversabilityType.RUN_OVER_TRAVERSABLE: "Traversable (Run-Over Allowed)",
    TraversabilityType.LETHAL_OBSTACLE: "Lethal Obstacle (Avoid)",
    TraversabilityType.NEGATIVE_HAZARD: "Negative Hazard (Ditch/Hole)",
    TraversabilityType.INCLINE_HAZARD: "Incline Hazard (Too Steep)",
    TraversabilityType.UNKNOWN: "Unknown / Unseen",
}

# Color palette for traversability overlay (RGB)
TRAVERSABILITY_COLORS_RGB: Dict[int, Tuple[int, int, int]] = {
    TraversabilityType.FREE: (46, 204, 113),                  # Green
    TraversabilityType.RUN_OVER_TRAVERSABLE: (52, 152, 219),  # Light Blue / Cyan
    TraversabilityType.LETHAL_OBSTACLE: (231, 76, 60),       # Bright Red
    TraversabilityType.NEGATIVE_HAZARD: (155, 89, 182),      # Purple / Magenta
    TraversabilityType.INCLINE_HAZARD: (230, 126, 34),       # Orange
    TraversabilityType.UNKNOWN: (127, 140, 141),             # Gray
}


@dataclass
class TraversabilityDecision:
    """Complete output of the traversability evaluation engine."""
    traversability_map: np.ndarray    # (H, W) TraversabilityType enum integers
    cost_map: np.ndarray              # (H, W) uint8 cost [0..255] (254=lethal, 255=fatal hazard)
    delta_h: np.ndarray               # (H, W) float height differential in meters
    can_run_over_mask: np.ndarray     # (H, W) bool: features safe to run over
    lethal_mask: np.ndarray           # (H, W) bool: lethal obstacles requiring avoidance
    negative_hazard_mask: np.ndarray  # (H, W) bool: drop-offs / ditches
    ground_plane: GroundPlane         # Estimated ground plane


class TraversabilityEngine:
    """
    Fuses 3D geometric depth metrics and semantic segmentation classifications
    with tracked vehicle physics parameters.
    """

    def __init__(self, vehicle_profile: Optional[TrackedVehicleProfile] = None):
        self.profile = vehicle_profile or TrackedVehicleProfile()

    def evaluate(
        self,
        semantic_mask: np.ndarray,
        pts_robot: np.ndarray,
        valid_mask: np.ndarray,
        ground_plane: GroundPlane,
    ) -> TraversabilityDecision:
        """
        Execute multi-factor traversability classification.
        
        Args:
            semantic_mask: (H, W) uint8 semantic classes (0..3)
            pts_robot: (H, W, 3) 3D coordinates in robot frame (meters)
            valid_mask: (H, W) boolean valid depth mask
            ground_plane: Estimated GroundPlane model
            
        Returns:
            TraversabilityDecision object with per-pixel types and cost scores
        """
        h, w = semantic_mask.shape
        trav_map = np.full((h, w), TraversabilityType.UNKNOWN, dtype=np.uint8)
        cost_map = np.full((h, w), 255, dtype=np.uint8)

        # 1. Compute relative height delta_h = Z_r - Z_ground
        X = pts_robot[..., 0]
        Y = pts_robot[..., 1]
        Z = pts_robot[..., 2]

        z_ground = ground_plane.height_at(X, Y)
        delta_h = np.zeros((h, w), dtype=np.float64)
        delta_h[valid_mask] = Z[valid_mask] - z_ground[valid_mask]

        # 2. Rule 1: Negative Hazards (Drop-offs / Ditches)
        # Height below ground by more than max_drop_step
        ditch_condition = valid_mask & (
            (delta_h < -self.profile.max_drop_step) |
            (semantic_mask == SemanticClass.NEGATIVE_HAZARD)
        )
        trav_map[ditch_condition] = TraversabilityType.NEGATIVE_HAZARD
        cost_map[ditch_condition] = 255

        # 3. Rule 2: Rigid Obstacles (Rocks, Tree Trunks, Barriers)
        rigid_mask = valid_mask & ~ditch_condition & (semantic_mask == SemanticClass.RIGID_OBSTACLE)
        
        # 3a. Rigid Obstacle > H_step -> Lethal
        lethal_rigid = rigid_mask & (delta_h > self.profile.max_climb_step)
        trav_map[lethal_rigid] = TraversabilityType.LETHAL_OBSTACLE
        cost_map[lethal_rigid] = 254

        # 3b. Rigid Feature <= H_step -> Traversable / Run Over by tracks
        climbable_rigid = rigid_mask & (delta_h <= self.profile.max_climb_step) & (delta_h > 0.03)
        trav_map[climbable_rigid] = TraversabilityType.RUN_OVER_TRAVERSABLE
        # Scaling cost with obstacle height: small bump = low cost, near-limit step = moderate cost
        step_ratio = np.clip(delta_h[climbable_rigid] / self.profile.max_climb_step, 0.0, 1.0)
        cost_map[climbable_rigid] = (25 + step_ratio * 55).astype(np.uint8)

        # 4. Rule 3: Soft Traversable (Tall Grass, Weeds, Brush)
        soft_mask = valid_mask & ~ditch_condition & (semantic_mask == SemanticClass.SOFT_TRAVERSABLE)
        
        # 4a. Crushable vegetation <= soft_vegetation_max_height -> Run-over allowed
        crushable_soft = soft_mask & (delta_h <= self.profile.soft_vegetation_max_height)
        trav_map[crushable_soft] = TraversabilityType.RUN_OVER_TRAVERSABLE
        cost_map[crushable_soft] = 15  # Slight friction penalty, but completely safe for continuous tracks

        # 4b. Dense tall brush / trees > soft_vegetation_max_height -> Lethal
        tall_soft = soft_mask & (delta_h > self.profile.soft_vegetation_max_height)
        trav_map[tall_soft] = TraversabilityType.LETHAL_OBSTACLE
        cost_map[tall_soft] = 254

        # 5. Rule 4: Free Drivable Ground (Soil, Pavement, Gravel)
        free_mask = valid_mask & ~ditch_condition & (semantic_mask == SemanticClass.FREE_DRIVABLE)
        
        # Flat surface within normal micro-roughness
        smooth_free = free_mask & (np.abs(delta_h) <= 0.05)
        trav_map[smooth_free] = TraversabilityType.FREE
        cost_map[smooth_free] = 0

        # Slightly uneven ground within track climb limit
        uneven_free = free_mask & (delta_h > 0.05) & (delta_h <= self.profile.max_climb_step)
        trav_map[uneven_free] = TraversabilityType.RUN_OVER_TRAVERSABLE
        cost_map[uneven_free] = 10

        # Physical safety override: Geometric height > max_climb_step is ALWAYS lethal
        excessive_height = free_mask & (delta_h > self.profile.max_climb_step)
        trav_map[excessive_height] = TraversabilityType.LETHAL_OBSTACLE
        cost_map[excessive_height] = 254

        # 6. Overall Ground Slope Check
        if ground_plane.slope_deg > self.profile.max_slope_deg:
            # Entire front slope is hazardous
            slope_hazard = valid_mask & (trav_map == TraversabilityType.FREE)
            trav_map[slope_hazard] = TraversabilityType.INCLINE_HAZARD
            cost_map[slope_hazard] = 254

        can_run_over = (trav_map == TraversabilityType.RUN_OVER_TRAVERSABLE)
        lethal = (trav_map == TraversabilityType.LETHAL_OBSTACLE) | (trav_map == TraversabilityType.INCLINE_HAZARD)
        negative = (trav_map == TraversabilityType.NEGATIVE_HAZARD)

        return TraversabilityDecision(
            traversability_map=trav_map,
            cost_map=cost_map,
            delta_h=delta_h,
            can_run_over_mask=can_run_over,
            lethal_mask=lethal,
            negative_hazard_mask=negative,
            ground_plane=ground_plane
        )
