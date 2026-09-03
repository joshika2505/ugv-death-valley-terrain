"""
Data classes and cell cost definitions for the multi-layer costmap.
"""

from dataclasses import dataclass
from enum import IntEnum
from typing import Dict


class CostmapCellType(IntEnum):
    """Standard costmap 8-bit cell cost values conforming to Nav2 standards."""
    FREE_SPACE = 0
    LOW_FRICTION_BRUSH = 50
    MEDIUM_FRICTION_BRUSH = 70
    HIGH_FRICTION_BRUSH = 80
    ELEVATED_ROUGHNESS = 120
    STEEP_SLOPE = 160
    INFLATED_OBSTACLE_MIN = 180
    INFLATED_OBSTACLE_MAX = 252
    LETHAL_PERSON = 253
    LETHAL_OBSTACLE = 254
    NO_INFORMATION = 255  # or -1 in int8 OccupancyGrid


@dataclass
class CostmapConfig:
    """Configuration parameters for 2.5D elevation and semantic costmap generation."""
    resolution: float = 0.1               # Meters per grid cell
    width_m: float = 20.0                 # Total grid width in meters
    height_m: float = 20.0                # Total grid height in meters
    robot_radius: float = 0.35            # Physical radius of UGV in meters
    inflation_radius: float = 0.70        # Total inflation decay radius in meters
    max_traversable_slope_deg: float = 25.0  # Slope threshold for lethal grade
    max_roughness_m: float = 0.08         # Standard deviation of height residuals threshold
    max_step_height_m: float = 0.15       # Step/drop height threshold (ground clearance limit)
    ground_clearance_m: float = 0.15      # Chassis clearance
    decay_factor: float = 3.0             # Exponential decay rate for obstacle inflation


# Semantic class string to Costmap cost mapping
DEFAULT_SEMANTIC_COST_MAP: Dict[str, int] = {
    # Free / Highly Traversable
    'road': CostmapCellType.FREE_SPACE,
    'path': CostmapCellType.FREE_SPACE,
    'dirt': CostmapCellType.FREE_SPACE,
    'trail': CostmapCellType.FREE_SPACE,
    'cleared_ground': CostmapCellType.FREE_SPACE,

    # Moderate Friction / Soft Foliage (drive-through at reduced speed)
    'grass': CostmapCellType.LOW_FRICTION_BRUSH,
    'tall_grass': CostmapCellType.MEDIUM_FRICTION_BRUSH,
    'bush': CostmapCellType.HIGH_FRICTION_BRUSH,
    'shrub': CostmapCellType.HIGH_FRICTION_BRUSH,
    'gravel': CostmapCellType.LOW_FRICTION_BRUSH,

    # Lethal Geometric / Tactical Obstacles
    'boulder': CostmapCellType.LETHAL_OBSTACLE,
    'rock': CostmapCellType.LETHAL_OBSTACLE,
    'tree': CostmapCellType.LETHAL_OBSTACLE,
    'trunk': CostmapCellType.LETHAL_OBSTACLE,
    'log': CostmapCellType.LETHAL_OBSTACLE,
    'wall': CostmapCellType.LETHAL_OBSTACLE,
    'barrier': CostmapCellType.LETHAL_OBSTACLE,
    'vehicle': CostmapCellType.LETHAL_OBSTACLE,
    'ditch': CostmapCellType.LETHAL_OBSTACLE,
    'cliff': CostmapCellType.LETHAL_OBSTACLE,

    # Safety Critical Dynamic Obstacles
    'person': CostmapCellType.LETHAL_PERSON,
    'human': CostmapCellType.LETHAL_PERSON,
    'animal': CostmapCellType.LETHAL_PERSON,
}
