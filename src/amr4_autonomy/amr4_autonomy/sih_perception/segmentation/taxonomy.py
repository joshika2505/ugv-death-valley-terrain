"""
Tracked UGV Semantic Segmentation Taxonomy and Color Palette.
"""

from enum import IntEnum
from typing import Dict, Tuple
import numpy as np


class SemanticClass(IntEnum):
    """Functional semantic classification for tracked vehicle navigation."""
    FREE_DRIVABLE = 0      # Flat dirt trail, packed earth, flat pavement, gravel road
    SOFT_TRAVERSABLE = 1   # Tall grass, light weeds, brush, small soft twigs (crushable)
    RIGID_OBSTACLE = 2     # Boulders, rocks > H_step, tree trunks, solid barriers, posts
    NEGATIVE_HAZARD = 3    # Ditches, trenches, step-downs, water/deep mud, cliffs


CLASS_NAMES: Dict[int, str] = {
    SemanticClass.FREE_DRIVABLE: "Free Drivable (Soil/Pavement)",
    SemanticClass.SOFT_TRAVERSABLE: "Soft Traversable (Grass/Brush)",
    SemanticClass.RIGID_OBSTACLE: "Rigid Lethal Obstacle (Rock/Tree)",
    SemanticClass.NEGATIVE_HAZARD: "Negative Hazard (Ditch/Drop-off)",
}

# Distinct BGR / RGB Color Palette for visualization
# 0: Green (Safe Drivable)
# 1: Lime / Cyan-Green (Soft crushable)
# 2: Crimson Red (Lethal Obstacle)
# 3: Deep Magenta / Purple (Negative Ditch Hazard)
CLASS_COLORS_RGB: Dict[int, Tuple[int, int, int]] = {
    SemanticClass.FREE_DRIVABLE: (46, 204, 113),     # Emerald Green
    SemanticClass.SOFT_TRAVERSABLE: (241, 196, 15),  # Sunflower Yellow
    SemanticClass.RIGID_OBSTACLE: (231, 76, 60),     # Alizarin Red
    SemanticClass.NEGATIVE_HAZARD: (155, 89, 182),   # Amethyst Purple
}

CLASS_COLORS_BGR: Dict[int, Tuple[int, int, int]] = {
    k: (v[2], v[1], v[0]) for k, v in CLASS_COLORS_RGB.items()
}

CLASS_COLORS = CLASS_COLORS_RGB



def colorize_mask(mask: np.ndarray) -> np.ndarray:
    """Convert (H, W) class integer mask to (H, W, 3) RGB colorized image."""
    h, w = mask.shape
    color_img = np.zeros((h, w, 3), dtype=np.uint8)
    for class_id, color in CLASS_COLORS_RGB.items():
        color_img[mask == class_id] = color
    return color_img
