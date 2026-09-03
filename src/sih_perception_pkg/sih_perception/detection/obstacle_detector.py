"""
3D Spatial Obstacle Extraction and Localization.
Segments point cloud into discrete 3D spatial obstacles, computing real-world
metric bounding boxes, centroids, radial distance, azimuth bearing, and traversability verdict.
"""

from dataclasses import dataclass, asdict
from typing import List, Tuple, Optional, Dict, Any
import numpy as np
import cv2

from ..traversability.decision_engine import TraversabilityDecision, TraversabilityType, TRAVERSABILITY_NAMES
from ..traversability.vehicle_profile import TrackedVehicleProfile
from ..segmentation.taxonomy import CLASS_NAMES


@dataclass
class SpatialObstacle:
    """3D localized object with spatial metric properties and traversability status."""
    id: int
    centroid_x: float          # Forward distance from robot base (meters)
    centroid_y: float          # Lateral distance (left = +Y, right = -Y)
    centroid_z: float          # Height relative to ground contact (meters)
    radial_distance_m: float   # 2D ground range sqrt(X^2 + Y^2)
    azimuth_deg: float         # Angle relative to robot heading (-180 to +180 deg)
    
    # Dimensions (meters)
    length_m: float            # X dimension
    width_m: float             # Y dimension
    height_m: float            # Z height above local ground
    
    # 2D image bounding box (u_min, v_min, u_max, v_max)
    bbox_2d: Tuple[int, int, int, int]
    
    # 3D bounding box (xmin, xmax, ymin, ymax, zmin, zmax)
    bbox_3d: Tuple[float, float, float, float, float, float]
    
    # Traversability assessment
    is_run_over_allowed: bool
    traversability_type: TraversabilityType
    traversability_name: str
    semantic_class: int
    semantic_name: str
    severity_cost: int
    num_points: int

    def to_dict(self) -> Dict[str, Any]:
        d = asdict(self)
        d['traversability_type'] = int(self.traversability_type)
        return d


class SpatialObstacleDetector:
    """
    Extracts, clusters, and formats 3D spatial obstacles from depth and traversability maps.
    """

    def __init__(
        self,
        min_cluster_pixels: int = 120,
        max_detection_range: float = 10.0,
        vehicle_profile: Optional[TrackedVehicleProfile] = None
    ):
        self.min_cluster_pixels = min_cluster_pixels
        self.max_detection_range = max_detection_range
        self.profile = vehicle_profile or TrackedVehicleProfile()

    def detect(
        self,
        pts_robot: np.ndarray,
        valid_mask: np.ndarray,
        semantic_mask: np.ndarray,
        decision: TraversabilityDecision
    ) -> List[SpatialObstacle]:
        """
        Extract discrete spatial obstacles from perception outputs.
        Extracts both Hazard Obstacles (lethal/negative) and Discrete Traversable Objects.
        """
        obstacles: List[SpatialObstacle] = []
        obs_id = 1

        # 1. Lethal / Negative Hazard Obstacles
        hazard_binary = (decision.lethal_mask | decision.negative_hazard_mask) & valid_mask
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        filtered_hazard = cv2.morphologyEx(hazard_binary.astype(np.uint8), cv2.MORPH_OPEN, kernel)
        
        num_labels_h, labels_h, stats_h, _ = cv2.connectedComponentsWithStats(filtered_hazard, connectivity=8)
        for label in range(1, num_labels_h):
            pixel_count = stats_h[label, cv2.CC_STAT_AREA]
            if pixel_count < self.min_cluster_pixels:
                continue

            u_min = int(stats_h[label, cv2.CC_STAT_LEFT])
            v_min = int(stats_h[label, cv2.CC_STAT_TOP])
            box_w = int(stats_h[label, cv2.CC_STAT_WIDTH])
            box_h = int(stats_h[label, cv2.CC_STAT_HEIGHT])
            u_max = u_min + box_w
            v_max = v_min + box_h

            cluster_mask = (labels_h == label) & valid_mask
            pts_cluster = pts_robot[cluster_mask]
            if len(pts_cluster) < 15:
                continue

            cx = float(np.median(pts_cluster[:, 0]))
            cy = float(np.median(pts_cluster[:, 1]))
            cz = float(np.median(pts_cluster[:, 2]))

            radial_dist = float(np.sqrt(cx**2 + cy**2))
            if radial_dist > self.max_detection_range or radial_dist < 0.2:
                continue

            azimuth = float(np.degrees(np.arctan2(cy, cx)))
            x_min, x_max = float(np.min(pts_cluster[:, 0])), float(np.max(pts_cluster[:, 0]))
            y_min, y_max = float(np.min(pts_cluster[:, 1])), float(np.max(pts_cluster[:, 1]))
            z_min, z_max = float(np.min(pts_cluster[:, 2])), float(np.max(pts_cluster[:, 2]))

            length_m = max(0.02, x_max - x_min)
            width_m = max(0.02, y_max - y_min)
            height_m = float(np.max(np.abs(decision.delta_h[cluster_mask])))

            cluster_trav = decision.traversability_map[cluster_mask]
            is_negative = np.count_nonzero(cluster_trav == TraversabilityType.NEGATIVE_HAZARD) > 0.3 * len(cluster_trav)
            dom_trav = TraversabilityType.NEGATIVE_HAZARD if is_negative else TraversabilityType.LETHAL_OBSTACLE
            
            cluster_sem = semantic_mask[cluster_mask]
            dom_sem = int(np.argmax(np.bincount(cluster_sem, minlength=4)))
            cost_val = int(np.max(decision.cost_map[cluster_mask]))

            obstacles.append(SpatialObstacle(
                id=obs_id,
                centroid_x=cx,
                centroid_y=cy,
                centroid_z=cz,
                radial_distance_m=radial_dist,
                azimuth_deg=azimuth,
                length_m=length_m,
                width_m=width_m,
                height_m=height_m,
                bbox_2d=(u_min, v_min, u_max, v_max),
                bbox_3d=(x_min, x_max, y_min, y_max, z_min, z_max),
                is_run_over_allowed=False,
                traversability_type=dom_trav,
                traversability_name=TRAVERSABILITY_NAMES.get(int(dom_trav), "Lethal Obstacle"),
                semantic_class=dom_sem,
                semantic_name=CLASS_NAMES.get(dom_sem, "Unknown"),
                severity_cost=cost_val,
                num_points=int(pixel_count)
            ))
            obs_id += 1

        # 2. Discrete Traversable Objects (Low bumps / grass mounds >= 3cm)
        trav_binary = decision.can_run_over_mask & valid_mask & (np.abs(decision.delta_h) >= 0.03) & ~hazard_binary
        filtered_trav = cv2.morphologyEx(trav_binary.astype(np.uint8), cv2.MORPH_OPEN, kernel)

        num_labels_t, labels_t, stats_t, _ = cv2.connectedComponentsWithStats(filtered_trav, connectivity=8)
        for label in range(1, num_labels_t):
            pixel_count = stats_t[label, cv2.CC_STAT_AREA]
            if pixel_count < self.min_cluster_pixels:
                continue

            u_min = int(stats_t[label, cv2.CC_STAT_LEFT])
            v_min = int(stats_t[label, cv2.CC_STAT_TOP])
            box_w = int(stats_t[label, cv2.CC_STAT_WIDTH])
            box_h = int(stats_t[label, cv2.CC_STAT_HEIGHT])
            u_max = u_min + box_w
            v_max = v_min + box_h

            cluster_mask = (labels_t == label) & valid_mask
            pts_cluster = pts_robot[cluster_mask]
            if len(pts_cluster) < 15:
                continue

            cx = float(np.median(pts_cluster[:, 0]))
            cy = float(np.median(pts_cluster[:, 1]))
            cz = float(np.median(pts_cluster[:, 2]))

            radial_dist = float(np.sqrt(cx**2 + cy**2))
            if radial_dist > self.max_detection_range or radial_dist < 0.2:
                continue

            azimuth = float(np.degrees(np.arctan2(cy, cx)))
            x_min, x_max = float(np.min(pts_cluster[:, 0])), float(np.max(pts_cluster[:, 0]))
            y_min, y_max = float(np.min(pts_cluster[:, 1])), float(np.max(pts_cluster[:, 1]))
            z_min, z_max = float(np.min(pts_cluster[:, 2])), float(np.max(pts_cluster[:, 2]))

            length_m = max(0.02, x_max - x_min)
            width_m = max(0.02, y_max - y_min)
            height_m = float(np.max(np.abs(decision.delta_h[cluster_mask])))

            cluster_sem = semantic_mask[cluster_mask]
            dom_sem = int(np.argmax(np.bincount(cluster_sem, minlength=4)))
            cost_val = int(np.max(decision.cost_map[cluster_mask]))

            obstacles.append(SpatialObstacle(
                id=obs_id,
                centroid_x=cx,
                centroid_y=cy,
                centroid_z=cz,
                radial_distance_m=radial_dist,
                azimuth_deg=azimuth,
                length_m=length_m,
                width_m=width_m,
                height_m=height_m,
                bbox_2d=(u_min, v_min, u_max, v_max),
                bbox_3d=(x_min, x_max, y_min, y_max, z_min, z_max),
                is_run_over_allowed=True,
                traversability_type=TraversabilityType.RUN_OVER_TRAVERSABLE,
                traversability_name=TRAVERSABILITY_NAMES.get(int(TraversabilityType.RUN_OVER_TRAVERSABLE), "Traversable"),
                semantic_class=dom_sem,
                semantic_name=CLASS_NAMES.get(dom_sem, "Unknown"),
                severity_cost=cost_val,
                num_points=int(pixel_count)
            ))
            obs_id += 1

        obstacles.sort(key=lambda o: o.radial_distance_m)
        return obstacles
