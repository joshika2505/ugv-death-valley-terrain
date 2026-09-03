"""
ROS2 Message Bridge and Standalone Serializer for Perception Outputs.
Supports standard ROS2 nav_msgs/OccupancyGrid and vision_msgs format dicts
as well as JSON serialization.
"""

import json
from typing import Dict, Any, List
import numpy as np

from .pipeline import PerceptionResult


class ROS2MessageBridge:
    """
    Translates PerceptionResult into ROS2 message structures or serialized payloads.
    """

    @staticmethod
    def create_occupancy_grid_msg(result: PerceptionResult, frame_id: str = "base_link") -> Dict[str, Any]:
        """
        Creates dictionary formatted identically to ROS2 `nav_msgs/msg/OccupancyGrid`.
        """
        costmap_grid = result.costmap_grid  # (cells_x, cells_y)
        cells_x, cells_y = costmap_grid.shape
        
        # Flatten row-major for ROS2 convention
        # Scale to [-1, 100]
        ros_data = np.full((cells_x, cells_y), -1, dtype=np.int8)
        ros_data[costmap_grid == 0] = 0
        trav_mask = (costmap_grid > 0) & (costmap_grid < 254)
        ros_data[trav_mask] = (1 + (costmap_grid[trav_mask] / 254.0) * 98).astype(np.int8)
        ros_data[costmap_grid >= 254] = 100

        # ROS2 OccupancyGrid expects 1D array
        flat_data = ros_data.T.flatten().tolist()

        return {
            "header": {
                "stamp": result.timestamp,
                "frame_id": frame_id
            },
            "info": {
                "map_load_time": result.timestamp,
                "resolution": 0.05,
                "width": cells_y,
                "height": cells_x,
                "origin": {
                    "position": {"x": -1.0, "y": -4.0, "z": 0.0},
                    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
                }
            },
            "data": flat_data
        }

    @staticmethod
    def create_detection_3d_array_msg(result: PerceptionResult, frame_id: str = "base_link") -> Dict[str, Any]:
        """
        Creates dictionary formatted identically to ROS2 `vision_msgs/msg/Detection3DArray`.
        """
        detections = []
        for obs in result.obstacles:
            detections.append({
                "header": {
                    "stamp": result.timestamp,
                    "frame_id": frame_id
                },
                "results": [{
                    "hypothesis": {
                        "class_id": obs.semantic_name,
                        "score": 1.0
                    }
                }],
                "bbox": {
                    "center": {
                        "position": {"x": obs.centroid_x, "y": obs.centroid_y, "z": obs.centroid_z},
                        "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
                    },
                    "size": {"x": obs.length_m, "y": obs.width_m, "z": obs.height_m}
                },
                "traversability": {
                    "is_run_over_allowed": obs.is_run_over_allowed,
                    "type": obs.traversability_name,
                    "severity_cost": obs.severity_cost,
                    "radial_distance_m": round(obs.radial_distance_m, 2),
                    "azimuth_deg": round(obs.azimuth_deg, 1)
                }
            })

        return {
            "header": {
                "stamp": result.timestamp,
                "frame_id": frame_id
            },
            "detections": detections
        }

    @staticmethod
    def export_summary_json(result: PerceptionResult) -> str:
        """Export high-level diagnostic summary to JSON."""
        summary = {
            "timestamp": result.timestamp,
            "ground_plane": {
                "equation": f"{result.ground_plane.a:.3f}x + {result.ground_plane.b:.3f}y + {result.ground_plane.c:.3f}z + {result.ground_plane.d:.3f} = 0",
                "slope_deg": round(result.ground_plane.slope_deg, 2),
                "inlier_ratio": round(result.ground_plane.inlier_ratio, 3)
            },
            "num_obstacles_detected": len(result.obstacles),
            "obstacles": [obs.to_dict() for obs in result.obstacles],
            "latency_ms": {
                "back_projection": round(result.timing.back_projection_ms, 2),
                "ground_plane": round(result.timing.ground_plane_ms, 2),
                "segmentation": round(result.timing.segmentation_ms, 2),
                "traversability": round(result.timing.traversability_ms, 2),
                "costmap": round(result.timing.costmap_ms, 2),
                "detection": round(result.timing.detection_ms, 2),
                "total": round(result.timing.total_ms, 2),
                "fps": round(result.timing.fps, 1)
            }
        }
        return json.dumps(summary, indent=2)
