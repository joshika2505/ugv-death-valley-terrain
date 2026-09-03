"""
Unified End-to-End Perception & Traversability Pipeline for Tracked UGV.
"""

import time
from dataclasses import dataclass, field
from typing import Optional, List, Dict, Any, Tuple
import numpy as np

from ..core.camera import StereoCameraModel, CameraParameters
from ..core.geometry import GroundPlaneFitter, GroundPlane, PointCloudProcessor
from ..segmentation.segmenter import BaseSegmenter, FeatureSegmenter
from ..segmentation.taxonomy import SemanticClass
from ..traversability.vehicle_profile import TrackedVehicleProfile
from ..traversability.decision_engine import TraversabilityEngine, TraversabilityDecision
from ..traversability.costmap import LocalCostmap, CostmapConfig
from ..detection.obstacle_detector import SpatialObstacle, SpatialObstacleDetector


@dataclass
class PipelineTiming:
    """Detailed execution latency breakdown (milliseconds)."""
    back_projection_ms: float = 0.0
    ground_plane_ms: float = 0.0
    segmentation_ms: float = 0.0
    traversability_ms: float = 0.0
    costmap_ms: float = 0.0
    detection_ms: float = 0.0
    total_ms: float = 0.0
    fps: float = 0.0


@dataclass
class PerceptionResult:
    """Comprehensive output of one perception inference cycle."""
    timestamp: float
    rgb: np.ndarray
    depth: np.ndarray
    pts_robot: np.ndarray
    valid_mask: np.ndarray
    ground_plane: GroundPlane
    semantic_mask: np.ndarray
    traversability: TraversabilityDecision
    costmap_grid: np.ndarray
    obstacles: List[SpatialObstacle]
    timing: PipelineTiming


class PerceptionPipeline:
    """
    Unified real-time perception pipeline.
    Orchestrates Stereo RGB-D backprojection, RANSAC ground extraction,
    semantic classification, physics-based traversability evaluation,
    2.5D costmap generation, and 3D spatial obstacle localization.
    """

    def __init__(
        self,
        camera_params: Optional[CameraParameters] = None,
        vehicle_profile: Optional[TrackedVehicleProfile] = None,
        costmap_config: Optional[CostmapConfig] = None,
        segmenter: Optional[BaseSegmenter] = None,
        ransac_iterations: int = 120,
    ):
        self.camera = StereoCameraModel(camera_params)
        self.vehicle_profile = vehicle_profile or TrackedVehicleProfile()
        self.costmap_cfg = costmap_config or CostmapConfig()
        
        self.ground_fitter = GroundPlaneFitter(
            distance_threshold=0.04,
            max_iterations=ransac_iterations,
            max_slope_deg=self.vehicle_profile.max_slope_deg
        )
        self.geometry_proc = PointCloudProcessor()
        self.segmenter = segmenter or FeatureSegmenter()
        self.traversability_engine = TraversabilityEngine(self.vehicle_profile)
        self.costmap_generator = LocalCostmap(self.costmap_cfg, self.vehicle_profile)
        self.obstacle_detector = SpatialObstacleDetector(
            min_cluster_pixels=80,
            vehicle_profile=self.vehicle_profile
        )

    def process_frame(
        self,
        rgb: np.ndarray,
        depth: np.ndarray,
        timestamp: Optional[float] = None
    ) -> PerceptionResult:
        """
        Process a single stereo RGB-D frame.
        
        Args:
            rgb: (H, W, 3) uint8 RGB image
            depth: (H, W) float32/float64 metric depth in meters
            timestamp: optional frame capture timestamp
            
        Returns:
            PerceptionResult container with all actionable outputs
        """
        t_start = time.perf_counter()
        ts = timestamp or time.time()
        timing = PipelineTiming()

        # Step 1: 3D Back-projection to Robot Frame
        t0 = time.perf_counter()
        pts_robot, valid_mask = self.camera.back_project_to_robot_frame(depth)
        t1 = time.perf_counter()
        timing.back_projection_ms = (t1 - t0) * 1000.0

        # Step 2: Ground Plane Fitting & Height Differentials
        ground_plane = self.ground_fitter.fit(pts_robot, valid_mask)
        self.geometry_proc.update_ground_plane(ground_plane)
        t2 = time.perf_counter()
        timing.ground_plane_ms = (t2 - t1) * 1000.0

        # Step 3: Semantic Segmentation
        semantic_mask = self.segmenter.predict(rgb, depth)
        t3 = time.perf_counter()
        timing.segmentation_ms = (t3 - t2) * 1000.0

        # Step 4: Hybrid Traversability Evaluation
        decision = self.traversability_engine.evaluate(
            semantic_mask=semantic_mask,
            pts_robot=pts_robot,
            valid_mask=valid_mask,
            ground_plane=ground_plane
        )
        t4 = time.perf_counter()
        timing.traversability_ms = (t4 - t3) * 1000.0

        # Step 5: 2.5D Local Grid Costmap Generation
        costmap_grid = self.costmap_generator.generate(
            pts_robot=pts_robot,
            valid_mask=valid_mask,
            decision=decision
        )
        t5 = time.perf_counter()
        timing.costmap_ms = (t5 - t4) * 1000.0

        # Step 6: 3D Spatial Obstacle Extraction & Classification
        obstacles = self.obstacle_detector.detect(
            pts_robot=pts_robot,
            valid_mask=valid_mask,
            semantic_mask=semantic_mask,
            decision=decision
        )
        t6 = time.perf_counter()
        timing.detection_ms = (t6 - t5) * 1000.0

        total_time = t6 - t_start
        timing.total_ms = total_time * 1000.0
        timing.fps = 1.0 / max(total_time, 1e-6)

        return PerceptionResult(
            timestamp=ts,
            rgb=rgb,
            depth=depth,
            pts_robot=pts_robot,
            valid_mask=valid_mask,
            ground_plane=ground_plane,
            semantic_mask=semantic_mask,
            traversability=decision,
            costmap_grid=costmap_grid,
            obstacles=obstacles,
            timing=timing
        )
