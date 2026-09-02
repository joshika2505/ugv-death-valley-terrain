"""
Synthetic 3D Scene and Stereo RGB-D Generator.
Generates realistic, physically exact depth and RGB sensor frames using
analytical 3D geometric raycasting:
- Traversable low debris (5 cm)
- Soft tall grass meadow (30 cm)
- Lethal rock boulder (25 cm)
- Negative trench / ditch (20 cm deep)
- Hazardous steep slope (42 deg)
"""

from enum import Enum
from typing import Tuple, Optional
import numpy as np
import cv2

from ..core.camera import StereoCameraModel, CameraParameters
from ..segmentation.taxonomy import SemanticClass


class ScenarioType(str, Enum):
    FLAT_TRAIL_SMALL_DEBRIS = "flat_trail_small_debris"
    TALL_GRASS_MEADOW = "tall_grass_meadow"
    LETHAL_ROCK_BOULDER = "lethal_rock_boulder"
    NEGATIVE_DITCH_TRENCH = "negative_ditch_trench"
    STEEP_INCLINE_SLOPE = "steep_incline_slope"


class SyntheticSceneGenerator:
    """
    Renders physically grounded RGB and Depth maps from 3D synthetic heightfields.
    """

    def __init__(self, camera_params: Optional[CameraParameters] = None):
        self.params = camera_params or CameraParameters()
        self.camera = StereoCameraModel(self.params)

    def generate_scenario(
        self,
        scenario: ScenarioType = ScenarioType.LETHAL_ROCK_BOULDER
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Generate (RGB, Depth) pair for a given test scenario using exact 3D ray-tracing.
        """
        w, h = self.params.width, self.params.height
        rgb = np.zeros((h, w, 3), dtype=np.uint8)

        # Unit ray directions in robot base frame (H, W, 3)
        ray_dir_robot = self.camera.ray_dir_robot
        ray_norm = np.linalg.norm(ray_dir_robot, axis=-1, keepdims=True)
        rays_unit = (ray_dir_robot / ray_norm).astype(np.float32)  # (H, W, 3)
        
        # Camera origin in robot base frame
        cam_origin = self.camera.t_cam_to_base.astype(np.float32)

        # Direction of camera optical axis in robot base frame
        opt_forward_robot = (self.camera.R_cam_to_base @ np.array([0.0, 0.0, 1.0], dtype=np.float32)).astype(np.float32)
        # Cosine factor converting distance along ray 's' to optical depth 'Z_opt'
        cos_opt = np.sum(rays_unit * opt_forward_robot, axis=-1)  # (H, W)

        # 1. Base Ground Plane Intersection: P_z = 0 -> cam_origin[2] + s * rays_unit[..., 2] = 0
        dir_z = rays_unit[..., 2]
        downward = dir_z < -0.01

        s_ground = np.full((h, w), np.nan, dtype=np.float32)
        s_ground[downward] = -cam_origin[2] / dir_z[downward]

        # Base 3D contact coordinates on ground
        pts_3d = cam_origin + s_ground[..., np.newaxis] * rays_unit
        X = pts_3d[..., 0]
        Y = pts_3d[..., 1]

        # Base dirt trail RGB texture
        X_clean = np.nan_to_num(X, nan=0.0)
        Y_clean = np.nan_to_num(Y, nan=0.0)
        rgb[..., 0] = np.clip(140 + np.sin(X_clean * 4) * 15, 0, 255).astype(np.uint8)  # R
        rgb[..., 1] = np.clip(110 + np.cos(Y_clean * 4) * 15, 0, 255).astype(np.uint8)  # G
        rgb[..., 2] = 80   # B

        s_final = s_ground.copy()

        # 2. Add Scenario-Specific 3D Geometric Features
        if scenario == ScenarioType.FLAT_TRAIL_SMALL_DEBRIS:
            # Small gravel / debris mound (5cm height, radius 18cm) at X=2.2m, Y=0.0m
            center_debris = np.array([2.2, 0.0, 0.0], dtype=np.float32)
            dist_debris = np.sqrt((X - center_debris[0])**2 + (Y - center_debris[1])**2)
            debris_mask = (dist_debris < 0.20) & ~np.isnan(s_ground)
            
            # 5 cm height delta
            h_debris = 0.05 * np.cos((dist_debris[debris_mask] / 0.20) * (np.pi / 2.0))
            s_final[debris_mask] = -(cam_origin[2] - h_debris) / dir_z[debris_mask]
            rgb[debris_mask] = [110, 95, 75]

        elif scenario == ScenarioType.TALL_GRASS_MEADOW:
            # Meadow with 30 cm tall grass everywhere beyond X > 1.2m
            grass_mask = (X > 1.2) & ~np.isnan(s_ground)
            h_grass = 0.30 + 0.04 * np.sin(X[grass_mask] * 8.0) * np.cos(Y[grass_mask] * 8.0)
            s_final[grass_mask] = -(cam_origin[2] - h_grass) / dir_z[grass_mask]
            
            # Lush vegetation color
            n_grass = int(grass_mask.sum())
            rgb[grass_mask, 0] = np.clip(60 + np.random.randint(0, 25, size=n_grass), 0, 255).astype(np.uint8)
            rgb[grass_mask, 1] = np.clip(185 + np.random.randint(0, 35, size=n_grass), 0, 255).astype(np.uint8)
            rgb[grass_mask, 2] = 45

        elif scenario == ScenarioType.LETHAL_ROCK_BOULDER:
            # 25 cm boulder at X=2.8m, Y=-0.2m (Radius = 0.25m, center Z = 0.0m -> top is at Z=0.25m)
            center_rock = np.array([2.8, -0.2, 0.0], dtype=np.float32)
            radius = 0.25
            
            v = cam_origin - center_rock
            b = 2.0 * np.sum(rays_unit * v, axis=-1)
            c = float(np.sum(v**2) - radius**2)
            disc = b**2 - 4.0 * c
            rock_hit = (disc >= 0)
            
            s_rock = (-b - np.sqrt(np.maximum(disc, 0.0))) / 2.0
            # Only where ray hits front surface of rock above ground
            rock_valid = rock_hit & (s_rock > 0.5) & (s_rock < s_ground)
            s_final[rock_valid] = s_rock[rock_valid]

            # High-contrast rough rock texture
            n_rock = int(rock_valid.sum())
            if n_rock > 0:
                noise_tex = np.random.randint(-35, 35, size=n_rock, dtype=np.int16)
                rgb[rock_valid, 0] = np.clip(95 + noise_tex, 0, 255).astype(np.uint8)
                rgb[rock_valid, 1] = np.clip(90 + noise_tex, 0, 255).astype(np.uint8)
                rgb[rock_valid, 2] = np.clip(85 + noise_tex, 0, 255).astype(np.uint8)

        elif scenario == ScenarioType.NEGATIVE_DITCH_TRENCH:
            # 20 cm deep trench spanning X in [2.3m, 2.8m], Y in [-1.5m, 1.5m]
            trench_mask = (X >= 2.3) & (X <= 2.8) & (np.abs(Y) <= 1.5) & ~np.isnan(s_ground)
            # Step down to Z = -0.20m
            s_final[trench_mask] = -(cam_origin[2] - (-0.20)) / dir_z[trench_mask]
            rgb[trench_mask] = [40, 35, 30]

        elif scenario == ScenarioType.STEEP_INCLINE_SLOPE:
            # 42 degree slope starting at X = 1.8m: Z = (X - 1.8) * tan(42 deg)
            slope_rad = np.radians(42.0)
            tan_slope = np.tan(slope_rad)
            # Intersection equation: cam_z + s * dir_z = (cam_x + s * dir_x - 1.8) * tan_slope
            # s * (dir_z - dir_x * tan_slope) = -cam_z - 1.8 * tan_slope + cam_x * tan_slope
            denom = dir_z - rays_unit[..., 0] * tan_slope
            numer = -cam_origin[2] - 1.8 * tan_slope + cam_origin[0] * tan_slope
            slope_hit = denom < -0.01
            s_slope = numer / denom
            
            pts_x_slope = cam_origin[0] + s_slope * rays_unit[..., 0]
            slope_valid = slope_hit & (pts_x_slope > 1.8) & (s_slope > 0.4) & (s_slope < 8.0)
            s_final[slope_valid] = s_slope[slope_valid]
            rgb[slope_valid] = [170, 130, 80]

        # Convert distance along ray s to optical depth Z_opt
        depth = s_final * cos_opt
        depth = np.nan_to_num(depth, nan=0.0)

        # Add slight sensor noise (5mm std dev)
        noise = np.random.normal(0.0, 0.005, size=depth.shape).astype(np.float32)
        depth_noisy = np.where(depth > 0.2, depth + noise, 0.0)
        depth_clean = np.clip(depth_noisy, 0.0, 12.0)

        return rgb, depth_clean
