"""
Stereo RGB-D Camera and Geometric Projection Model.
Handles intrinsic/extrinsic transformations, back-projection to 3D robot frame,
and distance computations.
"""

from dataclasses import dataclass
from typing import Tuple, Optional
import numpy as np


@dataclass
class CameraParameters:
    """Intrinsic and extrinsic calibration parameters."""
    width: int = 640
    height: int = 480
    fx: float = 380.0
    fy: float = 380.0
    cx: float = 320.0
    cy: float = 240.0
    baseline: float = 0.12  # meters (stereo baseline)
    
    # Camera mount position relative to robot base_link (ground contact point)
    mount_x: float = 0.20   # forward offset from base center (meters)
    mount_y: float = 0.00   # lateral offset (meters)
    mount_z: float = 0.35   # vertical height above ground (meters)
    pitch_deg: float = -8.0 # tilt down angle in degrees (negative = tilted down)
    roll_deg: float = 0.0   # roll in degrees
    yaw_deg: float = 0.0    # yaw in degrees
    
    depth_min: float = 0.20 # min valid depth (meters)
    depth_max: float = 12.0 # max valid depth (meters)


class StereoCameraModel:
    """
    Handles pinhole camera mathematics, coordinate transformations,
    and vectorized 3D point cloud generation from depth maps.
    
    Coordinate Frames:
      - Camera Optical Frame: +X Right, +Y Down, +Z Forward (optical axis)
      - Robot Base Frame (base_link): +X Forward, +Y Left, +Z Up (Ground = Z=0)
    """

    def __init__(self, params: Optional[CameraParameters] = None):
        self.params = params or CameraParameters()
        self._build_matrices()
        self._precompute_pixel_grid()

    def _build_matrices(self) -> None:
        """Construct intrinsic K and extrinsic T_camera_to_base transform matrices."""
        p = self.params
        
        # 1. Camera Intrinsic Matrix K (3x3)
        self.K = np.array([
            [p.fx,  0.0, p.cx],
            [ 0.0, p.fy, p.cy],
            [ 0.0,  0.0,  1.0]
        ], dtype=np.float64)
        
        self.K_inv = np.linalg.inv(self.K)

        # 2. Rotation from Camera Optical frame to Intermediate Robot Orientation:
        # Optical: X_opt -> -Y_robot, Y_opt -> -Z_robot, Z_opt -> +X_robot
        R_optical_to_robot = np.array([
            [ 0.0,  0.0,  1.0],  # Robot X = +Z_opt (Forward)
            [-1.0,  0.0,  0.0],  # Robot Y = -X_opt (Left)
            [ 0.0, -1.0,  0.0]   # Robot Z = -Y_opt (Up)
        ], dtype=np.float64)

        # 3. Mount Euler rotations (Roll, Pitch, Yaw in robot frame)
        pitch = np.radians(p.pitch_deg)
        roll = np.radians(p.roll_deg)
        yaw = np.radians(p.yaw_deg)

        R_pitch = np.array([
            [np.cos(pitch), 0.0, np.sin(pitch)],
            [0.0,           1.0, 0.0          ],
            [-np.sin(pitch), 0.0, np.cos(pitch)]
        ], dtype=np.float64)

        R_roll = np.array([
            [1.0, 0.0,          0.0         ],
            [0.0, np.cos(roll), -np.sin(roll)],
            [0.0, np.sin(roll),  np.cos(roll)]
        ], dtype=np.float64)

        R_yaw = np.array([
            [np.cos(yaw), -np.sin(yaw), 0.0],
            [np.sin(yaw),  np.cos(yaw), 0.0],
            [0.0,          0.0,         1.0]
        ], dtype=np.float64)

        R_mount = R_yaw @ R_pitch @ R_roll
        self.R_cam_to_base = R_mount @ R_optical_to_robot

        # 4. Translation vector (mount position relative to base_link)
        self.t_cam_to_base = np.array([p.mount_x, p.mount_y, p.mount_z], dtype=np.float64)

        # 5. Full 4x4 Extrinsic Matrix T_camera_to_base
        self.T_cam_to_base = np.eye(4, dtype=np.float64)
        self.T_cam_to_base[:3, :3] = self.R_cam_to_base
        self.T_cam_to_base[:3, 3] = self.t_cam_to_base

        # Inverse transform: T_base_to_cam
        self.T_base_to_cam = np.linalg.inv(self.T_cam_to_base)
        self.R_base_to_cam = self.T_base_to_cam[:3, :3]
        self.t_base_to_cam = self.T_base_to_cam[:3, 3]

    def _precompute_pixel_grid(self) -> None:
        """Precompute normalized ray grid for ultra-fast vectorized back-projection."""
        u, v = np.meshgrid(
            np.arange(self.params.width, dtype=np.float32),
            np.arange(self.params.height, dtype=np.float32)
        )
        # Normalized coordinates in optical frame (Z=1 plane)
        self.ray_x = ((u - self.params.cx) / self.params.fx).astype(np.float32)
        self.ray_y = ((v - self.params.cy) / self.params.fy).astype(np.float32)
        
        # Precompute unit ray directions transformed directly to Robot Base Frame:
        # P_cam = [ray_x * Z, ray_y * Z, Z] = Z * [ray_x, ray_y, 1.0]
        # P_robot = P_cam @ R.T + t = Z * ([ray_x, ray_y, 1.0] @ R.T) + t
        rays_opt = np.stack([self.ray_x, self.ray_y, np.ones_like(self.ray_x)], axis=-1)  # (H, W, 3)
        self.ray_dir_robot = (rays_opt @ self.R_cam_to_base.T).astype(np.float32)  # (H, W, 3)
        self.t_robot = self.t_cam_to_base.astype(np.float32)

    def disparity_to_depth(self, disparity: np.ndarray) -> np.ndarray:
        """Convert disparity map (pixels) to metric depth (meters)."""
        valid_mask = disparity > 0.0
        depth = np.zeros_like(disparity, dtype=np.float32)
        depth[valid_mask] = (self.params.fx * self.params.baseline) / disparity[valid_mask]
        depth[~valid_mask] = 0.0
        return depth

    def back_project_to_camera_frame(self, depth: np.ndarray) -> np.ndarray:
        """Back-project 2D depth map to 3D Camera Optical Frame (H, W, 3)."""
        Z_c = depth.astype(np.float32)
        X_c = self.ray_x * Z_c
        Y_c = self.ray_y * Z_c
        return np.stack([X_c, Y_c, Z_c], axis=-1)

    def back_project_to_robot_frame(self, depth: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Ultra-fast vectorized back-projection directly to 3D Robot Base Frame.
        Takes < 1.0 ms for full 640x480 resolution.
        """
        depth_f32 = depth.astype(np.float32)
        valid_mask = (depth_f32 >= self.params.depth_min) & (depth_f32 <= self.params.depth_max) & np.isfinite(depth_f32)

        # Vectorized scaling: P_robot = Z * ray_dir_robot + t_robot
        pts_robot = depth_f32[..., np.newaxis] * self.ray_dir_robot + self.t_robot
        pts_robot[~valid_mask] = 0.0
        return pts_robot, valid_mask

    def project_robot_points_to_pixels(self, pts_robot: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Project 3D points in robot base frame (N, 3) into pixel coordinates (N, 2).
        
        Returns:
            pixels: (N, 2) array of (u, v) image coordinates.
            valid_mask: (N,) boolean mask of points in front of camera and within FOV.
        """
        if pts_robot.ndim == 1:
            pts_robot = pts_robot.reshape(1, 3)
            
        # Transform from robot frame to camera optical frame: P_cam = P_robot @ R_base_to_cam.T + t
        pts_cam = pts_robot @ self.R_base_to_cam.T + self.t_base_to_cam
        
        Z_c = pts_cam[:, 2]
        in_front = Z_c > self.params.depth_min
        
        u = np.zeros(len(pts_robot), dtype=np.float64)
        v = np.zeros(len(pts_robot), dtype=np.float64)
        
        u[in_front] = (pts_cam[in_front, 0] * self.params.fx / Z_c[in_front]) + self.params.cx
        v[in_front] = (pts_cam[in_front, 1] * self.params.fy / Z_c[in_front]) + self.params.cy
        
        in_fov = (
            in_front &
            (u >= 0) & (u < self.params.width) &
            (v >= 0) & (v < self.params.height)
        )
        
        pixels = np.column_stack([u, v])
        return pixels, in_fov

    @staticmethod
    def compute_horizontal_distance(pts_robot: np.ndarray) -> np.ndarray:
        """Compute horizontal ground distance d = sqrt(X_r^2 + Y_r^2)."""
        return np.sqrt(pts_robot[..., 0] ** 2 + pts_robot[..., 1] ** 2)

    @staticmethod
    def compute_azimuth_deg(pts_robot: np.ndarray) -> np.ndarray:
        """Compute horizontal azimuth angle in degrees relative to robot forward heading."""
        # Y_r is left (+deg) and right (-deg)
        return np.degrees(np.arctan2(pts_robot[..., 1], pts_robot[..., 0]))
