"""
Unit tests for Camera Model, Intrinsics/Extrinsics, and 3D Back-projection.
"""

import pytest
import numpy as np

from sih_perception.core.camera import StereoCameraModel, CameraParameters


def test_camera_matrices():
    params = CameraParameters(
        width=640,
        height=480,
        fx=400.0,
        fy=400.0,
        cx=320.0,
        cy=240.0,
        mount_x=0.2,
        mount_z=0.35,
        pitch_deg=0.0
    )
    cam = StereoCameraModel(params)
    
    assert cam.K.shape == (3, 3)
    assert cam.T_cam_to_base.shape == (4, 4)
    # Check camera translation offset in base_link
    np.testing.assert_allclose(cam.t_cam_to_base, [0.2, 0.0, 0.35])


def test_disparity_to_depth():
    params = CameraParameters(fx=400.0, baseline=0.10)
    cam = StereoCameraModel(params)

    # Disparity of 20 pixels -> depth = (400 * 0.10) / 20 = 2.0 meters
    disp = np.full((10, 10), 20.0)
    depth = cam.disparity_to_depth(disp)
    np.testing.assert_allclose(depth, 2.0)


def test_back_project_and_projection_consistency():
    params = CameraParameters(
        width=640,
        height=480,
        fx=380.0,
        fy=380.0,
        cx=320.0,
        cy=240.0,
        mount_x=0.0,
        mount_y=0.0,
        mount_z=0.35,
        pitch_deg=0.0
    )
    cam = StereoCameraModel(params)

    # Create synthetic uniform depth map at 3.0 meters
    depth = np.full((480, 640), 3.0)
    pts_robot, valid = cam.back_project_to_robot_frame(depth)

    assert pts_robot.shape == (480, 640, 3)
    assert valid.all()

    # Optical center pixel (320, 240) should project directly forward at 3.0m
    center_pt = pts_robot[240, 320]
    np.testing.assert_allclose(center_pt[0], 3.0, atol=1e-3) # X_r = Forward
    np.testing.assert_allclose(center_pt[1], 0.0, atol=1e-3) # Y_r = Left/Right
    np.testing.assert_allclose(center_pt[2], 0.35, atol=1e-3) # Z_r = Height

    # Reproject to image plane
    pixels, in_fov = cam.project_robot_points_to_pixels(center_pt)
    assert in_fov[0]
    np.testing.assert_allclose(pixels[0], [320.0, 240.0], atol=1e-2)
