"""
Unit tests for RANSAC Ground Plane Fitting and Height Differentials.
"""

import pytest
import numpy as np

from sih_perception.core.geometry import GroundPlaneFitter, GroundPlane, PointCloudProcessor


def test_ground_plane_fitting_flat():
    fitter = GroundPlaneFitter(distance_threshold=0.03, max_iterations=100)

    # Generate synthetic flat ground points in robot frame at Z=0
    x = np.linspace(0.5, 4.0, 40)
    y = np.linspace(-1.5, 1.5, 40)
    xx, yy = np.meshgrid(x, y)
    zz = np.zeros_like(xx) + np.random.normal(0, 0.005, size=xx.shape)

    pts = np.stack([xx, yy, zz], axis=-1)
    valid = np.ones(xx.shape, dtype=bool)

    plane = fitter.fit(pts, valid)

    assert abs(plane.c - 1.0) < 0.05
    assert abs(plane.a) < 0.05
    assert abs(plane.b) < 0.05
    assert abs(plane.d) < 0.05
    assert plane.slope_deg < 5.0
    assert plane.inlier_ratio > 0.85


def test_ground_plane_fitting_tilted_ramp():
    fitter = GroundPlaneFitter(distance_threshold=0.03, max_iterations=150)

    # 15 degree slope upwards along X axis: Z = X * tan(15 deg)
    slope_angle = 15.0
    tan_slope = np.tan(np.radians(slope_angle))

    x = np.linspace(0.5, 3.5, 40)
    y = np.linspace(-1.0, 1.0, 40)
    xx, yy = np.meshgrid(x, y)
    zz = xx * tan_slope + np.random.normal(0, 0.005, size=xx.shape)

    pts = np.stack([xx, yy, zz], axis=-1)
    valid = np.ones(xx.shape, dtype=bool)

    plane = fitter.fit(pts, valid)

    np.testing.assert_allclose(plane.slope_deg, slope_angle, atol=2.0)


def test_height_differential_processor():
    # Ground at Z=0
    plane = GroundPlane(a=0.0, b=0.0, c=1.0, d=0.0)
    proc = PointCloudProcessor(plane)

    # Point at Z = 0.20m (20 cm obstacle)
    pts = np.zeros((10, 10, 3))
    pts[..., 0] = 2.0
    pts[..., 1] = 0.0
    pts[..., 2] = 0.20
    valid = np.ones((10, 10), dtype=bool)

    delta_h = proc.compute_height_differential(pts, valid)
    np.testing.assert_allclose(delta_h, 0.20)
