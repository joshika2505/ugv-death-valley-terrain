"""
Unit tests for 2.5D Local Costmap and ROS OccupancyGrid Export.
"""

import pytest
import numpy as np

from sih_perception.traversability.vehicle_profile import TrackedVehicleProfile
from sih_perception.traversability.decision_engine import TraversabilityDecision, TraversabilityType
from sih_perception.traversability.costmap import LocalCostmap, CostmapConfig
from sih_perception.core.geometry import GroundPlane


def test_costmap_world_to_grid_and_back():
    cfg = CostmapConfig(resolution=0.05, size_x=8.0, size_y=8.0, origin_x=-1.0, origin_y=-4.0)
    costmap = LocalCostmap(cfg)

    # Robot origin (0.0, 0.0)
    gx, gy, in_bounds = costmap.world_to_grid(np.array([0.0]), np.array([0.0]))
    assert in_bounds[0]
    assert gx[0] == 20  # (0.0 - (-1.0)) / 0.05 = 20
    assert gy[0] == 80  # (0.0 - (-4.0)) / 0.05 = 80

    # Convert back to world
    wx, wy = costmap.grid_to_world(int(gx[0]), int(gy[0]))
    np.testing.assert_allclose(wx, 0.025, atol=0.03)
    np.testing.assert_allclose(wy, 0.025, atol=0.03)


def test_costmap_generation_and_inflation():
    profile = TrackedVehicleProfile(inflation_radius=0.20)
    cfg = CostmapConfig(resolution=0.05, size_x=6.0, size_y=6.0, origin_x=-1.0, origin_y=-3.0)
    costmap = LocalCostmap(cfg, profile)

    h, w = 20, 20
    pts = np.zeros((h, w, 3))
    # Place lethal obstacle points at X=2.0m, Y=0.0m
    pts[..., 0] = 2.0
    pts[..., 1] = 0.0
    pts[..., 2] = 0.30
    valid = np.ones((h, w), dtype=bool)

    trav_map = np.full((h, w), TraversabilityType.LETHAL_OBSTACLE, dtype=np.uint8)
    cost_map = np.full((h, w), 254, dtype=np.uint8)
    delta_h = np.full((h, w), 0.30)
    
    decision = TraversabilityDecision(
        traversability_map=trav_map,
        cost_map=cost_map,
        delta_h=delta_h,
        can_run_over_mask=np.zeros((h, w), dtype=bool),
        lethal_mask=np.ones((h, w), dtype=bool),
        negative_hazard_mask=np.zeros((h, w), dtype=bool),
        ground_plane=GroundPlane()
    )

    grid = costmap.generate(pts, valid, decision)

    # Lethal cell location: gx = (2.0 - (-1.0))/0.05 = 60, gy = (0.0 - (-3.0))/0.05 = 60
    assert grid[60, 60] == 254
    # Neighboring cells within 20cm inflation radius should have elevated cost >= 180
    assert grid[61, 60] >= 180
    assert grid[60, 61] >= 180

    # Test conversion to ROS OccupancyGrid
    ros_grid = costmap.to_ros_occupancy_grid(grid)
    assert ros_grid[60, 60] == 100
