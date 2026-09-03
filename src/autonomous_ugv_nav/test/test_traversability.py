"""
Unit tests for pure TraversabilityAnalyzer filters.
"""

import numpy as np
import pytest

from autonomous_ugv_nav.costmap.traversability_analyzer import TraversabilityAnalyzer


def test_slope_flat_terrain():
    analyzer = TraversabilityAnalyzer(resolution=0.1, ground_clearance=0.15)
    # 50x50 flat plane at z = 1.0
    grid = np.ones((50, 50), dtype=np.float32)
    slope_cost = analyzer.compute_slope(grid, max_slope_deg=25.0)

    assert slope_cost.shape == (50, 50)
    assert np.all(slope_cost == 0), "Flat terrain must produce 0 slope cost."


def test_slope_steep_ramp():
    analyzer = TraversabilityAnalyzer(resolution=0.1, ground_clearance=0.15)
    # Linear ramp with 30 degree slope: dz/dx = tan(30 deg) = 0.577
    h, w = 50, 50
    x_coords = np.arange(w) * 0.1
    ramp = np.tile(x_coords * np.tan(np.radians(30.0)), (h, 1))

    slope_cost = analyzer.compute_slope(ramp, max_slope_deg=25.0)
    # Inner region away from boundaries should hit maximum cost (254) since 30 > 25 deg
    inner = slope_cost[5:-5, 5:-5]
    assert np.all(inner == 254), "Slope exceeding max_slope_deg must receive maximum cost 254."


def test_roughness_smooth_vs_noisy():
    analyzer = TraversabilityAnalyzer(resolution=0.1, ground_clearance=0.15)

    smooth_grid = np.ones((40, 40), dtype=np.float32) * 2.0
    roughness_smooth = analyzer.compute_roughness(smooth_grid, max_roughness_m=0.08)
    assert np.all(roughness_smooth == 0), "Smooth terrain must have 0 roughness cost."

    # Grid with high vertical variance (> 0.10m std dev)
    np.random.seed(42)
    noisy_grid = smooth_grid + np.random.normal(0.0, 0.12, size=(40, 40)).astype(np.float32)
    roughness_noisy = analyzer.compute_roughness(noisy_grid, max_roughness_m=0.08)
    assert np.mean(roughness_noisy) > 100, "High roughness terrain must produce elevated cost."


def test_step_height_detection():
    analyzer = TraversabilityAnalyzer(resolution=0.1, ground_clearance=0.15)
    grid = np.zeros((30, 30), dtype=np.float32)
    # Create an impassable boulder with step height 0.30m (exceeds 0.15m ground clearance)
    grid[10:20, 10:20] = 0.30

    step_cost = analyzer.compute_step_height(grid, ground_clearance=0.15)
    # Perimeter of the boulder should be flagged as lethal (254)
    boundary_cells = step_cost[10, 10:20]
    assert np.all(boundary_cells == 254), "Step exceeding ground clearance must be classified as lethal 254."


def test_fuse_traversability_lethal_preservation():
    analyzer = TraversabilityAnalyzer(resolution=0.1, ground_clearance=0.15)

    slope = np.zeros((20, 20), dtype=np.uint8)
    roughness = np.zeros((20, 20), dtype=np.uint8)
    step = np.zeros((20, 20), dtype=np.uint8)

    # Inject one lethal step cell
    step[10, 10] = 254

    fused = analyzer.fuse_traversability(slope, roughness, step)
    assert fused[10, 10] == 254, "Lethal cells must strictly remain 254 upon fusion."
    assert fused[0, 0] == 0, "Non-lethal empty cells must remain 0."


def test_costmap_inflation():
    analyzer = TraversabilityAnalyzer(resolution=0.1, ground_clearance=0.15)
    grid = np.zeros((50, 50), dtype=np.uint8)
    # Lethal obstacle in the center (cell 25, 25)
    grid[25, 25] = 254

    inflated = analyzer.inflate_costmap(
        grid,
        robot_radius=0.35,      # 3.5 cells
        inflation_radius=0.70,  # 7.0 cells
        decay_factor=3.0
    )

    # Inscribed radius (e.g. cell (25, 27) at dist = 0.2m <= 0.35m) should be >= 253
    assert inflated[25, 27] >= 253, "Cells within inscribed robot radius must have >= 253 cost."

    # Decay zone (e.g. cell (25, 30) at dist = 0.5m) should have decreasing positive cost
    assert 0 < inflated[25, 30] < 253, "Cells in inflation buffer must have decayed non-zero cost."

    # Outer zone (e.g. cell (25, 45) at dist = 2.0m) should be 0
    assert inflated[25, 45] == 0, "Cells beyond inflation radius must be 0."
