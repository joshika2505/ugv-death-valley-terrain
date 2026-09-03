"""
Unit tests for Hybrid Traversability Engine Rules and Tracked Vehicle Physics.
"""

import pytest
import numpy as np

from sih_perception.segmentation.taxonomy import SemanticClass
from sih_perception.core.geometry import GroundPlane
from sih_perception.traversability.vehicle_profile import TrackedVehicleProfile
from sih_perception.traversability.decision_engine import TraversabilityEngine, TraversabilityType


@pytest.fixture
def engine():
    profile = TrackedVehicleProfile(
        ground_clearance=0.10,
        max_climb_step=0.15,
        max_drop_step=0.12,
        soft_vegetation_max_height=0.40,
        max_slope_deg=35.0
    )
    return TraversabilityEngine(profile)


def test_rule_1_negative_hazard_ditch(engine):
    """Test ditch / trench drop-off detection."""
    h, w = 10, 10
    plane = GroundPlane()
    
    # 20 cm drop below ground surface (Z = -0.20m)
    pts = np.zeros((h, w, 3))
    pts[..., 0] = 2.5
    pts[..., 2] = -0.20
    valid = np.ones((h, w), dtype=bool)
    
    sem_mask = np.full((h, w), SemanticClass.FREE_DRIVABLE, dtype=np.uint8)

    decision = engine.evaluate(sem_mask, pts, valid, plane)

    assert (decision.traversability_map == TraversabilityType.NEGATIVE_HAZARD).all()
    assert (decision.cost_map == 255).all()
    assert decision.negative_hazard_mask.all()


def test_rule_2_rigid_lethal_rock(engine):
    """Test 25 cm rigid rock (> 15 cm step climb) is classified as Lethal Obstacle."""
    h, w = 10, 10
    plane = GroundPlane()

    pts = np.zeros((h, w, 3))
    pts[..., 0] = 2.5
    pts[..., 2] = 0.25  # 25 cm height
    valid = np.ones((h, w), dtype=bool)

    sem_mask = np.full((h, w), SemanticClass.RIGID_OBSTACLE, dtype=np.uint8)

    decision = engine.evaluate(sem_mask, pts, valid, plane)

    assert (decision.traversability_map == TraversabilityType.LETHAL_OBSTACLE).all()
    assert (decision.cost_map == 254).all()
    assert decision.lethal_mask.all()
    assert not decision.can_run_over_mask.any()


def test_rule_3_rigid_small_step_run_over(engine):
    """Test 8 cm curb / rock (<= 15 cm step climb) is classified as Run-Over Allowed."""
    h, w = 10, 10
    plane = GroundPlane()

    pts = np.zeros((h, w, 3))
    pts[..., 0] = 2.0
    pts[..., 2] = 0.08  # 8 cm height <= 15 cm step
    valid = np.ones((h, w), dtype=bool)

    sem_mask = np.full((h, w), SemanticClass.RIGID_OBSTACLE, dtype=np.uint8)

    decision = engine.evaluate(sem_mask, pts, valid, plane)

    assert (decision.traversability_map == TraversabilityType.RUN_OVER_TRAVERSABLE).all()
    assert decision.can_run_over_mask.all()
    # Cost should be moderate
    assert 25 <= decision.cost_map[0, 0] <= 90


def test_rule_4_tall_soft_grass_crushable(engine):
    """Test 30 cm tall grass (soft crushable) is classified as Run-Over Allowed."""
    h, w = 10, 10
    plane = GroundPlane()

    pts = np.zeros((h, w, 3))
    pts[..., 0] = 2.0
    pts[..., 2] = 0.30  # 30 cm tall grass <= 40 cm soft limit
    valid = np.ones((h, w), dtype=bool)

    sem_mask = np.full((h, w), SemanticClass.SOFT_TRAVERSABLE, dtype=np.uint8)

    decision = engine.evaluate(sem_mask, pts, valid, plane)

    assert (decision.traversability_map == TraversabilityType.RUN_OVER_TRAVERSABLE).all()
    assert decision.can_run_over_mask.all()
    assert decision.cost_map[0, 0] == 15  # Low friction cost
