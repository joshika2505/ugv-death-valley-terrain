"""
Unit tests for the Core MPPI Controller.
"""

import numpy as np
import pytest

from autonomous_ugv_nav.planner.skid_steer_model import SkidSteerModel
from autonomous_ugv_nav.planner.cost_critics import (
    ObstacleCritic,
    PathFollowCritic,
    GoalCritic,
    SmoothnessCritic,
)
from autonomous_ugv_nav.planner.mppi_core import MPPIController


def test_mppi_steers_towards_goal():
    dynamics = SkidSteerModel(max_v=1.2, min_v=0.0, max_omega=2.0, max_accel=2.0)
    critics = [
        GoalCritic(weight=20.0),
        SmoothnessCritic(weight=0.2),
    ]

    mppi = MPPIController(
        dynamics_model=dynamics,
        critics=critics,
        num_samples=300,
        time_horizon=20,
        dt=0.1,
        temperature=0.8,
        noise_std_v=0.45
    )

    current_state = np.array([0.0, 0.0, 0.0], dtype=np.float32)  # At (0,0) facing +X
    goal = np.array([10.0, 0.0, 0.0], dtype=np.float32)          # Goal at (10,0) ahead

    # Run for 10 steps to let warm-start sequence ramp up from rest
    cmd_v, cmd_omega = 0.0, 0.0
    for _ in range(10):
        cmd_v, cmd_omega, best_traj, _ = mppi.compute_control(
            current_state=current_state,
            costmap=None,
            costmap_origin=None,
            goal=goal,
            global_path=None
        )
        current_state = dynamics.step(current_state, np.array([cmd_v, cmd_omega]), dt=0.1)

    # Controller should smoothly accelerate forward toward the goal
    assert cmd_v > 0.3, f"MPPI must generate positive forward velocity (got {cmd_v:.2f} m/s)."
    assert abs(cmd_omega) < 0.2, f"MPPI should maintain straight heading (got {cmd_omega:.2f} rad/s)."
    assert best_traj.shape == (21, 3)


def test_mppi_obstacle_avoidance():
    dynamics = SkidSteerModel(max_v=1.0, min_v=0.0, max_omega=2.0)
    critics = [
        ObstacleCritic(weight=30.0, lethal_cost_thresh=70.0, lethal_penalty=1e5),
        GoalCritic(weight=5.0),
        SmoothnessCritic(weight=1.0),
    ]

    mppi = MPPIController(
        dynamics_model=dynamics,
        critics=critics,
        num_samples=250,
        time_horizon=20,
        dt=0.1,
        temperature=0.8
    )

    # Robot at (0, 0) facing +X
    current_state = np.array([0.0, 0.0, 0.0], dtype=np.float32)
    goal = np.array([6.0, 0.0, 0.0], dtype=np.float32)

    # 100x100 costmap (resolution 0.1m, origin at (-2, -5))
    costmap = np.zeros((100, 100), dtype=np.int8)
    origin = (-2.0, -5.0, 0.1)

    # Place lethal wall directly in front: X in [1.5, 2.5], Y in [-1.0, 1.0]
    # In grid coords: cols ~ (1.5 - (-2))/0.1 = 35 to 45, rows ~ (-1 - (-5))/0.1 = 40 to 60
    costmap[40:60, 35:45] = 100  # Lethal obstacle

    cmd_v, cmd_omega, best_traj, _ = mppi.compute_control(
        current_state=current_state,
        costmap=costmap,
        costmap_origin=origin,
        goal=goal,
        global_path=None
    )

    # MPPI should steer around the obstacle (non-zero angular velocity or slow down)
    assert abs(cmd_omega) > 0.1 or cmd_v < 0.3, (
        "MPPI must steer away or brake when facing a direct lethal obstacle."
    )
