"""
Unit tests for modular MPPI cost critics.
"""

import numpy as np
import pytest

from autonomous_ugv_nav.planner.cost_critics import (
    ObstacleCritic,
    PathFollowCritic,
    GoalCritic,
    SmoothnessCritic,
)


def test_obstacle_critic_lethal_rejection():
    critic = ObstacleCritic(weight=10.0, lethal_cost_thresh=200.0, lethal_penalty=1e5)

    costmap = np.zeros((50, 50), dtype=np.uint8)
    costmap[20:30, 20:30] = 254  # Lethal obstacle block
    origin = (0.0, 0.0, 0.1)

    # Trajectory 0: Clears obstacle (x from 0.0 to 1.0, y = 0.5)
    traj_clear = np.zeros((1, 10, 3), dtype=np.float32)
    traj_clear[0, :, 0] = np.linspace(0.0, 1.0, 10)
    traj_clear[0, :, 1] = 0.5

    # Trajectory 1: Hits lethal obstacle (x from 2.0 to 2.5, y = 2.5) -> (row 25, col 20-25)
    traj_hit = np.zeros((1, 10, 3), dtype=np.float32)
    traj_hit[0, :, 0] = np.linspace(2.0, 2.5, 10)
    traj_hit[0, :, 1] = 2.5

    trajs = np.vstack([traj_clear, traj_hit])
    controls = np.zeros((2, 9, 2), dtype=np.float32)

    costs = critic.score(trajs, controls, costmap, origin, goal=None, global_path=None)

    assert costs[0] < 10.0, "Clear trajectory must have low cost."
    assert costs[1] >= 1e5, "Colliding trajectory must receive massive lethal penalty."


def test_goal_critic():
    critic = GoalCritic(weight=10.0)

    goal = np.array([10.0, 0.0, 0.0])

    # Trajectory 0 ends at (9.0, 0.0) -> dist = 1.0 -> cost = 10.0
    traj_close = np.zeros((1, 5, 3), dtype=np.float32)
    traj_close[0, -1, 0] = 9.0

    # Trajectory 1 ends at (2.0, 0.0) -> dist = 8.0 -> cost = 80.0
    traj_far = np.zeros((1, 5, 3), dtype=np.float32)
    traj_far[0, -1, 0] = 2.0

    trajs = np.vstack([traj_close, traj_far])
    controls = np.zeros((2, 4, 2), dtype=np.float32)

    costs = critic.score(trajs, controls, None, None, goal=goal, global_path=None)

    assert costs[0] < costs[1], "Trajectory closer to goal must have lower cost."
    assert np.isclose(costs[0], 10.0, atol=1e-2)


def test_path_follow_critic():
    critic = PathFollowCritic(weight=5.0)

    # Reference path along X axis: (0,0), (1,0), (2,0), (3,0)
    path = np.array([[0.0, 0.0], [1.0, 0.0], [2.0, 0.0], [3.0, 0.0]], dtype=np.float32)

    # Trajectory 0 stays directly on path (y = 0.0)
    traj_on = np.zeros((1, 5, 3), dtype=np.float32)
    traj_on[0, :, 0] = np.linspace(0.0, 2.0, 5)

    # Trajectory 1 deviates significantly (y = 2.0)
    traj_off = np.zeros((1, 5, 3), dtype=np.float32)
    traj_off[0, :, 0] = np.linspace(0.0, 2.0, 5)
    traj_off[0, :, 1] = 2.0

    trajs = np.vstack([traj_on, traj_off])
    controls = np.zeros((2, 4, 2), dtype=np.float32)

    costs = critic.score(trajs, controls, None, None, goal=None, global_path=path)
    assert costs[0] < costs[1], "Trajectory on path must score much lower cost than divergent path."
