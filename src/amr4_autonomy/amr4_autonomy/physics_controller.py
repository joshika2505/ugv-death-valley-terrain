#!/usr/bin/env python3
import math
import numpy as np

class PhysicsBasedPathTracker:
    """
    Adaptive Pure Pursuit & Terrain-Aware Physics Controller for AMR-4.
    Regulates linear and angular velocity strictly through physical motor torque (cmd_vel)
    accounting for terrain slope, curvature, acceleration limits, and arrival deceleration.
    """
    def __init__(self):
        self.lookahead_dist_min = 0.8  # meters
        self.lookahead_dist_max = 2.5  # meters
        self.lookahead_gain = 1.2
        self.max_linear_speed = 1.0    # m/s
        self.min_linear_speed = 0.15   # m/s
        self.max_angular_speed = 1.2   # rad/s
        self.max_accel = 0.6           # m/s^2
        self.max_decel = 1.0           # m/s^2
        self.arrival_tolerance = 0.9   # meters

        self.current_speed = 0.0
        self.last_cmd_v = 0.0
        self.last_time = None

    def compute_control(self, robot_pose, waypoints, current_time):
        """
        Compute (linear_v, angular_w, arrival_status, distance_remaining).
        robot_pose: (x, y, yaw)
        waypoints: list of {'x', 'y', 'z', 'slope', 'speed'} dicts
        """
        if not waypoints or len(waypoints) == 0:
            return 0.0, 0.0, False, 0.0

        rx, ry, ryaw = robot_pose
        goal = waypoints[-1]
        dist_to_goal = math.hypot(goal['x'] - rx, goal['y'] - ry)

        # 1. Check Arrival Condition
        if dist_to_goal <= self.arrival_tolerance:
            return 0.0, 0.0, True, dist_to_goal

        # 2. Find Closest Waypoint
        dists = [math.hypot(w['x'] - rx, w['y'] - ry) for w in waypoints]
        closest_idx = int(np.argmin(dists))

        # 3. Determine Adaptive Lookahead Distance
        target_lookahead = np.clip(
            self.lookahead_gain * max(self.current_speed, 0.3),
            self.lookahead_dist_min,
            self.lookahead_dist_max
        )

        # 4. Find Lookahead Waypoint
        lookahead_pt = waypoints[-1]
        for i in range(closest_idx, len(waypoints)):
            if dists[i] >= target_lookahead:
                lookahead_pt = waypoints[i]
                break

        # 5. Pure Pursuit Steering & Curvature Calculation
        dx = lookahead_pt['x'] - rx
        dy = lookahead_pt['y'] - ry
        
        # Transform lookahead point to robot local frame
        local_x = math.cos(ryaw) * dx + math.sin(ryaw) * dy
        local_y = -math.sin(ryaw) * dx + math.cos(ryaw) * dy

        L = math.hypot(local_x, local_y)
        if L < 1e-3:
            curvature = 0.0
        else:
            curvature = (2.0 * local_y) / (L * L)

        # 6. Terrain-Aware Target Speed Selection
        nominal_speed = lookahead_pt.get('speed', 0.8)
        slope = lookahead_pt.get('slope', 0.0)

        # Reduce speed on steep terrain
        slope_factor = max(0.35, math.cos(math.radians(min(slope, 60.0))))
        target_v = nominal_speed * slope_factor

        # Reduce speed during sharp turns
        turn_factor = 1.0 / (1.0 + 2.2 * abs(curvature))
        target_v *= turn_factor

        # Deceleration profile approaching goal
        if dist_to_goal < 3.0:
            decel_factor = max(0.2, math.sqrt(dist_to_goal / 3.0))
            target_v *= decel_factor

        target_v = np.clip(target_v, self.min_linear_speed, self.max_linear_speed)

        # 7. Apply Acceleration Limits (Smooth Torque / Physics Response)
        dt = 0.1
        if self.last_time is not None:
            dt = max(0.01, min(0.5, current_time - self.last_time))
        self.last_time = current_time

        if target_v > self.last_cmd_v:
            cmd_v = min(target_v, self.last_cmd_v + self.max_accel * dt)
        else:
            cmd_v = max(target_v, self.last_cmd_v - self.max_decel * dt)

        self.last_cmd_v = cmd_v

        # 8. Compute Angular Velocity
        cmd_w = np.clip(curvature * cmd_v, -self.max_angular_speed, self.max_angular_speed)

        # Pivot in place if heading error is extreme (> 65 degrees)
        heading_err = math.atan2(local_y, local_x)
        if abs(heading_err) > math.radians(65.0):
            cmd_v = 0.05
            cmd_w = np.clip(heading_err * 1.5, -self.max_angular_speed, self.max_angular_speed)

        return cmd_v, cmd_w, False, dist_to_goal
