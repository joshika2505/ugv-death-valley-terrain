#!/usr/bin/env python3
import math
import time
import numpy as np

class PhysicsBasedPathTracker:
    """
    Advanced Terrain-Adaptive & Anti-Flip Physics Controller for Tracked UGV.
    Features:
    1. Pure Pursuit Waypoint Following
    2. Real-Time Slope & Elevation Speed Adaptation (climbing/descending power control)
    3. Anti-Tip & Side-Slope Rollover Prevention
    4. Autonomous Rollover Detection & Self-Righting Recovery
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

        # Stability & Rollover Prevention Thresholds
        self.max_safe_pitch = 20.0     # degrees (difficult terrain threshold)
        self.max_safe_roll = 16.0      # degrees (fall hazard threshold)
        self.rollover_threshold = 45.0 # degrees (rollover recovery)
        self.recovery_state = 'NORMAL' # 'NORMAL', 'STABILIZING', 'FLIPPED_RECOVERY'
        self.recovery_timer = 0.0

    def compute_control(self, robot_pose, robot_posture, waypoints, current_time):
        """
        Compute (linear_v, angular_w, arrival_status, distance_remaining, stability_mode).
        robot_pose: (x, y, yaw)
        robot_posture: (pitch_deg, roll_deg)
        waypoints: list of {'x', 'y', 'z', 'slope', 'speed'} dicts
        """
        pitch_deg, roll_deg = robot_posture
        abs_pitch = abs(pitch_deg)
        abs_roll = abs(roll_deg)

        # -------------------------------------------------------------
        # 1. EMERGENCY ROLLOVER / FLIP DETECTION & RECOVERY REFLEX
        # -------------------------------------------------------------
        if abs_pitch > self.rollover_threshold or abs_roll > self.rollover_threshold:
            self.recovery_state = 'FLIPPED_RECOVERY'
            # Apply rapid oscillatory track torque to dislodge and self-right
            oscillation = math.sin(current_time * 8.0)
            rec_v = 0.3 * oscillation
            rec_w = 1.2 * math.copysign(1.0, oscillation)
            return rec_v, rec_w, False, 999.0, 'CRITICAL_FLIPPED'

        # -------------------------------------------------------------
        # 2. PRE-TIP ANTI-ROLLOVER STABILIZATION REFLEX
        # -------------------------------------------------------------
        if abs_pitch > self.max_safe_pitch or abs_roll > self.max_safe_roll:
            self.recovery_state = 'STABILIZING'
            # Slow down to crawl immediately and counter-steer away from side-slope
            safe_crawl_v = 0.2
            counter_steer_w = -0.5 * math.copysign(1.0, roll_deg) if abs_roll > self.max_safe_roll else 0.0
            return safe_crawl_v, counter_steer_w, False, 0.0, 'ANTI_TIP_ACTIVE'

        # -------------------------------------------------------------
        # 2.5 STEEP RIDGE / INCLINE STALL DETECTION
        # -------------------------------------------------------------
        if abs_pitch > 18.0 and self.last_cmd_v > 0.25 and self.current_speed < 0.08:
            if not hasattr(self, 'stall_start_time') or self.stall_start_time is None:
                self.stall_start_time = current_time
            elif current_time - self.stall_start_time > 1.2:
                # Ridge is unscalable: back off gently and request alternative route
                return -0.25, 0.0, False, 0.0, 'RIDGE_UNCLIMBABLE'
        else:
            self.stall_start_time = None

        self.recovery_state = 'NORMAL'

        if not waypoints or len(waypoints) == 0:
            return 0.0, 0.0, False, 0.0, 'NORMAL'

        rx, ry, ryaw = robot_pose
        goal = waypoints[-1]
        dist_to_goal = math.hypot(goal['x'] - rx, goal['y'] - ry)

        # 3. Check Arrival Condition
        if dist_to_goal <= self.arrival_tolerance:
            return 0.0, 0.0, True, dist_to_goal, 'COMPLETED'

        # 4. Find Closest Waypoint & Lookahead
        dists = [math.hypot(w['x'] - rx, w['y'] - ry) for w in waypoints]
        closest_idx = int(np.argmin(dists))

        target_lookahead = np.clip(
            self.lookahead_gain * max(self.current_speed, 0.3),
            self.lookahead_dist_min,
            self.lookahead_dist_max
        )

        lookahead_pt = waypoints[-1]
        for i in range(closest_idx, len(waypoints)):
            if dists[i] >= target_lookahead:
                lookahead_pt = waypoints[i]
                break

        # 5. Pure Pursuit Steering & Curvature
        dx = lookahead_pt['x'] - rx
        dy = lookahead_pt['y'] - ry
        
        local_x = math.cos(ryaw) * dx + math.sin(ryaw) * dy
        local_y = -math.sin(ryaw) * dx + math.cos(ryaw) * dy

        L = math.hypot(local_x, local_y)
        curvature = 0.0 if L < 1e-3 else (2.0 * local_y) / (L * L)

        # -------------------------------------------------------------
        # 6. TERRAIN & ELEVATION SLOPE ADAPTIVE SPEED REGULATION
        # -------------------------------------------------------------
        nominal_speed = lookahead_pt.get('speed', 0.8)
        terrain_slope = lookahead_pt.get('slope', 0.0)

        # Incline / Elevation adaptation: Downshift to high-torque speed on climbs
        if abs_pitch > 10.0 or terrain_slope > 12.0:
            max_incline = max(abs_pitch, terrain_slope)
            incline_factor = max(0.3, math.cos(math.radians(min(max_incline, 50.0))))
            target_v = nominal_speed * incline_factor
            stability_status = 'CLIMBING_ELEVATION'
        else:
            target_v = nominal_speed
            stability_status = 'NORMAL'

        # Reduce speed during sharp turns to avoid track slippage and tipping
        turn_factor = 1.0 / (1.0 + 2.5 * abs(curvature))
        target_v *= turn_factor

        # Deceleration profile approaching Point B
        if dist_to_goal < 3.0:
            decel_factor = max(0.2, math.sqrt(dist_to_goal / 3.0))
            target_v *= decel_factor

        target_v = np.clip(target_v, self.min_linear_speed, self.max_linear_speed)

        # 7. Apply Smooth Acceleration Limits (Wheel Torque Ramp)
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

        # In-place pivot if heading error is large
        heading_err = math.atan2(local_y, local_x)
        if abs(heading_err) > math.radians(65.0):
            cmd_v = 0.05
            cmd_w = np.clip(heading_err * 1.5, -self.max_angular_speed, self.max_angular_speed)

        return cmd_v, cmd_w, False, dist_to_goal, stability_status
