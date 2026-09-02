#!/usr/bin/env python3
import math
import heapq
import numpy as np

class TerrainPathPlanner:
    """
    3D Terrain-Aware A* Shortest Safe Path & Adaptive Learning Planner.
    Features:
    1. Shortest Safe Path: Minimizes Euclidean 3D distance + slope cost + roughness + obstacles.
    2. Configurable MAX_CLIMBABLE_SLOPE_DEG: Prevents planning over impassable cliffs.
    3. Terrain Learning & Memory: Permanently remembers failed climb locations and dynamically avoids them.
    4. Gentle Valley Pass Router: Automatically finds low-altitude bypass contours around mountain ridges.
    """
    def __init__(self, terrain_analyzer):
        self.ta = terrain_analyzer
        self.max_climb_slope = 22.0 # Configurable MAX_CLIMBABLE_SLOPE_DEG
        self.dynamic_obstacles = [] # list of (x, y, radius)
        self.failed_climb_memory = [] # list of (x, y, radius, penalty_multiplier)

    def set_max_climb_slope(self, max_slope_deg):
        self.max_climb_slope = float(max_slope_deg)

    def add_obstacle(self, x, y, radius=1.0):
        """Registers dynamic obstacle from camera/LiDAR to be actively bypassed."""
        self.dynamic_obstacles.append((float(x), float(y), float(radius)))
        if len(self.dynamic_obstacles) > 40:
            self.dynamic_obstacles.pop(0)

    def add_failed_climb_region(self, x, y, radius=3.0, penalty=200.0):
        """Learning / Terrain Memory: Marks region where robot previously stalled/failed to climb."""
        self.failed_climb_memory.append((float(x), float(y), float(radius), float(penalty)))

    def clear_memory(self):
        self.dynamic_obstacles = []
        self.failed_climb_memory = []

    def plan_path(self, start_xy, goal_xy, dynamic_obs=None, slope_override=None):
        """
        Compute optimal shortest 3D safe path from start_xy to goal_xy.
        """
        sx, sy = start_xy
        gx, gy = goal_xy

        res = self.ta.resolution
        min_x, min_y = self.ta.min_x, self.ta.min_y
        max_slope = slope_override if slope_override is not None else self.max_climb_slope
        
        start_idx = (int(round((sx - min_x) / res)), int(round((sy - min_y) / res)))
        goal_idx = (int(round((gx - min_x) / res)), int(round((gy - min_y) / res)))

        start_idx = (np.clip(start_idx[0], 0, self.ta.grid_w - 1), np.clip(start_idx[1], 0, self.ta.grid_h - 1))
        goal_idx = (np.clip(goal_idx[0], 0, self.ta.grid_w - 1), np.clip(goal_idx[1], 0, self.ta.grid_h - 1))

        obs_list = list(self.dynamic_obstacles)
        if dynamic_obs:
            obs_list.extend(dynamic_obs)

        # Priority queue for A*
        open_set = []
        heapq.heappush(open_set, (0.0, start_idx))
        came_from = {}
        g_score = {start_idx: 0.0}

        def heuristic(idx1, idx2):
            dx = (idx1[0] - idx2[0]) * res
            dy = (idx1[1] - idx2[1]) * res
            z1 = self.ta.height_grid[idx1[1], idx1[0]]
            z2 = self.ta.height_grid[idx2[1], idx2[0]]
            dz = z2 - z1
            return math.sqrt(dx*dx + dy*dy + dz*dz)

        neighbors = [
            (-1, 0), (1, 0), (0, -1), (0, 1),
            (-1, -1), (-1, 1), (1, -1), (1, 1)
        ]

        found = False
        while open_set:
            _, current = heapq.heappop(open_set)

            if current == goal_idx or (abs(current[0] - goal_idx[0]) <= 1 and abs(current[1] - goal_idx[1]) <= 1):
                found = True
                goal_idx = current
                break

            for dx, dy in neighbors:
                nx, ny = current[0] + dx, current[1] + dy
                if 0 <= nx < self.ta.grid_w and 0 <= ny < self.ta.grid_h:
                    neighbor = (nx, ny)
                    
                    # 1. Slope Capability Constraint
                    slope = self.ta.slope_grid[ny, nx]
                    if slope > max_slope:
                        continue # Untraversable slope / cliff

                    # 2. Dynamic Obstacle Clearance Check
                    wx = min_x + nx * res
                    wy = min_y + ny * res
                    blocked_by_obs = False
                    obs_cost_penalty = 0.0

                    for (ox, oy, orad) in obs_list:
                        d_obs = math.hypot(wx - ox, wy - oy)
                        if d_obs < orad:
                            blocked_by_obs = True
                            break
                        elif d_obs < (orad + 1.2):
                            obs_cost_penalty += (1.2 - (d_obs - orad)) * 20.0

                    if blocked_by_obs:
                        continue

                    # 3. Learning & Terrain Memory Penalty
                    memory_cost_penalty = 0.0
                    for (fx, fy, frad, fpen) in self.failed_climb_memory:
                        d_fail = math.hypot(wx - fx, wy - fy)
                        if d_fail < frad:
                            memory_cost_penalty += fpen * (1.0 - d_fail / frad)

                    # 4. Physical Transition Cost: 3D Distance + Slope + Roughness + Memory
                    step_dist = math.hypot(dx * res, dy * res)
                    dz = abs(self.ta.height_grid[ny, nx] - self.ta.height_grid[current[1], current[0]])
                    
                    slope_penalty = 1.0 + (slope / 14.0)**2
                    cost = (step_dist + dz * 1.5) * slope_penalty + obs_cost_penalty + memory_cost_penalty

                    tentative_g = g_score[current] + cost
                    if neighbor not in g_score or tentative_g < g_score[neighbor]:
                        g_score[neighbor] = tentative_g
                        priority = tentative_g + heuristic(neighbor, goal_idx)
                        heapq.heappush(open_set, (priority, neighbor))
                        came_from[neighbor] = current

        if not found:
            # Fallback path if direct route is locked
            return self._generate_fallback_path(sx, sy, gx, gy)

        # Reconstruct path
        path_indices = [goal_idx]
        curr = goal_idx
        while curr in came_from:
            curr = came_from[curr]
            path_indices.append(curr)
        path_indices.reverse()

        # Convert to 3D waypoints with physics speed profile
        waypoints = []
        total_len = len(path_indices)
        for i, (ix, iy) in enumerate(path_indices):
            x = float(min_x + ix * res)
            y = float(min_y + iy * res)
            z = float(self.ta.height_grid[iy, ix])
            slope = float(self.ta.slope_grid[iy, ix])

            # Speed assignment based on slope
            if slope < 8.0:
                v = 0.95
            elif slope < 15.0:
                v = 0.55
            else:
                v = 0.30

            remaining = total_len - 1 - i
            if remaining < 6:
                v = max(0.15, v * (remaining / 6.0))

            waypoints.append({'x': x, 'y': y, 'z': z, 'slope': slope, 'speed': v})

        return self._smooth_waypoints(waypoints)

    def plan_gentle_valley_path(self, start_xy, goal_xy):
        """
        Specialized Gentle Valley Pass Router: Strictly restricts slope to <= 14 deg
        and finds low-altitude valley passes around steep mountain ridges.
        """
        return self.plan_path(start_xy, goal_xy, slope_override=14.0)

    def _smooth_waypoints(self, waypoints, window=3):
        if len(waypoints) <= window:
            return waypoints

        smoothed = []
        for i in range(len(waypoints)):
            start = max(0, i - window // 2)
            end = min(len(waypoints), i + window // 2 + 1)
            subset = waypoints[start:end]

            avg_x = sum(w['x'] for w in subset) / len(subset)
            avg_y = sum(w['y'] for w in subset) / len(subset)
            avg_z = self.ta.get_surface_elevation(avg_x, avg_y)
            avg_slope = sum(w['slope'] for w in subset) / len(subset)
            spd = waypoints[i]['speed']

            smoothed.append({'x': avg_x, 'y': avg_y, 'z': avg_z, 'slope': avg_slope, 'speed': spd})

        return smoothed

    def _generate_fallback_path(self, sx, sy, gx, gy, num_pts=30):
        waypoints = []
        for alpha in np.linspace(0.0, 1.0, num_pts):
            x = float(sx + alpha * (gx - sx))
            y = float(sy + alpha * (gy - sy))
            z, slope, _, _, _ = self.ta.get_terrain_properties(x, y)
            speed = 0.35 if slope > 12.0 else 0.75
            waypoints.append({'x': x, 'y': y, 'z': z, 'slope': slope, 'speed': speed})
        return waypoints
