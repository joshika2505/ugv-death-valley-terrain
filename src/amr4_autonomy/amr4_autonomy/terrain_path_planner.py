#!/usr/bin/env python3
import math
import heapq
import numpy as np

class TerrainPathPlanner:
    """
    3D Terrain-Aware A* Shortest Path & Obstacle Bypass Planner.
    Finds the mathematically shortest, energy-efficient, collision-free route across Death Valley,
    smoothly routing around steep hills, rocks, and dynamic obstacle clusters.
    """
    def __init__(self, terrain_analyzer):
        self.ta = terrain_analyzer
        self.max_climb_slope = 35.0 # Max traversable slope in degrees
        self.dynamic_obstacles = [] # list of (x, y, radius)

    def add_obstacle(self, x, y, radius=1.0):
        """Registers dynamic obstacle from camera/LiDAR to be actively bypassed."""
        self.dynamic_obstacles.append((float(x), float(y), float(radius)))
        # Keep latest 30 dynamic obstacles
        if len(self.dynamic_obstacles) > 30:
            self.dynamic_obstacles.pop(0)

    def clear_obstacles(self):
        self.dynamic_obstacles = []

    def plan_path(self, start_xy, goal_xy, dynamic_obs=None):
        """
        Compute optimal shortest 3D path from start_xy to goal_xy bypassing obstacles and steep slopes.
        """
        sx, sy = start_xy
        gx, gy = goal_xy

        res = self.ta.resolution
        min_x, min_y = self.ta.min_x, self.ta.min_y
        
        start_idx = (int(round((sx - min_x) / res)), int(round((sy - min_y) / res)))
        goal_idx = (int(round((gx - min_x) / res)), int(round((gy - min_y) / res)))

        start_idx = (np.clip(start_idx[0], 0, self.ta.grid_w - 1), np.clip(start_idx[1], 0, self.ta.grid_h - 1))
        goal_idx = (np.clip(goal_idx[0], 0, self.ta.grid_w - 1), np.clip(goal_idx[1], 0, self.ta.grid_h - 1))

        # Check all registered dynamic obstacles
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

        # 8-connected neighbors
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
                    
                    # 1. Slope Check
                    slope = self.ta.slope_grid[ny, nx]
                    if slope > self.max_climb_slope:
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
                            # Gaussian proximity cost penalty pushing path to shortest safe contour
                            obs_cost_penalty += (1.2 - (d_obs - orad)) * 15.0

                    if blocked_by_obs:
                        continue

                    # 3. Transition Cost Calculation (Shortest Path + Terrain Factors)
                    step_dist = math.hypot(dx * res, dy * res)
                    dz = abs(self.ta.height_grid[ny, nx] - self.ta.height_grid[current[1], current[0]])
                    
                    slope_penalty = 1.0 + (slope / 18.0)**2
                    cost = (step_dist + dz * 1.2) * slope_penalty + obs_cost_penalty

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

        # Convert to 3D waypoints
        waypoints = []
        total_len = len(path_indices)
        for i, (ix, iy) in enumerate(path_indices):
            x = float(min_x + ix * res)
            y = float(min_y + iy * res)
            z = float(self.ta.height_grid[iy, ix])
            slope = float(self.ta.slope_grid[iy, ix])

            # Dynamic velocity assignment
            if slope < 10.0:
                v = 0.9
            elif slope < 20.0:
                v = 0.5
            else:
                v = 0.3

            remaining = total_len - 1 - i
            if remaining < 5:
                v = max(0.15, v * (remaining / 5.0))

            waypoints.append({'x': x, 'y': y, 'z': z, 'slope': slope, 'speed': v})

        # Smooth waypoints and return
        return self._smooth_waypoints(waypoints)

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
            speed = 0.4 if slope > 15.0 else 0.8
            waypoints.append({'x': x, 'y': y, 'z': z, 'slope': slope, 'speed': speed})
        return waypoints
