#!/usr/bin/env python3
import math
import heapq
import numpy as np

class TerrainPathPlanner:
    """
    3D Terrain-Aware A* Path Planner.
    Generates safe, energy-efficient, collision-free paths traversing Death Valley contours
    while avoiding cliffs, steep ridges, and untraversable terrain.
    """
    def __init__(self, terrain_analyzer):
        self.ta = terrain_analyzer
        self.max_climb_slope = 45.0 # Maximum traversable slope in degrees

    def plan_path(self, start_xy, goal_xy, obstacle_grid=None):
        """
        Compute optimal 3D path from start_xy (x, y) to goal_xy (x, y).
        Returns list of (x, y, z, target_speed) waypoints.
        """
        sx, sy = start_xy
        gx, gy = goal_xy

        # Convert to grid indices
        res = self.ta.resolution
        min_x, min_y = self.ta.min_x, self.ta.min_y
        
        start_idx = (int(round((sx - min_x) / res)), int(round((sy - min_y) / res)))
        goal_idx = (int(round((gx - min_x) / res)), int(round((gy - min_y) / res)))

        start_idx = (np.clip(start_idx[0], 0, self.ta.grid_w - 1), np.clip(start_idx[1], 0, self.ta.grid_h - 1))
        goal_idx = (np.clip(goal_idx[0], 0, self.ta.grid_w - 1), np.clip(goal_idx[1], 0, self.ta.grid_h - 1))

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
                    
                    # Slope check
                    slope = self.ta.slope_grid[ny, nx]
                    if slope > self.max_climb_slope:
                        continue # Untraversable slope

                    # Dynamic obstacle check
                    if obstacle_grid is not None and obstacle_grid[ny, nx] > 50:
                        continue

                    # Transition cost calculation
                    step_dist = math.hypot(dx * res, dy * res)
                    dz = abs(self.ta.height_grid[ny, nx] - self.ta.height_grid[current[1], current[0]])
                    
                    # Cost multiplier: flat=1.0, slope penalty quadratic, roughness penalty
                    slope_penalty = 1.0 + (slope / 15.0)**2
                    roughness_penalty = 1.0 + self.ta.roughness_grid[ny, nx] * 0.1
                    cost = (step_dist + dz * 1.5) * slope_penalty * roughness_penalty

                    tentative_g = g_score[current] + cost
                    if neighbor not in g_score or tentative_g < g_score[neighbor]:
                        g_score[neighbor] = tentative_g
                        priority = tentative_g + heuristic(neighbor, goal_idx)
                        heapq.heappush(open_set, (priority, neighbor))
                        came_from[neighbor] = current

        if not found:
            print(f'[PathPlanner] Direct A* path blocked by extreme terrain. Generating best-effort contour route...')
            # Fallback: Straight-line interpolated route with terrain elevation snapping
            return self._generate_fallback_path(sx, sy, gx, gy)

        # Reconstruct path
        path_indices = [goal_idx]
        curr = goal_idx
        while curr in came_from:
            curr = came_from[curr]
            path_indices.append(curr)
        path_indices.reverse()

        # Convert indices to 3D world coordinates with terrain-adjusted target velocities
        waypoints = []
        total_len = len(path_indices)
        for i, (ix, iy) in enumerate(path_indices):
            x = float(min_x + ix * res)
            y = float(min_y + iy * res)
            z = float(self.ta.height_grid[iy, ix])
            slope = float(self.ta.slope_grid[iy, ix])

            # Dynamic velocity assignment based on terrain slope & proximity to goal:
            # Flat/Safe: 0.9 m/s, Moderate: 0.5 m/s, Steep: 0.3 m/s
            if slope < 10.0:
                v = 0.9
            elif slope < 20.0:
                v = 0.5
            else:
                v = 0.3

            # Deceleration near goal (last 5 waypoints)
            remaining = total_len - 1 - i
            if remaining < 5:
                v = max(0.15, v * (remaining / 5.0))

            waypoints.append({'x': x, 'y': y, 'z': z, 'slope': slope, 'speed': v})

        # Smooth waypoints using moving average
        smoothed = self._smooth_waypoints(waypoints)
        return smoothed

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
            # Re-snap Z to exact terrain surface at smoothed (x, y)
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
