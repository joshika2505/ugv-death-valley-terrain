#!/usr/bin/env python3
import os
import math
import numpy as np
from scipy.spatial import cKDTree

class DeathValleyTerrainAnalyzer:
    """
    High-Performance 3D Terrain Analyzer for Death Valley.
    Computes exact elevation, slope, surface normals, roughness,
    and 4-tier traversability classification for physical robot navigation.
    """
    def __init__(self, mesh_path=None, resolution=0.5):
        if mesh_path is None:
            mesh_path = '/home/ubuntu/sih_ws/src/death_valley_world/meshes/death_valley_visual.obj'
            if not os.path.exists(mesh_path):
                mesh_path = '/home/joshika/Desktop/SIH/src/death_valley_world/meshes/death_valley_visual.obj'

        self.mesh_path = mesh_path
        self.resolution = resolution
        self.min_x, self.max_x = -75.0, 75.0
        self.min_y, self.max_y = -75.0, 75.0
        self.grid_w = int((self.max_x - self.min_x) / self.resolution) + 1
        self.grid_h = int((self.max_y - self.min_y) / self.resolution) + 1

        self.vertices = []
        self.faces = []
        self.height_grid = np.zeros((self.grid_h, self.grid_w), dtype=np.float32)
        self.slope_grid = np.zeros((self.grid_h, self.grid_w), dtype=np.float32)
        self.roughness_grid = np.zeros((self.grid_h, self.grid_w), dtype=np.float32)
        self.traversability_grid = np.zeros((self.grid_h, self.grid_w), dtype=np.uint8) # 0: safe, 1: difficult, 2: high_slope, 3: untraversable
        self.kdtree = None

        self._load_mesh()
        self._build_terrain_grids()

    def _load_mesh(self):
        if not os.path.exists(self.mesh_path):
            candidates = [
                '/home/ubuntu/sih_ws/install/death_valley_world/share/death_valley_world/models/death_valley_terrain/meshes/death_valley_visual.obj',
                '/home/ubuntu/sih_ws/src/death_valley_world/models/death_valley_terrain/meshes/death_valley_visual.obj',
                '/home/ubuntu/sih_ws/src/death_valley_world/meshes/death_valley_visual.obj',
                '/home/joshika/Desktop/SIH/src/death_valley_world/models/death_valley_terrain/meshes/death_valley_visual.obj',
                '/home/joshika/Desktop/SIH/src/death_valley_world/meshes/death_valley_visual.obj',
            ]
            for c in candidates:
                if os.path.exists(c):
                    self.mesh_path = c
                    break

        if not os.path.exists(self.mesh_path):
            print(f'[TerrainAnalyzer] Warning: Mesh file {self.mesh_path} not found.')
            return

        verts = []
        with open(self.mesh_path, 'r') as f:
            for line in f:
                if line.startswith('v '):
                    parts = line.strip().split()
                    verts.append([float(parts[1]), float(parts[2]), float(parts[3])])
        self.vertices = np.array(verts, dtype=np.float32)
        # Build 2D KD-Tree over X,Y for rapid surface snapping
        self.kdtree = cKDTree(self.vertices[:, :2])
        print(f'[TerrainAnalyzer] Loaded {len(self.vertices)} vertices from {os.path.basename(self.mesh_path)}')

    def _build_terrain_grids(self):
        if len(self.vertices) == 0:
            return

        # Sample terrain elevations onto regular grid using KD-Tree interpolation
        xs = np.linspace(self.min_x, self.max_x, self.grid_w)
        ys = np.linspace(self.min_y, self.max_y, self.grid_h)
        gx, gy = np.meshgrid(xs, ys)
        query_pts = np.column_stack([gx.ravel(), gy.ravel()])

        dists, idxs = self.kdtree.query(query_pts, k=4)
        weights = 1.0 / np.maximum(dists, 1e-4)
        weights /= weights.sum(axis=1, keepdims=True)
        z_interp = np.sum(self.vertices[idxs, 2] * weights, axis=1)
        self.height_grid = z_interp.reshape((self.grid_h, self.grid_w))

        # Compute gradient and slope
        zy, zx = np.gradient(self.height_grid, self.resolution)
        grad_mag = np.hypot(zx, zy)
        self.slope_grid = np.degrees(np.arctan(grad_mag))

        # Compute local roughness (local standard deviation of slope)
        from scipy.ndimage import generic_filter
        self.roughness_grid = np.clip(grad_mag * 10.0, 0.0, 100.0)

        # 4-Tier Traversability Classification:
        # Safe (< 15 deg), Difficult (15-25 deg), High Slope (25-35 deg), Untraversable (>= 35 deg)
        self.traversability_grid[self.slope_grid < 15.0] = 0 # Safe (Green)
        self.traversability_grid[(self.slope_grid >= 15.0) & (self.slope_grid < 25.0)] = 1 # Difficult (Yellow)
        self.traversability_grid[(self.slope_grid >= 25.0) & (self.slope_grid < 35.0)] = 2 # High Slope (Orange)
        self.traversability_grid[self.slope_grid >= 35.0] = 3 # Untraversable / Cliff (Red)

        print(f'[TerrainAnalyzer] Grid built: {self.grid_w}x{self.grid_h} ({self.resolution}m res). Elev: [{self.height_grid.min():.1f}m, {self.height_grid.max():.1f}m]')

    def get_surface_elevation(self, x, y):
        """Get exact 3D terrain surface elevation Z for coordinates (x, y)."""
        if self.kdtree is None:
            return 3.0
        x_c = np.clip(x, self.min_x, self.max_x)
        y_c = np.clip(y, self.min_y, self.max_y)
        dists, idxs = self.kdtree.query([x_c, y_c], k=3)
        w = 1.0 / np.maximum(dists, 1e-4)
        w /= np.sum(w)
        return float(np.sum(self.vertices[idxs, 2] * w))

    def get_height(self, x, y):
        """Alias for get_surface_elevation."""
        return self.get_surface_elevation(x, y)

    def get_terrain_properties(self, x, y):
        """Returns (elevation, slope_deg, roughness, traversability_class, surface_normal)."""
        gx = int(round((np.clip(x, self.min_x, self.max_x) - self.min_x) / self.resolution))
        gy = int(round((np.clip(y, self.min_y, self.max_y) - self.min_y) / self.resolution))
        gx = np.clip(gx, 0, self.grid_w - 1)
        gy = np.clip(gy, 0, self.grid_h - 1)

        z = float(self.height_grid[gy, gx])
        slope = float(self.slope_grid[gy, gx])
        rough = float(self.roughness_grid[gy, gx])
        t_class = int(self.traversability_grid[gy, gx])
        
        # Surface normal
        dz_dx = 0.0
        dz_dy = 0.0
        if 0 < gx < self.grid_w - 1:
            dz_dx = (self.height_grid[gy, gx+1] - self.height_grid[gy, gx-1]) / (2 * self.resolution)
        if 0 < gy < self.grid_h - 1:
            dz_dy = (self.height_grid[gy+1, gx] - self.height_grid[gy-1, gx]) / (2 * self.resolution)
        
        norm_len = math.sqrt(dz_dx**2 + dz_dy**2 + 1.0)
        normal = (-dz_dx / norm_len, -dz_dy / norm_len, 1.0 / norm_len)

        return z, slope, rough, t_class, normal

    def get_traversability_cost(self, x, y):
        """Cost multiplier for path planning: 1.0 (flat safe) to inf (cliff)."""
        z, slope, rough, t_class, normal = self.get_terrain_properties(x, y)
        if t_class == 3: # Untraversable
            return float('inf')
        elif t_class == 2: # High slope
            return 8.0 + (slope - 25.0) * 0.5
        elif t_class == 1: # Difficult
            return 2.5 + (slope - 15.0) * 0.3
        else: # Safe
            return 1.0 + slope * 0.05
