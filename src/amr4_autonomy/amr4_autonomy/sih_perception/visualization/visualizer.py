"""
Multi-Panel Perception and Traversability Diagnostic Dashboard.
Renders high-contrast, publication-grade composite telemetry views:
1. Annotated RGB stream with semantic contours & 3D obstacle tags
2. Color-mapped Depth field with metric range rings
3. 2.5D Local Costmap with Tracked Robot Footprint & Inflation
4. Obstacle Radar & System Telemetry HUD
"""

from typing import Optional, List
import numpy as np
import cv2

from ..pipeline.pipeline import PerceptionResult
from ..traversability.decision_engine import TraversabilityType, TRAVERSABILITY_COLORS_RGB
from ..segmentation.taxonomy import CLASS_COLORS_RGB


class PerceptionVisualizer:
    """
    Renders 4-panel real-time perception dashboard.
    """

    def __init__(self, panel_width: int = 640, panel_height: int = 480):
        self.pw = panel_width
        self.ph = panel_height

    def render_rgb_panel(self, result: PerceptionResult) -> np.ndarray:
        """Render RGB stream with overlaid semantic contours and obstacle bounding boxes."""
        img = cv2.resize(result.rgb, (self.pw, self.ph)).copy()

        # 1. Overlay semantic segmentation with alpha blending
        sem_rgb = np.zeros_like(img)
        for class_id, color in CLASS_COLORS_RGB.items():
            sem_rgb[result.semantic_mask == class_id] = color
        
        sem_resized = cv2.resize(sem_rgb, (self.pw, self.ph))
        cv2.addWeighted(sem_resized, 0.35, img, 0.65, 0, img)

        # 2. Draw 2D Obstacle Bounding Boxes & Badges
        scale_x = self.pw / result.rgb.shape[1]
        scale_y = self.ph / result.rgb.shape[0]

        for obs in result.obstacles:
            u_min, v_min, u_max, v_max = obs.bbox_2d
            bx1, by1 = int(u_min * scale_x), int(v_min * scale_y)
            bx2, by2 = int(u_max * scale_x), int(v_max * scale_y)

            # Border color based on traversability
            if obs.is_run_over_allowed:
                box_color = (0, 220, 100)  # Bright Green
                badge = f"RUN-OVER (H={obs.height_m:.2f}m)"
            elif obs.traversability_type == TraversabilityType.NEGATIVE_HAZARD:
                box_color = (180, 50, 220) # Purple
                badge = f"DITCH HAZARD (Drop={obs.height_m:.2f}m)"
            else:
                box_color = (50, 50, 230)  # Red
                badge = f"AVOID (H={obs.height_m:.2f}m)"

            cv2.rectangle(img, (bx1, by1), (bx2, by2), box_color, 2)

            # Label banner
            label_text = f"ID#{obs.id} {obs.semantic_name.split('(')[0].strip()}"
            dist_text = f"{obs.radial_distance_m:.2f}m | {obs.azimuth_deg:+.1f} deg | {badge}"

            # Text background pill
            (tw1, th1), _ = cv2.getTextSize(label_text, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)
            (tw2, th2), _ = cv2.getTextSize(dist_text, cv2.FONT_HERSHEY_SIMPLEX, 0.40, 1)
            banner_w = max(tw1, tw2) + 12
            banner_h = th1 + th2 + 14

            banner_y1 = max(0, by1 - banner_h)
            banner_y2 = by1
            cv2.rectangle(img, (bx1, banner_y1), (bx1 + banner_w, banner_y2), (20, 20, 20), -1)
            cv2.rectangle(img, (bx1, banner_y1), (bx1 + banner_w, banner_y2), box_color, 1)

            cv2.putText(img, label_text, (bx1 + 6, banner_y1 + th1 + 3), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
            cv2.putText(img, dist_text, (bx1 + 6, banner_y1 + th1 + th2 + 8), cv2.FONT_HERSHEY_SIMPLEX, 0.40, box_color, 1)

        # Panel Title Bar
        cv2.rectangle(img, (0, 0), (self.pw, 28), (30, 30, 30), -1)
        cv2.putText(img, "[1] RGB Stream + Semantic Mask + 3D Detection", (12, 19), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (240, 240, 240), 1)
        return img

    def render_depth_panel(self, result: PerceptionResult) -> np.ndarray:
        """Render color-mapped depth field with range markings."""
        depth = result.depth
        valid = result.valid_mask & (depth > 0.1)

        depth_norm = np.zeros_like(depth, dtype=np.uint8)
        if valid.any():
            d_min = np.percentile(depth[valid], 2)
            d_max = np.percentile(depth[valid], 98)
            d_max = max(d_max, d_min + 1.0)
            d_scaled = np.clip((depth[valid] - d_min) / (d_max - d_min) * 255.0, 0, 255).astype(np.uint8)
            depth_norm[valid] = d_scaled

        # Apply Turbo Colormap
        depth_color = cv2.applyColorMap(depth_norm, cv2.COLORMAP_TURBO)
        depth_color[~valid] = [20, 20, 20]
        panel = cv2.resize(depth_color, (self.pw, self.ph))

        # Panel Title Bar
        cv2.rectangle(panel, (0, 0), (self.pw, 28), (30, 30, 30), -1)
        cv2.putText(panel, "[2] Metric Stereo Depth Map (0.2m - 10.0m)", (12, 19), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (240, 240, 240), 1)

        # Stats footer
        if valid.any():
            min_d = np.min(depth[valid])
            med_d = np.median(depth[valid])
            max_d = np.max(depth[valid])
            footer_text = f"Range: Min {min_d:.2f}m | Median {med_d:.2f}m | Max {max_d:.2f}m"
            cv2.rectangle(panel, (0, self.ph - 24), (self.pw, self.ph), (20, 20, 20), -1)
            cv2.putText(panel, footer_text, (12, self.ph - 7), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)

        return panel

    def render_costmap_panel(self, result: PerceptionResult) -> np.ndarray:
        """Render 2.5D top-down local grid costmap with vehicle footprint and range rings."""
        grid = result.costmap_grid  # (cells_x, cells_y) - X forward, Y left
        cells_x, cells_y = grid.shape

        # Build RGB image: (cells_x, cells_y, 3)
        # Flip so forward is UP: rotate 90 deg counter-clockwise
        cost_img = np.zeros((cells_x, cells_y, 3), dtype=np.uint8)

        # Free space (cost 0): Dark Green
        cost_img[grid == 0] = [30, 80, 40]
        # Soft / Traversable run-over (cost 1..100): Blue/Cyan
        trav_mask = (grid > 0) & (grid < 150)
        cost_img[trav_mask] = [180, 140, 40]
        # Inscribed inflation zone (cost 150..253): Orange
        inf_mask = (grid >= 150) & (grid < 254)
        cost_img[inf_mask] = [40, 140, 230]
        # Lethal obstacle (cost 254): Crimson Red
        cost_img[grid >= 254] = [40, 40, 230]

        # Rotate so X is UP and Y is RIGHT
        # OpenCV: rotate90 counter-clockwise
        cost_display = cv2.rotate(cost_img, cv2.ROTATE_90_COUNTERCLOCKWISE)
        cost_resized = cv2.resize(cost_display, (self.pw, self.ph), interpolation=cv2.INTER_NEAREST)

        # Draw range circles (1m, 2m, 3m, 5m) around robot
        # Robot position in costmap: X=0m, Y=0m
        # Origin is at (-1.0, -4.0), resolution=0.05
        # Cell: gx = (0 - (-1.0))/0.05 = 20, gy = (0 - (-4.0))/0.05 = 80
        res = 0.05
        rx_cell = int(1.0 / res)
        ry_cell = int(4.0 / res)
        
        # In rotated display: display_x = (cells_y - 1 - gy) * (pw / cells_y) -> actually in rot90:
        # rot90_counterclockwise: (x, y) -> (cells_y - 1 - y, x)
        center_px_x = int((ry_cell / cells_y) * self.pw)
        center_px_y = int((1.0 - (rx_cell / cells_x)) * self.ph)

        for radius_m in [1.0, 2.0, 3.0, 5.0]:
            r_px = int((radius_m / res) * (self.ph / cells_x))
            cv2.circle(cost_resized, (center_px_x, center_px_y), r_px, (90, 90, 90), 1, cv2.LINE_AA)
            cv2.putText(cost_resized, f"{radius_m:.0f}m", (center_px_x + r_px + 4, center_px_y), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (140, 140, 140), 1)

        # Draw Tracked Robot Footprint (0.8m x 0.6m)
        rw_px = int((0.6 / res) * (self.pw / cells_y))
        rl_px = int((0.8 / res) * (self.ph / cells_x))
        cv2.rectangle(
            cost_resized,
            (center_px_x - rw_px // 2, center_px_y - rl_px // 2),
            (center_px_x + rw_px // 2, center_px_y + rl_px // 2),
            (255, 255, 255),
            2
        )
        # Heading indicator arrow
        cv2.arrowedLine(
            cost_resized,
            (center_px_x, center_px_y),
            (center_px_x, center_px_y - rl_px),
            (0, 255, 255),
            2,
            tipLength=0.3
        )

        # Panel Title Bar
        cv2.rectangle(cost_resized, (0, 0), (self.pw, 28), (30, 30, 30), -1)
        cv2.putText(cost_resized, "[3] 2.5D Local Costmap (Robot Footprint & Inflation)", (12, 19), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (240, 240, 240), 1)
        return cost_resized

    def render_hud_panel(self, result: PerceptionResult) -> np.ndarray:
        """Render HUD Telemetry, Latency breakdown, and Obstacle Status Table."""
        hud = np.full((self.ph, self.pw, 3), 24, dtype=np.uint8)

        # Title Bar
        cv2.rectangle(hud, (0, 0), (self.pw, 28), (45, 52, 54), -1)
        cv2.putText(hud, "[4] System Telemetry & Obstacle Radar", (12, 19), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (240, 240, 240), 1)

        # Performance & Timing Section
        t = result.timing
        fps_color = (46, 204, 113) if t.fps >= 25.0 else (230, 126, 34)
        cv2.putText(hud, f"Real-Time FPS: {t.fps:.1f}", (16, 56), cv2.FONT_HERSHEY_SIMPLEX, 0.65, fps_color, 2)
        cv2.putText(hud, f"Total Pipeline: {t.total_ms:.1f} ms", (240, 56), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)

        # Ground Plane Telemetry
        plane = result.ground_plane
        plane_text = f"Ground Surface: Pitch/Slope = {plane.slope_deg:.1f} deg | Inlier Ratio = {plane.inlier_ratio*100:.1f}%"
        cv2.putText(hud, plane_text, (16, 85), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (180, 200, 220), 1)

        # Stage Latency Breakdown Bars
        stages = [
            ("3D Back-Proj", t.back_projection_ms, 6.0),
            ("RANSAC Plane", t.ground_plane_ms, 8.0),
            ("Segmentation", t.segmentation_ms, 12.0),
            ("Traversability", t.traversability_ms, 4.0),
            ("Costmap Gen", t.costmap_ms, 5.0),
            ("3D Detection", t.detection_ms, 3.0),
        ]

        bar_start_y = 112
        for i, (name, val, budget) in enumerate(stages):
            y = bar_start_y + i * 22
            cv2.putText(hud, f"{name}: {val:.1f}ms", (16, y + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (200, 200, 200), 1)
            
            # Progress bar
            bar_x = 160
            bar_w = 200
            fill_w = int(min(val / max(budget * 1.5, 1.0), 1.0) * bar_w)
            bar_col = (46, 204, 113) if val <= budget else (231, 76, 60)
            
            cv2.rectangle(hud, (bar_x, y), (bar_x + bar_w, y + 12), (50, 50, 50), -1)
            cv2.rectangle(hud, (bar_x, y), (bar_x + fill_w, y + 12), bar_col, -1)
            cv2.rectangle(hud, (bar_x, y), (bar_x + bar_w, y + 12), (100, 100, 100), 1)

        # Obstacle Table Section
        table_y = 265
        cv2.rectangle(hud, (12, table_y), (self.pw - 12, self.ph - 12), (35, 35, 35), -1)
        cv2.rectangle(hud, (12, table_y), (self.pw - 12, self.ph - 12), (70, 70, 70), 1)

        cv2.putText(hud, f"DETECTED 3D OBSTACLES ({len(result.obstacles)} active)", (20, table_y + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (255, 255, 255), 1)
        
        # Header
        headers = ["ID", "Range", "Bearing", "Height", "Verdict / Action", "Cost"]
        cols_x = [20, 55, 125, 200, 280, 560]
        for col_name, cx in zip(headers, cols_x):
            cv2.putText(hud, col_name, (cx, table_y + 42), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (160, 160, 160), 1)

        cv2.line(hud, (16, table_y + 48), (self.pw - 16, table_y + 48), (60, 60, 60), 1)

        for row_idx, obs in enumerate(result.obstacles[:5]):
            ry = table_y + 68 + row_idx * 24
            
            if obs.is_run_over_allowed:
                tag_col = (46, 204, 113)  # Green
                tag_text = "RUN-OVER (Safe)"
            elif obs.traversability_type == TraversabilityType.NEGATIVE_HAZARD:
                tag_col = (155, 89, 182) # Purple
                tag_text = "DITCH (Fatal Avoid)"
            else:
                tag_col = (231, 76, 60)   # Red
                tag_text = "LETHAL (Avoid)"

            cv2.putText(hud, f"#{obs.id}", (cols_x[0], ry), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (255, 255, 255), 1)
            cv2.putText(hud, f"{obs.radial_distance_m:.2f}m", (cols_x[1], ry), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (220, 220, 220), 1)
            cv2.putText(hud, f"{obs.azimuth_deg:+.1f} deg", (cols_x[2], ry), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (220, 220, 220), 1)
            cv2.putText(hud, f"{obs.height_m*100:.0f} cm", (cols_x[3], ry), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (220, 220, 220), 1)
            cv2.putText(hud, tag_text, (cols_x[4], ry), cv2.FONT_HERSHEY_SIMPLEX, 0.38, tag_col, 1)
            cv2.putText(hud, f"{obs.severity_cost}", (cols_x[5], ry), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (200, 200, 200), 1)

        return hud

    def render_composite_dashboard(self, result: PerceptionResult) -> np.ndarray:
        """Combine all 4 panels into a 2x2 grid (1280 x 960)."""
        p1 = self.render_rgb_panel(result)
        p2 = self.render_depth_panel(result)
        p3 = self.render_costmap_panel(result)
        p4 = self.render_hud_panel(result)

        top_row = np.hstack([p1, p2])
        bottom_row = np.hstack([p3, p4])
        composite = np.vstack([top_row, bottom_row])
        return composite

    def save_dashboard(self, result: PerceptionResult, output_path: str) -> None:
        """Save composite dashboard to disk (BGR conversion for OpenCV)."""
        composite = self.render_composite_dashboard(result)
        # Convert RGB to BGR for cv2.imwrite
        bgr = cv2.cvtColor(composite, cv2.COLOR_RGB2BGR)
        cv2.imwrite(output_path, bgr)
