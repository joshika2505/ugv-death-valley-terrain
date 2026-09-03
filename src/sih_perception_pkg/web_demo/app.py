"""
Interactive Flask Web Application for SIH Tracked UGV Perception & Traversability Demo.
Provides REST APIs and real-time processing for the interactive judging dashboard.
"""

import os
import sys
import base64
import json
import time
from typing import Dict, Any, Optional
import numpy as np
import cv2
from flask import Flask, render_template, request, jsonify, send_from_directory
from flask_cors import CORS

# Ensure repository root is on sys.path
REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, REPO_ROOT)

from sih_perception.core.camera import StereoCameraModel, CameraParameters
from sih_perception.traversability.vehicle_profile import TrackedVehicleProfile
from sih_perception.pipeline.pipeline import PerceptionPipeline, PerceptionResult
from sih_perception.pipeline.ros2_bridge import ROS2MessageBridge
from sih_perception.simulation.synthetic_scene import SyntheticSceneGenerator, ScenarioType
from sih_perception.visualization.visualizer import PerceptionVisualizer
from sih_perception.traversability.decision_engine import TraversabilityType
from sih_perception.segmentation.taxonomy import CLASS_COLORS_RGB, CLASS_NAMES

app = Flask(__name__, template_folder="templates", static_folder="static")
CORS(app)

# Global state / generator
camera_params = CameraParameters()
vehicle_profile = TrackedVehicleProfile()
pipeline = PerceptionPipeline(camera_params=camera_params, vehicle_profile=vehicle_profile)
sim = SyntheticSceneGenerator(camera_params=camera_params)
viz = PerceptionVisualizer()


def encode_image_base64(img_rgb: np.ndarray) -> str:
    """Convert RGB numpy array to base64 JPEG data string."""
    img_bgr = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2BGR)
    _, buffer = cv2.imencode('.jpg', img_bgr, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
    return "data:image/jpeg;base64," + base64.b64encode(buffer).decode('utf-8')


@app.route("/")
def index():
    return render_template("index.html")


@app.route("/api/process", methods=["POST"])
def process_scene():
    """
    Process requested scenario with dynamic parameters.
    Receives JSON with scenario type and optional vehicle / camera profile overrides.
    """
    data = request.json or {}
    scenario_str = data.get("scenario", "rock")
    
    # Extract user parameter overrides
    climb_step = float(data.get("max_climb_step", 0.15))
    ground_clearance = float(data.get("ground_clearance", 0.10))
    soft_height = float(data.get("soft_vegetation_max_height", 0.40))
    max_slope = float(data.get("max_slope_deg", 35.0))
    pitch_deg = float(data.get("pitch_deg", -8.0))
    custom_obs_dist = float(data.get("custom_obs_dist", 2.8))
    custom_obs_height = float(data.get("custom_obs_height", 0.25))

    # Update camera and vehicle profile
    cam_p = CameraParameters(pitch_deg=pitch_deg)
    veh_p = TrackedVehicleProfile(
        ground_clearance=ground_clearance,
        max_climb_step=climb_step,
        soft_vegetation_max_height=soft_height,
        max_slope_deg=max_slope
    )

    custom_pipeline = PerceptionPipeline(camera_params=cam_p, vehicle_profile=veh_p)
    custom_sim = SyntheticSceneGenerator(camera_params=cam_p)

    # Select scenario
    scenario_map = {
        "debris": ScenarioType.FLAT_TRAIL_SMALL_DEBRIS,
        "grass": ScenarioType.TALL_GRASS_MEADOW,
        "rock": ScenarioType.LETHAL_ROCK_BOULDER,
        "ditch": ScenarioType.NEGATIVE_DITCH_TRENCH,
        "slope": ScenarioType.STEEP_INCLINE_SLOPE,
    }

    if scenario_str == "custom":
        # Generate custom parametric scenario
        rgb, depth = custom_sim.generate_scenario(ScenarioType.LETHAL_ROCK_BOULDER)
        # Re-synthesize with custom rock distance & height
        w, h = cam_p.width, cam_p.height
        rays_unit = (custom_sim.camera.ray_dir_robot / np.linalg.norm(custom_sim.camera.ray_dir_robot, axis=-1, keepdims=True)).astype(np.float32)
        cam_origin = custom_sim.camera.t_cam_to_base.astype(np.float32)
        opt_forward_robot = (custom_sim.camera.R_cam_to_base @ np.array([0.0, 0.0, 1.0], dtype=np.float32)).astype(np.float32)
        cos_opt = np.sum(rays_unit * opt_forward_robot, axis=-1)

        dir_z = rays_unit[..., 2]
        downward = dir_z < -0.01
        s_ground = np.full((h, w), np.nan, dtype=np.float32)
        s_ground[downward] = -cam_origin[2] / dir_z[downward]

        center_rock = np.array([custom_obs_dist, 0.0, 0.0], dtype=np.float32)
        radius = max(0.04, custom_obs_height)
        v = cam_origin - center_rock
        b = 2.0 * np.sum(rays_unit * v, axis=-1)
        c = float(np.sum(v**2) - radius**2)
        disc = b**2 - 4.0 * c
        rock_hit = (disc >= 0)
        s_rock = (-b - np.sqrt(np.maximum(disc, 0.0))) / 2.0
        rock_valid = rock_hit & (s_rock > 0.4) & (s_rock < s_ground)
        s_final = s_ground.copy()
        s_final[rock_valid] = s_rock[rock_valid]

        depth = np.nan_to_num(s_final * cos_opt, nan=0.0)
        rgb = np.zeros((h, w, 3), dtype=np.uint8)
        rgb[..., 0] = 135; rgb[..., 1] = 110; rgb[..., 2] = 80
        if rock_valid.any():
            rgb[rock_valid] = [95, 90, 85]
    else:
        sc_type = scenario_map.get(scenario_str, ScenarioType.LETHAL_ROCK_BOULDER)
        rgb, depth = custom_sim.generate_scenario(sc_type)

    # Process frame through perception engine
    res = custom_pipeline.process_frame(rgb, depth)

    # Render individual visual panels
    rgb_panel = viz.render_rgb_panel(res)
    depth_panel = viz.render_depth_panel(res)
    costmap_panel = viz.render_costmap_panel(res)

    # Format obstacles payload
    obstacles_data = []
    for obs in res.obstacles:
        obstacles_data.append({
            "id": obs.id,
            "radial_distance_m": round(obs.radial_distance_m, 2),
            "azimuth_deg": round(obs.azimuth_deg, 1),
            "centroid_x": round(obs.centroid_x, 2),
            "centroid_y": round(obs.centroid_y, 2),
            "centroid_z": round(obs.centroid_z, 2),
            "height_m": round(obs.height_m, 3),
            "height_cm": round(obs.height_m * 100, 1),
            "width_cm": round(obs.width_m * 100, 1),
            "length_cm": round(obs.length_m * 100, 1),
            "is_run_over_allowed": obs.is_run_over_allowed,
            "traversability_type": int(obs.traversability_type),
            "traversability_name": obs.traversability_name,
            "semantic_class": obs.semantic_class,
            "semantic_name": obs.semantic_name,
            "severity_cost": obs.severity_cost,
            "bbox_2d": obs.bbox_2d,
        })

    # ROS2 payload preview
    ros_occupancy = ROS2MessageBridge.create_occupancy_grid_msg(res)
    ros_detections = ROS2MessageBridge.create_detection_3d_array_msg(res)

    # Generate 3D point cloud downsample for web visualizer probe
    pts_sub = res.pts_robot[::8, ::8].reshape(-1, 3)
    valid_sub = res.valid_mask[::8, ::8].reshape(-1)
    pts_valid = pts_sub[valid_sub]

    response_payload = {
        "success": True,
        "scenario": scenario_str,
        "images": {
            "rgb_annotated": encode_image_base64(rgb_panel),
            "depth_colormap": encode_image_base64(depth_panel),
            "costmap_25d": encode_image_base64(costmap_panel),
        },
        "telemetry": {
            "fps": round(res.timing.fps, 1),
            "total_latency_ms": round(res.timing.total_ms, 1),
            "latency_breakdown": {
                "back_projection_ms": round(res.timing.back_projection_ms, 2),
                "ground_plane_ms": round(res.timing.ground_plane_ms, 2),
                "segmentation_ms": round(res.timing.segmentation_ms, 2),
                "traversability_ms": round(res.timing.traversability_ms, 2),
                "costmap_ms": round(res.timing.costmap_ms, 2),
                "detection_ms": round(res.timing.detection_ms, 2),
            },
            "ground_plane": {
                "equation": f"{res.ground_plane.a:.3f}x + {res.ground_plane.b:.3f}y + {res.ground_plane.c:.3f}z + {res.ground_plane.d:.3f} = 0",
                "slope_deg": round(res.ground_plane.slope_deg, 2),
                "inlier_ratio_pct": round(res.ground_plane.inlier_ratio * 100, 1),
            },
            "num_obstacles": len(res.obstacles),
            "num_lethal": sum(1 for o in res.obstacles if not o.is_run_over_allowed),
            "num_run_over": sum(1 for o in res.obstacles if o.is_run_over_allowed),
        },
        "obstacles": obstacles_data,
        "ros2_sample": {
            "occupancy_grid_info": ros_occupancy["info"],
            "detection_3d_count": len(ros_detections["detections"]),
            "sample_detection": ros_detections["detections"][0] if ros_detections["detections"] else None,
        }
    }

    return jsonify(response_payload)


def start_server(port: int = 5050, debug: bool = False):
    print(f"Starting SIH Perception Demo Server on http://localhost:{port}")
    app.run(host="0.0.0.0", port=port, debug=debug)


if __name__ == "__main__":
    start_server(port=5050)
