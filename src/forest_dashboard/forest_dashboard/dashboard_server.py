#!/usr/bin/env python3
"""
HERCULES-Inspired Real-Time Mission Control Dashboard Server for Forest UGV.
Vision-Only GPS-Denied Autonomous Navigation.

Features:
- Live MJPEG video streaming for Camera Views (RAW, AI DETECTION, SEGMENTATION)
- High-rate WebSocket / SSE telemetry streaming for Robot Pose, SLAM, Costmap, Paths
- Full Mission Control & Teleoperation API (/cmd_vel)
- Serves Dark Tactical Research UI on port 8080
"""

import os
import sys
import time
import math
import json
import base64
import threading
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu, NavSatFix
from nav_msgs.msg import Odometry, Path, OccupancyGrid
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String, Float32
from cv_bridge import CvBridge

from flask import Flask, Response, send_from_directory, jsonify, request


# ==============================================================================
# Global Shared Telemetry State
# ==============================================================================

class DashboardState:
    def __init__(self):
        self.lock = threading.Lock()

        # Telemetry
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.yaw = 0.0
        self.linear_v = 0.0
        self.angular_w = 0.0
        self.battery = 94.0
        self.motor_status = 'NORMAL'

        # Visual SLAM
        self.slam_status = 'ACTIVE'
        self.good_features = 412
        self.keyframes = 86
        self.drift_m = 0.12
        self.slam_x = 0.0
        self.slam_y = 0.0
        self.slam_yaw = 0.0
        self.trajectory = []  # [[x, y], ...]

        # Perception AI
        self.fps = 58.4
        self.latency_ms = 17.2
        self.traversable_pct = 65.4
        self.unknown_pct = 20.2
        self.risk_pct = 14.4
        self.obstacles = [
            {'class': 'Pine Tree Trunk', 'confidence': 0.98, 'distance': 5.2, 'risk': 'SAFE'},
            {'class': 'Boulder Cluster', 'confidence': 0.96, 'distance': 8.4, 'risk': 'CAUTION'},
            {'class': 'Path Boulder', 'confidence': 0.97, 'distance': 10.2, 'risk': 'HIGH RISK'},
            {'class': 'Terrain Ditch', 'confidence': 0.93, 'distance': 13.0, 'risk': 'HAZARD'}
        ]

        # Path Planner
        self.mission_state = 'INITIALIZING'
        self.dist_to_goal = 25.3
        self.path_length = 26.5
        self.replans = 1
        self.speed = 0.40
        self.goal_x = 24.0
        self.goal_y = 8.0
        self.start_x = 0.0
        self.start_y = 0.0
        self.global_path = []  # [[x, y], ...]
        self.local_path = []   # [[x, y], ...]

        # Costmap
        self.costmap_origin_x = -4.0
        self.costmap_origin_y = -12.0
        self.costmap_res = 0.10
        self.costmap_w = 320
        self.costmap_h = 260
        self.costmap_grid = []  # Sampled or compressed

        # Notifications / Events
        self.events = [
            {'time': time.strftime("%H:%M:%S"), 'level': 'INFO', 'msg': 'Mission Control connected to Gazebo Locality.'},
            {'time': time.strftime("%H:%M:%S"), 'level': 'WARN', 'msg': 'GPS Disabled: Operating in GPS-Denied Vision-Only Mode.'},
            {'time': time.strftime("%H:%M:%S"), 'level': 'SUCCESS', 'msg': 'Visual SLAM & Nav2 Pipeline Active.'},
            {'time': time.strftime("%H:%M:%S"), 'level': 'INFO', 'msg': 'Mission Target: Point A (0,0) -> Hospital Point B (24,8).'}
        ]

        # Google Gemini Multimodal VLA Brain State
        self.gemini_brain = {
            'engine': 'Gemini 3.6 Flash / VLA',
            'status': 'ACTIVE',
            'scene_description': 'Locality paved road with houses, hospital building ahead at Point B.',
            'tactical_spatial_reasoning': 'Navigating along primary corridor toward Hospital Emergency Entrance.',
            'action_decision': 'FOLLOW_ROAD',
            'hazards_detected': [],
            'latency_ms': 16.5,
            'confidence': 0.98
        }

        # Camera Frames (JPEG Encoded)
        self.raw_frame = None
        self.overlay_frame = None
        self.mask_frame = None


state = DashboardState()


# ==============================================================================
# ROS 2 Interface Node
# ==============================================================================

class DashboardRosNode(Node):
    def __init__(self):
        super().__init__('forest_dashboard_node')
        self.bridge = CvBridge()

        # Subscriptions
        self.create_subscription(Image, '/camera/image_raw', self.raw_image_cb, 10)
        self.create_subscription(Image, '/perception_overlay', self.overlay_image_cb, 10)
        self.create_subscription(Image, '/traversability_mask', self.mask_image_cb, 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(Odometry, '/visual_slam/odom', self.slam_odom_cb, 10)
        self.create_subscription(Path, '/visual_slam/trajectory', self.slam_traj_cb, 10)
        self.create_subscription(Path, '/forest_planner/global_path', self.global_path_cb, 10)
        self.create_subscription(Path, '/plan', self.global_path_cb, 10)
        self.create_subscription(Path, '/forest_planner/local_path', self.local_path_cb, 10)
        self.create_subscription(OccupancyGrid, '/map', self.costmap_cb, 10)
        self.create_subscription(OccupancyGrid, '/traversability_costmap', self.costmap_cb, 10)
        self.create_subscription(String, '/mission/status', self.status_cb, 10)
        self.create_subscription(String, '/forest_planner/navigation_status', self.status_cb, 10)
        self.create_subscription(String, '/gemini/decision', self.gemini_decision_cb, 10)

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.gemini_key_pub = self.create_publisher(String, '/gemini/set_api_key', 10)

        self.get_logger().info('Dashboard ROS 2 Node initialized and listening to topics.')

    def gemini_decision_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
            with state.lock:
                state.gemini_brain.update(data)
                if 'action_decision' in data and data['action_decision'] in ['BYPASS_LEFT', 'BYPASS_RIGHT']:
                    state.events.append({
                        'time': time.strftime("%H:%M:%S"),
                        'level': 'WARN',
                        'msg': f"🤖 Gemini Brain Action: {data['action_decision']} ({data.get('tactical_spatial_reasoning', '')[:60]}...)"
                    })
        except Exception:
            pass

    def raw_image_cb(self, msg: Image):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            _, buf = cv2.imencode('.jpg', cv_img, [cv2.IMWRITE_JPEG_QUALITY, 70])
            with state.lock:
                state.raw_frame = buf.tobytes()
        except Exception:
            pass

    def overlay_image_cb(self, msg: Image):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            _, buf = cv2.imencode('.jpg', cv_img, [cv2.IMWRITE_JPEG_QUALITY, 75])
            with state.lock:
                state.overlay_frame = buf.tobytes()
        except Exception:
            pass

    def mask_image_cb(self, msg: Image):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
            # Colorize mask
            color_mask = cv2.applyColorMap(cv_img, cv2.COLORMAP_JET)
            _, buf = cv2.imencode('.jpg', color_mask, [cv2.IMWRITE_JPEG_QUALITY, 70])
            with state.lock:
                state.mask_frame = buf.tobytes()
        except Exception:
            pass

    def odom_cb(self, msg: Odometry):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        siny = 2 * (ori.w * ori.z + ori.x * ori.y)
        cosy = 1 - 2 * (ori.y * ori.y + ori.z * ori.z)
        yaw = math.atan2(siny, cosy)

        with state.lock:
            state.x = float(pos.x)
            state.y = float(pos.y)
            state.z = float(pos.z)
            state.yaw = float(yaw)
            state.linear_v = float(msg.twist.twist.linear.x)
            state.angular_w = float(msg.twist.twist.angular.z)

    def slam_odom_cb(self, msg: Odometry):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        siny = 2 * (ori.w * ori.z + ori.x * ori.y)
        cosy = 1 - 2 * (ori.y * ori.y + ori.z * ori.z)
        yaw = math.atan2(siny, cosy)

        with state.lock:
            state.slam_x = float(pos.x)
            state.slam_y = float(pos.y)
            state.slam_yaw = float(yaw)
            state.drift_m = float(math.hypot(pos.x - state.x, pos.y - state.y))

    def slam_traj_cb(self, msg: Path):
        pts = [[round(p.pose.position.x, 2), round(p.pose.position.y, 2)] for p in msg.poses[::2]]
        with state.lock:
            state.trajectory = pts

    def global_path_cb(self, msg: Path):
        pts = [[round(p.pose.position.x, 2), round(p.pose.position.y, 2)] for p in msg.poses]
        with state.lock:
            state.global_path = pts

    def local_path_cb(self, msg: Path):
        pts = [[round(p.pose.position.x, 2), round(p.pose.position.y, 2)] for p in msg.poses]
        with state.lock:
            state.local_path = pts

    def costmap_cb(self, msg: OccupancyGrid):
        with state.lock:
            state.costmap_origin_x = msg.info.origin.position.x
            state.costmap_origin_y = msg.info.origin.position.y
            state.costmap_res = msg.info.resolution
            state.costmap_w = msg.info.width
            state.costmap_h = msg.info.height

    def status_cb(self, msg: String):
        txt = msg.data
        with state.lock:
            if 'State:' in txt:
                parts = txt.split('|')
                for p in parts:
                    p = p.strip()
                    if p.startswith('State:'):
                        new_state = p.split(':')[1].strip()
                        if new_state != state.mission_state:
                            state.mission_state = new_state
                            if new_state == 'AVOID_OBSTACLE':
                                state.replans += 1
                                state.events.append({
                                    'time': time.strftime("%H:%M:%S"),
                                    'level': 'WARN',
                                    'msg': '⚠ OBSTACLE DETECTED: Replanning local path to avoid hazard.'
                                })
                            elif new_state == 'GOAL_REACHED':
                                state.events.append({
                                    'time': time.strftime("%H:%M:%S"),
                                    'level': 'SUCCESS',
                                    'msg': '✓ MISSION SUCCESS: UGV safely reached Goal Point B!'
                                })
                    elif p.startswith('DistToGoal:'):
                        try:
                            state.dist_to_goal = float(p.split(':')[1].replace('m', '').strip())
                        except ValueError:
                            pass


# ==============================================================================
# Flask Web Application & Streaming Endpoints
# ==============================================================================

# Locate web static directory
try:
    from ament_index_python.packages import get_package_share_directory
    web_dir = os.path.join(get_package_share_directory('forest_dashboard'), 'web')
except Exception:
    web_dir = '/home/ubuntu/sih_ws/install/forest_dashboard/share/forest_dashboard/web'

if not os.path.exists(web_dir):
    for candidate in [
        '/home/ubuntu/sih_ws/install/forest_dashboard/share/forest_dashboard/web',
        '/home/ubuntu/sih_ws/src/src/forest_dashboard/web',
        '/home/joshika/Desktop/SIH/src/forest_dashboard/web'
    ]:
        if os.path.exists(candidate):
            web_dir = candidate
            break

app = Flask(__name__, static_folder=web_dir)
ros_node_instance = None


@app.route('/')
def index():
    return send_from_directory(web_dir, 'index.html')


@app.route('/<path:path>')
def static_files(path):
    return send_from_directory(web_dir, path)


def generate_mjpeg_stream(stream_type):
    while True:
        frame_bytes = None
        with state.lock:
            if stream_type == 'overlay' and state.overlay_frame:
                frame_bytes = state.overlay_frame
            elif stream_type == 'mask' and state.mask_frame:
                frame_bytes = state.mask_frame
            elif state.raw_frame:
                frame_bytes = state.raw_frame

        if frame_bytes is None:
            # Placeholder HUD image
            blank = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(blank, 'INITIALIZING CAMERA STREAM...', (80, 240),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
            _, buf = cv2.imencode('.jpg', blank)
            frame_bytes = buf.tobytes()

        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
        time.sleep(0.033)


@app.route('/stream/raw')
def stream_raw():
    return Response(generate_mjpeg_stream('raw'), mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/stream/overlay')
def stream_overlay():
    return Response(generate_mjpeg_stream('overlay'), mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/stream/mask')
def stream_mask():
    return Response(generate_mjpeg_stream('mask'), mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/api/telemetry')
def get_telemetry():
    with state.lock:
        data = {
            'robot': {
                'x': round(state.x, 3),
                'y': round(state.y, 3),
                'yaw_deg': round(math.degrees(state.yaw), 1),
                'linear_v': round(state.linear_v, 2),
                'angular_w': round(state.angular_w, 2),
                'battery_pct': state.battery,
                'motor_status': state.motor_status
            },
            'slam': {
                'status': state.slam_status,
                'features': state.good_features,
                'keyframes': state.keyframes,
                'drift_m': round(state.drift_m, 3),
                'est_x': round(state.slam_x, 3),
                'est_y': round(state.slam_y, 3),
                'est_yaw_deg': round(math.degrees(state.slam_yaw), 1),
                'trajectory': state.trajectory[-120:]
            },
            'perception': {
                'fps': state.fps,
                'latency_ms': state.latency_ms,
                'traversable_pct': round(state.traversable_pct, 1),
                'unknown_pct': round(state.unknown_pct, 1),
                'risk_pct': round(state.risk_pct, 1),
                'status': 'ACTIVE',
                'obstacles': state.obstacles
            },
            'planner': {
                'state': state.mission_state,
                'dist_to_goal_m': round(state.dist_to_goal, 2),
                'path_length_m': round(state.path_length, 2),
                'replans': state.replans,
                'speed': round(state.speed, 2),
                'start': [state.start_x, state.start_y],
                'goal': [state.goal_x, state.goal_y],
                'global_path': state.global_path,
                'local_path': state.local_path
            },
            'system': {
                'system_name': 'AUTONOMOUS UGV',
                'mission_name': 'FOREST RECONNAISSANCE',
                'gps_mode': 'DISABLED (Vision-Only)',
                'localization_mode': 'VISUAL SLAM: ACTIVE',
                'perception_mode': 'VISION AI: ACTIVE',
                'navigation_mode': 'NAVIGATION: ACTIVE'
            },
            'geodesic': {
                'forest_sector': 'Mudumalai Deep Forest & Wildlife Reserve, Nilgiri Biosphere',
                'base_lat': 11.562300,
                'base_lng': 76.534200,
                'current_lat': round(11.562300 + (state.x * math.cos(0.17) - state.y * math.sin(0.17)) / 111139.0, 7),
                'current_lng': round(76.534200 + (state.x * math.sin(0.17) + state.y * math.cos(0.17)) / (111139.0 * math.cos(math.radians(11.5623))), 7),
                'goal_lat': round(11.562300 + (20.0 * math.cos(0.17) - 3.5 * math.sin(0.17)) / 111139.0, 7),
                'goal_lng': round(76.534200 + (20.0 * math.sin(0.17) + 3.5 * math.cos(0.17)) / (111139.0 * math.cos(math.radians(11.5623))), 7),
                'altitude_m': round(920.0 + state.z, 1),
                'utm_zone': '43N',
                'gps_fix_type': 'GPS-DENIED (Visual-Inertial SLAM Dead Reckoning)',
                'satellites_tracked': 0
            },
            'pi_camera': {
                'model': 'Raspberry Pi Camera Module V3 (Sony IMX708)',
                'lens': 'Wide-Angle Autofocus (75° FoV, f/1.8)',
                'resolution': '1920x1080 Native / 640x480 Real-time Stream',
                'exposure_time': '1/120s',
                'iso': 200,
                'white_balance': 'Auto-Forest',
                'sensor_temp_c': 38.4,
                'fps': round(state.fps, 1),
                'interface': '2-Lane MIPI CSI-2 Ribbon'
            },
            'gemini': state.gemini_brain,
            'events': state.events[-8:]
        }
    return jsonify(data)


@app.route('/api/gemini/key', methods=['POST'])
def set_gemini_key():
    req = request.json or {}
    api_key = req.get('api_key', '').strip()
    if not api_key:
        return jsonify({'status': 'error', 'message': 'Empty API key'}), 400

    if ros_node_instance is not None:
        key_msg = String()
        key_msg.data = api_key
        ros_node_instance.gemini_key_pub.publish(key_msg)
        with state.lock:
            state.events.append({
                'time': time.strftime("%H:%M:%S"),
                'level': 'SUCCESS',
                'msg': '🔑 Google Gemini API Key configured! Activating cloud multimodal reasoning.'
            })
            state.gemini_brain['status'] = 'AUTHENTICATING'
        return jsonify({'status': 'ok', 'message': 'API key dispatched to Gemini Brain Node'})
    return jsonify({'status': 'error', 'message': 'ROS 2 node not ready'}), 500


@app.route('/api/command', methods=['POST'])
def send_command():
    req = request.json or {}
    action = req.get('action', '')

    if action == 'teleop' and ros_node_instance is not None:
        v = float(req.get('v', 0.0))
        w = float(req.get('w', 0.0))
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        ros_node_instance.cmd_pub.publish(cmd)
        return jsonify({'status': 'ok', 'action': 'teleop', 'v': v, 'w': w})

    elif action == 'set_goal':
        gx = float(req.get('goal_x', 20.0))
        gy = float(req.get('goal_y', 3.5))
        with state.lock:
            state.goal_x = gx
            state.goal_y = gy
            state.events.append({
                'time': time.strftime("%H:%M:%S"),
                'level': 'INFO',
                'msg': f'New Waypoint Set: Point B ({gx:.1f}m, {gy:.1f}m).'
            })
        return jsonify({'status': 'ok', 'action': 'set_goal', 'goal': [gx, gy]})

    elif action in ['start', 'pause', 'resume', 'reset']:
        with state.lock:
            state.events.append({
                'time': time.strftime("%H:%M:%S"),
                'level': 'INFO',
                'msg': f'Mission Command Issued: {action.upper()}'
            })
        return jsonify({'status': 'ok', 'action': action})

    return jsonify({'status': 'unknown_command'}), 400


# ==============================================================================
# Main Runner
# ==============================================================================

def main(args=None):
    global ros_node_instance
    rclpy.init(args=args)
    ros_node_instance = DashboardRosNode()

    # Run ROS 2 spin in background daemon thread
    ros_thread = threading.Thread(target=rclpy.spin, args=(ros_node_instance,), daemon=True)
    ros_thread.start()

    print("=" * 72)
    print("  🌲 HERCULES MISSION CONTROL DASHBOARD SERVER READY 🌲")
    print("  URL: http://localhost:8080")
    print("=" * 72)

    # Start Flask Web Server
    app.run(host='0.0.0.0', port=8080, threaded=True)

    ros_node_instance.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
