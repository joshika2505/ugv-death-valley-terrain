#!/usr/bin/env python3
import os
import sys
import json
import math
import time
import threading
from http.server import ThreadingHTTPServer, SimpleHTTPRequestHandler
from ament_index_python.packages import get_package_share_directory
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PointStamped, PoseStamped, PoseWithCovarianceStamped, Twist, Point
from nav_msgs.msg import Odometry, Path, OccupancyGrid
from sensor_msgs.msg import LaserScan, Image
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge

# Import modular autonomy components
try:
    from amr4_autonomy.terrain_analyzer import DeathValleyTerrainAnalyzer
    from amr4_autonomy.terrain_path_planner import TerrainPathPlanner
    from amr4_autonomy.physics_controller import PhysicsBasedPathTracker
except ImportError:
    from terrain_analyzer import DeathValleyTerrainAnalyzer
    from terrain_path_planner import TerrainPathPlanner
    from physics_controller import PhysicsBasedPathTracker

class NavigationManagerNode(Node):
    """
    Master Point A -> Point B Autonomous Physics-Based Navigation Coordinator for AMR-4.
    Integrates terrain raycasting, 3D A* planning, pure pursuit physics tracking,
    dynamic obstacle replanning, RViz marker visualizers, and interactive Web UI server.
    """
    def __init__(self):
        super().__init__('navigation_manager_node')

        # Declare parameters
        self.declare_parameter('web_port', 8080)
        self.declare_parameter('auto_start', False)
        self.declare_parameter('mesh_path', '/home/ubuntu/sih_ws/src/death_valley_world/meshes/death_valley_visual.obj')

        self.web_port = int(self.get_parameter('web_port').value)
        self.auto_start = bool(self.get_parameter('auto_start').value)
        mesh_path = self.get_parameter('mesh_path').value

        # Initialize core modular systems
        self.get_logger().info('Initializing Death Valley 3D Terrain Analyzer...')
        self.terrain_analyzer = DeathValleyTerrainAnalyzer(mesh_path=mesh_path)
        self.path_planner = TerrainPathPlanner(self.terrain_analyzer)
        self.path_tracker = PhysicsBasedPathTracker()
        self.bridge = CvBridge()

        # State Variables
        self.point_a = {'x': 0.0, 'y': 0.0, 'z': self.terrain_analyzer.get_surface_elevation(0.0, 0.0)}
        self.point_b = {'x': 15.0, 'y': 15.0, 'z': self.terrain_analyzer.get_surface_elevation(15.0, 15.0)}
        self.points_set = {'a': True, 'b': True}
        self.selection_mode = 'NONE' # 'SET_A', 'SET_B', 'NONE'
        
        self.path_status = 'Ready' # 'Idle', 'Planning', 'Ready', 'Navigating', 'Replanning', 'Completed'
        self.planned_waypoints = []
        self.robot_pose = (0.0, 0.0, 0.0) # x, y, yaw
        self.robot_speed = 0.0
        self.robot_pitch = 0.0
        self.robot_roll = 0.0
        self.distance_remaining = math.hypot(self.point_b['x'] - self.point_a['x'], self.point_b['y'] - self.point_a['y'])
        self.min_clearance = 10.0
        self.terrain_class_str = 'Safe'
        self.follow_robot_mode = False
        self.latest_camera_jpg = None

        # ROS 2 Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/navigation_markers', 10)
        self.costmap_pub = self.create_publisher(OccupancyGrid, '/traversability_grid', 10)

        # ROS 2 Subscriptions
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.sub_scan = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)
        self.sub_click = self.create_subscription(PointStamped, '/clicked_point', self.clicked_point_callback, 10)
        self.sub_init_pose = self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.initialpose_callback, 10)
        self.sub_goal_pose = self.create_subscription(PoseStamped, '/goal_pose', self.goalpose_callback, 10)
        self.sub_cam = self.create_subscription(Image, '/camera/image_raw', self.camera_callback, 10)

        # Main Navigation Control Loop (20 Hz)
        self.nav_timer = self.create_timer(0.05, self.control_loop)
        
        # Marker & Telemetry Publish Loop (2 Hz)
        self.vis_timer = self.create_timer(0.5, self.publish_visuals)

        # Initial Path Planning
        self.trigger_plan_path()

        # Start Embedded Web Server in Background Thread
        self.start_web_server()

        self.get_logger().info('====================================================')
        self.get_logger().info(' AMR-4 Point A -> Point B Autonomous Navigation Node')
        self.get_logger().info(f' Web Navigation Dashboard: http://localhost:{self.web_port}')
        self.get_logger().info('====================================================')

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        
        # Compute yaw from quaternion
        siny_cosp = 2.0 * (ori.w * ori.z + ori.x * ori.y)
        cosy_cosp = 1.0 - 2.0 * (ori.y * ori.y + ori.z * ori.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        # Compute pitch and roll
        sinr_cosp = 2.0 * (ori.w * ori.x + ori.y * ori.z)
        cosr_cosp = 1.0 - 2.0 * (ori.x * ori.x + ori.y * ori.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        sinp = 2.0 * (ori.w * ori.y - ori.z * ori.x)
        pitch = math.asin(np.clip(sinp, -1.0, 1.0))

        self.robot_pose = (pos.x, pos.y, yaw)
        self.robot_pitch = math.degrees(pitch)
        self.robot_roll = math.degrees(roll)

        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.robot_speed = math.hypot(vx, vy)
        self.path_tracker.current_speed = self.robot_speed

        # Query terrain properties under robot
        _, slope, _, t_class, _ = self.terrain_analyzer.get_terrain_properties(pos.x, pos.y)
        t_names = ['Safe', 'Difficult', 'High-Slope', 'Critical']
        self.terrain_class_str = t_names[min(t_class, 3)]

    def scan_callback(self, msg):
        valid = [r for r in msg.ranges if msg.range_min < r < msg.range_max]
        if valid:
            self.min_clearance = min(valid)

        # Dynamic Obstacle Perception & Smooth Detour Trigger
        num_readings = len(msg.ranges)
        if num_readings > 0:
            angle_min = msg.angle_min
            angle_inc = msg.angle_increment
            forward_obstacles = []
            
            for i, r in enumerate(msg.ranges):
                if 0.55 < r < 1.4: # Obstacle in path range
                    angle = angle_min + i * angle_inc
                    if abs(angle) < math.radians(45.0): # Forward cone
                        forward_obstacles.append((r, angle))

            if len(forward_obstacles) > 4:
                # Find closest obstacle distance and angle
                min_r, min_ang = min(forward_obstacles, key=lambda x: x[0])
                rx, ry, ryaw = self.robot_pose
                obs_world_x = rx + min_r * math.cos(ryaw + min_ang)
                obs_world_y = ry + min_r * math.sin(ryaw + min_ang)

                # Register obstacle into A* planner
                self.path_planner.add_obstacle(obs_world_x, obs_world_y, radius=0.9)

                now_t = time.time()
                if self.path_status == 'Navigating' and (not hasattr(self, 'last_replan_time') or (now_t - self.last_replan_time > 2.5)):
                    self.last_replan_time = now_t
                    self.get_logger().warn(f'[ObstaclePerception] Obstacle at {min_r:.2f}m ({obs_world_x:.1f}, {obs_world_y:.1f})! Finding shortest detour...')
                    self.path_status = 'Replanning'
                    self.trigger_plan_path(start_from_current=True)
                    self.path_status = 'Navigating'

    def camera_callback(self, msg):
        try:
            import cv2
            if msg.encoding in ['rgb8', '8UC3']:
                cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            elif msg.encoding == 'bgr8':
                cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            else:
                cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
                if len(cv_img.shape) == 2:
                    cv_img = cv2.cvtColor(cv_img, cv2.COLOR_GRAY2BGR)

            h, w, _ = cv_img.shape

            # Visual HUD Overlay
            # 1. Horizon & Center Reticle
            cv2.line(cv_img, (w//2 - 25, h//2), (w//2 + 25, h//2), (0, 255, 136), 1)
            cv2.line(cv_img, (w//2, h//2 - 25), (w//2, h//2 + 25), (0, 255, 136), 1)

            # 2. Header Telemetry Banner
            status_txt = f"MODE: {self.path_status.upper()} | SPD: {self.robot_speed:.2f} m/s | REMAIN: {self.distance_remaining:.1f}m"
            cv2.rectangle(cv_img, (10, 10), (w - 10, 42), (15, 20, 25), -1)
            cv2.rectangle(cv_img, (10, 10), (w - 10, 42), (0, 210, 255), 1)
            cv2.putText(cv_img, status_txt, (20, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (0, 210, 255), 2)

            # 3. Bottom Clearance & Terrain Banner
            clr_color = (0, 255, 136) if self.min_clearance > 2.0 else ((0, 180, 255) if self.min_clearance > 1.0 else (0, 50, 255))
            terr_txt = f"TERRAIN: {self.terrain_class_str.upper()} | CLEARANCE: {self.min_clearance:.2f}m"
            cv2.rectangle(cv_img, (10, h - 45), (w - 10, h - 12), (15, 20, 25), -1)
            cv2.rectangle(cv_img, (10, h - 45), (w - 10, h - 12), clr_color, 1)
            cv2.putText(cv_img, terr_txt, (20, h - 22), cv2.FONT_HERSHEY_SIMPLEX, 0.52, clr_color, 2)

            _, encoded = cv2.imencode('.jpg', cv_img, [cv2.IMWRITE_JPEG_QUALITY, 80])
            self.latest_camera_jpg = encoded.tobytes()
        except Exception as e:
            pass

    def clicked_point_callback(self, msg):
        """Handles RViz /clicked_point interaction."""
        x, y = msg.point.x, msg.point.y
        z = self.terrain_analyzer.get_surface_elevation(x, y)
        if self.selection_mode == 'SET_A' or not self.points_set['a']:
            self.set_point_a(x, y, z)
            self.selection_mode = 'SET_B'
        elif self.selection_mode == 'SET_B' or not self.points_set['b']:
            self.set_point_b(x, y, z)
            self.selection_mode = 'NONE'
            self.trigger_plan_path(start_from_current=True)

    def initialpose_callback(self, msg):
        p = msg.pose.pose.position
        z = self.terrain_analyzer.get_surface_elevation(p.x, p.y)
        self.set_point_a(p.x, p.y, z)

    def goalpose_callback(self, msg):
        p = msg.pose.position
        z = self.terrain_analyzer.get_surface_elevation(p.x, p.y)
        self.set_point_b(p.x, p.y, z)
        self.trigger_plan_path(start_from_current=True)

    def set_point_a(self, x, y, z=None):
        if z is None:
            z = self.terrain_analyzer.get_surface_elevation(x, y)
        self.point_a = {'x': float(x), 'y': float(y), 'z': float(z)}
        self.points_set['a'] = True
        self.get_logger().info(f'[PointSelector] Point A (Start) set to: ({x:.2f}, {y:.2f}, {z:.2f})')

    def set_point_b(self, x, y, z=None):
        if z is None:
            z = self.terrain_analyzer.get_surface_elevation(x, y)
        self.point_b = {'x': float(x), 'y': float(y), 'z': float(z)}
        self.points_set['b'] = True
        dist = math.hypot(self.point_b['x'] - self.point_a['x'], self.point_b['y'] - self.point_a['y'])
        self.get_logger().info(f'[PointSelector] Point B (Destination) set to: ({x:.2f}, {y:.2f}, {z:.2f}) | Dist: {dist:.2f}m')

    def trigger_plan_path(self, start_from_current=False):
        """Runs the 3D Terrain A* Path Planner."""
        if not self.points_set['b']:
            return

        self.path_status = 'Planning'
        rx, ry = self.robot_pose[0], self.robot_pose[1]
        if start_from_current and abs(rx) < 48.0 and abs(ry) < 48.0 and math.hypot(rx, ry) > 0.5:
            start_coord = (rx, ry)
        else:
            start_coord = (self.point_a['x'], self.point_a['y'])
        goal_coord = (self.point_b['x'], self.point_b['y'])

        self.get_logger().info(f'[PathPlanner] Planning 3D terrain route from {start_coord} to {goal_coord}...')
        self.planned_waypoints = self.path_planner.plan_path(start_coord, goal_coord)
        self.path_status = 'Ready'
        self.get_logger().info(f'[PathPlanner] Route ready with {len(self.planned_waypoints)} safe 3D contour waypoints.')

    def start_navigation(self):
        if len(self.planned_waypoints) == 0:
            self.trigger_plan_path(start_from_current=True)
        if len(self.planned_waypoints) > 0:
            self.path_status = 'Terrain_Scanning'
            self.scan_start_time = time.time()
            self.get_logger().info('[Observation] Performing 3D terrain & environmental sector scan...')

    def stop_navigation(self):
        self.path_status = 'Ready'
        stop_cmd = Twist()
        self.cmd_pub.publish(stop_cmd)
        self.get_logger().info('[Navigation] Robot stopped.')

    def reset_navigation(self):
        self.path_status = 'Idle'
        self.planned_waypoints = []
        self.path_planner.clear_memory()
        stop_cmd = Twist()
        self.cmd_pub.publish(stop_cmd)
        self.get_logger().info('[Navigation] Navigation state reset.')

    def control_loop(self):
        """20 Hz Pure Pursuit Physics Control Loop."""
        now_sec = time.time()

        # 1. Terrain Scanning Observation Phase
        if self.path_status == 'Terrain_Scanning':
            if now_sec - getattr(self, 'scan_start_time', now_sec) > 0.8:
                self.path_status = 'Navigating'
                self.get_logger().info('[Navigation] Autonomous physics navigation started!')
            else:
                stop_cmd = Twist()
                self.cmd_pub.publish(stop_cmd)
                return

        # 2. Autonomous Reverse-and-Replan on Climb Failure
        if self.path_status == 'Reversing':
            if now_sec - getattr(self, 'reverse_start_time', now_sec) < 2.0:
                bk = Twist()
                bk.linear.x = -0.30
                self.cmd_pub.publish(bk)
                return
            else:
                self.get_logger().info('[ClimbRecovery] Reversal to safe terrain complete. Replanning alternative valley bypass...')
                self.path_status = 'Replanning'
                self.planned_waypoints = self.path_planner.plan_gentle_valley_path(
                    (self.robot_pose[0], self.robot_pose[1]),
                    (self.point_b['x'], self.point_b['y'])
                )
                self.path_status = 'Navigating'
                return

        if self.path_status != 'Navigating' or len(self.planned_waypoints) == 0:
            return

        cmd_v, cmd_w, arrived, dist_rem, stab_status = self.path_tracker.compute_control(
            self.robot_pose, (self.robot_pitch, self.robot_roll), self.planned_waypoints, now_sec
        )
        self.distance_remaining = dist_rem
        self.stability_status = stab_status

        if stab_status == 'CRITICAL_FLIPPED':
            self.get_logger().warn('[SafetyReflex] CRITICAL: Rollover detected! Activating flip recovery bursts...', throttle_duration_sec=1.0)
        elif stab_status == 'ANTI_TIP_ACTIVE':
            self.get_logger().warn('[SafetyReflex] Steep side-slope detected! Engaging anti-tip stabilization...', throttle_duration_sec=1.5)
        elif stab_status == 'RIDGE_UNCLIMBABLE':
            self.get_logger().warn('[ClimbFailure] Unscalable slope encountered! Initiating reverse-and-replan reflex...')
            self.path_status = 'Reversing'
            self.reverse_start_time = now_sec
            # Add to terrain failure memory
            self.path_planner.add_failed_climb_region(self.robot_pose[0], self.robot_pose[1], radius=3.5, penalty=350.0)
            bk = Twist()
            bk.linear.x = -0.30
            self.cmd_pub.publish(bk)
            return

        if arrived:
            self.path_status = 'Completed'
            stop_cmd = Twist()
            self.cmd_pub.publish(stop_cmd)
            self.get_logger().info('====================================================')
            self.get_logger().info(' DESTINATION REACHED: AMR-4 Arrived at Point B!')
            self.get_logger().info(f' Final Position: ({self.robot_pose[0]:.2f}, {self.robot_pose[1]:.2f})')
            self.get_logger().info('====================================================')
            return

        # Publish physics command to robot wheels
        twist = Twist()
        twist.linear.x = float(cmd_v)
        twist.angular.z = float(cmd_w)
        self.cmd_pub.publish(twist)

    def publish_visuals(self):
        """Publishes 3D RViz markers for Point A, Point B, planned path, and traversability."""
        markers = MarkerArray()

        # 1. Point A Marker (Green 3D Cylinder / Pin + Text)
        if self.points_set['a']:
            ma = Marker()
            ma.header.frame_id = 'map'
            ma.header.stamp = self.get_clock().now().to_msg()
            ma.ns = 'point_a'
            ma.id = 0
            ma.type = Marker.CYLINDER
            ma.action = Marker.ADD
            ma.pose.position.x = self.point_a['x']
            ma.pose.position.y = self.point_a['y']
            ma.pose.position.z = self.point_a['z'] + 1.0
            ma.scale.x = 1.0
            ma.scale.y = 1.0
            ma.scale.z = 2.0
            ma.color.r = 0.0
            ma.color.g = 1.0
            ma.color.b = 0.0
            ma.color.a = 0.85
            markers.markers.append(ma)

            # Text label
            mat = Marker()
            mat.header.frame_id = 'map'
            mat.header.stamp = self.get_clock().now().to_msg()
            mat.ns = 'point_a_text'
            mat.id = 1
            mat.type = Marker.TEXT_VIEW_FACING
            mat.action = Marker.ADD
            mat.pose.position.x = self.point_a['x']
            mat.pose.position.y = self.point_a['y']
            mat.pose.position.z = self.point_a['z'] + 2.5
            mat.scale.z = 0.9
            mat.text = f'START A ({self.point_a["x"]:.1f}, {self.point_a["y"]:.1f}, {self.point_a["z"]:.1f}m)'
            mat.color.r = 0.2
            mat.color.g = 1.0
            mat.color.b = 0.2
            mat.color.a = 1.0
            markers.markers.append(mat)

        # 2. Point B Marker (Red 3D Cylinder / Pin + Text)
        if self.points_set['b']:
            mb = Marker()
            mb.header.frame_id = 'map'
            mb.header.stamp = self.get_clock().now().to_msg()
            mb.ns = 'point_b'
            mb.id = 2
            mb.type = Marker.CYLINDER
            mb.action = Marker.ADD
            mb.pose.position.x = self.point_b['x']
            mb.pose.position.y = self.point_b['y']
            mb.pose.position.z = self.point_b['z'] + 1.0
            mb.scale.x = 1.0
            mb.scale.y = 1.0
            mb.scale.z = 2.0
            mb.color.r = 1.0
            mb.color.g = 0.0
            mb.color.b = 0.0
            mb.color.a = 0.85
            markers.markers.append(mb)

            # Text label
            mbt = Marker()
            mbt.header.frame_id = 'map'
            mbt.header.stamp = self.get_clock().now().to_msg()
            mbt.ns = 'point_b_text'
            mbt.id = 3
            mbt.type = Marker.TEXT_VIEW_FACING
            mbt.action = Marker.ADD
            mbt.pose.position.x = self.point_b['x']
            mbt.pose.position.y = self.point_b['y']
            mbt.pose.position.z = self.point_b['z'] + 2.5
            mbt.scale.z = 0.9
            mbt.text = f'DESTINATION B ({self.point_b["x"]:.1f}, {self.point_b["y"]:.1f}, {self.point_b["z"]:.1f}m)'
            mbt.color.r = 1.0
            mbt.color.g = 0.2
            mbt.color.b = 0.2
            mbt.color.a = 1.0
            markers.markers.append(mbt)

        # 3. 3D Planned Path Line Strip
        if len(self.planned_waypoints) > 0:
            mp = Marker()
            mp.header.frame_id = 'map'
            mp.header.stamp = self.get_clock().now().to_msg()
            mp.ns = 'planned_route'
            mp.id = 4
            mp.type = Marker.LINE_STRIP
            mp.action = Marker.ADD
            mp.scale.x = 0.35 # Line width
            mp.color.r = 0.0
            mp.color.g = 0.9
            mp.color.b = 0.3
            mp.color.a = 0.95
            
            ros_path = Path()
            ros_path.header.frame_id = 'map'
            ros_path.header.stamp = mp.header.stamp

            for w in self.planned_waypoints:
                p = Point()
                p.x = w['x']
                p.y = w['y']
                p.z = w['z'] + 0.15 # Slightly elevated above surface
                mp.points.append(p)

                ps = PoseStamped()
                ps.header.frame_id = 'map'
                ps.header.stamp = mp.header.stamp
                ps.pose.position.x = w['x']
                ps.pose.position.y = w['y']
                ps.pose.position.z = w['z']
                ros_path.poses.append(ps)

            markers.markers.append(mp)
            self.path_pub.publish(ros_path)

        self.marker_pub.publish(markers)

    def start_web_server(self):
        """Runs asynchronous HTTP REST server for Web Navigation UI."""
        node_ref = self
        try:
            web_dir = os.path.join(get_package_share_directory('amr4_autonomy'), 'web')
        except Exception:
            web_dir = '/home/ubuntu/sih_ws/src/amr4_autonomy/web'
        if not os.path.exists(web_dir):
            web_dir = '/home/joshika/Desktop/SIH/src/amr4_autonomy/web'

        class WebRequestHandler(SimpleHTTPRequestHandler):
            def __init__(self, *args, **kwargs):
                super().__init__(*args, directory=web_dir, **kwargs)

            def do_GET(self):
                if self.path == '/api/status':
                    self.send_response(200)
                    self.send_header('Content-Type', 'application/json')
                    self.send_header('Access-Control-Allow-Origin', '*')
                    self.end_headers()
                    status_data = {
                        'start_a': node_ref.point_a,
                        'destination_b': node_ref.point_b,
                        'distance_ab': round(math.hypot(node_ref.point_b['x'] - node_ref.point_a['x'], node_ref.point_b['y'] - node_ref.point_a['y']), 2),
                        'distance_remaining': round(node_ref.distance_remaining, 2),
                        'path_status': node_ref.path_status,
                        'robot_speed': round(node_ref.robot_speed, 2),
                        'robot_pose': {'x': round(node_ref.robot_pose[0], 2), 'y': round(node_ref.robot_pose[1], 2), 'yaw': round(node_ref.robot_pose[2], 2)},
                        'robot_pitch': round(node_ref.robot_pitch, 1),
                        'robot_roll': round(node_ref.robot_roll, 1),
                        'min_clearance': round(node_ref.min_clearance, 2),
                        'terrain_class': node_ref.terrain_class_str,
                        'stability_status': getattr(node_ref, 'stability_status', 'NORMAL'),
                        'waypoints_count': len(node_ref.planned_waypoints),
                        'waypoints': node_ref.planned_waypoints[::2] # Decimated for light payload
                    }
                    self.wfile.write(json.dumps(status_data).encode('utf-8'))
                elif self.path == '/api/camera_stream':
                    self.send_response(200)
                    self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
                    self.send_header('Access-Control-Allow-Origin', '*')
                    self.end_headers()
                    try:
                        while True:
                            if node_ref.latest_camera_jpg:
                                self.wfile.write(b"--frame\r\n")
                                self.wfile.write(b"Content-Type: image/jpeg\r\n\r\n")
                                self.wfile.write(node_ref.latest_camera_jpg)
                                self.wfile.write(b"\r\n")
                            time.sleep(0.04) # ~25 FPS
                    except Exception:
                        pass
                elif self.path.startswith('/api/camera_frame'):
                    if node_ref.latest_camera_jpg:
                        self.send_response(200)
                        self.send_header('Content-Type', 'image/jpeg')
                        self.send_header('Access-Control-Allow-Origin', '*')
                        self.end_headers()
                        self.wfile.write(node_ref.latest_camera_jpg)
                    else:
                        self.send_response(404)
                        self.end_headers()
                else:
                    super().do_GET()

            def do_POST(self):
                content_len = int(self.headers.get('Content-Length', 0))
                post_body = self.rfile.read(content_len) if content_len > 0 else b'{}'
                try:
                    data = json.loads(post_body.decode('utf-8'))
                except Exception:
                    data = {}

                if self.path == '/api/set_point_a':
                    node_ref.set_point_a(data.get('x', 0.0), data.get('y', 0.0))
                    res = {'status': 'success', 'point_a': node_ref.point_a}
                elif self.path == '/api/set_point_b':
                    node_ref.set_point_b(data.get('x', 15.0), data.get('y', 15.0))
                    res = {'status': 'success', 'point_b': node_ref.point_b}
                elif self.path == '/api/plan_path':
                    node_ref.trigger_plan_path(start_from_current=False)
                    res = {'status': 'success', 'waypoints': len(node_ref.planned_waypoints)}
                elif self.path == '/api/plan_alternative_path':
                    node_ref.get_logger().info('[PathPlanner] Planning alternative gentle valley pass...')
                    node_ref.path_status = 'Planning'
                    node_ref.planned_waypoints = node_ref.path_planner.plan_gentle_valley_path(
                        (node_ref.point_a['x'], node_ref.point_a['y']),
                        (node_ref.point_b['x'], node_ref.point_b['y'])
                    )
                    node_ref.path_status = 'Ready'
                    res = {'status': 'success', 'waypoints': len(node_ref.planned_waypoints)}
                elif self.path == '/api/start_navigation':
                    node_ref.start_navigation()
                    res = {'status': 'success', 'state': node_ref.path_status}
                elif self.path == '/api/stop':
                    node_ref.stop_navigation()
                    res = {'status': 'success', 'state': node_ref.path_status}
                elif self.path == '/api/reset':
                    node_ref.reset_navigation()
                    res = {'status': 'success', 'state': node_ref.path_status}
                elif self.path == '/api/recover_robot':
                    # Apply emergency pulse burst to self-right
                    node_ref.stability_status = 'RECOVERED'
                    burst = Twist()
                    burst.linear.x = 0.5
                    burst.angular.z = 1.5
                    node_ref.cmd_pub.publish(burst)
                    res = {'status': 'success', 'message': 'Flip recovery maneuver executed'}
                else:
                    res = {'status': 'unknown_endpoint'}

                self.send_response(200)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(json.dumps(res).encode('utf-8'))

        def run_server():
            try:
                server = ThreadingHTTPServer(('0.0.0.0', self.web_port), WebRequestHandler)
                server.serve_forever()
            except Exception as e:
                self.get_logger().warn(f'Web server warning: {e}')

        t = threading.Thread(target=run_server, daemon=True)
        t.start()

def main(args=None):
    rclpy.init(args=args)
    node = NavigationManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
