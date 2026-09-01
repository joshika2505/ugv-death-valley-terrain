#!/usr/bin/env python3
"""
Autonomous Mission Navigator & Dynamic Collision Avoidance Controller for Outdoor UGV.
SIH GPS-Denied Outdoor Autonomous Navigation.

Features:
- Full GPS-denied autonomous mission execution from Point A (Start) to Point B (Goal Beacon)
- Multi-sensor fusion: AI Path Segmentation Guidance + 360 LiDAR Dynamic Obstacle Avoidance + Visual Beacon Homing
- Reactive Vector Field / Dynamic Window obstacle avoidance around rocks, boulders, and ditches
- Visual RViz2 telemetry marker array generation (trail, target corridor, obstacle vectors)
- Precision terminal docking at Goal Point B
"""

import time
import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import Twist, Point, PoseStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, Bool, String, Header
from visualization_msgs.msg import Marker, MarkerArray


class AutonomousNavigator(Node):
    # Navigation States
    STATE_IDLE = 'IDLE'
    STATE_FOLLOW_PATH = 'FOLLOW_PATH'
    STATE_AVOID_OBSTACLE = 'AVOID_OBSTACLE'
    STATE_HOMING_BEACON = 'HOMING_BEACON'
    STATE_GOAL_REACHED = 'GOAL_REACHED'

    def __init__(self):
        super().__init__('autonomous_navigator')
        self.get_logger().info('Initializing Autonomous Mission Navigator...')

        # Mission Waypoints (GPS-Denied local coordinate frame)
        self.start_pos = (0.0, 0.0)
        self.goal_pos = (18.0, 4.2)
        self.coarse_waypoints = [
            (3.0, 0.0),
            (6.0, 0.2),
            (9.0, 1.2),
            (12.0, 2.2),
            (15.0, 3.2),
            (18.0, 4.2)
        ]
        self.current_wp_idx = 0

        # Robot Pose State
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.pose_initialized = False

        # Perception & Sensor Inputs
        self.visual_path_offset = 0.0
        self.last_path_offset_time = 0.0

        self.beacon_detected = False
        self.beacon_bearing = 0.0
        self.beacon_range = 999.0
        self.last_beacon_time = 0.0

        self.min_front_dist = 999.0
        self.min_left_dist = 999.0
        self.min_right_dist = 999.0

        # Controller Parameters
        self.max_linear_speed = 0.75     # m/s
        self.avoid_linear_speed = 0.38   # m/s
        self.max_angular_speed = 1.2     # rad/s
        self.obstacle_stop_dist = 0.65   # m
        self.obstacle_avoid_dist = 1.65  # m
        self.goal_reach_threshold = 0.75 # m

        # State Machine
        self.state = self.STATE_FOLLOW_PATH
        self.mission_start_time = time.time()
        self.distance_traveled = 0.0
        self.last_pos = None

        # Subscriptions
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        self.offset_sub = self.create_subscription(
            Float32,
            '/sih_ugv/perception/path_center_offset',
            self.path_offset_callback,
            10
        )
        self.beacon_det_sub = self.create_subscription(
            Bool,
            '/sih_ugv/beacon/detected',
            self.beacon_detected_callback,
            10
        )
        self.beacon_bearing_sub = self.create_subscription(
            Float32,
            '/sih_ugv/beacon/bearing',
            self.beacon_bearing_callback,
            10
        )
        self.beacon_range_sub = self.create_subscription(
            Float32,
            '/sih_ugv/beacon/range',
            self.beacon_range_callback,
            10
        )

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/sih_ugv/navigation/status', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/sih_ugv/navigation/markers', 10)

        # Control Loop Timer (20 Hz)
        self.control_timer = self.create_timer(0.05, self.control_loop)
        self.telemetry_timer = self.create_timer(0.2, self.publish_telemetry_and_markers)

        # Trajectory History for RViz
        self.trajectory_points = []

        self.get_logger().info('Autonomous Navigator initialized. Starting mission toward Point B...')

    def odom_callback(self, msg: Odometry):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation

        # Quaternion to Yaw
        siny_cosp = 2 * (ori.w * ori.z + ori.x * ori.y)
        cosy_cosp = 1 - 2 * (ori.y * ori.y + ori.z * ori.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        if self.last_pos is not None:
            step_dist = math.hypot(pos.x - self.last_pos[0], pos.y - self.last_pos[1])
            self.distance_traveled += step_dist
        self.last_pos = (pos.x, pos.y)

        self.current_x = pos.x
        self.current_y = pos.y
        self.current_yaw = yaw
        self.pose_initialized = True

        if len(self.trajectory_points) == 0 or math.hypot(pos.x - self.trajectory_points[-1][0], pos.y - self.trajectory_points[-1][1]) > 0.2:
            self.trajectory_points.append((pos.x, pos.y, 0.05))

    def scan_callback(self, msg: LaserScan):
        ranges = np.array(msg.ranges)
        num_samples = len(ranges)
        if num_samples == 0:
            return

        angles = msg.angle_min + np.arange(num_samples) * msg.angle_increment
        valid = (ranges >= msg.range_min) & (ranges <= msg.range_max) & (~np.isnan(ranges))

        # Split into sectors (Front: -25 to +25 deg, Left: +25 to +75 deg, Right: -75 to -25 deg)
        front_mask = valid & (angles >= -math.radians(25)) & (angles <= math.radians(25))
        left_mask = valid & (angles > math.radians(25)) & (angles <= math.radians(75))
        right_mask = valid & (angles >= -math.radians(75)) & (angles < -math.radians(25))

        self.min_front_dist = float(np.min(ranges[front_mask])) if np.any(front_mask) else 999.0
        self.min_left_dist = float(np.min(ranges[left_mask])) if np.any(left_mask) else 999.0
        self.min_right_dist = float(np.min(ranges[right_mask])) if np.any(right_mask) else 999.0

    def path_offset_callback(self, msg: Float32):
        self.visual_path_offset = float(msg.data)
        self.last_path_offset_time = time.time()

    def beacon_detected_callback(self, msg: Bool):
        self.beacon_detected = bool(msg.data)

    def beacon_bearing_callback(self, msg: Float32):
        self.beacon_bearing = float(msg.data)
        self.last_beacon_time = time.time()

    def beacon_range_callback(self, msg: Float32):
        self.beacon_range = float(msg.data)

    def control_loop(self):
        if not self.pose_initialized:
            return

        cmd = Twist()
        dist_to_goal = math.hypot(self.goal_pos[0] - self.current_x, self.goal_pos[1] - self.current_y)

        # ======================================================================
        # State Machine Transitions
        # ======================================================================
        if dist_to_goal <= self.goal_reach_threshold:
            self.state = self.STATE_GOAL_REACHED
        elif self.beacon_detected and dist_to_goal < 6.0:
            self.state = self.STATE_HOMING_BEACON
        elif self.min_front_dist < self.obstacle_avoid_dist:
            self.state = self.STATE_AVOID_OBSTACLE
        else:
            self.state = self.STATE_FOLLOW_PATH

        # ======================================================================
        # State Execution
        # ======================================================================
        if self.state == self.STATE_GOAL_REACHED:
            # Reached Goal Point B - Stop
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        elif self.state == self.STATE_HOMING_BEACON:
            # Visual Servoing / Target Beacon Homing
            # Turn toward beacon bearing and advance smoothly
            ang_err = self.beacon_bearing
            cmd.angular.z = float(np.clip(ang_err * 1.8, -self.max_angular_speed, self.max_angular_speed))
            
            # Decelerate as target is approached
            speed_factor = np.clip(self.beacon_range / 4.0, 0.25, 0.7)
            cmd.linear.x = float(self.max_linear_speed * speed_factor * max(0.2, math.cos(ang_err)))

        elif self.state == self.STATE_AVOID_OBSTACLE:
            # Dynamic Collision Avoidance
            # If too close, brake and turn away
            if self.min_front_dist < self.obstacle_stop_dist:
                cmd.linear.x = 0.05
                # Turn toward whichever side has more clearance
                turn_direction = 1.0 if self.min_left_dist >= self.min_right_dist else -1.0
                cmd.angular.z = float(turn_direction * self.max_angular_speed)
            else:
                # Dynamic routing around obstacle
                # Repulsive potential from obstacle + attractive force to safe path
                clearance_diff = self.min_left_dist - self.min_right_dist
                evasion_bias = np.clip(clearance_diff * 0.8, -1.0, 1.0)
                
                # If front is blocked, prioritize clear side
                if evasion_bias >= 0:
                    turn_steer = 0.85
                else:
                    turn_steer = -0.85

                cmd.linear.x = float(self.avoid_linear_speed)
                cmd.angular.z = float(turn_steer)

        elif self.state == self.STATE_FOLLOW_PATH:
            # Autonomous Path Following
            # 1. Waypoint bearing to stay on the outdoor trail route
            target_wp = self.coarse_waypoints[self.current_wp_idx]
            dist_to_wp = math.hypot(target_wp[0] - self.current_x, target_wp[1] - self.current_y)
            if dist_to_wp < 1.5 and self.current_wp_idx < len(self.coarse_waypoints) - 1:
                self.current_wp_idx += 1
                target_wp = self.coarse_waypoints[self.current_wp_idx]

            desired_yaw = math.atan2(target_wp[1] - self.current_y, target_wp[0] - self.current_x)
            yaw_diff = math.atan2(math.sin(desired_yaw - self.current_yaw), math.cos(desired_yaw - self.current_yaw))

            # 2. Visual AI path centering correction (offset in [-1, 1])
            ai_vision_steer = -self.visual_path_offset * 1.5

            # Combined Steering: Waypoint Course Guidance (45%) + AI Path Traversability (55%)
            combined_steer = 0.45 * (yaw_diff * 1.2) + 0.55 * ai_vision_steer

            # Side clearance adjustment (stay centered between flanking hazards)
            if self.min_left_dist < 0.9:
                combined_steer -= (0.9 - self.min_left_dist) * 0.8
            elif self.min_right_dist < 0.9:
                combined_steer += (0.9 - self.min_right_dist) * 0.8

            cmd.angular.z = float(np.clip(combined_steer, -self.max_angular_speed, self.max_angular_speed))
            
            # Speed scaling based on curvature and obstacles
            curve_slowdown = max(0.4, 1.0 - abs(combined_steer) / 1.5)
            cmd.linear.x = float(self.max_linear_speed * curve_slowdown)

        self.cmd_vel_pub.publish(cmd)

    def publish_telemetry_and_markers(self):
        if not self.pose_initialized:
            return

        dist_to_goal = math.hypot(self.goal_pos[0] - self.current_x, self.goal_pos[1] - self.current_y)
        elapsed_time = time.time() - self.mission_start_time

        # Publish Status String
        status_text = (
            f"State: {self.state} | DistToGoal: {dist_to_goal:.2f}m | "
            f"Traveled: {self.distance_traveled:.2f}m | FrontObs: {self.min_front_dist:.2f}m | "
            f"Time: {elapsed_time:.1f}s | Beacon: {'LOCKED' if self.beacon_detected else 'SEARCHING'}"
        )
        status_msg = String()
        status_msg.data = status_text
        self.status_pub.publish(status_msg)

        # Generate RViz Markers
        marker_array = MarkerArray()
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'odom'

        # 1. Trajectory Line
        traj_marker = Marker()
        traj_marker.header = header
        traj_marker.ns = 'trajectory'
        traj_marker.id = 0
        traj_marker.type = Marker.LINE_STRIP
        traj_marker.action = Marker.ADD
        traj_marker.scale.x = 0.08
        traj_marker.color.r = 0.0
        traj_marker.color.g = 0.9
        traj_marker.color.b = 1.0
        traj_marker.color.a = 0.85
        for pt in self.trajectory_points:
            p = Point()
            p.x, p.y, p.z = pt
            traj_marker.points.append(p)
        marker_array.markers.append(traj_marker)

        # 2. Start Point A Marker (Blue Cylinder)
        start_marker = Marker()
        start_marker.header = header
        start_marker.ns = 'waypoints'
        start_marker.id = 1
        start_marker.type = Marker.CYLINDER
        start_marker.action = Marker.ADD
        start_marker.pose.position.x = self.start_pos[0]
        start_marker.pose.position.y = self.start_pos[1]
        start_marker.pose.position.z = 0.1
        start_marker.scale.x = 1.2
        start_marker.scale.y = 1.2
        start_marker.scale.z = 0.1
        start_marker.color.r = 0.1
        start_marker.color.g = 0.4
        start_marker.color.b = 0.9
        start_marker.color.a = 0.7
        marker_array.markers.append(start_marker)

        # 3. Goal Point B Marker (Green Cylinder with Text)
        goal_marker = Marker()
        goal_marker.header = header
        goal_marker.ns = 'waypoints'
        goal_marker.id = 2
        goal_marker.type = Marker.CYLINDER
        goal_marker.action = Marker.ADD
        goal_marker.pose.position.x = self.goal_pos[0]
        goal_marker.pose.position.y = self.goal_pos[1]
        goal_marker.pose.position.z = 0.1
        goal_marker.scale.x = 1.4
        goal_marker.scale.y = 1.4
        goal_marker.scale.z = 0.1
        goal_marker.color.r = 0.1
        goal_marker.color.g = 0.9
        goal_marker.color.b = 0.2
        goal_marker.color.a = 0.75
        marker_array.markers.append(goal_marker)

        # Goal Label
        text_marker = Marker()
        text_marker.header = header
        text_marker.ns = 'waypoints'
        text_marker.id = 3
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        text_marker.pose.position.x = self.goal_pos[0]
        text_marker.pose.position.y = self.goal_pos[1]
        text_marker.pose.position.z = 1.8
        text_marker.scale.z = 0.6
        text_marker.color.r = 1.0
        text_marker.color.g = 1.0
        text_marker.color.b = 1.0
        text_marker.color.a = 1.0
        text_marker.text = "POINT B (Goal Beacon)"
        marker_array.markers.append(text_marker)

        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    navigator = AutonomousNavigator()
    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        pass
    finally:
        navigator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
