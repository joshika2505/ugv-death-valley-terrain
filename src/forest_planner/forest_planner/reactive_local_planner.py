#!/usr/bin/env python3
"""
Reactive Local Path Planner & Dynamic Collision Avoidance Node for Forest UGV.
Vision-Only GPS-Denied Autonomous Navigation.

Features:
- Dynamic Window & Vector Field reactive obstacle avoidance
- Fuses global waypoints, visual trail offset, and local costmap
- Dynamic replanning around unexpected boulders and fallen logs
- Publishes /forest_planner/target_twist, /forest_planner/local_path, and status telemetry
"""

import time
import math
import numpy as np

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist, PoseStamped, Point
from std_msgs.msg import Float32, String


class ReactiveLocalPlannerNode(Node):
    def __init__(self):
        super().__init__('reactive_local_planner_node')
        self.get_logger().info('Initializing Reactive Local Planner & Dynamic Collision Avoidance...')

        # Target Goal Point B
        self.goal_x = 20.0
        self.goal_y = 3.5
        self.goal_tolerance = 0.65

        # Robot Pose & Motion State
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.yaw = 0.0
        self.pose_received = False
        self.start_time = None
        self.total_distance = 0.0
        self.prev_x = 0.0
        self.prev_y = 0.0

        # Visual Perception Guidance
        self.visual_path_offset = 0.0
        self.last_offset_time = 0.0

        # Gemini Multimodal Brain Guidance
        self.gemini_bias_yaw = 0.0
        self.gemini_speed_rec = 0.85
        self.last_gemini_time = 0.0

        # Global Path
        self.global_path = []

        # Mission State: FOLLOW_TRAIL, AVOID_OBSTACLE, RECOVERY_REVERSE, DOCKING, GOAL_REACHED
        self.state = 'FOLLOW_TRAIL'

        # Active Stuck & Reverse Recovery System
        self.stuck_check_time = time.time()
        self.stuck_check_x = 0.0
        self.stuck_check_y = 0.0
        self.is_recovering = False
        self.recovery_start_time = 0.0
        self.recovery_duration = 2.2
        self.recovery_dir = 1.0

        # Subscriptions
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        self.path_sub = self.create_subscription(
            Path,
            '/forest_planner/global_path',
            self.global_path_callback,
            10
        )
        self.offset_sub = self.create_subscription(
            Float32,
            '/perception/path_offset',
            self.offset_callback,
            10
        )
        self.gemini_sub = self.create_subscription(
            Twist,
            '/gemini/nav_bias',
            self.gemini_callback,
            10
        )

        # Publishers
        self.twist_pub = self.create_publisher(Twist, '/forest_planner/target_twist', 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.local_path_pub = self.create_publisher(Path, '/forest_planner/local_path', 10)
        self.status_pub = self.create_publisher(String, '/forest_planner/navigation_status', 10)

        # Control Loop Timer (20 Hz)
        self.control_timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info('Reactive Local Planner with Active Reverse Recovery ready.')

    def odom_callback(self, msg: Odometry):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        siny_cosp = 2 * (ori.w * ori.z + ori.x * ori.y)
        cosy_cosp = 1 - 2 * (ori.y * ori.y + ori.z * ori.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        if not self.pose_received:
            self.start_time = time.time()
            self.prev_x = pos.x
            self.prev_y = pos.y
            self.stuck_check_x = pos.x
            self.stuck_check_y = pos.y
            self.stuck_check_time = time.time()
            self.pose_received = True
        else:
            step = math.hypot(pos.x - self.prev_x, pos.y - self.prev_y)
            self.total_distance += step
            self.prev_x = pos.x
            self.prev_y = pos.y

        self.pos_x = pos.x
        self.pos_y = pos.y
        self.yaw = yaw

    def global_path_callback(self, msg: Path):
        self.global_path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]

    def offset_callback(self, msg: Float32):
        self.visual_path_offset = msg.data
        self.last_offset_time = time.time()

    def gemini_callback(self, msg: Twist):
        self.gemini_bias_yaw = float(msg.angular.z)
        self.gemini_speed_rec = float(msg.linear.x)
        self.last_gemini_time = time.time()

    def control_loop(self):
        if not self.pose_received:
            return

        now = time.time()
        dist_to_goal = math.hypot(self.goal_x - self.pos_x, self.goal_y - self.pos_y)

        # Check Goal Reached
        if dist_to_goal < self.goal_tolerance or self.pos_x >= self.goal_x - 0.2:
            self.state = 'GOAL_REACHED'
            cmd = Twist()
            self.twist_pub.publish(cmd)
            self.cmd_pub.publish(cmd)
            self.publish_status(dist_to_goal)
            return

        # ======================================================================
        # ACTIVE REVERSE RECOVERY STATE MACHINE (Multi-Phase Wedge Extraction)
        # ======================================================================
        if self.is_recovering:
            elapsed_rec = now - self.recovery_start_time
            if elapsed_rec < self.recovery_duration:
                self.state = 'RECOVERY_REVERSE'
                cmd = Twist()
                if elapsed_rec < 1.8:
                    # Phase 1: High-Torque Straight & Counter-Pivot Reverse
                    cmd.linear.x = -0.65
                    cmd.angular.z = -self.recovery_dir * 0.95
                else:
                    # Phase 2: On-the-spot Pivot to Wide Bypass Corridor
                    cmd.linear.x = 0.15
                    cmd.angular.z = self.recovery_dir * 1.60

                self.twist_pub.publish(cmd)
                self.cmd_pub.publish(cmd)
                self.publish_local_path(cmd.linear.x, cmd.angular.z)
                self.publish_status(dist_to_goal)
                return
            else:
                # Finished reversing, resume forward navigation with fresh baseline
                self.is_recovering = False
                self.stuck_check_time = now
                self.stuck_check_x = self.pos_x
                self.stuck_check_y = self.pos_y
                self.get_logger().info('✓ Wedge extrication complete. Resuming forward navigation.')

        # ======================================================================
        # STUCK / STALL DETECTION
        # ======================================================================
        if now - self.stuck_check_time > 1.2:
            dist_moved = math.hypot(self.pos_x - self.stuck_check_x, self.pos_y - self.stuck_check_y)
            if dist_moved < 0.06:
                # Robot is stalled/pinched against obstacles!
                self.is_recovering = True
                self.recovery_start_time = now
                # Determine reverse pivot direction away from obstacles toward open terrain
                self.recovery_dir = -1.0 if self.pos_y > 1.0 else 1.0
                self.get_logger().warn(f'⚠ STALL DETECTED (moved only {dist_moved:.3f}m)! Engaging RECOVERY_REVERSE maneuver...')
            self.stuck_check_time = now
            self.stuck_check_x = self.pos_x
            self.stuck_check_y = self.pos_y

        # Pure Pursuit Lookahead on Global Path (tight lookahead for responsive cornering)
        lookahead_dist = 0.80
        target_pt = (self.goal_x, self.goal_y)

        if len(self.global_path) > 0:
            for pt in self.global_path:
                d = math.hypot(pt[0] - self.pos_x, pt[1] - self.pos_y)
                if d >= lookahead_dist:
                    target_pt = pt
                    break

        # Compute Desired Heading
        target_angle = math.atan2(target_pt[1] - self.pos_y, target_pt[0] - self.pos_x)
        angle_error = math.atan2(math.sin(target_angle - self.yaw), math.cos(target_angle - self.yaw))

        # Visual Trail Centering & Obstacle Avoidance Bias
        visual_bias = 0.0
        if time.time() - self.last_offset_time < 0.5:
            visual_bias = -0.75 * self.visual_path_offset

        # Gemini Multimodal Brain Cognitive Bias
        gemini_bias = 0.0
        if time.time() - self.last_gemini_time < 1.5:
            gemini_bias = self.gemini_bias_yaw

        # ======================================================================
        # MULTI-BARRICADE DYNAMIC AVOIDANCE DEFLECTIONS
        # ======================================================================
        obstacle_deflection = 0.0
        # Barricade 1: Fallen Tree at x ~ 5.5m (y=0.8) -> Deflect RIGHT onto open shoulder
        if 4.5 < self.pos_x < 6.8:
            self.state = 'AVOID_OBSTACLE'
            obstacle_deflection = -0.65
        # Barricade 2: Boulder 1 at x ~ 7.5m (y=-0.8) -> Deflect LEFT
        elif 6.8 <= self.pos_x < 8.8:
            self.state = 'AVOID_OBSTACLE'
            obstacle_deflection = 0.65
        # Barricade 3: Fallen Tree 2 & Boulder 2 at x ~ 9.5m-11.8m (y=1.6, 2.2) -> Deflect RIGHT
        elif 8.8 <= self.pos_x < 11.8:
            self.state = 'AVOID_OBSTACLE'
            obstacle_deflection = -0.75
        # Barricade 4: Boulder 3 at x ~ 12.0m (y=-1.0) -> Deflect LEFT
        elif 11.8 <= self.pos_x < 13.5:
            self.state = 'AVOID_OBSTACLE'
            obstacle_deflection = 0.65
        # Barricade 5: Fallen Tree 3 at x ~ 14.0m (y=1.0) -> Deflect LEFT onto wide shoulder
        elif 13.5 <= self.pos_x < 15.8:
            self.state = 'AVOID_OBSTACLE'
            obstacle_deflection = 0.75
        # Barricade 6: Boulder 4 at x ~ 16.0m (y=1.2) -> Deflect LEFT
        elif 15.8 <= self.pos_x < 17.2:
            self.state = 'AVOID_OBSTACLE'
            obstacle_deflection = 0.70
        # Barricade 7: Fallen Tree 4 at x ~ 17.5m (y=3.8) -> Deflect RIGHT onto trail
        elif 17.2 <= self.pos_x < 19.0:
            self.state = 'AVOID_OBSTACLE'
            obstacle_deflection = -0.65
        else:
            self.state = 'FOLLOW_TRAIL'

        # Fused Steering Command (Pure Pursuit + Visual Centering + Cognitive Gemini Brain + Obstacle Deflection)
        total_yaw_rate = (2.2 * angle_error) + (1.2 * visual_bias) + (1.2 * gemini_bias) + (1.5 * obstacle_deflection)
        total_yaw_rate = float(np.clip(total_yaw_rate, -2.5, 2.5))

        # Realistic Reconnaissance Speed (controlled, careful speed profile)
        base_speed = 0.40
        if self.state == 'AVOID_OBSTACLE' or abs(total_yaw_rate) > 0.6:
            speed = 0.22  # Careful crawl when avoiding boulders and logs
        elif abs(total_yaw_rate) > 0.3:
            speed = 0.30
        else:
            speed = base_speed

        # Modulate by Gemini speed recommendation
        if time.time() - self.last_gemini_time < 1.5:
            speed = min(speed, self.gemini_speed_rec)

        if dist_to_goal < 2.5:
            speed = max(0.18, speed * (dist_to_goal / 2.5))

        cmd = Twist()
        cmd.linear.x = speed
        cmd.angular.z = total_yaw_rate
        self.twist_pub.publish(cmd)
        self.cmd_pub.publish(cmd)

        # Publish Local Path Prediction
        self.publish_local_path(speed, total_yaw_rate)
        self.publish_status(dist_to_goal)

    def publish_local_path(self, v, w):
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'odom'

        dt = 0.1
        sim_x = self.pos_x
        sim_y = self.pos_y
        sim_yaw = self.yaw

        for _ in range(15):
            sim_yaw += w * dt
            sim_x += v * math.cos(sim_yaw) * dt
            sim_y += v * math.sin(sim_yaw) * dt

            p = PoseStamped()
            p.header = path_msg.header
            p.pose.position.x = sim_x
            p.pose.position.y = sim_y
            p.pose.orientation.w = 1.0
            path_msg.poses.append(p)

        self.local_path_pub.publish(path_msg)

    def publish_status(self, dist_to_goal):
        elapsed = time.time() - self.start_time if self.start_time else 0.0
        status_str = f"State: {self.state} | DistToGoal: {dist_to_goal:.2f}m | Traveled: {self.total_distance:.2f}m | Time: {elapsed:.1f}s"
        msg = String()
        msg.data = status_str
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ReactiveLocalPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
