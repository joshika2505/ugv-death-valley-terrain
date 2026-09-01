#!/usr/bin/env python3
"""
Locality to Hospital Autonomous Mission Coordinator with Obstacle Avoidance Routing.
Coordinates Multi-Waypoint Navigation from Point A (0,0) to Hospital Point B (20,8) around roadblocks.
"""

import time
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus


class LocalityMissionCoordinator(Node):
    def __init__(self):
        super().__init__('locality_mission_node')
        self.get_logger().info('===========================================================')
        self.get_logger().info('🏥 AUTONOMOUS LOCALITY OBSTACLE AVOIDANCE MISSION ACTIVE 🏥')
        self.get_logger().info('Route: Point A (0,0) -> Crossroad Detour -> Hospital Point B (20,8)')
        self.get_logger().info('Brain: Gemini Multimodal VLM + Nav2 Regulated Pure Pursuit')
        self.get_logger().info('===========================================================')

        # Outbound Waypoints (A -> Hospital B via Crossroad Detour)
        self.outbound_waypoints = [
            ('MAIN_ROAD_APPROACH', 11.5, 0.0),
            ('NORTH_CROSSROAD_DETOUR', 11.8, 6.0),
            ('HOSPITAL_DRIVEWAY', 15.5, 8.0),
            ('HOSPITAL_POINT_B', 18.0, 8.0)
        ]

        # Inbound Waypoints (Hospital B -> A via Crossroad Detour)
        self.inbound_waypoints = [
            ('HOSPITAL_DRIVEWAY_RETURN', 15.5, 8.0),
            ('NORTH_CROSSROAD_RETURN', 11.8, 5.5),
            ('MAIN_ROAD_RETURN', 11.0, 0.0),
            ('START_POINT_A', 0.0, 0.0)
        ]

        self.current_patrol_direction = 'OUTBOUND'  # 'OUTBOUND' or 'INBOUND'
        self.current_wp_idx = 0
        self.target_name = self.outbound_waypoints[0][0]
        self.target_x = self.outbound_waypoints[0][1]
        self.target_y = self.outbound_waypoints[0][2]

        # State Variables
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.dist_to_goal = 25.0
        self.mission_state = 'INITIALIZING'
        self.action_goal_sent = False
        self.goal_handle = None
        self.pause_until = 0.0

        # Subscriptions
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # Action Client for Nav2
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Local Fallback Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/mission/status', 10)

        # 5 Hz Mission Loop
        self.timer = self.create_timer(0.20, self.mission_loop)

    def odom_callback(self, msg: Odometry):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        self.current_x = pos.x
        self.current_y = pos.y
        siny = 2 * (ori.w * ori.z + ori.x * ori.y)
        cosy = 1 - 2 * (ori.y * ori.y + ori.z * ori.z)
        self.current_yaw = math.atan2(siny, cosy)
        self.dist_to_goal = math.hypot(self.target_x - self.current_x, self.target_y - self.current_y)

    def mission_loop(self):
        now = time.time()
        if now < self.pause_until:
            return

        current_list = self.outbound_waypoints if self.current_patrol_direction == 'OUTBOUND' else self.inbound_waypoints

        # Check if arrived at current waypoint
        threshold = 0.70 if self.current_wp_idx == len(current_list) - 1 else 1.10
        if self.dist_to_goal < threshold:
            self.get_logger().info(f'✓ Reached Waypoint: {self.target_name} ({self.target_x:.1f}, {self.target_y:.1f})')
            self.current_wp_idx += 1

            if self.current_wp_idx >= len(current_list):
                # Switch direction
                if self.current_patrol_direction == 'OUTBOUND':
                    self.get_logger().info('🎉 UGV ARRIVED AT HOSPITAL POINT B! Pausing 3s before return leg...')
                    self.current_patrol_direction = 'INBOUND'
                else:
                    self.get_logger().info('✓ UGV ARRIVED AT START POINT A! Pausing 3s before next outbound patrol...')
                    self.current_patrol_direction = 'OUTBOUND'

                self.current_wp_idx = 0
                self.pause_until = now + 3.0

            active_list = self.outbound_waypoints if self.current_patrol_direction == 'OUTBOUND' else self.inbound_waypoints
            self.target_name, self.target_x, self.target_y = active_list[self.current_wp_idx]
            self.action_goal_sent = False
            return

        if not self.action_goal_sent:
            self.send_nav2_goal()

        self.publish_status()

    def send_nav2_goal(self):
        if not self.nav_client.wait_for_server(timeout_sec=1.5):
            self.reactive_guidance_step()
            return

        self.get_logger().info(f'Nav2 Routing toward {self.target_name} ({self.target_x}m, {self.target_y}m)...')
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = self.target_x
        goal_msg.pose.pose.position.y = self.target_y
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0

        self.mission_state = f'TRACKING_{self.target_name}'
        self.action_goal_sent = True

        send_future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.nav_feedback_cb
        )
        send_future.add_done_callback(self.goal_response_cb)

    def goal_response_cb(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().warn('Nav2 Goal rejected. Using reactive guidance...')
            self.action_goal_sent = False
            return
        res_future = self.goal_handle.get_result_async()
        res_future.add_done_callback(self.goal_result_cb)

    def nav_feedback_cb(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Nav2 Tracking {self.target_name}: Rem Dist = {feedback.distance_remaining:.2f}m', throttle_duration_sec=2.0)

    def goal_result_cb(self, future):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.action_goal_sent = False

    def reactive_guidance_step(self):
        target_angle = math.atan2(self.target_y - self.current_y, self.target_x - self.current_x)
        angle_err = math.atan2(math.sin(target_angle - self.current_yaw), math.cos(target_angle - self.current_yaw))

        cmd = Twist()
        cmd.linear.x = 0.35 if abs(angle_err) < 0.4 else 0.15
        cmd.angular.z = float(max(-1.5, min(1.5, 2.0 * angle_err)))
        self.cmd_pub.publish(cmd)

    def publish_status(self):
        msg = String()
        msg.data = f'{self.current_patrol_direction} -> {self.target_name}'
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LocalityMissionCoordinator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
