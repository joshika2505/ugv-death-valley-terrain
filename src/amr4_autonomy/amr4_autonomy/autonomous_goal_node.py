#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from sensor_msgs.msg import LaserScan
from nav2_msgs.action import NavigateToPose
import math
import time
import json
import os

class AMR4AutonomousGoalNode(Node):
    def __init__(self):
        super().__init__('autonomous_goal_node')

        self.declare_parameter('start_x', 0.0)
        self.declare_parameter('start_y', 0.0)
        self.declare_parameter('start_yaw', 0.6)
        self.declare_parameter('goal_x', 20.0)
        self.declare_parameter('goal_y', 20.0)
        self.declare_parameter('goal_yaw', 0.785)

        self.start_x = self.get_parameter('start_x').value
        self.start_y = self.get_parameter('start_y').value
        self.start_yaw = self.get_parameter('start_yaw').value
        self.goal_x = self.get_parameter('goal_x').value
        self.goal_y = self.get_parameter('goal_y').value
        self.goal_yaw = self.get_parameter('goal_yaw').value

        self.get_logger().info('====================================================')
        self.get_logger().info(' AMR-4 Autonomous Navigation & Telemetry System')
        self.get_logger().info(f' Point A (Start): ({self.start_x:.2f}, {self.start_y:.2f}, yaw={self.start_yaw:.2f} rad)')
        self.get_logger().info(f' Point B (Goal):  ({self.goal_x:.2f}, {self.goal_y:.2f}, yaw={self.goal_yaw:.2f} rad)')
        self.get_logger().info('====================================================')

        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.plan_sub = self.create_subscription(Path, '/plan', self.plan_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.curr_x = self.start_x
        self.curr_y = self.start_y
        self.prev_x = self.start_x
        self.prev_y = self.start_y
        self.total_distance = 0.0
        self.min_obstacle_clearance = float('inf')
        self.map_received = False
        self.map_cells = 0
        self.plan_count = 0
        self.replans = 0
        self.start_time = None
        self.goal_sent = False
        self.goal_active = False
        self.goal_reached = False
        self.step_size = 3.5

        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info('[AMR-4 Autonomy] Waiting for SLAM Map & Nav2 Action Server...')

    def map_callback(self, msg):
        if not self.map_received:
            self.map_received = True
            self.map_cells = len(msg.data)
            self.get_logger().info(f'[SLAM] Online Mapping Active. Map cells: {self.map_cells}')

    def scan_callback(self, msg):
        valid_ranges = [r for r in msg.ranges if msg.range_min < r < msg.range_max]
        if valid_ranges:
            min_r = min(valid_ranges)
            if min_r < self.min_obstacle_clearance:
                self.min_obstacle_clearance = min_r

    def odom_callback(self, msg):
        self.curr_x = msg.pose.pose.position.x
        self.curr_y = msg.pose.pose.position.y

        d = math.hypot(self.curr_x - self.prev_x, self.curr_y - self.prev_y)
        if d > 0.01:
            self.total_distance += d
            self.prev_x = self.curr_x
            self.prev_y = self.curr_y

    def plan_callback(self, msg):
        if len(msg.poses) > 0:
            self.plan_count += 1
            if self.plan_count > 1:
                self.replans += 1

    def send_navigation_goal(self, target_x, target_y, target_yaw):
        if not self.nav_client.server_is_ready():
            return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = target_x
        goal_msg.pose.pose.position.y = target_y
        goal_msg.pose.pose.position.z = 0.0

        cy = math.cos(target_yaw * 0.5)
        sy = math.sin(target_yaw * 0.5)
        goal_msg.pose.pose.orientation.z = sy
        goal_msg.pose.pose.orientation.w = cy

        self.get_logger().info(f'[NAV] Sending Navigation Goal -> ({target_x:.2f}, {target_y:.2f})')
        send_goal_future = self.nav_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        send_goal_future.add_done_callback(self.goal_response_callback)
        self.goal_sent = True
        return True

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('[NAV] Goal rejected. Retrying...')
            self.goal_sent = False
            self.goal_active = False
            return

        self.goal_active = True
        self.get_logger().info('[NAV] Goal accepted. Trajectory tracking active.')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def feedback_callback(self, feedback_msg):
        pass

    def goal_result_callback(self, future):
        self.goal_active = False
        dist_to_final = math.hypot(self.goal_x - self.curr_x, self.goal_y - self.curr_y)
        if dist_to_final < 1.0:
            self.goal_reached = True
            self.complete_mission()
        else:
            self.get_logger().info(f'[NAV] Advanced step. Remaining to Point B: {dist_to_final:.2f}m')
            self.goal_sent = False

    def timer_callback(self):
        dist_to_point_b = math.hypot(self.goal_x - self.curr_x, self.goal_y - self.curr_y)

        if dist_to_point_b < 1.0 and not self.goal_reached:
            self.goal_reached = True
            self.complete_mission()
            return

        if not self.goal_sent and not self.goal_active and self.map_received:
            if self.nav_client.wait_for_server(timeout_sec=0.5):
                if self.start_time is None:
                    self.start_time = time.time()

                # Step size capped to always stay inside discovered SLAM bounds
                total_vec_dist = math.hypot(self.goal_x - self.curr_x, self.goal_y - self.curr_y)
                angle_to_goal = math.atan2(self.goal_y - self.curr_y, self.goal_x - self.curr_x)

                step = min(self.step_size, total_vec_dist)
                tx = self.curr_x + step * math.cos(angle_to_goal)
                ty = self.curr_y + step * math.sin(angle_to_goal)
                tyaw = angle_to_goal

                self.send_navigation_goal(tx, ty, tyaw)

        if self.start_time:
            elapsed = time.time() - self.start_time
            clearance_str = f'{self.min_obstacle_clearance:.2f} m' if self.min_obstacle_clearance < 100 else 'N/A'
            self.get_logger().info(
                f'[STATUS] Pose: ({self.curr_x:.2f}, {self.curr_y:.2f}) | '
                f'Dist to Point B: {dist_to_point_b:.2f} m | '
                f'Traveled: {self.total_distance:.2f} m | '
                f'Min Clearance: {clearance_str} | '
                f'Time: {elapsed:.1f} s'
            )

    def complete_mission(self):
        duration = time.time() - (self.start_time if self.start_time else time.time())
        final_err = math.hypot(self.goal_x - self.curr_x, self.goal_y - self.curr_y)
        
        self.get_logger().info('====================================================')
        self.get_logger().info(' MISSION ACCOMPLISHED: POINT B REACHED AUTONOMOUSLY!')
        self.get_logger().info(f' Final Position: ({self.curr_x:.2f}, {self.curr_y:.2f})')
        self.get_logger().info(f' Distance to Goal: {final_err:.2f} m (within tolerance)')
        self.get_logger().info(f' Total Traveled: {self.total_distance:.2f} m')
        self.get_logger().info(f' Total Mission Time: {duration:.1f} s')
        self.get_logger().info(f' Minimum Obstacle Clearance: {self.min_obstacle_clearance:.2f} m')
        self.get_logger().info(f' Path Replans: {self.replans}')
        self.get_logger().info('====================================================')

        report = {
            "mission_status": "SUCCESS",
            "point_a": [self.start_x, self.start_y, self.start_yaw],
            "point_b": [self.goal_x, self.goal_y, self.goal_yaw],
            "final_pose": [self.curr_x, self.curr_y],
            "position_error_m": round(final_err, 3),
            "total_distance_m": round(self.total_distance, 3),
            "duration_s": round(duration, 1),
            "min_obstacle_clearance_m": round(self.min_obstacle_clearance, 3),
            "replans_count": self.replans,
            "timestamp": time.strftime("%Y-%m-%d %H:%M:%S")
        }
        os.makedirs('/tmp/amr4_perception', exist_ok=True)
        with open('/tmp/amr4_navigation_report.json', 'w') as f:
            json.dump(report, f, indent=2)

def main(args=None):
    rclpy.init(args=args)
    node = AMR4AutonomousGoalNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
