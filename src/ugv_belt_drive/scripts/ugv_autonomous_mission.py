#!/usr/bin/env python3
"""
UGV Autonomous Mission & Navigation Controller
Dispatches goals to Nav2 (/navigate_to_pose action) and monitors live execution,
odometry, and LiDAR obstacle perception in real-time.
"""

import sys
import math
import time
import argparse
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy


from visualization_msgs.msg import Marker, MarkerArray


def yaw_to_quaternion(yaw_rad):
    """Convert yaw angle in radians to quaternion (x, y, z, w)."""
    return (0.0, 0.0, math.sin(yaw_rad / 2.0), math.cos(yaw_rad / 2.0))


class UGVAutonomousMission(Node):
    def __init__(self, goal_x=3.0, goal_y=0.0, goal_yaw=0.0, timeout_sec=60.0):
        super().__init__('ugv_autonomous_mission')
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)
        self.get_logger().info('Initializing UGV Autonomous Mission Controller...')

        self.target_x = float(goal_x)
        self.target_y = float(goal_y)
        self.target_yaw = float(goal_yaw)
        self.timeout_sec = float(timeout_sec)

        self.current_pose = None
        self.min_front_scan = 999.0
        self.mission_completed = False
        self.mission_succeeded = False

        # Robust QoS Profile matching both reliable and best-effort bridge publishers
        qos_robust = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscriptions
        self.odom_sub = self.create_subscription(
            Odometry, '/ugv/odom', self.odom_callback, 10
        )
        self.odom_sub_be = self.create_subscription(
            Odometry, '/ugv/odom', self.odom_callback, qos_robust
        )
        self.odom_sub2 = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_robust
        )
        self.cmd_pub = self.create_publisher(
            Twist, '/ugv/cmd_vel', 10
        )
        self.cmd_pub2 = self.create_publisher(
            Twist, '/cmd_vel', 10
        )

        # RViz Waypoint Marker Publisher (Point A Green, Point B Red)
        self.marker_pub = self.create_publisher(
            MarkerArray, '/waypoint_markers', 10
        )
        self.marker_timer = self.create_timer(1.0, self.publish_waypoint_markers)

        # Nav2 Action Client
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def publish_waypoint_markers(self):
        ma = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        # 1. Point A (Green Cylinder)
        mA_cyl = Marker()
        mA_cyl.header.frame_id = 'map'
        mA_cyl.header.stamp = stamp
        mA_cyl.ns = 'waypoints'
        mA_cyl.id = 0
        mA_cyl.type = Marker.CYLINDER
        mA_cyl.action = Marker.ADD
        mA_cyl.pose.position.x = 0.0
        mA_cyl.pose.position.y = 0.0
        mA_cyl.pose.position.z = 1.0
        mA_cyl.scale.x = 1.2
        mA_cyl.scale.y = 1.2
        mA_cyl.scale.z = 2.0
        mA_cyl.color.r = 0.0
        mA_cyl.color.g = 1.0
        mA_cyl.color.b = 0.2
        mA_cyl.color.a = 0.85
        ma.markers.append(mA_cyl)

        # 2. Point A Text Label
        mA_txt = Marker()
        mA_txt.header.frame_id = 'map'
        mA_txt.header.stamp = stamp
        mA_txt.ns = 'waypoints'
        mA_txt.id = 1
        mA_txt.type = Marker.TEXT_VIEW_FACING
        mA_txt.action = Marker.ADD
        mA_txt.pose.position.x = 0.0
        mA_txt.pose.position.y = 0.0
        mA_txt.pose.position.z = 2.5
        mA_txt.scale.z = 0.8
        mA_txt.color.r = 0.2
        mA_txt.color.g = 1.0
        mA_txt.color.b = 0.4
        mA_txt.color.a = 1.0
        mA_txt.text = 'POINT A (START)'
        ma.markers.append(mA_txt)

        # 3. Point B (Red Cylinder & Sphere)
        mB_cyl = Marker()
        mB_cyl.header.frame_id = 'map'
        mB_cyl.header.stamp = stamp
        mB_cyl.ns = 'waypoints'
        mB_cyl.id = 2
        mB_cyl.type = Marker.CYLINDER
        mB_cyl.action = Marker.ADD
        mB_cyl.pose.position.x = self.target_x
        mB_cyl.pose.position.y = self.target_y
        mB_cyl.pose.position.z = 1.5
        mB_cyl.scale.x = 1.4
        mB_cyl.scale.y = 1.4
        mB_cyl.scale.z = 3.0
        mB_cyl.color.r = 1.0
        mB_cyl.color.g = 0.1
        mB_cyl.color.b = 0.1
        mB_cyl.color.a = 0.9
        ma.markers.append(mB_cyl)

        # 4. Point B Text Label
        mB_txt = Marker()
        mB_txt.header.frame_id = 'map'
        mB_txt.header.stamp = stamp
        mB_txt.ns = 'waypoints'
        mB_txt.id = 3
        mB_txt.type = Marker.TEXT_VIEW_FACING
        mB_txt.action = Marker.ADD
        mB_txt.pose.position.x = self.target_x
        mB_txt.pose.position.y = self.target_y
        mB_txt.pose.position.z = 3.5
        mB_txt.scale.z = 0.8
        mB_txt.color.r = 1.0
        mB_txt.color.g = 0.2
        mB_txt.color.b = 0.2
        mB_txt.color.a = 1.0
        mB_txt.text = 'POINT B (GOAL)'
        ma.markers.append(mB_txt)

        self.marker_pub.publish(ma)

    def odom_callback(self, msg: Odometry):
        self.current_pose = msg.pose.pose

    def scan_callback(self, msg: LaserScan):
        if not msg.ranges:
            return
        # Calculate minimum distance in front cone (-30 to +30 deg)
        num_samples = len(msg.ranges)
        mid = num_samples // 2
        cone = int(num_samples * (30.0 / 360.0))
        front_ranges = msg.ranges[max(0, mid - cone):min(num_samples, mid + cone)]
        valid_ranges = [r for r in front_ranges if 0.40 < r < msg.range_max and not math.isinf(r) and not math.isnan(r)]
        if valid_ranges:
            self.min_front_scan = min(valid_ranges)
        else:
            self.min_front_scan = 999.0

    def run_mission(self):
        self.get_logger().info(f"Target Goal: X={self.target_x:.2f}m, Y={self.target_y:.2f}m, Yaw={self.target_yaw:.2f}rad")
        
        # Wait for odometry
        self.get_logger().info("Waiting for /ugv/odom telemetry...")
        start_wait = time.time()
        while self.current_pose is None and (time.time() - start_wait) < 10.0:
            rclpy.spin_once(self, timeout_sec=0.1)

        if self.current_pose:
            init_x = self.current_pose.position.x
            init_y = self.current_pose.position.y
            self.get_logger().info(f"Current Robot Pose: X={init_x:.2f}m, Y={init_y:.2f}m")
        else:
            self.get_logger().warn("Odometry not received yet; proceeding with Nav2 dispatch.")

        # Check if Nav2 action server is available
        self.get_logger().info("Connecting to Nav2 action server (/navigate_to_pose)...")
        nav2_ready = self.nav_to_pose_client.wait_for_server(timeout_sec=5.0)

        if nav2_ready:
            self.get_logger().info("Nav2 action server connected! Dispatching goal pose...")
            self.dispatch_nav2_goal()
        else:
            self.get_logger().warn("Nav2 action server not responding. Executing autonomous reactive terrain navigation...")
            self.execute_reactive_navigation()

    def dispatch_nav2_goal(self):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position.x = self.target_x
        goal_msg.pose.pose.position.y = self.target_y
        goal_msg.pose.pose.position.z = 0.0

        qx, qy, qz, qw = yaw_to_quaternion(self.target_yaw)
        goal_msg.pose.pose.orientation.x = qx
        goal_msg.pose.pose.orientation.y = qy
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw

        goal_handle = None
        for attempt in range(1, 15):
            goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
            self.get_logger().info(f"Sending goal to Nav2 (Attempt {attempt}/15)...")
            send_goal_future = self.nav_to_pose_client.send_goal_async(
                goal_msg, feedback_callback=self.nav2_feedback_callback
            )
            rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=5.0)

            if send_goal_future.done():
                goal_handle = send_goal_future.result()
                if goal_handle and goal_handle.accepted:
                    self.get_logger().info("Nav2 Goal accepted! Tracking progress to destination...")
                    break
                else:
                    self.get_logger().warn(f"Nav2 goal not accepted yet (waiting for SLAM map/costmap to initialize)... retry in 2s")
            time.sleep(2.0)

        if not goal_handle or not goal_handle.accepted:
            self.get_logger().warn("Nav2 planner not ready yet. Falling back to autonomous reactive obstacle navigation...")
            self.execute_reactive_navigation()
            return
        res_future = goal_handle.get_result_async()
        start_time = time.time()

        while rclpy.ok() and not res_future.done():
            rclpy.spin_once(self, timeout_sec=0.2)
            if (time.time() - start_time) > self.timeout_sec:
                self.get_logger().warn("Mission timeout reached. Canceling goal...")
                goal_handle.cancel_goal_async()
                break

        if res_future.done():
            result = res_future.result()
            status = result.status
            if status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info("SUCCESS: Nav2 Goal Reached!")
                self.mission_succeeded = True
            else:
                self.get_logger().warn(f"Nav2 goal finished with status code: {status}")
        self.mission_completed = True

    def nav2_feedback_callback(self, feedback_msg):
        fb = feedback_msg.feedback
        dist = fb.distance_remaining
        time_elapsed = fb.navigation_time.sec
        self.get_logger().info(
            f"[Nav2 Progress] Distance to Goal: {dist:.2f}m | Time: {time_elapsed}s | Obstacle Proximity: {self.min_front_scan:.2f}m",
            throttle_duration_sec=1.5
        )

    def execute_reactive_navigation(self):
        """Autonomous reactive obstacle avoidance towards target waypoint."""
        rate_hz = 10.0
        start_time = time.time()
        self.get_logger().info("Starting closed-loop obstacle avoidance towards target...")

        while rclpy.ok() and (time.time() - start_time) < self.timeout_sec:
            rclpy.spin_once(self, timeout_sec=1.0 / rate_hz)

            if self.current_pose is None:
                continue

            cx = self.current_pose.position.x
            cy = self.current_pose.position.y
            dist_to_goal = math.hypot(self.target_x - cx, self.target_y - cy)

            if dist_to_goal < 0.4:
                self.get_logger().info(f"SUCCESS: Target reached! Final Distance: {dist_to_goal:.2f}m")
                self.mission_succeeded = True
                break

            target_angle = math.atan2(self.target_y - cy, self.target_x - cx)
            # Robot yaw from quaternion
            q = self.current_pose.orientation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            current_yaw = math.atan2(siny_cosp, cosy_cosp)

            angle_diff = (target_angle - current_yaw + math.pi) % (2 * math.pi) - math.pi

            cmd = Twist()
            # Obstacle avoidance using LiDAR
            if self.min_front_scan < 0.65:
                # Obstacle detected in front: slow down and pivot away
                cmd.linear.x = 0.05
                cmd.angular.z = 0.8 if angle_diff >= 0 else -0.8
                self.get_logger().info(f"[Avoiding Obstacle] Front Clearance: {self.min_front_scan:.2f}m", throttle_duration_sec=1.0)
            elif abs(angle_diff) > 0.4:
                # Align heading
                cmd.linear.x = 0.15
                cmd.angular.z = max(-1.0, min(1.0, 1.5 * angle_diff))
            else:
                # Drive forward towards goal
                cmd.linear.x = min(0.6, max(0.2, 0.5 * dist_to_goal))
                cmd.angular.z = max(-0.8, min(0.8, 1.0 * angle_diff))

            self.cmd_pub.publish(cmd)
            self.get_logger().info(
                f"[Navigation Progress] Current: ({cx:.2f}, {cy:.2f}) | Distance to Goal: {dist_to_goal:.2f}m | Front Clearance: {self.min_front_scan:.2f}m",
                throttle_duration_sec=1.5
            )

        # Stop robot
        stop_cmd = Twist()
        for _ in range(5):
            self.cmd_pub.publish(stop_cmd)
            time.sleep(0.05)
        self.mission_completed = True


def main():
    parser = argparse.ArgumentParser(description='UGV Autonomous Navigation Mission')
    parser.add_argument('--goal_x', type=float, default=3.0, help='Goal X coordinate in meters')
    parser.add_argument('--goal_y', type=float, default=0.0, help='Goal Y coordinate in meters')
    parser.add_argument('--goal_yaw', type=float, default=0.0, help='Goal Yaw angle in radians')
    parser.add_argument('--timeout', type=float, default=60.0, help='Mission timeout in seconds')
    args, unknown = parser.parse_known_args()

    rclpy.init(args=unknown)
    mission_node = UGVAutonomousMission(
        goal_x=args.goal_x,
        goal_y=args.goal_y,
        goal_yaw=args.goal_yaw,
        timeout_sec=args.timeout
    )

    try:
        mission_node.run_mission()
    except KeyboardInterrupt:
        pass
    finally:
        # Publish 0 velocity
        try:
            mission_node.cmd_pub.publish(Twist())
        except Exception:
            pass
        mission_node.destroy_node()
        rclpy.shutdown()

    if mission_node.mission_succeeded:
        sys.exit(0)
    else:
        sys.exit(1)


if __name__ == '__main__':
    main()
