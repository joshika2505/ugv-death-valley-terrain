#!/usr/bin/env python3
import os
import cv2
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge

class PerceptionDisplayNode(Node):
    """
    Dedicated Standalone AMR-4 Robot POV Interface with Live Perception HUD
    """
    def __init__(self):
        super().__init__('perception_display_node')
        self.bridge = CvBridge()
        self.output_dir = '/tmp/amr4_perception'
        os.makedirs(self.output_dir, exist_ok=True)

        self.declare_parameter('goal_x', 12.0)
        self.declare_parameter('goal_y', 12.0)
        self.goal_x = float(self.get_parameter('goal_x').value)
        self.goal_y = float(self.get_parameter('goal_y').value)

        # Telemetry state
        self.curr_x = 0.0
        self.curr_y = 0.0
        self.curr_speed = 0.0
        self.min_clearance = 10.0
        self.frame_count = 0

        # Subscriptions
        self.sub_cam = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, qos_profile_sensor_data
        )
        self.sub_odom = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )
        self.sub_scan = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data
        )

        # Initialize dedicated POV GUI window
        self.window_name = "AMR-4 Robot First-Person POV [Death Valley]"
        try:
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self.window_name, 720, 540)
        except Exception:
            pass

        self.get_logger().info('====================================================')
        self.get_logger().info(' [POV] AMR-4 Standalone Robot Camera Interface Ready')
        self.get_logger().info('====================================================')

    def odom_callback(self, msg):
        self.curr_x = msg.pose.pose.position.x
        self.curr_y = msg.pose.pose.position.y
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.curr_speed = math.hypot(vx, vy)

    def scan_callback(self, msg):
        valid = [r for r in msg.ranges if msg.range_min < r < msg.range_max]
        if valid:
            self.min_clearance = min(valid)

    def image_callback(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.frame_count += 1

            # Render Heads-Up Display (HUD)
            vis = cv_img.copy()
            h, w = vis.shape[:2]

            # Top Title Banner
            cv2.rectangle(vis, (0, 0), (w, 45), (20, 20, 20), -1)
            cv2.putText(vis, "AMR-4 ROBOT POV | DEATH VALLEY AUTONOMOUS EXPEDITION",
                        (15, 28), cv2.FONT_HERSHEY_DUPLEX, 0.55, (0, 230, 255), 1, cv2.LINE_AA)

            # Telemetry Box (Top Right)
            dist_to_goal = math.hypot(self.goal_x - self.curr_x, self.goal_y - self.curr_y)
            cv2.rectangle(vis, (w - 230, 52), (w - 10, 145), (0, 0, 0), -1)
            cv2.rectangle(vis, (w - 230, 52), (w - 10, 145), (0, 200, 255), 1)
            cv2.putText(vis, f"POSE: ({self.curr_x:.1f}, {self.curr_y:.1f}) m",
                        (w - 220, 72), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
            cv2.putText(vis, f"STOP DIST: {dist_to_goal:.1f} m",
                        (w - 220, 95), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)
            cv2.putText(vis, f"SPEED: {self.curr_speed:.2f} m/s",
                        (w - 220, 118), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 255, 200), 1)
            
            # Clearance Indicator (Bottom Left)
            clear_col = (0, 255, 0) if self.min_clearance > 1.5 else ((0, 255, 255) if self.min_clearance > 0.8 else (0, 0, 255))
            cv2.rectangle(vis, (10, h - 55), (240, h - 10), (0, 0, 0), -1)
            cv2.rectangle(vis, (10, h - 55), (240, h - 10), clear_col, 1)
            cv2.putText(vis, f"LIDAR CLEARANCE: {self.min_clearance:.2f} m",
                        (20, h - 33), cv2.FONT_HERSHEY_SIMPLEX, 0.45, clear_col, 1)
            cv2.putText(vis, "SENSING: 360 LiDAR + RGB-D",
                        (20, h - 16), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (180, 180, 180), 1)

            # Center Crosshair
            cx, cy = w // 2, h // 2
            cv2.line(vis, (cx - 15, cy), (cx + 15, cy), (0, 255, 0), 1)
            cv2.line(vis, (cx, cy - 15), (cx, cy + 15), (0, 255, 0), 1)

            # Save snapshot to disk periodically
            if self.frame_count % 5 == 0:
                cv2.imwrite(os.path.join(self.output_dir, 'latest_robot_view.jpg'), vis)

            # Render to Dedicated Window
            try:
                cv2.imshow(self.window_name, vis)
                cv2.waitKey(1)
            except Exception:
                pass

        except Exception as e:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = PerceptionDisplayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    try:
        cv2.destroyAllWindows()
    except Exception:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
