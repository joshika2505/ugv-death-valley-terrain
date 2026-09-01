#!/usr/bin/env python3
"""
Dedicated Real-Time Desktop GUI Window for Raspberry Pi Camera Module 3 Robot View.
Features robust non-blocking UI rendering, HUD overlays, and zero-freeze event loop.
"""

import sys
import os
import time
import math
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge


class CameraViewerGUI(Node):
    def __init__(self):
        super().__init__('camera_viewer_gui')
        self.bridge = CvBridge()
        self.window_name = 'PI CAMERA V3 (SONY IMX708) - ROBOT EYE LIVE VIEW'
        
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, 960, 540)

        self.latest_frame = None
        self.last_frame_time = time.time()
        self.robot_x = 0.0
        self.robot_y = 0.0

        # Subscriptions
        self.overlay_sub = self.create_subscription(
            Image,
            '/perception_overlay',
            self.overlay_cb,
            10
        )
        self.raw_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.raw_cb,
            10
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_cb,
            10
        )

        # 30 Hz GUI Refresh Timer
        self.gui_timer = self.create_timer(0.033, self.render_frame)
        self.get_logger().info('Camera Viewer GUI active at 30 FPS.')

    def overlay_cb(self, msg: Image):
        try:
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.last_frame_time = time.time()
        except Exception:
            pass

    def raw_cb(self, msg: Image):
        if self.latest_frame is None or (time.time() - self.last_frame_time > 0.5):
            try:
                self.latest_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                self.last_frame_time = time.time()
            except Exception:
                pass

    def odom_cb(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

    def render_frame(self):
        now = time.time()
        
        if self.latest_frame is not None and (now - self.last_frame_time < 2.0):
            frame = self.latest_frame.copy()
            h, w = frame.shape[:2]

            # Coordinate readout in top-right corner
            pos_str = f'POS: ({self.robot_x:.2f}m, {self.robot_y:.2f}m)'
            cv2.putText(frame, pos_str, (w - 220, 28),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 180), 2)

            # Central Flight Aiming Reticle
            cx, cy = int(w / 2), int(h / 2)
            cv2.line(frame, (cx - 18, cy), (cx - 6, cy), (0, 240, 255), 2)
            cv2.line(frame, (cx + 6, cy), (cx + 18, cy), (0, 240, 255), 2)
            cv2.line(frame, (cx, cy - 18), (cx, cy - 6), (0, 240, 255), 2)
            cv2.line(frame, (cx, cy + 6), (cx, cy + 18), (0, 240, 255), 2)

            cv2.imshow(self.window_name, frame)
        else:
            # Standby graphic
            standby = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(standby, 'CONNECTING TO PI CAMERA MODULE 3 STREAM...', (50, 240),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 200, 255), 2)
            cv2.imshow(self.window_name, standby)

        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = CameraViewerGUI()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
