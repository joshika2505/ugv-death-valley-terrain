#!/usr/bin/env python3
"""
Dynamic Obstacle Injection & Movement Engine for Real-World Digital Twin.
Simulates moving civilian actors, delivery carts, and temporary roadblocks.
Publishes 3D bounding boxes and updates the dynamic costmap layer in real time.
"""

import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import Header

class DynamicObstacleNode(Node):
    def __init__(self):
        super().__init__('dynamic_obstacle_node')
        self.get_logger().info('Initializing Dynamic Obstacle Engine for Real Digital Twin...')

        # Dynamic Obstacle State: Moving across the trail at x=11.8m, y=3.0m -> 7.0m
        self.obs_x = 11.8
        self.obs_y = 2.5
        self.obs_vy = 0.25  # 0.25 m/s moving velocity
        self.y_min = 1.5
        self.y_max = 6.5

        # Subscriptions
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.robot_x = 0.0
        self.robot_y = 0.0

        # Publishers
        self.marker_pub = self.create_publisher(MarkerArray, '/perception/dynamic_obstacles', 10)

        # 10 Hz Movement Update Loop
        self.timer = self.create_timer(0.1, self.update_and_publish)

    def odom_callback(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

    def update_and_publish(self):
        # Update dynamic obstacle position
        self.obs_y += self.obs_vy * 0.1
        if self.obs_y >= self.y_max or self.obs_y <= self.y_min:
            self.obs_vy = -self.obs_vy

        # Distance to robot
        dist = math.hypot(self.obs_x - self.robot_x, self.obs_y - self.robot_y)

        # Publish 3D Visual Marker
        markers = MarkerArray()
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'dynamic_obstacle'
        m.id = 1
        m.type = Marker.CUBE
        m.action = Marker.ADD
        m.pose.position.x = self.obs_x
        m.pose.position.y = self.obs_y
        m.pose.position.z = 0.60
        m.scale.x = 0.90
        m.scale.y = 0.90
        m.scale.z = 1.20
        m.color.r = 1.0
        m.color.g = 0.2
        m.color.b = 0.1
        m.color.a = 0.85
        markers.markers.append(m)

        # Text Label Marker
        txt = Marker()
        txt.header.frame_id = 'map'
        txt.header.stamp = self.get_clock().now().to_msg()
        txt.ns = 'dynamic_obstacle_label'
        txt.id = 2
        txt.type = Marker.TEXT_VIEW_FACING
        txt.action = Marker.ADD
        txt.pose.position.x = self.obs_x
        txt.pose.position.y = self.obs_y
        txt.pose.position.z = 1.45
        txt.scale.z = 0.40
        txt.color.r = 1.0
        txt.color.g = 0.9
        txt.color.b = 0.1
        txt.color.a = 1.0
        txt.text = f'DYNAMIC ACTOR ({dist:.1f}m)'
        markers.markers.append(txt)

        self.marker_pub.publish(markers)

def main(args=None):
    rclpy.init(args=args)
    node = DynamicObstacleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
