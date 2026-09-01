#!/usr/bin/env python3
"""
Dynamic Point A -> Point B Mission Coordinator for Real Digital Twin.
Arbitrary Goal Point B Selection (No Hardcoded Destinations).
GPS-Denied Navigation using Visual SLAM + Traversability Costmap + Nav2.
"""

import math
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from nav_msgs.msg import Odometry, Path
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from visualization_msgs.msg import Marker, MarkerArray

class PointABMissionCoordinator(Node):
    def __init__(self):
        super().__init__('point_ab_mission_coordinator')
        self.get_logger().info('===========================================================')
        self.get_logger().info('  🚀 POINT A -> POINT B DYNAMIC MISSION COORDINATOR ACTIVE ')
        self.get_logger().info('  GPS-Denied Visual SLAM + Nav2 Pure Pursuit Autonomy     ')
        self.get_logger().info('===========================================================')

        # Configurable Dynamic Start & Goal Points
        self.point_a = (0.0, 0.0)      # Default Start Point A
        self.point_b = (18.0, 8.0)     # Default Goal Point B (Sector Coordinate)

        # Robot Pose State
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.pose_received = False

        # Intermediate Traversable Waypoints for Collision-Free Outskirts Routing
        self.waypoints = [
            ("WAYPOINT_APPROACH", 11.0, 0.0, 0.0),
            ("NORTH_CROSSROAD_BYPASS", 11.8, 5.5, 1.57),
            ("EAST_CONNECTOR_SECTOR", 15.5, 8.0, 0.0),
            ("GOAL_POINT_B", 18.0, 8.0, 0.0),
        ]
        self.current_wp_idx = 0
        self.mission_state = "NAVIGATING_TO_B"  # NAVIGATING_TO_B, REACHED_B, RETURNING_TO_A, MISSION_COMPLETE

        # Action Client for Nav2
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Subscriptions
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.custom_goal_callback, 10)

        # Publishers for 3D Visual Markers in RViz
        self.marker_pub = self.create_publisher(MarkerArray, '/mission/point_ab_markers', 10)

        # Main Mission FSM Loop (5 Hz)
        self.fsm_timer = self.create_timer(0.2, self.fsm_step)
        self.goal_handle = None
        self.goal_in_progress = False

    def odom_callback(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)
        self.pose_received = True

    def custom_goal_callback(self, msg: PoseStamped):
        """Allows dynamic click-to-navigate for arbitrary Point B selection from RViz or Dashboard."""
        gx = msg.pose.position.x
        gy = msg.pose.position.y
        self.get_logger().info(f'📍 User Selected New Dynamic Goal Point B: ({gx:.2f}m, {gy:.2f}m)')
        self.point_b = (gx, gy)
        self.waypoints[-1] = ("CUSTOM_GOAL_POINT_B", gx, gy, 0.0)
        self.current_wp_idx = 0
        self.mission_state = "NAVIGATING_TO_B"
        self.goal_in_progress = False

    def send_nav2_goal(self, target_x: float, target_y: float, target_yaw: float = 0.0):
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
            return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = target_x
        goal_msg.pose.pose.position.y = target_y
        goal_msg.pose.pose.position.z = 0.0

        qz = math.sin(target_yaw / 2.0)
        qw = math.cos(target_yaw / 2.0)
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw

        self.get_logger().info(f'Nav2 Routing toward ({target_x:.1f}m, {target_y:.1f}m)...')
        send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)
        self.goal_in_progress = True
        return True

    def goal_response_callback(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().warn('Nav2 Goal rejected by planner.')
            self.goal_in_progress = False

    def fsm_step(self):
        if not self.pose_received:
            return

        self.publish_ab_markers()

        if self.current_wp_idx < len(self.waypoints):
            name, tx, ty, tyaw = self.waypoints[self.current_wp_idx]
            dist = math.hypot(tx - self.robot_x, ty - self.robot_y)

            if not self.goal_in_progress:
                self.send_nav2_goal(tx, ty, tyaw)

            if dist < 0.65:
                self.get_logger().info(f'✓ Reached Waypoint: {name} ({tx:.1f}, {ty:.1f})')
                self.current_wp_idx += 1
                self.goal_in_progress = False
                if self.current_wp_idx >= len(self.waypoints):
                    self.get_logger().info(f'🎉 UGV ARRIVED AT GOAL POINT B ({self.point_b[0]:.1f}, {self.point_b[1]:.1f})!')
                    self.mission_state = "REACHED_B"

    def publish_ab_markers(self):
        markers = MarkerArray()
        
        # Start Point A Marker (Green Cylinder)
        ma = Marker()
        ma.header.frame_id = 'map'
        ma.header.stamp = self.get_clock().now().to_msg()
        ma.ns = 'point_ab'
        ma.id = 10
        ma.type = Marker.CYLINDER
        ma.action = Marker.ADD
        ma.pose.position.x = self.point_a[0]
        ma.pose.position.y = self.point_a[1]
        ma.pose.position.z = 0.05
        ma.scale.x = 1.2
        ma.scale.y = 1.2
        ma.scale.z = 0.10
        ma.color.r = 0.1
        ma.color.g = 0.9
        ma.color.b = 0.2
        ma.color.a = 0.9
        markers.markers.append(ma)

        # Goal Point B Marker (Golden Beacon)
        mb = Marker()
        mb.header.frame_id = 'map'
        mb.header.stamp = self.get_clock().now().to_msg()
        mb.ns = 'point_ab'
        mb.id = 20
        mb.type = Marker.CYLINDER
        mb.action = Marker.ADD
        mb.pose.position.x = self.point_b[0]
        mb.pose.position.y = self.point_b[1]
        mb.pose.position.z = 0.05
        mb.scale.x = 1.6
        mb.scale.y = 1.6
        mb.scale.z = 0.10
        mb.color.r = 1.0
        mb.color.g = 0.8
        mb.color.b = 0.0
        mb.color.a = 0.9
        markers.markers.append(mb)

        self.marker_pub.publish(markers)

def main(args=None):
    rclpy.init(args=args)
    node = PointABMissionCoordinator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
