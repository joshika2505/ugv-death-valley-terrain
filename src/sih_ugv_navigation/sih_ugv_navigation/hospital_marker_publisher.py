#!/usr/bin/env python3
"""
Publishes 3D Visualization Markers for Point A (Start) and Point B (Hospital) in RViz2.
"""

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

class HospitalMarkerPublisher(Node):
    def __init__(self):
        super().__init__('hospital_marker_publisher')
        self.pub = self.create_publisher(MarkerArray, '/hospital_goal_marker', 10)
        self.timer = self.create_timer(1.0, self.publish_markers)
        self.get_logger().info('Hospital & Start Marker Publisher Active.')

    def publish_markers(self):
        markers = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        # 1. Point A: Start Gate Marker (Green Cylinder + Text)
        start_cyl = Marker()
        start_cyl.header.frame_id = 'map'
        start_cyl.header.stamp = stamp
        start_cyl.ns = 'mission_points'
        start_cyl.id = 1
        start_cyl.type = Marker.CYLINDER
        start_cyl.action = Marker.ADD
        start_cyl.pose.position.x = 0.0
        start_cyl.pose.position.y = 0.0
        start_cyl.pose.position.z = 0.15
        start_cyl.scale.x = 1.0
        start_cyl.scale.y = 1.0
        start_cyl.scale.z = 0.3
        start_cyl.color.r = 0.0
        start_cyl.color.g = 1.0
        start_cyl.color.b = 0.3
        start_cyl.color.a = 0.8
        markers.markers.append(start_cyl)

        start_txt = Marker()
        start_txt.header.frame_id = 'map'
        start_txt.header.stamp = stamp
        start_txt.ns = 'mission_points'
        start_txt.id = 2
        start_txt.type = Marker.TEXT_VIEW_FACING
        start_txt.action = Marker.ADD
        start_txt.pose.position.x = 0.0
        start_txt.pose.position.y = 0.0
        start_txt.pose.position.z = 1.2
        start_txt.scale.z = 0.6
        start_txt.color.r = 1.0
        start_txt.color.g = 1.0
        start_txt.color.b = 1.0
        start_txt.color.a = 1.0
        start_txt.text = "POINT A: START LOCATION"
        markers.markers.append(start_txt)

        # 2. Point B: Hospital Marker (Red Cross Cylinder + Text)
        hosp_cyl = Marker()
        hosp_cyl.header.frame_id = 'map'
        hosp_cyl.header.stamp = stamp
        hosp_cyl.ns = 'mission_points'
        hosp_cyl.id = 3
        hosp_cyl.type = Marker.CYLINDER
        hosp_cyl.action = Marker.ADD
        hosp_cyl.pose.position.x = 18.0
        hosp_cyl.pose.position.y = 8.0
        hosp_cyl.pose.position.z = 0.2
        hosp_cyl.scale.x = 1.5
        hosp_cyl.scale.y = 1.5
        hosp_cyl.scale.z = 0.4
        hosp_cyl.color.r = 1.0
        hosp_cyl.color.g = 0.1
        hosp_cyl.color.b = 0.1
        hosp_cyl.color.a = 0.85
        markers.markers.append(hosp_cyl)

        hosp_txt = Marker()
        hosp_txt.header.frame_id = 'map'
        hosp_txt.header.stamp = stamp
        hosp_txt.ns = 'mission_points'
        hosp_txt.id = 4
        hosp_txt.type = Marker.TEXT_VIEW_FACING
        hosp_txt.action = Marker.ADD
        hosp_txt.pose.position.x = 18.0
        hosp_txt.pose.position.y = 8.0
        hosp_txt.pose.position.z = 2.0
        hosp_txt.scale.z = 0.7
        hosp_txt.color.r = 1.0
        hosp_txt.color.g = 0.2
        hosp_txt.color.b = 0.2
        hosp_txt.color.a = 1.0
        hosp_txt.text = "🏥 POINT B: HOSPITAL EMERGENCY"
        markers.markers.append(hosp_txt)

        self.pub.publish(markers)

def main(args=None):
    rclpy.init(args=args)
    node = HospitalMarkerPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
