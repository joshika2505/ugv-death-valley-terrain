#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros

class OdomTfBroadcaster(Node):
    def __init__(self):
        super().__init__('odom_tf_broadcaster')
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        static_t = TransformStamped()
        static_t.header.stamp = self.get_clock().now().to_msg()
        static_t.header.frame_id = 'map'
        static_t.child_frame_id = 'odom'
        static_t.transform.translation.x = 0.0
        static_t.transform.translation.y = 0.0
        static_t.transform.translation.z = 0.0
        static_t.transform.rotation.w = 1.0
        self.static_tf_broadcaster.sendTransform(static_t)
        self.sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 50)
        self.get_logger().info('Odom TF Broadcaster initialized (map -> odom, odom -> base_footprint)')

    def odom_callback(self, msg):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = OdomTfBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
