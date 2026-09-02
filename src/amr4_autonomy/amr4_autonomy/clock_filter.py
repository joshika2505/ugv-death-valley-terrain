#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock

class ClockMonotonicFilter(Node):
    def __init__(self):
        super().__init__('clock_monotonic_filter')
        self.sub = self.create_subscription(Clock, '/gz/clock', self.clock_cb, 50)
        self.pub = self.create_publisher(Clock, '/clock', 50)
        self.last_total_ns = 0
        self.get_logger().info('Clock Monotonic Filter Active: Zero time-jump guaranteed.')

    def clock_cb(self, msg):
        current_ns = msg.clock.sec * 1_000_000_000 + msg.clock.nanosec
        
        # Handle simulator reset
        if current_ns < self.last_total_ns - 2_000_000_000:
            self.last_total_ns = current_ns
            self.pub.publish(msg)
            return

        if current_ns >= self.last_total_ns:
            self.last_total_ns = current_ns
            self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ClockMonotonicFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
