#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock

class ClockMonotonicFilter(Node):
    def __init__(self):
        super().__init__('clock_monotonic_filter')
        self.sub = self.create_subscription(Clock, '/gz/clock', self.clock_cb, 50)
        self.pub = self.create_publisher(Clock, '/clock', 50)
        self.last_sec = 0
        self.last_nanosec = 0
        self.get_logger().info('Clock Monotonic Filter Active: Preventing time jitter.')

    def clock_cb(self, msg):
        if msg.clock.sec > self.last_sec or (msg.clock.sec == self.last_sec and msg.clock.nanosec >= self.last_nanosec):
            self.last_sec = msg.clock.sec
            self.last_nanosec = msg.clock.nanosec
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
