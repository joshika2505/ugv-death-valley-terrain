#!/usr/bin/env python3
"""
Motor Velocity Controller & Command Interface for Forest UGV.
Vision-Only GPS-Denied Autonomous Navigation.

Features:
- Subscribes to /forest_planner/target_twist
- Slew-rate acceleration limiter for smooth off-road traction and wheel torque
- Publishes /cmd_vel to Gazebo / ROS-GZ Bridge
"""

import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class MotorControllerNode(Node):
    def __init__(self):
        super().__init__('motor_controller_node')
        self.get_logger().info('Initializing Forest Motor Controller Node...')

        # Velocity limits
        self.max_lin_accel = 2.5   # m/s^2
        self.max_ang_accel = 4.0   # rad/s^2

        self.current_v = 0.0
        self.current_w = 0.0
        self.target_v = 0.0
        self.target_w = 0.0
        self.last_time = time.time()

        # Subscriptions
        self.target_sub = self.create_subscription(
            Twist,
            '/forest_planner/target_twist',
            self.target_callback,
            10
        )

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # High-rate control timer (30 Hz)
        self.timer = self.create_timer(0.033, self.control_update)

        self.get_logger().info('Motor Controller ready. Publishing /cmd_vel.')

    def target_callback(self, msg: Twist):
        self.target_v = msg.linear.x
        self.target_w = msg.angular.z

    def control_update(self):
        now = time.time()
        dt = max(now - self.last_time, 1e-4)
        self.last_time = now

        # Linear velocity slew rate
        dv_max = self.max_lin_accel * dt
        dv = self.target_v - self.current_v
        if abs(dv) > dv_max:
            self.current_v += dv_max if dv > 0 else -dv_max
        else:
            self.current_v = self.target_v

        # Angular velocity slew rate
        dw_max = self.max_ang_accel * dt
        dw = self.target_w - self.current_w
        if abs(dw) > dw_max:
            self.current_w += dw_max if dw > 0 else -dw_max
        else:
            self.current_w = self.target_w

        cmd = Twist()
        cmd.linear.x = float(self.current_v)
        cmd.angular.z = float(self.current_w)
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = MotorControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
