"""
ROS 2 Safety Monitor Node.
Centralized watchdog monitoring planner frequency, feature count, slip events,
and executing the Behavior State Machine to publish safety overrides.
"""

from collections import deque
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool, Int32

from autonomous_ugv_nav.safety.behavior_state_machine import BehaviorStateMachine, NavState


class SafetyMonitorNode(Node):
    """
    Central safety watchdog enforcing safe recovery and emergency stops.
    """

    def __init__(self):
        super().__init__('safety_monitor_node')

        # Parameters
        self.declare_parameter('watchdog_rate_hz', 20.0)
        self.declare_parameter('min_features', 50)
        self.declare_parameter('min_planner_rate_hz', 5.0)

        watchdog_rate = float(self.get_parameter('watchdog_rate_hz').value)
        min_features = int(self.get_parameter('min_features').value)
        min_planner_rate = float(self.get_parameter('min_planner_rate_hz').value)

        self.fsm = BehaviorStateMachine(
            min_features=min_features,
            recovery_timeout_sec=4.0,
            min_planner_freq_hz=min_planner_rate
        )

        # Heartbeat and rate tracking
        self.cmd_timestamps = deque(maxlen=20)
        self.latest_nominal_cmd = Twist()
        self.tracked_features = 100  # Default nominal
        self.is_slipping = False

        # Subscriptions
        self.cmd_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_callback,
            10
        )
        self.feature_sub = self.create_subscription(
            Int32,
            '/visual_slam/tracked_features',
            self.feature_callback,
            10
        )
        self.slip_sub = self.create_subscription(
            Bool,
            '/ugv/diagnostics/slip',
            self.slip_callback,
            10
        )

        # Publishers
        self.safety_cmd_pub = self.create_publisher(Twist, '/ugv/safety/cmd_vel_override', 10)
        self.state_pub = self.create_publisher(String, '/ugv/safety/state', 10)

        # Timer
        self.timer = self.create_timer(1.0 / watchdog_rate, self.watchdog_cycle)

        self.get_logger().info('SafetyMonitorNode active with hardware/software watchdog.')

    def cmd_callback(self, msg: Twist):
        self.cmd_timestamps.append(time.time())
        self.latest_nominal_cmd = msg

    def feature_callback(self, msg: Int32):
        self.tracked_features = msg.data

    def slip_callback(self, msg: Bool):
        self.is_slipping = msg.data

    def watchdog_cycle(self):
        # 1. Calculate actual MPPI / Planner rate
        now = time.time()
        planner_freq = 0.0
        if len(self.cmd_timestamps) >= 2:
            dt_total = self.cmd_timestamps[-1] - self.cmd_timestamps[0]
            if dt_total > 0.0:
                planner_freq = (len(self.cmd_timestamps) - 1) / dt_total

        # If no commands received recently, frequency is 0
        if len(self.cmd_timestamps) > 0 and (now - self.cmd_timestamps[-1]) > 0.5:
            planner_freq = 0.0

        # 2. Update FSM
        state = self.fsm.update(
            tracked_features=self.tracked_features,
            planner_freq=planner_freq
        )

        # 3. Publish State String
        state_msg = String()
        state_msg.data = state.value
        self.state_pub.publish(state_msg)

        # 4. Handle State Actions & Overrides
        if state == NavState.SAFE_STOP:
            # Issue zero-velocity override
            stop_twist = Twist()
            self.safety_cmd_pub.publish(stop_twist)

        elif state == NavState.FEATURE_RECOVERY:
            # Issue recovery oscillation override
            v_crawl, omega_osc = self.fsm.get_recovery_cmd(self.latest_nominal_cmd.linear.x)
            recovery_twist = Twist()
            recovery_twist.linear.x = v_crawl
            recovery_twist.angular.z = omega_osc
            self.safety_cmd_pub.publish(recovery_twist)


def main(args=None):
    rclpy.init(args=args)
    node = SafetyMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
