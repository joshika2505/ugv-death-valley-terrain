"""
ROS 2 EKF State Estimator Node.
Fuses Visual Odometry, IMU, and Wheel Encoders with Slip Detection.
Broadcasts TF odom -> base_link and publishes /ugv/odom_filtered.
"""

import math
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Header
from geometry_msgs.msg import TransformStamped, Quaternion

import tf2_ros

from autonomous_ugv_nav.estimator.ekf_core import EKFCore
from autonomous_ugv_nav.estimator.slip_detector import SlipDetector


def quaternion_from_yaw(yaw: float) -> Quaternion:
    """Creates geometry_msgs/Quaternion from a 2D yaw angle."""
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def yaw_from_quaternion(q) -> float:
    """Extracts yaw angle from quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class EKFStateEstimatorNode(Node):
    """
    State Estimator fusing high-rate IMU, wheel encoder odometry, and VIO
    into a continuous, slip-robust pose estimate.
    """

    def __init__(self):
        super().__init__('ekf_state_estimator_node')

        # Parameters
        self.declare_parameter('publish_rate_hz', 30.0)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', True)

        self.publish_rate = float(self.get_parameter('publish_rate_hz').value)
        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.publish_tf = bool(self.get_parameter('publish_tf').value)

        # Instantiate EKF and Slip Detector
        self.ekf = EKFCore()
        self.slip_detector = SlipDetector(slip_threshold=0.30, window_size=5)

        self.last_predict_time = self.get_clock().now()
        self.latest_a_imu = 0.0
        self.latest_omega_gyro = 0.0
        self.wheel_v = 0.0
        self.wheel_omega = 0.0
        self.vio_v = 0.0

        # Subscriptions
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )
        self.wheel_odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.wheel_odom_callback,
            10
        )
        self.vio_sub = self.create_subscription(
            Odometry,
            '/visual_slam/odom',
            self.vio_callback,
            10
        )
        self.vio_fallback_sub = self.create_subscription(
            Odometry,
            '/rtabmap/odom',
            self.vio_callback,
            10
        )

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/ugv/odom_filtered', 10)
        self.slip_pub = self.create_publisher(Bool, '/ugv/diagnostics/slip', 10)

        # TF Broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Timer
        self.timer = self.create_timer(1.0 / self.publish_rate, self.update_and_publish)

        self.get_logger().info('EKFStateEstimatorNode active with adaptive slip gating.')

    def imu_callback(self, msg: Imu):
        self.latest_a_imu = float(msg.linear_acceleration.x)
        self.latest_omega_gyro = float(msg.angular_velocity.z)

    def wheel_odom_callback(self, msg: Odometry):
        self.wheel_v = float(msg.twist.twist.linear.x)
        self.wheel_omega = float(msg.twist.twist.angular.z)

        # Check slip
        is_slipping = self.slip_detector.update(
            wheel_v=self.wheel_v,
            vio_v=self.vio_v,
            gyro_omega=self.latest_omega_gyro,
            wheel_omega=self.wheel_omega
        )

        cov_scale = self.slip_detector.get_encoder_covariance_scale()

        # Update EKF with wheel odometry
        z_wheel = np.array([self.wheel_v, self.wheel_omega], dtype=np.float64)
        self.ekf.update_wheel_odometry(z_wheel, cov_scale=cov_scale)

    def vio_callback(self, msg: Odometry):
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)

        self.vio_v = float(msg.twist.twist.linear.x)

        z_vio = np.array([px, py, yaw], dtype=np.float64)
        self.ekf.update_vio(z_vio)

    def update_and_publish(self):
        now = self.get_clock().now()
        dt = (now - self.last_predict_time).nanoseconds * 1e-9
        self.last_predict_time = now

        if dt <= 0.0 or dt > 0.5:
            dt = 1.0 / self.publish_rate

        # Predict step
        state = self.ekf.predict(
            a_imu=self.latest_a_imu,
            omega_gyro=self.latest_omega_gyro,
            dt=dt
        )
        cov = self.ekf.P

        px, py, theta, v, omega = state
        stamp = now.to_msg()

        # 1. Publish /ugv/odom_filtered
        odom_msg = Odometry()
        odom_msg.header = Header(stamp=stamp, frame_id=self.odom_frame)
        odom_msg.child_frame_id = self.base_frame

        odom_msg.pose.pose.position.x = float(px)
        odom_msg.pose.pose.position.y = float(py)
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation = quaternion_from_yaw(theta)

        # Pose covariance (6x6)
        odom_msg.pose.covariance[0] = float(cov[0, 0])
        odom_msg.pose.covariance[7] = float(cov[1, 1])
        odom_msg.pose.covariance[35] = float(cov[2, 2])

        odom_msg.twist.twist.linear.x = float(v)
        odom_msg.twist.twist.angular.z = float(omega)
        odom_msg.twist.covariance[0] = float(cov[3, 3])
        odom_msg.twist.covariance[35] = float(cov[4, 4])

        self.odom_pub.publish(odom_msg)

        # 2. Publish Slip Diagnostic
        slip_msg = Bool()
        slip_msg.data = self.slip_detector.is_slipping
        self.slip_pub.publish(slip_msg)

        # 3. Broadcast TF
        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame

            t.transform.translation.x = float(px)
            t.transform.translation.y = float(py)
            t.transform.translation.z = 0.0
            t.transform.rotation = quaternion_from_yaw(theta)

            self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = EKFStateEstimatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
