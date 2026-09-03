"""
ROS 2 Node for the MPPI Local Controller.
Subscribes to costmaps, filtered odometry, and global path, and publishes /cmd_vel.
"""

import math
import numpy as np

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from geometry_msgs.msg import Twist, PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header

from autonomous_ugv_nav.planner.skid_steer_model import SkidSteerModel
from autonomous_ugv_nav.planner.cost_critics import (
    ObstacleCritic,
    PathFollowCritic,
    PathAlignCritic,
    GoalCritic,
    GoalAngleCritic,
    SmoothnessCritic,
    SemanticSpeedCritic,
)
from autonomous_ugv_nav.planner.mppi_core import MPPIController


def yaw_from_quaternion(q) -> float:
    """Extracts yaw angle (in radians) from a geometry_msgs/Quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class MPPIControllerNode(Node):
    """
    Executes the MPPI control loop at 10–15 Hz and publishes /cmd_vel.
    """

    def __init__(self):
        super().__init__('mppi_controller_node')

        # Parameters
        self.declare_parameter('control_rate_hz', 12.0)
        self.declare_parameter('num_samples', 300)
        self.declare_parameter('time_horizon', 20)
        self.declare_parameter('dt', 0.1)
        self.declare_parameter('temperature', 1.0)
        self.declare_parameter('max_v', 1.2)
        self.declare_parameter('min_v', -0.3)
        self.declare_parameter('max_omega', 2.0)
        self.declare_parameter('goal_tolerance_m', 0.4)

        rate_hz = float(self.get_parameter('control_rate_hz').value)
        num_samples = int(self.get_parameter('num_samples').value)
        time_horizon = int(self.get_parameter('time_horizon').value)
        dt = float(self.get_parameter('dt').value)
        temperature = float(self.get_parameter('temperature').value)
        max_v = float(self.get_parameter('max_v').value)
        min_v = float(self.get_parameter('min_v').value)
        max_omega = float(self.get_parameter('max_omega').value)
        self.goal_tol = float(self.get_parameter('goal_tolerance_m').value)

        # Initialize Dynamics and Critics
        dynamics = SkidSteerModel(
            max_v=max_v,
            min_v=min_v,
            max_omega=max_omega,
            max_accel=1.5,
            max_yaw_accel=3.0
        )

        critics = [
            ObstacleCritic(weight=20.0, lethal_cost_thresh=75.0, lethal_penalty=1e5),  # 75 on [0, 100] occupancy grid
            PathFollowCritic(weight=6.0),
            PathAlignCritic(weight=3.5),
            GoalCritic(weight=8.0),
            GoalAngleCritic(weight=3.0, trigger_dist=1.2),
            SmoothnessCritic(weight=2.0),
            SemanticSpeedCritic(weight=4.0, friction_cost_range=(20.0, 60.0)),
        ]

        self.mppi = MPPIController(
            dynamics_model=dynamics,
            critics=critics,
            num_samples=num_samples,
            time_horizon=time_horizon,
            dt=dt,
            temperature=temperature
        )

        # State cache
        self.current_state = None      # [x, y, theta]
        self.current_costmap = None    # 2D numpy array
        self.costmap_origin = None     # (ox, oy, res)
        self.global_path = None        # (N, 2) numpy array
        self.current_goal = None       # [gx, gy, gtheta]
        self.goal_reached = False

        # Subscriptions
        self.costmap_sub = self.create_subscription(
            OccupancyGrid,
            '/ugv/semantic_costmap',
            self.costmap_callback,
            10
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            '/ugv/odom_filtered',
            self.odom_callback,
            10
        )
        self.odom_fallback_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        self.path_sub = self.create_subscription(
            Path,
            '/ugv/global_plan',
            self.path_callback,
            10
        )
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/ugv/goal_pose',
            self.goal_callback,
            10
        )

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.best_traj_pub = self.create_publisher(Path, '/ugv/mppi/best_trajectory', 10)
        self.sampled_trajs_pub = self.create_publisher(MarkerArray, '/ugv/mppi/trajectories', 10)

        # Timer
        self.control_timer = self.create_timer(1.0 / rate_hz, self.control_loop)

        self.get_logger().info(f'MPPIControllerNode active at {rate_hz} Hz (K={num_samples}, T={time_horizon})')

    def odom_callback(self, msg: Odometry):
        pos = msg.pose.pose.position
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self.current_state = np.array([pos.x, pos.y, yaw], dtype=np.float32)

    def costmap_callback(self, msg: OccupancyGrid):
        w = msg.info.width
        h = msg.info.height
        res = msg.info.resolution
        ox = msg.info.origin.position.x
        oy = msg.info.origin.position.y

        data = np.array(msg.data, dtype=np.int8).reshape((h, w))
        # Map unknown (-1) to 0 or neutral for local planning
        data = np.where(data < 0, 0, data)

        self.current_costmap = data
        self.costmap_origin = (ox, oy, res)

    def path_callback(self, msg: Path):
        if len(msg.poses) == 0:
            self.global_path = None
            return

        pts = []
        for p in msg.poses:
            pts.append([p.pose.position.x, p.pose.position.y])
        self.global_path = np.array(pts, dtype=np.float32)

        # Update goal from terminal pose of the path if no explicit goal set
        if self.current_goal is None and len(msg.poses) > 0:
            last_p = msg.poses[-1].pose
            last_yaw = yaw_from_quaternion(last_p.orientation)
            self.current_goal = np.array([last_p.position.x, last_p.position.y, last_yaw], dtype=np.float32)
            self.goal_reached = False

    def goal_callback(self, msg: PoseStamped):
        pos = msg.pose.position
        yaw = yaw_from_quaternion(msg.pose.orientation)
        self.current_goal = np.array([pos.x, pos.y, yaw], dtype=np.float32)
        self.goal_reached = False
        self.get_logger().info(f'Received new goal: ({pos.x:.2f}, {pos.y:.2f})')

    def control_loop(self):
        if self.current_state is None:
            return

        # Check goal proximity
        if self.current_goal is not None:
            dist_to_goal = math.hypot(
                self.current_state[0] - self.current_goal[0],
                self.current_state[1] - self.current_goal[1]
            )
            if dist_to_goal <= self.goal_tol:
                if not self.goal_reached:
                    self.get_logger().info('Goal reached! Holding position.')
                    self.goal_reached = True
                self.publish_stop()
                return

        # Compute MPPI control
        cmd_v, cmd_omega, best_traj, sampled_trajs = self.mppi.compute_control(
            current_state=self.current_state,
            costmap=self.current_costmap,
            costmap_origin=self.costmap_origin,
            goal=self.current_goal,
            global_path=self.global_path
        )

        # Publish Twist
        twist = Twist()
        twist.linear.x = float(cmd_v)
        twist.angular.z = float(cmd_omega)
        self.cmd_pub.publish(twist)

        # Publish Best Trajectory Path
        self.publish_best_trajectory(best_traj)

    def publish_stop(self):
        twist = Twist()
        self.cmd_pub.publish(twist)
        self.mppi.reset()

    def publish_best_trajectory(self, best_traj: np.ndarray):
        path_msg = Path()
        path_msg.header = Header(stamp=self.get_clock().now().to_msg(), frame_id='odom')

        for pt in best_traj:
            ps = PoseStamped()
            ps.header = path_msg.header
            ps.pose.position.x = float(pt[0])
            ps.pose.position.y = float(pt[1])
            ps.pose.position.z = 0.05
            path_msg.poses.append(ps)

        self.best_traj_pub.publish(path_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MPPIControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
