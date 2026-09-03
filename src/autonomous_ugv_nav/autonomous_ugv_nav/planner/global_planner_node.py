"""
Weighted A* Global Planner Node.
Computes globally optimal paths across continuous traversability costmaps.
"""

import heapq
import math
import numpy as np

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from std_msgs.msg import Header


def heuristic(a: tuple, b: tuple) -> float:
    """Euclidean distance heuristic for 2D grid cells."""
    return math.hypot(a[0] - b[0], a[1] - b[1])


def weighted_a_star(
    costmap: np.ndarray,
    start_cell: tuple,
    goal_cell: tuple,
    lethal_threshold: int = 70,
    cost_weight: float = 3.0
) -> list:
    """
    Executes Weighted A* path search over an 8-connected 2D grid.

    Args:
        costmap: 2D array of cost values (shape [H, W]).
        start_cell: (col, row) start cell indices.
        goal_cell: (col, row) goal cell indices.
        lethal_threshold: Cost threshold above which cells are untraversable.
        cost_weight: Weight alpha scaling traversability cost penalty.

    Returns:
        path: List of (col, row) tuples from start to goal, or empty list if no path found.
    """
    h, w = costmap.shape
    start = (int(start_cell[0]), int(start_cell[1]))
    goal = (int(goal_cell[0]), int(goal_cell[1]))

    # Bounds check
    if not (0 <= start[0] < w and 0 <= start[1] < h):
        return []
    if not (0 <= goal[0] < w and 0 <= goal[1] < h):
        return []

    # If goal or start is on a lethal obstacle, find closest non-lethal cell
    if costmap[goal[1], goal[0]] >= lethal_threshold:
        # Simple radial search for closest valid cell
        found = False
        for r in range(1, 10):
            for dx in range(-r, r + 1):
                for dy in range(-r, r + 1):
                    gx, gy = goal[0] + dx, goal[1] + dy
                    if 0 <= gx < w and 0 <= gy < h and costmap[gy, gx] < lethal_threshold:
                        goal = (gx, gy)
                        found = True
                        break
                if found:
                    break
            if found:
                break
        if not found:
            return []

    # 8-connectivity movements (dx, dy, step_cost)
    motions = [
        (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
        (1, 1, 1.414), (-1, 1, 1.414), (1, -1, 1.414), (-1, -1, 1.414)
    ]

    open_set = []
    # (f_score, g_score, (x, y))
    heapq.heappush(open_set, (heuristic(start, goal), 0.0, start))

    came_from = {}
    g_score = {start: 0.0}

    while open_set:
        f, current_g, current = heapq.heappop(open_set)

        if current == goal:
            # Reconstruct path
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            path.reverse()
            return path

        if current_g > g_score.get(current, float('inf')):
            continue

        cx, cy = current

        for dx, dy, dist in motions:
            nx, ny = cx + dx, cy + dy

            if not (0 <= nx < w and 0 <= ny < h):
                continue

            cell_cost = costmap[ny, nx]
            if cell_cost >= lethal_threshold or cell_cost < 0:
                continue

            # Traversability cost scaling
            step_cost = dist * (1.0 + cost_weight * (float(cell_cost) / 100.0))
            tentative_g = current_g + step_cost

            neighbor = (nx, ny)
            if tentative_g < g_score.get(neighbor, float('inf')):
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score = tentative_g + heuristic(neighbor, goal)
                heapq.heappush(open_set, (f_score, tentative_g, neighbor))

    return []


class GlobalPlannerNode(Node):
    """
    ROS 2 Node for Global Path Planning using Weighted A*.
    Subscribes to costmaps, robot pose, and destination goal, publishing /ugv/global_plan.
    """

    def __init__(self):
        super().__init__('global_planner_node')

        # Parameters
        self.declare_parameter('replan_rate_hz', 1.0)
        self.declare_parameter('lethal_cost_thresh', 70)
        self.declare_parameter('cost_weight', 3.0)
        self.declare_parameter('frame_id', 'odom')

        self.replan_rate = float(self.get_parameter('replan_rate_hz').value)
        self.lethal_thresh = int(self.get_parameter('lethal_cost_thresh').value)
        self.cost_weight = float(self.get_parameter('cost_weight').value)
        self.frame_id = str(self.get_parameter('frame_id').value)

        # State Cache
        self.current_costmap = None
        self.costmap_origin_x = 0.0
        self.costmap_origin_y = 0.0
        self.costmap_res = 0.1
        self.robot_pose = None       # (x, y)
        self.goal_pose = None        # (x, y)

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
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/ugv/goal_pose',
            self.goal_callback,
            10
        )

        # Publisher
        self.path_pub = self.create_publisher(Path, '/ugv/global_plan', 10)

        # Timer
        self.timer = self.create_timer(1.0 / self.replan_rate, self.plan_cycle)

        self.get_logger().info('GlobalPlannerNode (Weighted A*) initialized.')

    def costmap_callback(self, msg: OccupancyGrid):
        w = msg.info.width
        h = msg.info.height
        self.costmap_res = msg.info.resolution
        self.costmap_origin_x = msg.info.origin.position.x
        self.costmap_origin_y = msg.info.origin.position.y

        self.current_costmap = np.array(msg.data, dtype=np.int8).reshape((h, w))

    def odom_callback(self, msg: Odometry):
        self.robot_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def goal_callback(self, msg: PoseStamped):
        self.goal_pose = (msg.pose.position.x, msg.pose.position.y)
        self.get_logger().info(f'New goal set: ({self.goal_pose[0]:.2f}, {self.goal_pose[1]:.2f})')
        self.plan_cycle()

    def plan_cycle(self):
        if self.current_costmap is None or self.robot_pose is None or self.goal_pose is None:
            return

        # Convert world coordinates to cell indices
        start_c = int((self.robot_pose[0] - self.costmap_origin_x) / self.costmap_res)
        start_r = int((self.robot_pose[1] - self.costmap_origin_y) / self.costmap_res)

        goal_c = int((self.goal_pose[0] - self.costmap_origin_x) / self.costmap_res)
        goal_r = int((self.goal_pose[1] - self.costmap_origin_y) / self.costmap_res)

        cell_path = weighted_a_star(
            costmap=self.current_costmap,
            start_cell=(start_c, start_r),
            goal_cell=(goal_c, goal_r),
            lethal_threshold=self.lethal_thresh,
            cost_weight=self.cost_weight
        )

        if not cell_path:
            return

        # Convert cell path back to metric world coordinates
        path_msg = Path()
        path_msg.header = Header(stamp=self.get_clock().now().to_msg(), frame_id=self.frame_id)

        for c, r in cell_path:
            wx = self.costmap_origin_x + (c + 0.5) * self.costmap_res
            wy = self.costmap_origin_y + (r + 0.5) * self.costmap_res

            ps = PoseStamped()
            ps.header = path_msg.header
            ps.pose.position.x = float(wx)
            ps.pose.position.y = float(wy)
            ps.pose.position.z = 0.0
            ps.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            path_msg.poses.append(ps)

        self.path_pub.publish(path_msg)


def main(args=None):
    rclpy.init(args=args)
    node = GlobalPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
