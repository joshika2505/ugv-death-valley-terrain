#!/usr/bin/env python3
"""
Global A* Path Planner Node for Forest UGV Navigation.
Vision-Only GPS-Denied Autonomous Navigation.

Features:
- Computes optimal global path across traversability costmap from Point A to Point B
- Heuristic search with obstacle cost penalties and trail alignment
- Continuous replanning upon dynamic obstacle detection
- Publishes /forest_planner/global_path
"""

import time
import math
import heapq
import numpy as np

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path, Odometry
from geometry_msgs.msg import PoseStamped, Point, Quaternion


class GlobalPathPlannerNode(Node):
    def __init__(self):
        super().__init__('global_path_planner_node')
        self.get_logger().info('Initializing Global A* Path Planner...')

        # Mission Waypoints
        self.start_x = 0.0
        self.start_y = 0.0
        self.goal_x = 20.0
        self.goal_y = 3.5

        # Robot Current Pose
        self.current_x = 0.0
        self.current_y = 0.0
        self.pose_received = False

        # Costmap State
        self.costmap = None
        self.map_info = None

        # Subscriptions
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        self.costmap_sub = self.create_subscription(
            OccupancyGrid,
            '/traversability_costmap',
            self.costmap_callback,
            10
        )

        # Publishers
        self.path_pub = self.create_publisher(Path, '/forest_planner/global_path', 10)

        # Periodic Replanning Timer (2 Hz)
        self.timer = self.create_timer(0.50, self.plan_path)

        self.get_logger().info(f'Global Planner initialized. Target Goal: Point B ({self.goal_x}m, {self.goal_y}m).')

    def odom_callback(self, msg: Odometry):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.pose_received = True

    def costmap_callback(self, msg: OccupancyGrid):
        self.map_info = msg.info
        self.costmap = np.array(msg.data, dtype=np.int8).reshape((msg.info.height, msg.info.width))

    def world_to_map(self, wx, wy):
        if self.map_info is None:
            return 0, 0
        mx = int((wx - self.map_info.origin.position.x) / self.map_info.resolution)
        my = int((wy - self.map_info.origin.position.y) / self.map_info.resolution)
        return mx, my

    def map_to_world(self, mx, my):
        if self.map_info is None:
            return 0.0, 0.0
        wx = self.map_info.origin.position.x + (mx + 0.5) * self.map_info.resolution
        wy = self.map_info.origin.position.y + (my + 0.5) * self.map_info.resolution
        return wx, wy

    def a_star_search(self, start_m, goal_m):
        """A* Search on the 2D Costmap Grid."""
        h, w = self.costmap.shape
        sx, sy = start_m
        gx, gy = goal_m

        sx = max(0, min(w - 1, sx))
        sy = max(0, min(h - 1, sy))
        gx = max(0, min(w - 1, gx))
        gy = max(0, min(h - 1, gy))

        open_set = []
        heapq.heappush(open_set, (0.0, (sx, sy)))
        came_from = {}
        g_score = { (sx, sy): 0.0 }

        motions = [
            (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
            (1, 1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (-1, -1, 1.414)
        ]

        iterations = 0
        while open_set and iterations < 6000:
            iterations += 1
            _, current = heapq.heappop(open_set)

            if math.hypot(current[0] - gx, current[1] - gy) <= 2:
                # Reconstruct Path
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                path.reverse()
                return path

            for dx, dy, cost in motions:
                nx, ny = current[0] + dx, current[1] + dy
                if 0 <= nx < w and 0 <= ny < h:
                    cell_val = self.costmap[ny, nx]
                    if cell_val >= 90:  # Lethal obstacle
                        continue

                    # Penalty for high-risk or unknown cells
                    risk_penalty = 1.0
                    if cell_val > 0:
                        risk_penalty = 1.0 + (cell_val / 20.0)

                    tentative_g = g_score[current] + cost * risk_penalty

                    if (nx, ny) not in g_score or tentative_g < g_score[(nx, ny)]:
                        came_from[(nx, ny)] = current
                        g_score[(nx, ny)] = tentative_g
                        h_dist = math.hypot(nx - gx, ny - gy)
                        f_score = tentative_g + h_dist
                        heapq.heappush(open_set, (f_score, (nx, ny)))

        # Fallback straight line if path blocked
        return [start_m, goal_m]

    def plan_path(self):
        if not self.pose_received:
            return

        # Start from current robot pose
        start_wx = self.current_x
        start_wy = self.current_y

        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'odom'

        if self.costmap is not None:
            start_m = self.world_to_map(start_wx, start_wy)
            goal_m = self.world_to_map(self.goal_x, self.goal_y)

            grid_path = self.a_star_search(start_m, goal_m)

            # Subsample and smooth waypoints
            step = max(1, len(grid_path) // 35)
            waypoints = grid_path[::step]
            if grid_path[-1] not in waypoints:
                waypoints.append(grid_path[-1])

            for mx, my in waypoints:
                wx, wy = self.map_to_world(mx, my)
                p = PoseStamped()
                p.header = path_msg.header
                p.pose.position.x = wx
                p.pose.position.y = wy
                p.pose.orientation.w = 1.0
                path_msg.poses.append(p)
        else:
            # Default trail interpolation
            num_pts = 25
            for i in range(num_pts + 1):
                t = i / float(num_pts)
                wx = start_wx + t * (self.goal_x - start_wx)
                wy = start_wy + t * (self.goal_y - start_wy)
                p = PoseStamped()
                p.header = path_msg.header
                p.pose.position.x = wx
                p.pose.position.y = wy
                p.pose.orientation.w = 1.0
                path_msg.poses.append(p)

        self.path_pub.publish(path_msg)


def main(args=None):
    rclpy.init(args=args)
    node = GlobalPathPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
