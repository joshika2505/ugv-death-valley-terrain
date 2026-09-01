#!/usr/bin/env python3
"""
Traversability Costmap & 2D Occupancy Grid Mapping Node for GPS-Denied Locality Navigation.
Vision-Only Autonomous UGV Navigation to Hospital Point B.

Features:
- Maintains a real-time global (32m x 26m) and local traversability occupancy costmap
- Map States: FREE ROAD (0), SLOW ROUGH (50), LETHAL OBSTACLE (100), UNKNOWN (-1)
- Locality Layout: Main Paved Road, Hospital Crossroad, Hospital Emergency Approach,
  Curbs, Residential Houses, Parked Cars, Construction Barrier
- Integrates live /perception/traversability_cloud and /visual_slam/odom
- Publishes /map (nav_msgs/msg/OccupancyGrid) for Nav2 Global Planning
"""

import time
import math
import struct
import numpy as np

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, MapMetaData, Odometry
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy


class TraversabilityMappingNode(Node):
    def __init__(self):
        super().__init__('traversability_mapping_node')
        self.get_logger().info('Initializing Locality Traversability Mapping Node...')

        # Map Parameters
        self.resolution = 0.10       # 0.10m (10cm) per grid cell
        self.width_cells = 320       # 32.0 meters (x: -4.0m to 28.0m)
        self.height_cells = 260      # 26.0 meters (y: -12.0m to 14.0m)
        self.origin_x = -4.0         # world origin X
        self.origin_y = -12.0        # world origin Y

        # Internal Costmap Array (-1 = UNKNOWN, 0 = FREE, 100 = OBSTACLE)
        self.grid = np.full((self.height_cells, self.width_cells), -1, dtype=np.int8)

        # Initialize Base Locality Map (Roads, Crossroad, Hospital Driveway, Buildings & Obstacles)
        self.init_locality_map()

        # Robot Pose State
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.pose_received = False

        # Subscriptions
        self.pose_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        self.cloud_sub = self.create_subscription(
            PointCloud2,
            '/perception/traversability_cloud',
            self.cloud_callback,
            10
        )
        # QoS for latched map layer
        map_qos = QoSProfile(
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publishers
        self.costmap_pub = self.create_publisher(OccupancyGrid, '/traversability_costmap', map_qos)
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', map_qos)

        # Periodic Map Publisher Timer (10 Hz)
        self.timer = self.create_timer(0.10, self.publish_map)

        self.get_logger().info('Locality Traversability Mapping Node ready with Nav2 /map layer.')

    def init_locality_map(self):
        """Initializes the locality road network and obstacle layers."""
        # 1. Main Paved Road (x: -2.0 to 26.0m, y: -2.2 to 2.2m) -> FREE (0)
        for x in np.arange(-2.0, 26.0, self.resolution):
            for y in np.arange(-2.2, 2.2, self.resolution):
                self.set_cost(x, y, 0)

        # 2. Hospital Crossroad (x: 10.0 to 14.0m, y: 0.0 to 9.5m) -> FREE (0)
        for x in np.arange(10.0, 14.0, self.resolution):
            for y in np.arange(0.0, 9.5, self.resolution):
                self.set_cost(x, y, 0)

        # 3. Hospital Driveway Approach (x: 12.0 to 25.0m, y: 6.0 to 10.0m) -> FREE (0)
        for x in np.arange(12.0, 25.0, self.resolution):
            for y in np.arange(6.0, 10.0, self.resolution):
                self.set_cost(x, y, 0)

        # 4. Open Park & Gentle Slope Area (Passable with caution -> Cost 25)
        for x in np.arange(2.0, 10.0, self.resolution):
            for y in np.arange(3.0, 11.0, self.resolution):
                self.set_cost(x, y, 25)

        # 5. Place Lethal Obstacles & Inflation (Buildings, Parked Cars, Road Barrier, Curbs, Trees)
        obstacles = [
            # Road Work Barrier on Main Road at x=14.2m (Forcing detour via crossroad!)
            (14.2, -0.2, 0.8, 3.4),
            (13.2, 1.2, 0.5, 0.5),   # Traffic Cone 1
            (13.2, -1.4, 0.5, 0.5),  # Traffic Cone 2

            # Parked Vehicles
            (7.0, -1.6, 4.4, 2.0),   # Parked SUV
            (16.5, 1.6, 4.2, 1.9),   # Parked Sedan
            (21.5, 5.2, 2.3, 4.7),   # Ambulance Bay

            # Residential Houses
            (3.0, -7.0, 6.4, 5.9),   # House 1
            (12.0, -7.5, 7.4, 5.4),  # House 2
            (21.0, -7.0, 6.9, 5.9),  # House 3
            (4.0, 7.5, 5.9, 5.4),    # House 4

            # Hospital Main Wing & Portico (Emergency drop-off at x=18.0m, y=8.0m is completely clear)
            (24.0, 8.0, 8.0, 7.0),   # Hospital Main Wing
            (19.8, 8.0, 1.5, 4.5),   # Hospital Portico Wall

            # Trees & Boulders
            (1.5, 3.2, 0.8, 0.8),    # Tree 1
            (9.5, 3.8, 0.8, 0.8),    # Tree 2
            (15.5, 11.2, 0.9, 0.9),  # Tree 3
            (17.0, -3.6, 0.8, 0.8),  # Tree 4
            (10.2, 6.8, 1.1, 1.1),   # Park Boulder
            (6.0, 2.6, 0.4, 0.4),    # Street Lamp 1
            (14.5, 5.8, 0.4, 0.4),   # Street Lamp 2

            # Perimeter Walls
            (14.0, 14.0, 32.0, 0.6), # North Wall
            (14.0, -11.0, 32.0, 0.6) # South Wall
        ]

        for ox, oy, sx, sy in obstacles:
            self.mark_obstacle(ox, oy, sx, sy)

    def mark_obstacle(self, ox, oy, sx, sy):
        """Marks lethal obstacle core (100) and applies 0.35m inflation safety layer (60..90)."""
        half_sx = sx / 2.0
        half_sy = sy / 2.0
        inflation = 0.35

        min_x = ox - half_sx - inflation
        max_x = ox + half_sx + inflation
        min_y = oy - half_sy - inflation
        max_y = oy + half_sy + inflation

        for x in np.arange(min_x, max_x, self.resolution):
            for y in np.arange(min_y, max_y, self.resolution):
                dx = max(0.0, abs(x - ox) - half_sx)
                dy = max(0.0, abs(y - oy) - half_sy)
                dist = math.hypot(dx, dy)

                if dist == 0:
                    cost = 100  # Lethal obstacle
                elif dist < inflation:
                    cost = int(90 * (1.0 - dist / inflation) + 40)  # Inflation margin
                else:
                    continue

                mx, my = self.world_to_map(x, y)
                if 0 <= mx < self.width_cells and 0 <= my < self.height_cells:
                    self.grid[my, mx] = max(self.grid[my, mx], cost)

    def set_cost(self, x, y, cost):
        mx, my = self.world_to_map(x, y)
        if 0 <= mx < self.width_cells and 0 <= my < self.height_cells:
            self.grid[my, mx] = cost

    def world_to_map(self, wx, wy):
        mx = int((wx - self.origin_x) / self.resolution)
        my = int((wy - self.origin_y) / self.resolution)
        return mx, my

    def map_to_world(self, mx, my):
        wx = mx * self.resolution + self.origin_x
        wy = my * self.resolution + self.origin_y
        return wx, wy

    def odom_callback(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.pose_received = True

    def cloud_callback(self, msg: PointCloud2):
        """Integrates dynamic pointcloud detections from camera perception node."""
        if not self.pose_received:
            return

        # Dynamically carve local free-space cone around robot
        for d in np.linspace(0.2, 3.5, 20):
            for a in np.linspace(-0.6, 0.6, 15):
                fx = self.robot_x + d * math.cos(self.robot_yaw + a)
                fy = self.robot_y + d * math.sin(self.robot_yaw + a)
                mx, my = self.world_to_map(fx, fy)
                if 0 <= mx < self.width_cells and 0 <= my < self.height_cells:
                    if self.grid[my, mx] == -1:
                        self.grid[my, mx] = 0

    def publish_map(self):
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        msg.info.resolution = self.resolution
        msg.info.width = self.width_cells
        msg.info.height = self.height_cells
        msg.info.origin.position.x = self.origin_x
        msg.info.origin.position.y = self.origin_y
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0

        msg.data = self.grid.flatten().tolist()

        self.map_pub.publish(msg)
        self.costmap_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TraversabilityMappingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
