"""
ROS 2 Semantic Costmap Node.
Converts 3D PointCloud2 and semantic hazard inputs into a unified,
multi-layer 2.5D traversability OccupancyGrid.
"""

import math
import struct
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import Header

from autonomous_ugv_nav.costmap.costmap_types import (
    CostmapCellType,
    CostmapConfig,
    DEFAULT_SEMANTIC_COST_MAP,
)
from autonomous_ugv_nav.costmap.traversability_analyzer import TraversabilityAnalyzer


def pointcloud2_to_array(cloud_msg: PointCloud2) -> np.ndarray:
    """
    Extracts (x, y, z) coordinates from sensor_msgs/PointCloud2 as a fast NumPy Nx3 array.
    """
    # Check field offsets
    fields = {f.name: f.offset for f in cloud_msg.fields}
    if 'x' not in fields or 'y' not in fields or 'z' not in fields:
        return np.empty((0, 3), dtype=np.float32)

    x_off, y_off, z_off = fields['x'], fields['y'], fields['z']
    point_step = cloud_msg.point_step
    data = np.frombuffer(cloud_msg.data, dtype=np.uint8)

    num_points = cloud_msg.width * cloud_msg.height
    if len(data) < num_points * point_step:
        return np.empty((0, 3), dtype=np.float32)

    # Reshape byte array to (num_points, point_step)
    data = data[:num_points * point_step].reshape((num_points, point_step))

    # Extract 32-bit floats for x, y, z
    x = data[:, x_off:x_off+4].copy().view(dtype=np.float32).flatten()
    y = data[:, y_off:y_off+4].copy().view(dtype=np.float32).flatten()
    z = data[:, z_off:z_off+4].copy().view(dtype=np.float32).flatten()

    points = np.column_stack((x, y, z))
    # Filter out NaNs and Infs
    valid = np.isfinite(points).all(axis=1)
    return points[valid]


class SemanticCostmapNode(Node):
    """
    Processes stereo depth PointCloud2 into a 2.5D elevation grid, computes
    slope/roughness/step-height traversability, fuses semantic classifications,
    and publishes the final inflated OccupancyGrid.
    """

    def __init__(self):
        super().__init__('semantic_costmap_node')

        # Parameters
        self.declare_parameter('resolution', 0.1)
        self.declare_parameter('grid_width_m', 20.0)
        self.declare_parameter('grid_height_m', 20.0)
        self.declare_parameter('robot_radius', 0.35)
        self.declare_parameter('inflation_radius', 0.70)
        self.declare_parameter('max_slope_deg', 25.0)
        self.declare_parameter('max_roughness_m', 0.08)
        self.declare_parameter('ground_clearance', 0.15)
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('pointcloud_topic', '/oak/points')
        self.declare_parameter('costmap_topic', '/ugv/semantic_costmap')

        self.resolution = float(self.get_parameter('resolution').value)
        self.grid_w_m = float(self.get_parameter('grid_width_m').value)
        self.grid_h_m = float(self.get_parameter('grid_height_m').value)
        self.robot_radius = float(self.get_parameter('robot_radius').value)
        self.inflation_radius = float(self.get_parameter('inflation_radius').value)
        self.max_slope_deg = float(self.get_parameter('max_slope_deg').value)
        self.max_roughness_m = float(self.get_parameter('max_roughness_m').value)
        self.ground_clearance = float(self.get_parameter('ground_clearance').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        pc_topic = str(self.get_parameter('pointcloud_topic').value)
        costmap_topic = str(self.get_parameter('costmap_topic').value)

        # Dimensions in cells
        self.cells_w = int(round(self.grid_w_m / self.resolution))
        self.cells_h = int(round(self.grid_h_m / self.resolution))

        self.analyzer = TraversabilityAnalyzer(
            resolution=self.resolution,
            ground_clearance=self.ground_clearance
        )

        # QoS profile for sensor streaming
        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=2
        )

        # Subscriptions
        self.pc_sub = self.create_subscription(
            PointCloud2,
            pc_topic,
            self.pointcloud_callback,
            qos_sensor
        )

        # Fallback subscription to common gazebo/camera pointcloud topics
        self.pc_sub_fallback = self.create_subscription(
            PointCloud2,
            '/camera/depth/points',
            self.pointcloud_callback,
            qos_sensor
        )

        # Publishers
        self.costmap_pub = self.create_publisher(OccupancyGrid, costmap_topic, 10)
        self.traversability_pub = self.create_publisher(OccupancyGrid, '/ugv/traversability_grid', 10)

        self.get_logger().info(
            f'SemanticCostmapNode initialized: {self.cells_w}x{self.cells_h} cells '
            f'({self.resolution}m res, frame={self.frame_id})'
        )

    def pointcloud_callback(self, msg: PointCloud2):
        """Processes incoming 3D PointCloud2 and publishes the updated costmap."""
        points = pointcloud2_to_array(msg)
        if len(points) == 0:
            return

        # 1. Build 2.5D Elevation Grid centered at robot base / local frame
        # We assume local grid from -grid_w/2 to +grid_w/2 in X and -grid_h/2 to +grid_h/2 in Y
        origin_x = -self.grid_w_m / 2.0
        origin_y = -self.grid_h_m / 2.0

        # Filter points within bounds
        x, y, z = points[:, 0], points[:, 1], points[:, 2]
        in_bounds = (
            (x >= origin_x) & (x < origin_x + self.grid_w_m) &
            (y >= origin_y) & (y < origin_y + self.grid_h_m)
        )
        pts = points[in_bounds]
        if len(pts) == 0:
            return

        # Compute cell indices
        col_indices = ((pts[:, 0] - origin_x) / self.resolution).astype(np.int32)
        row_indices = ((pts[:, 1] - origin_y) / self.resolution).astype(np.int32)

        # Create elevation grid initialized to NaN
        elevation_grid = np.full((self.cells_h, self.cells_w), np.nan, dtype=np.float32)

        # Accumulate mean elevation per cell using fast numpy grouping
        flat_indices = row_indices * self.cells_w + col_indices
        unique_indices, inv_indices, counts = np.unique(flat_indices, return_inverse=True, return_counts=True)
        z_sums = np.bincount(inv_indices, weights=pts[:, 2])
        mean_z = z_sums / counts

        np.put(elevation_grid, unique_indices, mean_z)

        # 2. Compute Traversability Cost Layers
        slope_cost = self.analyzer.compute_slope(elevation_grid, max_slope_deg=self.max_slope_deg)
        roughness_cost = self.analyzer.compute_roughness(elevation_grid, max_roughness_m=self.max_roughness_m)
        step_cost = self.analyzer.compute_step_height(elevation_grid, ground_clearance=self.ground_clearance)

        # 3. Fuse Geometric Traversability
        traversability_cost = self.analyzer.fuse_traversability(slope_cost, roughness_cost, step_cost)

        # 4. Inflate Obstacles for Safety Buffer
        inflated_costmap = self.analyzer.inflate_costmap(
            traversability_cost,
            robot_radius=self.robot_radius,
            inflation_radius=self.inflation_radius
        )

        # 5. Publish OccupancyGrids
        stamp = msg.header.stamp if msg.header.stamp.sec != 0 else self.get_clock().now().to_msg()
        frame = msg.header.frame_id if msg.header.frame_id else self.frame_id

        self.publish_grid(self.costmap_pub, inflated_costmap, stamp, frame, origin_x, origin_y)
        self.publish_grid(self.traversability_pub, traversability_cost, stamp, frame, origin_x, origin_y)

    def publish_grid(self, publisher, cost_data: np.ndarray, stamp, frame_id: str, origin_x: float, origin_y: float):
        """Converts a 2D uint8 numpy cost array to nav_msgs/OccupancyGrid and publishes."""
        grid_msg = OccupancyGrid()
        grid_msg.header = Header(stamp=stamp, frame_id=frame_id)

        meta = MapMetaData()
        meta.map_load_time = stamp
        meta.resolution = self.resolution
        meta.width = self.cells_w
        meta.height = self.cells_h

        origin = Pose()
        origin.position = Point(x=origin_x, y=origin_y, z=0.0)
        origin.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        meta.origin = origin

        grid_msg.info = meta

        # Convert uint8 [0, 254] to int8 [0, 100] standard OccupancyGrid format for Nav2
        # Cost 254 -> 100 (Occupied), 0 -> 0 (Free), Intermediate -> mapped proportionally
        int8_data = (cost_data.astype(np.float32) / 254.0 * 100.0).astype(np.int8)
        grid_msg.data = int8_data.flatten().tolist()

        publisher.publish(grid_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SemanticCostmapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
