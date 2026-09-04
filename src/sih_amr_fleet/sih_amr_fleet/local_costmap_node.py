import math
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

from .common import POSE_QOS


class LocalCostmapNode(Node):
    """Creates a local robot-frame occupancy grid and closest-obstacle measurement."""
    def __init__(self):
        super().__init__('local_costmap_node')
        self.resolution = self.declare_parameter('resolution_m', 0.20).value
        self.size = self.declare_parameter('size_m', 8.0).value
        self.frame = self.declare_parameter('base_frame', 'base_link').value
        self.grid_pub = self.create_publisher(OccupancyGrid, 'local_costmap', POSE_QOS)
        self.distance_pub = self.create_publisher(Float32, 'nearest_obstacle_m', POSE_QOS)
        self.create_subscription(LaserScan, 'scan', self.on_scan, POSE_QOS)

    def on_scan(self, scan):
        cells = int(self.size / self.resolution)
        grid = OccupancyGrid()
        grid.header = scan.header; grid.header.frame_id = self.frame
        grid.info.resolution, grid.info.width, grid.info.height = self.resolution, cells, cells
        grid.info.origin.position.x = -self.size / 2.0; grid.info.origin.position.y = -self.size / 2.0
        grid.data = [0] * (cells * cells)
        nearest = math.inf
        for index, distance in enumerate(scan.ranges):
            if not math.isfinite(distance) or distance < scan.range_min or distance > scan.range_max:
                continue
            nearest = min(nearest, distance)
            angle = scan.angle_min + index * scan.angle_increment
            x, y = distance * math.cos(angle), distance * math.sin(angle)
            gx, gy = int((x + self.size / 2.0) / self.resolution), int((y + self.size / 2.0) / self.resolution)
            if 0 <= gx < cells and 0 <= gy < cells: grid.data[gy * cells + gx] = 100
        self.grid_pub.publish(grid)
        self.distance_pub.publish(Float32(data=nearest if math.isfinite(nearest) else scan.range_max))


def main():
    rclpy.init(); node = LocalCostmapNode()
    try: rclpy.spin(node)
    finally: node.destroy_node(); rclpy.shutdown()
