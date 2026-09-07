import math
import pathlib
import rclpy
import yaml
from geometry_msgs.msg import Pose
from nav_msgs.msg import MapMetaData, OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

MAP_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL
)


class WarehouseMapNode(Node):
    """Parses warehouse layout YAML and publishes standard OccupancyGrid /map and /map_metadata."""

    def __init__(self):
        super().__init__('warehouse_map_node')
        self.map_file = self.declare_parameter('map_file', '').value
        self.resolution = self.declare_parameter('resolution_m', 0.5).value
        self.origin_x = self.declare_parameter('origin_x', -22.5).value
        self.origin_y = self.declare_parameter('origin_y', -30.0).value
        self.width = self.declare_parameter('width_cells', 90).value
        self.height = self.declare_parameter('height_cells', 120).value
        self.blocked_cells = set()

        if self.map_file:
            self.load_map_file(self.map_file)

        self.map_pub = self.create_publisher(OccupancyGrid, '/map', MAP_QOS)
        self.meta_pub = self.create_publisher(MapMetaData, '/map_metadata', MAP_QOS)

        # Publish immediately and periodically at 1 Hz for late joiners
        self.timer = self.create_timer(1.0, self.publish_map)
        self.publish_map()
        self.get_logger().info(
            f'WarehouseMapNode initialized ({self.width}x{self.height} cells @ {self.resolution}m/cell, '
            f'{len(self.blocked_cells)} blocked cells)'
        )

    def load_map_file(self, filename):
        path = pathlib.Path(filename)
        if not path.is_file():
            self.get_logger().warn(f'Map file {filename} does not exist; using defaults.')
            return

        try:
            data = yaml.safe_load(path.read_text())
            self.resolution = float(data.get('resolution_m', self.resolution))
            self.width = int(data.get('width', self.width))
            self.height = int(data.get('height', self.height))
            origin = data.get('origin', [self.origin_x, self.origin_y])
            self.origin_x, self.origin_y = float(origin[0]), float(origin[1])

            self.blocked_cells = {tuple(c) for c in data.get('blocked_cells', [])}

            layout = data.get('shelf_layout')
            if layout:
                footprint = layout.get('footprint_m', [3.92, 0.90])
                half_x = math.ceil(footprint[0] / self.resolution / 2.0)
                half_y = math.ceil(footprint[1] / self.resolution / 2.0)
                excluded = {tuple(p) for p in layout.get('excluded_zones', [])}

                for y_zone, rows in layout.get('y_zones', {}).items():
                    for x_zone, columns in layout.get('x_zones', {}).items():
                        if (y_zone, x_zone) in excluded:
                            continue
                        for x in columns:
                            for y in rows:
                                cx = round((x - self.origin_x) / self.resolution)
                                cy = round((y - self.origin_y) / self.resolution)
                                for dx in range(-half_x, half_x + 1):
                                    for dy in range(-half_y, half_y + 1):
                                        bx, by = cx + dx, cy + dy
                                        if 0 <= bx < self.width and 0 <= by < self.height:
                                            self.blocked_cells.add((bx, by))
        except Exception as e:
            self.get_logger().error(f'Error reading map file {filename}: {e}')

    def publish_map(self):
        now = self.get_clock().now().to_msg()

        meta = MapMetaData()
        meta.map_load_time = now
        meta.resolution = float(self.resolution)
        meta.width = int(self.width)
        meta.height = int(self.height)
        meta.origin.position.x = float(self.origin_x)
        meta.origin.position.y = float(self.origin_y)
        meta.origin.position.z = 0.0
        meta.origin.orientation.w = 1.0

        grid = OccupancyGrid()
        grid.header.stamp = now
        grid.header.frame_id = 'map'
        grid.info = meta

        # Row-major order data array: 0 = free, 100 = occupied
        data = [0] * (self.width * self.height)
        for (x, y) in self.blocked_cells:
            if 0 <= x < self.width and 0 <= y < self.height:
                data[y * self.width + x] = 100

        grid.data = data
        self.map_pub.publish(grid)
        self.meta_pub.publish(meta)


def main(args=None):
    rclpy.init(args=args)
    node = WarehouseMapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
