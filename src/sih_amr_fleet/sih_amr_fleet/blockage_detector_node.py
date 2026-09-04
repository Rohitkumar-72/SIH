import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from sih_amr_interfaces.msg import BlockageObservation, GridCell, RobotState

from .common import FLEET_STATE_QOS, PROTOCOL_QOS, header, new_session_id


class BlockageDetectorNode(Node):
    """Publishes only persistent, expiring LiDAR-derived blockage observations."""
    def __init__(self):
        super().__init__('blockage_detector_node')
        self.robot_id = self.declare_parameter('robot_id', 'robot_1').value
        self.persistence = self.declare_parameter('persistence_frames', 5).value
        self.session_id, self.sequence, self.pose, self.hits = new_session_id(), 0, None, {}
        self.pub = self.create_publisher(BlockageObservation, '/fleet/blockage_observation', PROTOCOL_QOS)
        self.create_subscription(RobotState, 'state', self.on_state, FLEET_STATE_QOS)
        self.create_subscription(OccupancyGrid, 'local_costmap', self.on_grid, FLEET_STATE_QOS)

    def on_state(self, msg): self.pose = msg.pose

    def on_grid(self, grid):
        if self.pose is None: return
        active = set()
        width = grid.info.width
        for i, value in enumerate(grid.data):
            if value < 80: continue
            x = (i % width) * grid.info.resolution + grid.info.origin.position.x
            y = (i // width) * grid.info.resolution + grid.info.origin.position.y
            key = (round(self.pose.x + x), round(self.pose.y + y))
            active.add(key); self.hits[key] = self.hits.get(key, 0) + 1
        self.hits = {key: count for key, count in self.hits.items() if key in active}
        cells = [key for key, count in self.hits.items() if count >= self.persistence]
        if not cells: return
        self.sequence += 1
        msg = BlockageObservation()
        msg.fleet_header = header(self, self.robot_id, self.session_id, self.sequence, 2.0)
        msg.cells = [GridCell(x=x, y=y, time_slot=0) for x, y in cells]
        msg.confidence, msg.source = 0.8, 'lidar_persistence'
        self.pub.publish(msg)


def main():
    rclpy.init(); node = BlockageDetectorNode()
    try: rclpy.spin(node)
    finally: node.destroy_node(); rclpy.shutdown()
