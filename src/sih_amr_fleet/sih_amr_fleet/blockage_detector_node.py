import pathlib
import yaml
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from sih_amr_interfaces.msg import BlockageObservation, GridCell, RobotState

from .algorithms import (
    expand_grid_cells, filter_unexpected_blockages, local_point_to_grid_cell,
)
from .common import (
    FLEET_STATE_QOS, POSE_QOS, PROTOCOL_QOS, header, new_session_id,
    now_seconds, stamp_seconds,
)
from .map_geometry import map_geometry_from_data


class BlockageDetectorNode(Node):
    """Publishes only persistent, expiring LiDAR-derived blockage observations."""
    def __init__(self):
        super().__init__('blockage_detector_node')
        self.robot_id = self.declare_parameter('robot_id', 'robot_1').value
        # Ten 20 Hz observations reject one-frame edges and moving scan smear
        # while adding only 0.5 simulated seconds before a real fixed obstacle
        # is shared. Local safety still reacts to every scan immediately.
        self.persistence = self.declare_parameter('persistence_frames', 10).value
        self.observation_ttl_s = self.declare_parameter(
            'blockage_observation_ttl_s', 0.75).value
        self.static_clearance = self.declare_parameter(
            'blockage_static_clearance_cells', 1).value
        self.robot_clearance = self.declare_parameter(
            'blockage_robot_clearance_cells', 2).value
        self.boundary_clearance = self.declare_parameter(
            'blockage_boundary_clearance_cells', 1).value
        self.map_resolution = self.declare_parameter('grid_resolution_m', 0.5).value
        self.map_origin_x, self.map_origin_y = -22.5, -30.0
        self.map_width, self.map_height = 90, 120
        self.static_blocked = set()
        self.fixture_cells = set()
        map_file = self.declare_parameter('map_file', '').value
        if map_file:
            try:
                map_data = yaml.safe_load(pathlib.Path(map_file).read_text())
                (self.map_resolution, self.map_width, self.map_height,
                 self.map_origin_x, self.map_origin_y,
                 self.static_blocked) = map_geometry_from_data(
                    map_data, default_resolution=self.map_resolution,
                    default_width=self.map_width,
                    default_height=self.map_height,
                    default_origin=(self.map_origin_x, self.map_origin_y))
                self.fixture_cells = {
                    tuple(int(value) for value in anchor['cell'])
                    for anchor in map_data.get('anchors', {}).values()
                    if isinstance(anchor, dict) and len(anchor.get('cell', [])) == 2
                }
            except Exception as error:
                self.get_logger().error(f'Failed loading blockage grid transform: {error}')
        # LiDAR endpoints on a shelf surface can quantize one cell outside the
        # analytically generated footprint.  Precompute this immutable halo,
        # including known dock fixtures, instead of rebuilding it per scan.
        self.expected_cells = expand_grid_cells(
            self.static_blocked, self.static_clearance,
            self.map_width, self.map_height)
        self.expected_cells.update(expand_grid_cells(
            self.fixture_cells, self.robot_clearance,
            self.map_width, self.map_height))
        self.session_id, self.sequence, self.pose, self.hits = new_session_id(), 0, None, {}
        self.robot_poses = {}
        self.pub = self.create_publisher(BlockageObservation, '/fleet/blockage_observation', PROTOCOL_QOS)
        self.create_subscription(RobotState, 'state', self.on_state, POSE_QOS)
        self.create_subscription(
            RobotState, '/fleet/robot_state', self.on_fleet_state,
            FLEET_STATE_QOS)
        self.create_subscription(OccupancyGrid, 'local_costmap', self.on_grid, POSE_QOS)

    def on_state(self, msg):
        if not msg.localization_valid:
            return
        self.pose = msg.pose
        self.robot_poses[self.robot_id] = (
            msg.pose, stamp_seconds(msg.fleet_header.valid_until))

    def on_fleet_state(self, msg):
        robot_id = msg.fleet_header.robot_id
        if not robot_id or not msg.localization_valid:
            return
        valid_until = stamp_seconds(msg.fleet_header.valid_until)
        if valid_until < now_seconds(self):
            return
        self.robot_poses[robot_id] = (msg.pose, valid_until)

    def pose_to_cell(self, pose):
        return (
            round((pose.x - self.map_origin_x) / self.map_resolution),
            round((pose.y - self.map_origin_y) / self.map_resolution),
        )

    def on_grid(self, grid):
        if self.pose is None: return
        active = set()
        width = grid.info.width
        for i, value in enumerate(grid.data):
            if value < 80: continue
            # The local costmap is in base_link coordinates, whereas WHCA*
            # consumes integer cells in the map grid.  The old code merely
            # added local metres to map metres and published those rounded
            # metre values as cell indices, both omitting robot yaw and using
            # the wrong coordinate system.
            local_x = ((i % width) + 0.5) * grid.info.resolution + grid.info.origin.position.x
            local_y = ((i // width) + 0.5) * grid.info.resolution + grid.info.origin.position.y
            key = local_point_to_grid_cell(
                (self.pose.x, self.pose.y, self.pose.theta), (local_x, local_y),
                (self.map_origin_x, self.map_origin_y), self.map_resolution,
            )
            active.add(key)

        now = now_seconds(self)
        self.robot_poses = {
            robot_id: record for robot_id, record in self.robot_poses.items()
            if record[1] >= now
        }
        active = filter_unexpected_blockages(
            active, self.expected_cells,
            {self.pose_to_cell(record[0]) for record in self.robot_poses.values()},
            self.map_width, self.map_height,
            boundary_clearance_cells=self.boundary_clearance,
            robot_clearance_cells=self.robot_clearance,
        )
        for key in active:
            self.hits[key] = self.hits.get(key, 0) + 1
        self.hits = {key: count for key, count in self.hits.items() if key in active}
        cells = [key for key, count in self.hits.items() if count >= self.persistence]
        if not cells: return
        self.sequence += 1
        msg = BlockageObservation()
        msg.fleet_header = header(
            self, self.robot_id, self.session_id, self.sequence,
            self.observation_ttl_s)
        msg.cells = [GridCell(x=x, y=y, time_slot=0) for x, y in cells]
        msg.confidence, msg.source = 0.8, 'lidar_persistence'
        self.pub.publish(msg)


def main():
    rclpy.init(); node = BlockageDetectorNode()
    try: rclpy.spin(node)
    finally: node.destroy_node(); rclpy.shutdown()
