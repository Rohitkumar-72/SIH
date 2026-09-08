import pathlib
import uuid
import math
import yaml
import rclpy
from rclpy.node import Node
from sih_amr_interfaces.msg import CorridorProtocol, FleetHealth, PeerTrack, PeerTrackArray, RobotState, RoutePlan
from std_msgs.msg import Bool, Float32, String

from .algorithms import approach_policy
from .common import FLEET_STATE_QOS, POSE_QOS, PROTOCOL_QOS, header, new_session_id, now_seconds, stamp_seconds


class CorridorMutexNode(Node):
    """Ricart-Agrawala corridor mutual exclusion node. Network grant never bypasses physical clearance."""

    def __init__(self):
        super().__init__('corridor_mutex_node')
        self.robot_id = self.declare_parameter('robot_id', 'robot_1').value
        self.resolution = self.declare_parameter('grid_resolution_m', 0.5).value
        map_file = self.declare_parameter('map_file', '').value

        self.session_id = new_session_id()
        self.sequence = 0
        self.clock = 0
        self.origin_x = 0.0
        self.origin_y = 0.0
        self.request = None
        self.deferred = {}  # (corridor_id, request_id) -> requesting_robot_id
        self.peers = {}     # robot_id -> lease_until_s
        self.peer_corridors = {}  # robot_id -> corridor currently announced ENTER
        self.suspect_corridors = set()
        self.entrance_clear = True
        self.corridors = {}
        self.approach_cells = {}
        self.armed_corridor = None
        self.in_approach = False
        self.communication_degraded = False
        self.approach_distance_m = self.declare_parameter('approach_distance_m', 2.0).value
        self.approach_speed_mps = self.declare_parameter('approach_speed_mps', 0.4).value

        if map_file:
            self.load_map(map_file)

        self.pub = self.create_publisher(CorridorProtocol, '/fleet/corridor_protocol', PROTOCOL_QOS)
        self.allowed_pub = self.create_publisher(Bool, 'corridor_motion_allowed', FLEET_STATE_QOS)
        self.speed_pub = self.create_publisher(Float32, 'corridor_speed_cap', FLEET_STATE_QOS)
        self.protected_pub = self.create_publisher(Bool, 'corridor_protected', FLEET_STATE_QOS)

        self.create_subscription(CorridorProtocol, '/fleet/corridor_protocol', self.on_protocol, PROTOCOL_QOS)
        self.create_subscription(FleetHealth, '/fleet/health', self.on_health, FLEET_STATE_QOS)
        self.create_subscription(PeerTrackArray, 'peer_tracks', self.on_tracks, FLEET_STATE_QOS)
        self.create_subscription(String, 'request_corridor', self.on_request, FLEET_STATE_QOS)
        self.create_subscription(RoutePlan, 'planned_route', self.on_route, FLEET_STATE_QOS)
        self.create_subscription(RobotState, 'state', self.on_state, POSE_QOS)
        self.create_subscription(Bool, 'entrance_clear', self.on_entrance_clear, FLEET_STATE_QOS)
        self.create_timer(0.1, self.tick)

    def load_map(self, map_file):
        try:
            data = yaml.safe_load(pathlib.Path(map_file).read_text())
            origin = data.get('origin', [0.0, 0.0])
            self.origin_x, self.origin_y = float(origin[0]), float(origin[1])
            self.resolution = float(data.get('resolution_m', self.resolution))

            # Only constrained narrow lanes and narrow junctions use the
            # Ricart-Agrawala mutex.  Spacious main junctions remain WHCA*
            # reservation resources, not one-robot-at-a-time bottlenecks.
            raw_corridors = data.get('mutex_resources', data.get('corridors', {}))
            for name, resource in raw_corridors.items():
                cells = resource.get('cells', []) if isinstance(resource, dict) else resource
                cell_set = set()
                if len(cells) == 2 and isinstance(cells[0], list) and isinstance(cells[1], list):
                    # Bounding endpoints / line segment
                    x1, y1 = cells[0]
                    x2, y2 = cells[1]
                    for x in range(min(x1, x2), max(x1, x2) + 1):
                        for y in range(min(y1, y2), max(y1, y2) + 1):
                            cell_set.add((x, y))
                else:
                    for c in cells:
                        cell_set.add(tuple(c))
                self.corridors[name] = cell_set
            if data.get('generate_narrow_lane_mutex_resources', False):
                # Generate one distinct resource for every shelf-row aisle.
                # This keeps unrelated parallel aisles independent while
                # making every physically narrow aisle one-robot-at-a-time.
                zones = data.get('shelf_layout', {}).get('y_zones', {})
                for zone, rows in zones.items():
                    for side, x_start, x_end in (
                        ('WEST', -7.5, -21.5), ('EAST', 7.5, 21.5),
                    ):
                        for index, (lower, upper) in enumerate(zip(rows, rows[1:]), start=1):
                            y = (float(lower) + float(upper)) / 2.0
                            cy = round((y - self.origin_y) / self.resolution)
                            left = round((min(x_start, x_end) - self.origin_x) / self.resolution)
                            right = round((max(x_start, x_end) - self.origin_x) / self.resolution)
                            self.corridors[f'NC-{zone.upper()}-{side}-{index:02d}'] = {
                                (x, cy) for x in range(left, right + 1)
                            }
                for name, y_start, y_end in (
                    ('NC-MIDDLE-CENTRE', -7.8, 7.8),
                    ('NC-NORTH-CENTRE', 12.2, 29.0),
                ):
                    cx = round((0.0 - self.origin_x) / self.resolution)
                    lower = round((y_start - self.origin_y) / self.resolution)
                    upper = round((y_end - self.origin_y) / self.resolution)
                    self.corridors[name] = {(cx, y) for y in range(lower, upper + 1)}
            radius_cells = max(1, round(self.approach_distance_m / self.resolution))
            for name, cells in self.corridors.items():
                self.approach_cells[name] = {
                    (x + dx, y + dy) for x, y in cells
                    for dx in range(-radius_cells, radius_cells + 1)
                    for dy in range(-radius_cells, radius_cells + 1)
                } - cells
            self.get_logger().info(f'Loaded {len(self.corridors)} corridors from {map_file}')
        except Exception as e:
            self.get_logger().error(f'Failed loading corridors from {map_file}: {e}')

    def on_entrance_clear(self, msg):
        self.entrance_clear = msg.data

    def on_health(self, msg):
        if msg.fleet_header.robot_id != self.robot_id:
            self.peers[msg.fleet_header.robot_id] = stamp_seconds(msg.fleet_header.valid_until)

    def on_tracks(self, msg):
        self.communication_degraded = any(track.freshness != PeerTrack.NORMAL for track in msg.tracks)

    def send(self, event, corridor_id, request_id, target=''):
        self.sequence += 1
        self.clock += 1
        msg = CorridorProtocol()
        msg.fleet_header = header(self, self.robot_id, self.session_id, self.sequence, 2.0)
        msg.corridor_id = corridor_id
        msg.request_id = request_id
        msg.target_robot_id = target
        msg.lamport_time = self.clock
        msg.occupancy_epoch = self.sequence
        msg.event = event
        self.pub.publish(msg)

    def begin_request(self, corridor_id):
        if self.request is not None or not corridor_id:
            return
        request_id = str(uuid.uuid4())
        self.clock += 1
        self.request = {
            'corridor': corridor_id,
            'id': request_id,
            'ts': self.clock,
            'grants': set(),
            'entered': False,
            'was_inside': False
        }
        self.send(CorridorProtocol.REQUEST, corridor_id, request_id)
        self.get_logger().info(f'[{self.robot_id}] Requested corridor mutex for {corridor_id} (req={request_id[:8]})')

    def on_request(self, msg):
        self.begin_request(msg.data)

    def on_route(self, route):
        if not route.route_feasible:
            return
        for cell in route.cells[1:]:
            corridor = next((name for name, cells in self.corridors.items() if (cell.x, cell.y) in cells), '')
            if corridor:
                self.armed_corridor = corridor
                return

    def on_state(self, state):
        cell = (
            round((state.pose.x - self.origin_x) / self.resolution),
            round((state.pose.y - self.origin_y) / self.resolution)
        )
        self.in_approach = bool(self.armed_corridor and cell in self.approach_cells.get(self.armed_corridor, set()))
        if self.in_approach and self.request is None:
            self.begin_request(self.armed_corridor)
        if not self.request or not self.request['entered']:
            return
        corridor_cells = self.corridors.get(self.request['corridor'], set())
        if cell in corridor_cells:
            self.request['was_inside'] = True
        elif self.request['was_inside']:
            # Exited the corridor
            self.get_logger().info(f"[{self.robot_id}] Exited corridor {self.request['corridor']}; releasing mutex.")
            self.send(CorridorProtocol.EXIT, self.request['corridor'], self.request['id'])
            for (corridor, request_id), peer in list(self.deferred.items()):
                if corridor == self.request['corridor']:
                    self.send(CorridorProtocol.GRANT, corridor, request_id, peer)
                    del self.deferred[(corridor, request_id)]
            self.request = None

    def on_protocol(self, msg):
        if msg.fleet_header.robot_id == self.robot_id:
            return
        self.clock = max(self.clock, msg.lamport_time) + 1

        if msg.event == CorridorProtocol.REQUEST:
            incoming = (msg.lamport_time, msg.fleet_header.robot_id)
            mine = (self.request['ts'], self.robot_id) if self.request and self.request['corridor'] == msg.corridor_id else None
            if mine is None or (not self.request.get('entered', False) and incoming < mine):
                self.send(CorridorProtocol.GRANT, msg.corridor_id, msg.request_id, msg.fleet_header.robot_id)
            else:
                self.deferred[(msg.corridor_id, msg.request_id)] = msg.fleet_header.robot_id

        elif msg.event == CorridorProtocol.GRANT and self.request:
            if msg.target_robot_id == self.robot_id and msg.request_id == self.request['id']:
                self.request['grants'].add(msg.fleet_header.robot_id)

        elif msg.event in (CorridorProtocol.EXIT, CorridorProtocol.RELEASE, CorridorProtocol.CANCEL):
            if self.peer_corridors.get(msg.fleet_header.robot_id) == msg.corridor_id:
                self.peer_corridors.pop(msg.fleet_header.robot_id, None)
            for (corridor, request_id), peer in list(self.deferred.items()):
                if corridor == msg.corridor_id:
                    self.send(CorridorProtocol.GRANT, corridor, request_id, peer)
                    del self.deferred[(corridor, request_id)]

        elif msg.event == CorridorProtocol.ENTER:
            # Network ownership is not physical clearance.  Retain this fact
            # if the owner later disappears, until an operator/sensor-backed
            # recovery procedure explicitly clears the corridor.
            self.peer_corridors[msg.fleet_header.robot_id] = msg.corridor_id

    def tick(self):
        if self.request is None:
            cap = approach_policy(self.in_approach, False, self.communication_degraded, self.approach_speed_mps)
            self.allowed_pub.publish(Bool(data=math.isinf(cap)))
            self.speed_pub.publish(Float32(data=float(cap if math.isfinite(cap) else 1e6)))
            self.protected_pub.publish(Bool(data=False))
            return

        now = now_seconds(self)
        for robot_id, corridor_id in list(self.peer_corridors.items()):
            if self.peers.get(robot_id, 0.0) < now:
                self.suspect_corridors.add(corridor_id)
        active_peers = {robot for robot, until in self.peers.items() if until >= now}
        suspected = self.request['corridor'] in self.suspect_corridors
        permitted = active_peers.issubset(self.request['grants']) and not suspected and not (self.in_approach and self.communication_degraded)

        if permitted and self.entrance_clear and not self.request['entered']:
            self.request['entered'] = True
            self.send(CorridorProtocol.ENTER, self.request['corridor'], self.request['id'])
            self.get_logger().info(f"[{self.robot_id}] Entered corridor {self.request['corridor']} with full grants.")

        allowed = permitted and self.entrance_clear
        cap = approach_policy(self.in_approach, allowed, self.communication_degraded, self.approach_speed_mps)
        self.allowed_pub.publish(Bool(data=allowed))
        self.speed_pub.publish(Float32(data=float(cap if math.isfinite(cap) else 1e6)))
        self.protected_pub.publish(Bool(data=self.request['entered']))


def main(args=None):
    rclpy.init(args=args)
    node = CorridorMutexNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
