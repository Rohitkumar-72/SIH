import pathlib
import uuid
import yaml
import rclpy
from rclpy.node import Node
from sih_amr_interfaces.msg import CorridorProtocol, FleetHealth, RobotState, RoutePlan
from std_msgs.msg import Bool, String

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

        if map_file:
            self.load_map(map_file)

        self.pub = self.create_publisher(CorridorProtocol, '/fleet/corridor_protocol', PROTOCOL_QOS)
        self.allowed_pub = self.create_publisher(Bool, 'corridor_motion_allowed', FLEET_STATE_QOS)

        self.create_subscription(CorridorProtocol, '/fleet/corridor_protocol', self.on_protocol, PROTOCOL_QOS)
        self.create_subscription(FleetHealth, '/fleet/health', self.on_health, FLEET_STATE_QOS)
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

            raw_corridors = data.get('corridors', {})
            for name, cells in raw_corridors.items():
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
            self.get_logger().info(f'Loaded {len(self.corridors)} corridors from {map_file}')
        except Exception as e:
            self.get_logger().error(f'Failed loading corridors from {map_file}: {e}')

    def on_entrance_clear(self, msg):
        self.entrance_clear = msg.data

    def on_health(self, msg):
        if msg.fleet_header.robot_id != self.robot_id:
            self.peers[msg.fleet_header.robot_id] = stamp_seconds(msg.fleet_header.valid_until)

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
        if self.request is not None or not route.route_feasible:
            return
        for cell in route.cells[1:]:
            corridor = next((name for name, cells in self.corridors.items() if (cell.x, cell.y) in cells), '')
            if corridor:
                self.begin_request(corridor)
                return

    def on_state(self, state):
        if not self.request or not self.request['entered']:
            return
        cell = (
            round((state.pose.x - self.origin_x) / self.resolution),
            round((state.pose.y - self.origin_y) / self.resolution)
        )
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
            self.allowed_pub.publish(Bool(data=True))
            return

        now = now_seconds(self)
        for robot_id, corridor_id in list(self.peer_corridors.items()):
            if self.peers.get(robot_id, 0.0) < now:
                self.suspect_corridors.add(corridor_id)
        active_peers = {robot for robot, until in self.peers.items() if until >= now}
        suspected = self.request['corridor'] in self.suspect_corridors
        permitted = active_peers.issubset(self.request['grants']) and not suspected

        if permitted and self.entrance_clear and not self.request['entered']:
            self.request['entered'] = True
            self.send(CorridorProtocol.ENTER, self.request['corridor'], self.request['id'])
            self.get_logger().info(f"[{self.robot_id}] Entered corridor {self.request['corridor']} with full grants.")

        self.allowed_pub.publish(Bool(data=permitted and self.entrance_clear))


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
