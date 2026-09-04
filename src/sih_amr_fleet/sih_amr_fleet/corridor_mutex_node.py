import uuid
import pathlib
import yaml
import rclpy
from rclpy.node import Node
from sih_amr_interfaces.msg import CorridorProtocol, FleetHealth, RobotState, RoutePlan
from std_msgs.msg import Bool, String

from .common import FLEET_STATE_QOS, PROTOCOL_QOS, header, new_session_id, now_seconds, stamp_seconds


class CorridorMutexNode(Node):
    """Ricart-Agrawala corridor mutex; network permission never implies clearance."""
    def __init__(self):
        super().__init__('corridor_mutex_node')
        self.robot_id = self.declare_parameter('robot_id', 'robot_1').value
        self.resolution = self.declare_parameter('grid_resolution_m', 0.5).value
        map_file = self.declare_parameter('map_file', '').value
        self.session_id, self.sequence, self.clock = new_session_id(), 0, 0
        self.request, self.deferred, self.peers, self.entrance_clear, self.corridors = None, {}, {}, True, {}
        if map_file:
            self.corridors = {name: {tuple(cell) for cell in cells} for name, cells in yaml.safe_load(pathlib.Path(map_file).read_text()).get('corridors', {}).items()}
        self.pub = self.create_publisher(CorridorProtocol, '/fleet/corridor_protocol', PROTOCOL_QOS)
        self.allowed_pub = self.create_publisher(Bool, 'corridor_motion_allowed', FLEET_STATE_QOS)
        self.create_subscription(CorridorProtocol, '/fleet/corridor_protocol', self.on_protocol, PROTOCOL_QOS)
        self.create_subscription(FleetHealth, '/fleet/health', self.on_health, FLEET_STATE_QOS)
        self.create_subscription(String, 'request_corridor', self.on_request, FLEET_STATE_QOS)
        self.create_subscription(RoutePlan, 'planned_route', self.on_route, FLEET_STATE_QOS)
        self.create_subscription(RobotState, 'state', self.on_state, FLEET_STATE_QOS)
        self.create_subscription(Bool, 'entrance_clear', lambda msg: setattr(self, 'entrance_clear', msg.data), FLEET_STATE_QOS)
        self.create_timer(0.1, self.tick)

    def on_health(self, msg):
        if msg.fleet_header.robot_id != self.robot_id: self.peers[msg.fleet_header.robot_id] = stamp_seconds(msg.fleet_header.valid_until)

    def send(self, event, corridor_id, request_id, target=''):
        self.sequence += 1; self.clock += 1; msg = CorridorProtocol()
        msg.fleet_header = header(self, self.robot_id, self.session_id, self.sequence, 2.0)
        msg.corridor_id, msg.request_id, msg.target_robot_id, msg.lamport_time, msg.occupancy_epoch, msg.event = corridor_id, request_id, target, self.clock, self.sequence, event
        self.pub.publish(msg)

    def begin_request(self, corridor_id):
        if self.request is not None or not corridor_id: return
        request_id = str(uuid.uuid4()); self.clock += 1
        self.request = {'corridor': corridor_id, 'id': request_id, 'ts': self.clock, 'grants': set(), 'entered': False, 'was_inside': False}
        self.send(CorridorProtocol.REQUEST, corridor_id, request_id)

    def on_request(self, msg): self.begin_request(msg.data)

    def on_route(self, route):
        if self.request is not None or not route.route_feasible: return
        for cell in route.cells[1:]:
            corridor = next((name for name, cells in self.corridors.items() if (cell.x, cell.y) in cells), '')
            if corridor:
                self.begin_request(corridor)
                return

    def on_state(self, state):
        if not self.request or not self.request['entered']: return
        cell = (round(state.pose.x / self.resolution), round(state.pose.y / self.resolution))
        if cell in self.corridors.get(self.request['corridor'], set()): self.request['was_inside'] = True
        elif self.request['was_inside']:
            self.send(CorridorProtocol.EXIT, self.request['corridor'], self.request['id'])
            for (corridor, request_id), peer in list(self.deferred.items()):
                if corridor == self.request['corridor']:
                    self.send(CorridorProtocol.GRANT, corridor, request_id, peer); del self.deferred[(corridor, request_id)]
            self.request = None

    def on_protocol(self, msg):
        if msg.fleet_header.robot_id == self.robot_id: return
        self.clock = max(self.clock, msg.lamport_time) + 1
        if msg.event == CorridorProtocol.REQUEST:
            incoming = (msg.lamport_time, msg.fleet_header.robot_id)
            mine = (self.request['ts'], self.robot_id) if self.request and self.request['corridor'] == msg.corridor_id else None
            if mine is None or (not self.request.get('entered', False) and incoming < mine):
                self.send(CorridorProtocol.GRANT, msg.corridor_id, msg.request_id, msg.fleet_header.robot_id)
            else:
                self.deferred[(msg.corridor_id, msg.request_id)] = msg.fleet_header.robot_id
        elif msg.event == CorridorProtocol.GRANT and self.request and msg.target_robot_id == self.robot_id and msg.request_id == self.request['id']:
            self.request['grants'].add(msg.fleet_header.robot_id)
        elif msg.event in (CorridorProtocol.EXIT, CorridorProtocol.RELEASE, CorridorProtocol.CANCEL):
            for (corridor, request_id), peer in list(self.deferred.items()):
                if corridor == msg.corridor_id:
                    self.send(CorridorProtocol.GRANT, corridor, request_id, peer); del self.deferred[(corridor, request_id)]

    def tick(self):
        if self.request is None:
            self.allowed_pub.publish(Bool(data=True)); return
        active = {robot for robot, until in self.peers.items() if until >= now_seconds(self)}
        permitted = active.issubset(self.request['grants'])
        if permitted and self.entrance_clear and not self.request['entered']:
            self.request['entered'] = True; self.send(CorridorProtocol.ENTER, self.request['corridor'], self.request['id'])
        self.allowed_pub.publish(Bool(data=permitted and self.entrance_clear))


def main():
    rclpy.init(); node = CorridorMutexNode()
    try: rclpy.spin(node)
    finally: node.destroy_node(); rclpy.shutdown()
