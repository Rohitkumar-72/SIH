"""Decentralized leased dock selection and final-dock target publication."""
import pathlib
import uuid
import yaml

import rclpy
from geometry_msgs.msg import Pose2D
from rclpy.duration import Duration
from rclpy.node import Node
from sih_amr_interfaces.msg import DockProtocol, RobotState
from std_msgs.msg import Bool

from .algorithms import DockLeaseTable
from .common import FLEET_STATE_QOS, PROTOCOL_QOS, header, new_session_id, now_seconds, stamp_seconds


class DockingCoordinatorNode(Node):
    """Each AMR owns one deterministic replica; this is not a fleet manager."""
    def __init__(self):
        super().__init__('docking_coordinator_node')
        self.robot_id = self.declare_parameter('robot_id', 'robot_1').value
        self.map_file = self.declare_parameter('map_file', '').value
        self.lease_s = self.declare_parameter('dock_lease_s', 4.0).value
        self.arbitration_s = self.declare_parameter('dock_arbitration_s', 0.4).value
        self.final_distance_m = self.declare_parameter('dock_final_distance_m', 1.2).value
        self.need_dock = self.declare_parameter('needs_dock_on_start', False).value
        self.session_id, self.sequence, self.clock = new_session_id(), 0, 0
        self.table, self.anchors, self.pose = DockLeaseTable(), {}, None
        self.candidate = None
        self.pending_at = 0.0
        self.confirmed = False
        self.pub = self.create_publisher(DockProtocol, '/fleet/dock_protocol', PROTOCOL_QOS)
        self.target_pub = self.create_publisher(Pose2D, 'docking/target', FLEET_STATE_QOS)
        self.final_pub = self.create_publisher(Bool, 'docking/final_active', FLEET_STATE_QOS)
        self.create_subscription(DockProtocol, '/fleet/dock_protocol', self.on_protocol, PROTOCOL_QOS)
        self.create_subscription(Bool, 'docking/need_dock', self.on_need_dock, FLEET_STATE_QOS)
        self.create_subscription(RobotState, '/fleet/robot_state', self.on_state, FLEET_STATE_QOS)
        self.load_anchors()
        self.create_timer(0.1, self.tick)

    def load_anchors(self):
        if not self.map_file:
            return
        data = yaml.safe_load(pathlib.Path(self.map_file).read_text()) or {}
        for dock_id, spec in (data.get('anchors', {}) or {}).items():
            pose = spec.get('map_pose', [])
            if len(pose) >= 3:
                self.anchors[dock_id] = Pose2D(x=float(pose[0]), y=float(pose[1]), theta=float(pose[2]))

    def on_need_dock(self, msg):
        self.need_dock = bool(msg.data)
        if not self.need_dock:
            self.release('RELEASE')

    def on_state(self, msg):
        if msg.fleet_header.robot_id == self.robot_id:
            self.pose = msg.pose
            if not msg.localization_valid:
                self.need_dock = True

    def on_protocol(self, msg):
        now = now_seconds(self)
        if stamp_seconds(msg.lease_until) < now and msg.event not in (DockProtocol.RELEASE, DockProtocol.CANCEL):
            return
        event = ('REQUEST', 'CLAIM', 'RELEASE', 'CANCEL', 'CONFIRMED', 'BLOCKED')[msg.event]
        self.clock = max(self.clock, msg.lamport_time) + 1
        if event in ('REQUEST', 'CLAIM'):
            self.table.observe(msg.dock_id, msg.fleet_header.robot_id, msg.request_id,
                               msg.lamport_time, stamp_seconds(msg.lease_until), event, now)
        elif event in ('RELEASE', 'CANCEL'):
            self.table.observe(msg.dock_id, msg.fleet_header.robot_id, msg.request_id,
                               msg.lamport_time, 0.0, event, now)
        if (event == 'CONFIRMED' and msg.fleet_header.robot_id == self.robot_id
                and self.candidate and msg.dock_id == self.candidate['dock_id']):
            self.confirmed = True

    def send(self, event):
        if not self.candidate:
            return
        self.sequence += 1
        self.clock += 1
        now = now_seconds(self)
        msg = DockProtocol()
        msg.fleet_header = header(self, self.robot_id, self.session_id, self.sequence, self.lease_s)
        msg.dock_id, msg.request_id = self.candidate['dock_id'], self.candidate['request_id']
        msg.lamport_time, msg.lease_until = self.clock, (self.get_clock().now() + Duration(seconds=self.lease_s)).to_msg()
        msg.event = getattr(DockProtocol, event)
        self.pub.publish(msg)
        self.table.observe(msg.dock_id, self.robot_id, msg.request_id, msg.lamport_time,
                           now + self.lease_s, event, now)

    def release(self, event):
        if self.candidate:
            self.send(event)
        self.candidate, self.confirmed = None, False

    def tick(self):
        now = now_seconds(self)
        self.table.expire(now)
        if not self.need_dock:
            self.final_pub.publish(Bool(data=False)); return
        if self.candidate is None:
            dock_id = self.table.choose(self.anchors, self.robot_id, now)
            if dock_id is None:
                self.final_pub.publish(Bool(data=False)); return
            self.candidate = {'dock_id': dock_id, 'request_id': str(uuid.uuid4()), 'last_claim': 0.0}
            self.pending_at = now
            self.send('REQUEST')
            return
        winner = self.table.owner(self.candidate['dock_id'], now)
        if winner and winner != (self.robot_id, self.candidate['request_id']):
            self.release('CANCEL')
            return
        if now - self.pending_at >= self.arbitration_s and now - self.candidate['last_claim'] >= self.lease_s / 2.0:
            self.send('CLAIM'); self.candidate['last_claim'] = now
        target = self.anchors.get(self.candidate['dock_id'])
        if target:
            self.target_pub.publish(target)
            final = self.pose is not None and ((self.pose.x-target.x)**2 + (self.pose.y-target.y)**2) ** 0.5 <= self.final_distance_m
            self.final_pub.publish(Bool(data=final and not self.confirmed))


def main(args=None):
    rclpy.init(args=args); node = DockingCoordinatorNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()
