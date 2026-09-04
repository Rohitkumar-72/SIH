import json
import rclpy
from rclpy.node import Node
from sih_amr_interfaces.msg import CorridorProtocol, FleetHealth, RobotState, SafetyState, TaskConsensus
from std_msgs.msg import String

from .common import FLEET_STATE_QOS, PROTOCOL_QOS


class DashboardBridgeNode(Node):
    """Read-only JSON telemetry bridge; it never publishes control or allocation input."""
    def __init__(self):
        super().__init__('dashboard_bridge_node')
        self.robots, self.health, self.events = {}, {}, []
        self.pub = self.create_publisher(String, '/fleet/dashboard_telemetry', FLEET_STATE_QOS)
        self.create_subscription(RobotState, '/fleet/robot_state', self.on_state, FLEET_STATE_QOS)
        self.create_subscription(FleetHealth, '/fleet/health', self.on_health, FLEET_STATE_QOS)
        self.create_subscription(CorridorProtocol, '/fleet/corridor_protocol', self.on_corridor, PROTOCOL_QOS)
        self.create_subscription(TaskConsensus, '/fleet/task_consensus', self.on_task, PROTOCOL_QOS)
        self.create_subscription(SafetyState, '/fleet/safety_state', self.on_safety, FLEET_STATE_QOS)
        self.create_timer(0.25, self.publish)

    def on_state(self, msg): self.robots[msg.fleet_header.robot_id] = {'x': msg.pose.x, 'y': msg.pose.y, 'theta': msg.pose.theta}
    def on_health(self, msg): self.health[msg.fleet_header.robot_id] = {'battery': msg.battery_percent, 'safe': msg.safety_ok}
    def add_event(self, kind, detail): self.events = (self.events + [{'type': kind, 'detail': detail}])[-30:]
    def on_corridor(self, msg): self.add_event('corridor', f'{msg.fleet_header.robot_id}:{msg.corridor_id}:{msg.event}')
    def on_task(self, msg): self.add_event('task', f'{msg.task_id}->{msg.winner_robot_id}@{msg.assignment_epoch}')
    def on_safety(self, msg): self.add_event('safety', f'{msg.fleet_header.robot_id}:{msg.level}:{msg.reason}')
    def publish(self): self.pub.publish(String(data=json.dumps({'robots': self.robots, 'health': self.health, 'events': self.events})))


def main():
    rclpy.init(); node = DashboardBridgeNode()
    try: rclpy.spin(node)
    finally: node.destroy_node(); rclpy.shutdown()
