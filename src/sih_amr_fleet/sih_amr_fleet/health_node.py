import rclpy
from rclpy.node import Node
from sih_amr_interfaces.msg import FleetHealth, PeerTrackArray, SafetyState
from std_msgs.msg import Float32

from .common import FLEET_STATE_QOS, header, new_session_id


class HealthNode(Node):
    """Publishes low-rate liveness/freshness for CBBA and corridor participants."""
    def __init__(self):
        super().__init__('health_node')
        self.robot_id = self.declare_parameter('robot_id', 'robot_1').value
        self.battery = self.declare_parameter('initial_battery_percent', 100.0).value
        self.session_id, self.sequence, self.safety, self.peer_state = new_session_id(), 0, SafetyState.CLEAR, 0
        self.pub = self.create_publisher(FleetHealth, '/fleet/health', FLEET_STATE_QOS)
        self.create_subscription(SafetyState, '/fleet/safety_state', self.on_safety, FLEET_STATE_QOS)
        self.create_subscription(PeerTrackArray, 'peer_tracks', self.on_tracks, FLEET_STATE_QOS)
        self.create_subscription(Float32, 'charging/battery_percent', self.on_battery, 10)
        self.create_timer(0.5, self.publish_health)

    def on_safety(self, msg):
        if msg.fleet_header.robot_id == self.robot_id: self.safety = msg.level
    def on_tracks(self, msg): self.peer_state = max([track.freshness for track in msg.tracks], default=0)
    def on_battery(self, msg): self.battery = max(0.0, min(100.0, msg.data))
    def publish_health(self):
        self.sequence += 1; msg = FleetHealth()
        msg.fleet_header = header(self, self.robot_id, self.session_id, self.sequence, 1.5)
        msg.battery_percent, msg.communication_state = self.battery, self.peer_state
        msg.task_feasible, msg.safety_ok, msg.active_task_id = True, self.safety != SafetyState.STOP, ''
        self.pub.publish(msg)


def main():
    rclpy.init(); node = HealthNode()
    try: rclpy.spin(node)
    finally: node.destroy_node(); rclpy.shutdown()
