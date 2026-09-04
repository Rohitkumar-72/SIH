import rclpy
from rclpy.node import Node
from sih_amr_interfaces.msg import RoutePlan, TrajectoryIntent

from .common import FLEET_STATE_QOS, PROTOCOL_QOS, header, new_session_id


class ReservationManagerNode(Node):
    """Converts a local WHCA* route into the fleet's expiring trajectory intent."""
    def __init__(self):
        super().__init__('reservation_manager_node')
        self.robot_id = self.declare_parameter('robot_id', 'robot_1').value
        self.dt = self.declare_parameter('time_slot_seconds', 0.5).value
        self.priority = self.declare_parameter('priority', 100).value
        self.session_id, self.sequence, self.current = new_session_id(), 0, None
        self.pub = self.create_publisher(TrajectoryIntent, '/fleet/trajectory_intent', PROTOCOL_QOS)
        self.create_subscription(RoutePlan, 'planned_route', self.on_route, FLEET_STATE_QOS)
        self.create_timer(0.75, self.publish_intent)

    def on_route(self, route):
        if route.route_feasible: self.current = route
    def publish_intent(self):
        if self.current is None: return
        self.sequence += 1; msg = TrajectoryIntent()
        msg.fleet_header = header(self, self.robot_id, self.session_id, self.sequence, 1.5)
        msg.plan_id, msg.t0, msg.dt_seconds, msg.reservations, msg.priority = self.current.plan_id, msg.fleet_header.sent_at, self.dt, self.current.cells, self.priority
        self.pub.publish(msg)


def main():
    rclpy.init(); node = ReservationManagerNode()
    try: rclpy.spin(node)
    finally: node.destroy_node(); rclpy.shutdown()
