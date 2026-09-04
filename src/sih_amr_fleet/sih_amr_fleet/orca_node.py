import math
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sih_amr_interfaces.msg import PeerTrackArray, RobotState

from .algorithms import avoidance_velocity
from .common import FLEET_STATE_QOS


class OrcaNode(Node):
    """Produces an uncertainty-inflated reciprocal-velocity safety candidate."""
    def __init__(self):
        super().__init__('orca_node')
        self.radius = self.declare_parameter('robot_radius_m', 0.28).value
        self.max_speed = self.declare_parameter('max_speed_mps', 0.45).value
        self.pose, self.desired, self.tracks = None, Twist(), []
        self.pub = self.create_publisher(Twist, 'cmd_vel_candidate', FLEET_STATE_QOS)
        self.create_subscription(RobotState, 'state', lambda msg: setattr(self, 'pose', msg.pose), FLEET_STATE_QOS)
        self.create_subscription(Twist, 'cmd_vel_desired', lambda msg: setattr(self, 'desired', msg), FLEET_STATE_QOS)
        self.create_subscription(PeerTrackArray, 'peer_tracks', lambda msg: setattr(self, 'tracks', msg.tracks), FLEET_STATE_QOS)
        self.create_timer(0.05, self.control)

    def control(self):
        result = Twist()
        if self.pose is not None:
            direction = (math.cos(self.pose.theta), math.sin(self.pose.theta))
            preferred = (self.desired.linear.x * direction[0], self.desired.linear.x * direction[1])
            peers = [{'x': p.pose.x, 'y': p.pose.y, 'vx': p.twist.linear.x, 'vy': p.twist.linear.y,
                      'radius_inflation': 2.0 * math.sqrt(max(p.covariance_trace, 0.0))} for p in self.tracks]
            vx, vy = avoidance_velocity(preferred, (self.pose.x, self.pose.y), peers, self.radius, 1.5, self.max_speed)
            result.linear.x = vx * direction[0] + vy * direction[1]
            result.angular.z = self.desired.angular.z
        self.pub.publish(result)


def main():
    rclpy.init(); node = OrcaNode()
    try: rclpy.spin(node)
    finally: node.destroy_node(); rclpy.shutdown()
