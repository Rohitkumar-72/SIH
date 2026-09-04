import math
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from sih_amr_interfaces.msg import RobotState, SafetyState
from std_msgs.msg import Bool

from .common import FLEET_STATE_QOS, header, new_session_id, now_seconds


class SafetySupervisorNode(Node):
    """The final local authority before Gazebo/robot cmd_vel; all failures stop."""
    def __init__(self):
        super().__init__('safety_supervisor_node')
        self.robot_id = self.declare_parameter('robot_id', 'robot_1').value
        self.deceleration = self.declare_parameter('braking_deceleration_mps2', 0.8).value
        self.margin = self.declare_parameter('braking_margin_m', 0.25).value
        self.session_id, self.sequence, self.pose_time = new_session_id(), 0, -math.inf
        self.nearest, self.candidate, self.estop = math.inf, Twist(), False
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', FLEET_STATE_QOS)
        self.state_pub = self.create_publisher(SafetyState, '/fleet/safety_state', FLEET_STATE_QOS)
        self.clear_pub = self.create_publisher(Bool, 'entrance_clear', FLEET_STATE_QOS)
        self.create_subscription(RobotState, 'state', self.on_state, FLEET_STATE_QOS)
        self.create_subscription(LaserScan, 'scan', self.on_scan, FLEET_STATE_QOS)
        self.create_subscription(Twist, 'cmd_vel_candidate', lambda msg: setattr(self, 'candidate', msg), FLEET_STATE_QOS)
        self.create_subscription(Bool, 'emergency_stop', lambda msg: setattr(self, 'estop', msg.data), FLEET_STATE_QOS)
        self.create_timer(0.025, self.enforce)

    def on_state(self, msg): self.pose_time = now_seconds(self) if msg.localization_valid else -math.inf
    def on_scan(self, scan): self.nearest = min((value for value in scan.ranges if math.isfinite(value) and value >= scan.range_min), default=math.inf)
    def enforce(self):
        age = now_seconds(self) - self.pose_time; speed = abs(self.candidate.linear.x)
        braking = speed * speed / (2.0 * self.deceleration) + self.margin
        stop = self.estop or age > 0.5 or self.nearest <= braking
        level = SafetyState.STOP if stop else (SafetyState.SLOW if self.nearest < braking + 0.5 else SafetyState.CLEAR)
        cmd = Twist() if stop else self.candidate
        if level == SafetyState.SLOW: cmd.linear.x *= 0.4
        self.cmd_pub.publish(cmd); self.sequence += 1
        state = SafetyState(); state.fleet_header = header(self, self.robot_id, self.session_id, self.sequence, 0.3)
        state.level, state.nearest_obstacle_m = level, self.nearest
        state.time_to_collision_s = self.nearest / max(speed, 0.01)
        state.reason = 'emergency stop' if self.estop else ('localization stale' if age > 0.5 else ('braking envelope' if stop else 'clear'))
        self.state_pub.publish(state); self.clear_pub.publish(Bool(data=not stop and self.nearest > braking))


def main():
    rclpy.init(); node = SafetySupervisorNode()
    try: rclpy.spin(node)
    finally: node.destroy_node(); rclpy.shutdown()
