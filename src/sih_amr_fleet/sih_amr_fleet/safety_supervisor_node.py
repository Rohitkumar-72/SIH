import math
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from sih_amr_interfaces.msg import RobotState, SafetyState
from std_msgs.msg import Bool

from .algorithms import directional_scan_minimum, finite_command
from .common import FLEET_STATE_QOS, POSE_QOS, header, new_session_id, now_seconds


class SafetySupervisorNode(Node):
    """The final local authority before Gazebo/robot cmd_vel; all failures stop."""
    def __init__(self):
        super().__init__('safety_supervisor_node')
        self.robot_id = self.declare_parameter('robot_id', 'robot_1').value
        self.deceleration = self.declare_parameter('braking_deceleration_mps2', 0.8).value
        self.margin = self.declare_parameter('braking_margin_m', 0.25).value
        self.forward_half_angle = self.declare_parameter('braking_sector_half_angle_rad', math.pi / 3.0).value
        self.session_id, self.sequence, self.pose_time, self.scan_time = new_session_id(), 0, -math.inf, -math.inf
        self.nearest, self.candidate, self.estop = math.inf, Twist(), False
        self.scan_ranges, self.scan_angle_min, self.scan_angle_increment, self.scan_range_min = (), 0.0, 0.0, 0.0
        self._received_local_state = False
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', FLEET_STATE_QOS)
        self.state_pub = self.create_publisher(SafetyState, '/fleet/safety_state', FLEET_STATE_QOS)
        self.clear_pub = self.create_publisher(Bool, 'entrance_clear', FLEET_STATE_QOS)
        self.create_subscription(RobotState, '/fleet/robot_state', self.on_state, FLEET_STATE_QOS)
        self.create_subscription(RobotState, 'state', self.on_local_state, POSE_QOS)
        self.create_subscription(LaserScan, 'scan', self.on_scan, POSE_QOS)
        self.create_subscription(Twist, 'cmd_vel_candidate', lambda msg: setattr(self, 'candidate', msg), FLEET_STATE_QOS)
        self.create_subscription(Bool, 'emergency_stop', lambda msg: setattr(self, 'estop', msg.data), FLEET_STATE_QOS)
        self.create_timer(0.025, self.enforce)

    def on_state(self, msg):
        if msg.fleet_header.robot_id == self.robot_id:
            self.pose_time = now_seconds(self) if msg.localization_valid else -math.inf

    def on_local_state(self, msg):
        if msg.localization_valid:
            self.pose_time = now_seconds(self)
            if not self._received_local_state:
                self._received_local_state = True
                self.get_logger().info('Safety Supervisor received first local RobotState sample')

    def on_scan(self, scan):
        self.scan_ranges = tuple(scan.ranges)
        self.scan_angle_min = scan.angle_min
        self.scan_angle_increment = scan.angle_increment
        self.scan_range_min = scan.range_min
        self.scan_time = now_seconds(self)
        self.nearest = min((value for value in scan.ranges if math.isfinite(value) and value >= scan.range_min), default=math.inf)

    def directional_clearance(self, linear_x):
        if abs(linear_x) < 1e-4:
            return math.inf
        direction = 0.0 if linear_x > 0.0 else math.pi
        return directional_scan_minimum(
            self.scan_ranges, self.scan_angle_min, self.scan_angle_increment,
            direction, self.forward_half_angle, self.scan_range_min)

    def enforce(self):
        age = now_seconds(self) - self.pose_time
        scan_age = now_seconds(self) - self.scan_time
        speed = abs(self.candidate.linear.x)
        braking = speed * speed / (2.0 * self.deceleration) + self.margin
        invalid_command = not finite_command(self.candidate.linear.x, self.candidate.angular.z)
        travel_clearance = self.directional_clearance(self.candidate.linear.x)
        stop = (self.estop or invalid_command or age > 0.5 or scan_age > 0.5
                or travel_clearance <= braking)
        level = SafetyState.STOP if stop else (SafetyState.SLOW if travel_clearance < braking + 0.5 else SafetyState.CLEAR)
        cmd = Twist() if stop else self.candidate
        if level == SafetyState.SLOW: cmd.linear.x *= 0.4
        self.cmd_pub.publish(cmd); self.sequence += 1
        state = SafetyState(); state.fleet_header = header(self, self.robot_id, self.session_id, self.sequence, 0.3)
        state.level, state.nearest_obstacle_m = level, self.nearest
        state.time_to_collision_s = self.nearest / max(speed, 0.01)
        state.reason = ('emergency stop' if self.estop else
                        ('nonfinite command' if invalid_command else
                         ('localization stale' if age > 0.5 else
                          ('scan stale' if scan_age > 0.5 else
                           ('braking envelope' if stop else 'clear')))))
        self.state_pub.publish(state)
        self.clear_pub.publish(Bool(data=not stop and travel_clearance > braking))


def main():
    rclpy.init(); node = SafetySupervisorNode()
    try: rclpy.spin(node)
    finally: node.destroy_node(); rclpy.shutdown()
