import math
import rclpy
from geometry_msgs.msg import Pose2D, Twist
from rclpy.node import Node
from sih_amr_interfaces.msg import FleetEvent, RobotState, RoutePlan, SafetyState, TaskExecutionStatus
from std_msgs.msg import Bool, Float32

from .algorithms import reverse_recovery_allowed
from .common import FLEET_STATE_QOS, POSE_QOS, PROTOCOL_QOS, clamp, header, new_session_id, now_seconds


class PathFollowerNode(Node):
    """Simple simulator controller: follows the first forward WHCA* waypoint."""
    def __init__(self):
        super().__init__('path_follower_node')
        self.max_speed = self.declare_parameter('max_speed_mps', 6.0).value
        self.robot_id = self.declare_parameter('robot_id', 'robot_1').value
        self.kp = self.declare_parameter('linear_kp', 0.8).value
        self.recovery_reverse_m = self.declare_parameter('recovery_reverse_distance_m', 2.0).value
        self.recovery_speed_mps = self.declare_parameter('recovery_speed_mps', 0.20).value
        self.recovery_margin_m = self.declare_parameter('recovery_margin_m', 0.5).value
        self.final_docking_speed_mps = self.declare_parameter('final_docking_speed_mps', 0.15).value
        self.pose, self.route, self.clear, self.hold = None, None, True, False
        self.docking_target, self.docking_final, self.protected = None, False, False
        self.speed_cap, self.nearest = math.inf, math.inf
        self.recovery_state, self.recovery_started = 'IDLE', 0.0
        self.session_id, self.sequence = new_session_id(), 0
        self._received_local_state = False
        self.pub = self.create_publisher(Twist, 'cmd_vel_desired', FLEET_STATE_QOS)
        self.event_pub = self.create_publisher(FleetEvent, '/fleet/recovery_event', PROTOCOL_QOS)
        self.create_subscription(RobotState, '/fleet/robot_state', self.on_state, FLEET_STATE_QOS)
        self.create_subscription(RobotState, 'state', self.on_local_state, POSE_QOS)
        self.create_subscription(RoutePlan, 'planned_route', self.on_route, FLEET_STATE_QOS)
        self.create_subscription(Bool, 'corridor_motion_allowed', lambda msg: setattr(self, 'clear', msg.data), FLEET_STATE_QOS)
        self.create_subscription(Bool, 'corridor_protected', lambda msg: setattr(self, 'protected', msg.data), FLEET_STATE_QOS)
        self.create_subscription(Float32, 'corridor_speed_cap', lambda msg: setattr(self, 'speed_cap', msg.data), FLEET_STATE_QOS)
        self.create_subscription(Float32, 'nearest_obstacle_m', lambda msg: setattr(self, 'nearest', msg.data), POSE_QOS)
        self.create_subscription(Pose2D, 'docking/target', lambda msg: setattr(self, 'docking_target', msg), FLEET_STATE_QOS)
        self.create_subscription(Bool, 'docking/final_active', lambda msg: setattr(self, 'docking_final', msg.data), FLEET_STATE_QOS)
        self.create_subscription(SafetyState, '/fleet/safety_state', self.on_safety, FLEET_STATE_QOS)
        self.create_subscription(TaskExecutionStatus, '/fleet/task_execution_status', self.on_execution, FLEET_STATE_QOS)
        self.create_timer(0.05, self.control)

    def on_state(self, msg):
        if msg.fleet_header.robot_id == self.robot_id:
            self.pose = msg.pose
    def on_local_state(self, msg):
        if msg.localization_valid:
            self.pose = msg.pose
            if not self._received_local_state:
                self._received_local_state = True
                self.get_logger().info('Path follower received first local RobotState sample')
    def on_route(self, msg): self.route = msg if msg.route_feasible else None
    def on_execution(self, msg):
        if msg.owner_robot_id == self.robot_id:
            self.hold = msg.phase in (TaskExecutionStatus.PICKUP_WAIT, TaskExecutionStatus.DROPOFF_WAIT, TaskExecutionStatus.COMPLETED)
    def on_safety(self, msg):
        if msg.fleet_header.robot_id == self.robot_id and msg.level == SafetyState.STOP and self.recovery_state == 'IDLE':
            self.recovery_state, self.recovery_started = 'VERIFY', now_seconds(self)
            self.publish_event('recovery_stop', f'safety trigger: {msg.reason}')
    def publish_event(self, event_type, detail):
        self.sequence += 1
        msg = FleetEvent(); msg.fleet_header = header(self, self.robot_id, self.session_id, self.sequence, 3.0)
        msg.severity, msg.event_type = 2, event_type
        pose = 'unknown' if self.pose is None else f'map=({self.pose.x:.2f},{self.pose.y:.2f})'
        msg.detail = f'{detail}; {pose}; nearest={self.nearest:.2f}; protected={self.protected}'
        self.event_pub.publish(msg)
    def control(self):
        cmd = Twist()
        now = now_seconds(self)
        if self.recovery_state == 'VERIFY':
            if now - self.recovery_started >= 0.5:
                if reverse_recovery_allowed(self.nearest, self.recovery_reverse_m, self.recovery_margin_m,
                                           self.protected, self.docking_final):
                    self.recovery_state, self.recovery_started = 'REVERSING', now
                    self.publish_event('recovery_retreat_started', f'reverse={self.recovery_reverse_m:.1f}m')
                else:
                    self.recovery_state = 'IDLE'; self.publish_event('recovery_blocked', 'unsafe reverse rejected')
        elif self.recovery_state == 'REVERSING':
            if now - self.recovery_started < self.recovery_reverse_m / max(self.recovery_speed_mps, 0.01):
                cmd.linear.x = -self.recovery_speed_mps
            else:
                self.recovery_state = 'IDLE'; self.publish_event('recovery_retreat_complete', 'requesting normal replanning')
        elif self.pose is not None and self.docking_final and self.docking_target and self.clear:
            target = self.docking_target; dx, dy = target.x - self.pose.x, target.y - self.pose.y
            desired = math.atan2(dy, dx); angular_error = math.atan2(math.sin(desired-self.pose.theta), math.cos(desired-self.pose.theta))
            cmd.linear.x = min(self.final_docking_speed_mps, self.kp * math.hypot(dx, dy)) * max(0.0, math.cos(angular_error))
            cmd.angular.z = clamp(2.0 * angular_error, -0.8, 0.8)
        elif self.pose is not None and self.route and len(self.route.waypoints) > 1 and self.clear and not self.hold:
            target = self.route.waypoints[1]; dx, dy = target.x - self.pose.x, target.y - self.pose.y
            desired = math.atan2(dy, dx); angular_error = math.atan2(math.sin(desired-self.pose.theta), math.cos(desired-self.pose.theta))
            cmd.linear.x = clamp(self.kp * math.hypot(dx, dy), 0.0, min(self.max_speed, self.speed_cap)) * max(0.0, math.cos(angular_error))
            cmd.angular.z = clamp(2.0 * angular_error, -1.2, 1.2)
        self.pub.publish(cmd)


def main():
    rclpy.init(); node = PathFollowerNode()
    try: rclpy.spin(node)
    finally: node.destroy_node(); rclpy.shutdown()
