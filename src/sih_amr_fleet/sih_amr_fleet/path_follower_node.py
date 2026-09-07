import math
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sih_amr_interfaces.msg import RobotState, RoutePlan, TaskExecutionStatus
from std_msgs.msg import Bool

from .common import FLEET_STATE_QOS, clamp


class PathFollowerNode(Node):
    """Simple simulator controller: follows the first forward WHCA* waypoint."""
    def __init__(self):
        super().__init__('path_follower_node')
        self.max_speed = self.declare_parameter('max_speed_mps', 0.45).value
        self.robot_id = self.declare_parameter('robot_id', 'robot_1').value
        self.kp = self.declare_parameter('linear_kp', 0.8).value
        self.pose, self.route, self.clear, self.hold = None, None, True, False
        self.pub = self.create_publisher(Twist, 'cmd_vel_desired', FLEET_STATE_QOS)
        self.create_subscription(RobotState, 'state', self.on_state, FLEET_STATE_QOS)
        self.create_subscription(RoutePlan, 'planned_route', self.on_route, FLEET_STATE_QOS)
        self.create_subscription(Bool, 'corridor_motion_allowed', lambda msg: setattr(self, 'clear', msg.data), FLEET_STATE_QOS)
        self.create_subscription(TaskExecutionStatus, '/fleet/task_execution_status', self.on_execution, FLEET_STATE_QOS)
        self.create_timer(0.05, self.control)

    def on_state(self, msg): self.pose = msg.pose
    def on_route(self, msg): self.route = msg if msg.route_feasible else None
    def on_execution(self, msg):
        if msg.owner_robot_id == self.robot_id:
            self.hold = msg.phase in (TaskExecutionStatus.PICKUP_WAIT, TaskExecutionStatus.DROPOFF_WAIT, TaskExecutionStatus.COMPLETED)
    def control(self):
        cmd = Twist()
        if self.pose is not None and self.route and len(self.route.waypoints) > 1 and self.clear and not self.hold:
            target = self.route.waypoints[1]; dx, dy = target.x - self.pose.x, target.y - self.pose.y
            desired = math.atan2(dy, dx); angular_error = math.atan2(math.sin(desired-self.pose.theta), math.cos(desired-self.pose.theta))
            cmd.linear.x = clamp(self.kp * math.hypot(dx, dy), 0.0, self.max_speed) * max(0.0, math.cos(angular_error))
            cmd.angular.z = clamp(2.0 * angular_error, -1.2, 1.2)
        self.pub.publish(cmd)


def main():
    rclpy.init(); node = PathFollowerNode()
    try: rclpy.spin(node)
    finally: node.destroy_node(); rclpy.shutdown()
