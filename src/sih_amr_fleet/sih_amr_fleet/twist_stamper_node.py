"""Adapt the fleet's Twist command contract to Gazebo's stamped controller input."""

import time
import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from .algorithms import finite_command


class TwistStamper(Node):
    """Republish ``cmd_vel`` as ``diffdrive_controller/cmd_vel`` with a timestamp.

    The SIH fleet intentionally uses the standard ``geometry_msgs/Twist`` API.
    The TurtleBot 4 Gazebo controller uses ``TwistStamped``.  Keeping this small
    adapter in the project makes the simulator integration explicit and avoids
    giving fleet nodes a simulator-specific command type.
    """

    def __init__(self):
        super().__init__('twist_stamper')
        self.publisher = self.create_publisher(
            TwistStamped, 'diffdrive_controller/cmd_vel', QoSProfile(
                history=HistoryPolicy.KEEP_LAST, depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(Twist, 'cmd_vel', self.on_command, 10)
        self.last_command_at = None
        # Before the fleet starts, a controller still requires a finite
        # reference.  Keep it stopped instead of leaving reference interfaces
        # uninitialised (which previously produced NaN command warnings).
        # Publish often enough to seed a newly activated controller before its
        # first command update, without changing the finite-stop contract.
        self.create_timer(0.05, self.publish_idle_stop)

    def publish(self, command):
        stamped = TwistStamped()
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.twist = command
        self.publisher.publish(stamped)

    def on_command(self, command):
        if not finite_command(command.linear.x, command.angular.z):
            self.get_logger().warning('Rejected nonfinite cmd_vel before controller bridge')
            self.publish(Twist())
            return
        self.last_command_at = time.monotonic()
        self.publish(command)

    def publish_idle_stop(self):
        if self.last_command_at is None or time.monotonic() - self.last_command_at >= 0.2:
            self.publish(Twist())


def main():
    rclpy.init()
    node = TwistStamper()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
