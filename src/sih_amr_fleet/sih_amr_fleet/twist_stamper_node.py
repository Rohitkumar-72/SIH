"""Adapt the fleet's Twist command contract to Gazebo's stamped controller input."""

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node


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
            TwistStamped, 'diffdrive_controller/cmd_vel', 10)
        self.create_subscription(Twist, 'cmd_vel', self.on_command, 10)

    def on_command(self, command):
        stamped = TwistStamped()
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.twist = command
        self.publisher.publish(stamped)


def main():
    rclpy.init()
    node = TwistStamper()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
