"""Verify that one simulated AMR exposes its baseline interfaces."""

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from .common import POSE_QOS


class InterfaceReadiness(Node):
    """Report readiness only after actual baseline data has arrived.

    This runs inside the robot namespace and avoids repeatedly launching ROS
    command-line discovery clients, which can become unreliable when four
    robots share the VM.  The controller's odometry publisher, the bridged
    scan publisher, and the existing Twist-to-TwistStamped adapter are all
    required.  Endpoint discovery alone can be true even when Gazebo has
    stopped delivering samples, so callbacks are used as the launch gate.
    """

    def __init__(self):
        super().__init__('interface_readiness')
        self._reported = False
        self._received_odom = False
        self._received_scan = False
        self.create_subscription(Odometry, 'odom', self._on_odom, POSE_QOS)
        self.create_subscription(LaserScan, 'scan', self._on_scan, POSE_QOS)
        self.create_timer(0.5, self._check)

    def _on_odom(self, _msg: Odometry) -> None:
        self._received_odom = True

    def _on_scan(self, _msg: LaserScan) -> None:
        self._received_scan = True

    def _check(self) -> None:
        if self._reported:
            return
        adapter_present = any(
            endpoint.node_name == 'twist_stamper'
            for endpoint in self.get_publishers_info_by_topic(
                'diffdrive_controller/cmd_vel'))
        if self._received_odom and self._received_scan and adapter_present:
            self._reported = True
            self.get_logger().info(
                'Interface readiness passed: odom and scan samples received; '
                'twist_stamper publisher detected.')


def main():
    rclpy.init()
    node = InterfaceReadiness()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
