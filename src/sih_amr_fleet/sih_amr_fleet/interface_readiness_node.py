"""Verify that one simulated AMR exposes its baseline interfaces."""

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node


class InterfaceReadiness(Node):
    """Report readiness after discovering the required namespaced endpoints.

    This runs inside the robot namespace and avoids repeatedly launching ROS
    command-line discovery clients, which can become unreliable when four
    robots share the VM.  The controller's odometry publisher, the bridged
    scan publisher, and the existing Twist-to-TwistStamped adapter are all
    required.  Publisher availability is intentional: this headless VM does
    not always deliver sensor samples until a renderer consumer is present.
    """

    def __init__(self):
        super().__init__('interface_readiness')
        self._reported = False
        self.create_timer(0.5, self._check)

    def _check(self) -> None:
        if self._reported:
            return
        odom_present = bool(self.get_publishers_info_by_topic('odom'))
        scan_present = bool(self.get_publishers_info_by_topic('scan'))
        adapter_present = any(
            endpoint.node_name == 'twist_stamper'
            for endpoint in self.get_publishers_info_by_topic(
                'diffdrive_controller/cmd_vel'))
        if odom_present and scan_present and adapter_present:
            self._reported = True
            self.get_logger().info(
                'Interface readiness passed: odom and scan publishers detected; '
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
