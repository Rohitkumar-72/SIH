import rclpy
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sih_amr_interfaces.msg import RobotState

from .common import FLEET_STATE_QOS, POSE_QOS, header, new_session_id, yaw_from_quaternion


class LocalizationNode(Node):
    """Normalizes simulator odometry into the fleet's validated RobotState contract."""
    def __init__(self):
        super().__init__('localization_node')
        self.robot_id = self.declare_parameter('robot_id', 'robot_1').value
        self.session_id, self.sequence = new_session_id(), 0
        self.publisher = self.create_publisher(RobotState, '/fleet/robot_state', FLEET_STATE_QOS)
        self.local_publisher = self.create_publisher(RobotState, 'state', POSE_QOS)
        self.create_subscription(Odometry, 'odom', self.on_odom, POSE_QOS)

    def on_odom(self, odom):
        self.sequence += 1
        msg = RobotState()
        msg.fleet_header = header(self, self.robot_id, self.session_id, self.sequence, 0.5)
        msg.pose = Pose2D(x=odom.pose.pose.position.x, y=odom.pose.pose.position.y,
                          theta=yaw_from_quaternion(odom.pose.pose.orientation))
        msg.twist = odom.twist.twist
        msg.position_covariance_xy = [odom.pose.covariance[0], odom.pose.covariance[1],
                                      odom.pose.covariance[6], odom.pose.covariance[7]]
        msg.localization_valid = True
        self.publisher.publish(msg)
        self.local_publisher.publish(msg)


def main():
    rclpy.init(); node = LocalizationNode()
    try: rclpy.spin(node)
    finally: node.destroy_node(); rclpy.shutdown()
