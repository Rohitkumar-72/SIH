import rclpy
import math
from geometry_msgs.msg import Pose2D, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sih_amr_interfaces.msg import RobotState

from .common import FLEET_STATE_QOS, POSE_QOS, header, new_session_id, yaw_from_quaternion


class LocalizationNode(Node):
    """Normalizes simulator odometry into the fleet's validated RobotState contract."""
    def __init__(self):
        super().__init__('localization_node')
        self.robot_id = self.declare_parameter('robot_id', 'robot_1').value
        self.odom_origin_x = self.declare_parameter('odom_origin_x', 0.0).value
        self.odom_origin_y = self.declare_parameter('odom_origin_y', 0.0).value
        self.odom_origin_yaw = self.declare_parameter('odom_origin_yaw', 0.0).value
        self.session_id, self.sequence = new_session_id(), 0
        self.publisher = self.create_publisher(RobotState, '/fleet/robot_state', FLEET_STATE_QOS)
        # Retain the latest local pose too: planners are intentionally started
        # after the controller bring-up and must not wait for a DDS rediscovery
        # cycle before they can bid on a task.
        self.local_publisher = self.create_publisher(RobotState, 'state', FLEET_STATE_QOS)
        self.amcl_publisher = self.create_publisher(PoseWithCovarianceStamped, 'amcl_pose', POSE_QOS)
        self.create_subscription(Odometry, 'odom', self.on_odom, POSE_QOS)

    def on_odom(self, odom):
        self.sequence += 1
        local_x, local_y = odom.pose.pose.position.x, odom.pose.pose.position.y
        cosine, sine = math.cos(self.odom_origin_yaw), math.sin(self.odom_origin_yaw)
        msg = RobotState()
        msg.fleet_header = header(self, self.robot_id, self.session_id, self.sequence, 0.5)
        msg.pose = Pose2D(
            x=self.odom_origin_x + cosine * local_x - sine * local_y,
            y=self.odom_origin_y + sine * local_x + cosine * local_y,
            theta=self.odom_origin_yaw + yaw_from_quaternion(odom.pose.pose.orientation))
        msg.twist = odom.twist.twist
        msg.position_covariance_xy = [odom.pose.covariance[0], odom.pose.covariance[1],
                                      odom.pose.covariance[6], odom.pose.covariance[7]]
        msg.localization_valid = True
        self.publisher.publish(msg)
        self.local_publisher.publish(msg)
        # Gazebo odometry transformed into the warehouse map frame. This keeps
        # AMCL-compatible consumers usable before a physical AMCL integration.
        pose = PoseWithCovarianceStamped()
        pose.header.stamp = odom.header.stamp
        pose.header.frame_id = 'map'
        pose.pose.pose.position.x, pose.pose.pose.position.y = msg.pose.x, msg.pose.y
        pose.pose.pose.orientation.z = math.sin(msg.pose.theta / 2.0)
        pose.pose.pose.orientation.w = math.cos(msg.pose.theta / 2.0)
        pose.pose.covariance[0], pose.pose.covariance[7], pose.pose.covariance[35] = 0.02, 0.02, 0.05
        self.amcl_publisher.publish(pose)


def main():
    rclpy.init(); node = LocalizationNode()
    try: rclpy.spin(node)
    finally: node.destroy_node(); rclpy.shutdown()
