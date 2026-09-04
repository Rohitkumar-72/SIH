"""Shared ROS 2 message, QoS, and geometry helpers."""
import math
import uuid
from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sih_amr_interfaces.msg import FleetHeader

POSE_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
FLEET_STATE_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL)
PROTOCOL_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST, depth=30, reliability=ReliabilityPolicy.RELIABLE)


def new_session_id():
    return str(uuid.uuid4())


def header(node, robot_id, session_id, sequence_no, validity_s=1.0):
    result = FleetHeader()
    result.robot_id = robot_id
    result.session_id = session_id
    result.sequence_no = sequence_no
    result.sent_at = node.get_clock().now().to_msg()
    result.valid_until = (node.get_clock().now() + Duration(seconds=validity_s)).to_msg()
    return result


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def now_seconds(node):
    return node.get_clock().now().nanoseconds * 1e-9


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def clamp(value, lower, upper):
    return max(lower, min(value, upper))


def distance(a, b):
    return math.hypot(a.x - b.x, a.y - b.y)
