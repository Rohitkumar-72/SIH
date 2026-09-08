import rclpy
from geometry_msgs.msg import Pose2D, Twist
from rclpy.node import Node
from sih_amr_interfaces.msg import PeerTrack, PeerTrackArray, RobotState

from .algorithms import ConstantVelocityTrack
from .common import FLEET_STATE_QOS, header, new_session_id, now_seconds, stamp_seconds


class PeerTrackerNode(Node):
    """Validates fleet state by session/sequence then predicts each peer locally."""
    def __init__(self):
        super().__init__('peer_tracker_node')
        self.robot_id = self.declare_parameter('robot_id', 'robot_1').value
        self.session_id, self.sequence, self.peers = new_session_id(), 0, {}
        self.last_publish = now_seconds(self)
        self.pub = self.create_publisher(PeerTrackArray, 'peer_tracks', FLEET_STATE_QOS)
        self.create_subscription(RobotState, '/fleet/robot_state', self.on_state, FLEET_STATE_QOS)
        self.create_timer(0.05, self.publish_tracks)

    def on_state(self, msg):
        sender = msg.fleet_header.robot_id
        if sender == self.robot_id or not msg.localization_valid: return
        now = now_seconds(self)
        if stamp_seconds(msg.fleet_header.valid_until) < now: return
        old = self.peers.get(sender)
        if old and old['session'] == msg.fleet_header.session_id and msg.fleet_header.sequence_no <= old['sequence']:
            return
        vx, vy = msg.twist.linear.x, msg.twist.linear.y
        if not old or old['session'] != msg.fleet_header.session_id:
            track = ConstantVelocityTrack(msg.pose.x, msg.pose.y, vx, vy)
        else:
            track = old['track']; track.update(msg.pose.x, msg.pose.y, vx, vy)
        self.peers[sender] = {'session': msg.fleet_header.session_id, 'sequence': msg.fleet_header.sequence_no,
                              'track': track, 'last_seen': now, 'theta': msg.pose.theta}

    def publish_tracks(self):
        now = now_seconds(self); output = PeerTrackArray(); self.sequence += 1
        dt = max(0.0, min(now - self.last_publish, 1.0)); self.last_publish = now
        output.fleet_header = header(self, self.robot_id, self.session_id, self.sequence, 0.3)
        for robot_id, peer in self.peers.items():
            # Prediction intentionally continues after loss; covariance grows
            # with the true elapsed time so downstream ORCA/corridor policy can
            # become more conservative instead of treating a missing peer as gone.
            age = now - peer['last_seen']; peer['track'].predict(dt)
            item = PeerTrack(); item.robot_id, item.session_id = robot_id, peer['session']
            item.pose = Pose2D(x=peer['track'].x, y=peer['track'].y, theta=peer['theta'])
            item.twist = Twist(); item.twist.linear.x, item.twist.linear.y = peer['track'].vx, peer['track'].vy
            item.covariance_trace = peer['track'].variance * 2.0
            item.freshness = PeerTrack.NORMAL if age <= 0.5 else (PeerTrack.COMM_DEGRADED if age <= 2.0 else PeerTrack.PEER_UNREACHABLE)
            output.tracks.append(item)
        self.pub.publish(output)


def main():
    rclpy.init(); node = PeerTrackerNode()
    try: rclpy.spin(node)
    finally: node.destroy_node(); rclpy.shutdown()
