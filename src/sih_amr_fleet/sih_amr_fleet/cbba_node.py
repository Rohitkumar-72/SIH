import math
import rclpy
from rclpy.node import Node
from sih_amr_interfaces.msg import RobotState, Task, TaskAnnouncement, TaskAssignment, TaskConsensus

from .common import FLEET_STATE_QOS, PROTOCOL_QOS, header, new_session_id, now_seconds, stamp_seconds


class CbbaNode(Node):
    """Bounded, one-task CBBA/CBAA implementation with lease and epoch protection."""
    def __init__(self):
        super().__init__('cbba_node')
        self.robot_id = self.declare_parameter('robot_id', 'robot_1').value
        self.max_speed = self.declare_parameter('nominal_speed_mps', 0.45).value
        self.session_id, self.sequence, self.pose, self.tasks, self.winners = new_session_id(), 0, None, {}, {}
        self.consensus_pub = self.create_publisher(TaskConsensus, '/fleet/task_consensus', PROTOCOL_QOS)
        self.assignment_pub = self.create_publisher(TaskAssignment, 'task_assignment', FLEET_STATE_QOS)
        self.create_subscription(TaskAnnouncement, '/fleet/task_announcement', self.on_task, PROTOCOL_QOS)
        self.create_subscription(TaskConsensus, '/fleet/task_consensus', self.on_consensus, PROTOCOL_QOS)
        self.create_subscription(RobotState, 'state', self.on_state, FLEET_STATE_QOS)
        self.create_timer(0.5, self.run_round)

    def on_state(self, msg): self.pose = msg.pose
    def on_task(self, msg): self.tasks[msg.task.task_id] = msg.task

    def on_consensus(self, msg):
        if msg.fleet_header.robot_id == self.robot_id or stamp_seconds(msg.lease_until) < now_seconds(self): return
        old = self.winners.get(msg.task_id)
        if (old is None or msg.assignment_epoch > old.assignment_epoch or
                (msg.assignment_epoch == old.assignment_epoch and
                 (msg.winning_bid, msg.winner_robot_id) < (old.winning_bid, old.winner_robot_id))):
            self.winners[msg.task_id] = msg

    def bid(self, task):
        if self.pose is None: return math.inf
        return (math.hypot(task.pickup.x - self.pose.x, task.pickup.y - self.pose.y) +
                math.hypot(task.dropoff.x - task.pickup.x, task.dropoff.y - task.pickup.y)) / self.max_speed

    def run_round(self):
        if self.pose is None: return
        now = now_seconds(self)
        for task in self.tasks.values():
            if stamp_seconds(task.expires_at) < now: continue
            bid = self.bid(task); old = self.winners.get(task.task_id)
            epoch = 1 if old is None else old.assignment_epoch + (1 if stamp_seconds(old.lease_until) < now else 0)
            winner = self.robot_id if old is None or (bid, self.robot_id) < (old.winning_bid, old.winner_robot_id) else old.winner_robot_id
            winning_bid = bid if winner == self.robot_id else old.winning_bid
            self.sequence += 1; consensus = TaskConsensus()
            consensus.fleet_header = header(self, self.robot_id, self.session_id, self.sequence, 2.0)
            consensus.task_id, consensus.winner_robot_id = task.task_id, winner
            consensus.winner_session_id, consensus.winning_bid, consensus.assignment_epoch = self.session_id if winner == self.robot_id else old.winner_session_id, winning_bid, epoch
            consensus.lease_until = header(self, self.robot_id, self.session_id, self.sequence, 2.0).valid_until
            consensus.event = TaskConsensus.CLAIM if winner == self.robot_id else TaskConsensus.BID
            self.winners[task.task_id] = consensus; self.consensus_pub.publish(consensus)
            if winner == self.robot_id:
                assignment = TaskAssignment(); assignment.fleet_header = consensus.fleet_header; assignment.task = task
                assignment.owner_robot_id, assignment.owner_session_id = self.robot_id, self.session_id
                assignment.assignment_epoch, assignment.lease_until, assignment.active = epoch, consensus.lease_until, True
                self.assignment_pub.publish(assignment)


def main():
    rclpy.init(); node = CbbaNode()
    try: rclpy.spin(node)
    finally: node.destroy_node(); rclpy.shutdown()
