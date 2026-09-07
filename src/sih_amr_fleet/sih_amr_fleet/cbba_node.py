import math
import rclpy
from rclpy.node import Node
from sih_amr_interfaces.msg import (
    RobotState, Task, TaskAnnouncement, TaskAssignment, TaskConsensus, TaskExecutionStatus
)

from .common import FLEET_STATE_QOS, PROTOCOL_QOS, header, new_session_id, now_seconds, stamp_seconds


class CbbaNode(Node):
    """Consensus-Based Bundle Algorithm (CBBA) node with epoch protection and workload balance."""

    def __init__(self):
        super().__init__('cbba_node')
        self.robot_id = self.declare_parameter('robot_id', 'robot_1').value
        self.max_speed = self.declare_parameter('nominal_speed_mps', 0.45).value
        self.session_id = new_session_id()
        self.sequence = 0
        self.pose = None
        self.tasks = {}
        self.winners = {}  # task_id -> TaskConsensus

        self.consensus_pub = self.create_publisher(TaskConsensus, '/fleet/task_consensus', PROTOCOL_QOS)
        self.assignment_pub = self.create_publisher(TaskAssignment, 'task_assignment', FLEET_STATE_QOS)

        self.create_subscription(TaskAnnouncement, '/fleet/task_announcement', self.on_task, PROTOCOL_QOS)
        self.create_subscription(TaskConsensus, '/fleet/task_consensus', self.on_consensus, PROTOCOL_QOS)
        self.create_subscription(RobotState, 'state', self.on_state, FLEET_STATE_QOS)
        self.create_subscription(TaskExecutionStatus, '/fleet/task_execution_status', self.on_execution, FLEET_STATE_QOS)
        self.create_timer(0.5, self.run_round)
        self.get_logger().info(f'CbbaNode initialized for {self.robot_id}')

    def on_state(self, msg):
        self.pose = msg.pose

    def on_task(self, msg):
        self.tasks[msg.task.task_id] = msg.task

    def on_execution(self, msg):
        if msg.phase == TaskExecutionStatus.COMPLETED:
            self.tasks.pop(msg.task_id, None)
            self.winners.pop(msg.task_id, None)

    def on_consensus(self, msg):
        if msg.fleet_header.robot_id == self.robot_id:
            return
        now = now_seconds(self)
        if stamp_seconds(msg.lease_until) < now:
            return

        old = self.winners.get(msg.task_id)
        # Update winning claim if higher epoch or better bid at same epoch
        if (old is None or msg.assignment_epoch > old.assignment_epoch or
                (msg.assignment_epoch == old.assignment_epoch and
                 (msg.winning_bid, msg.winner_robot_id) < (old.winning_bid, old.winner_robot_id))):
            self.winners[msg.task_id] = msg

    def bid(self, task):
        if self.pose is None:
            return math.inf

        # Base travel time: AMR -> Pickup -> Dropoff
        travel_dist = (
            math.hypot(task.pickup.x - self.pose.x, task.pickup.y - self.pose.y) +
            math.hypot(task.dropoff.x - task.pickup.x, task.dropoff.y - task.pickup.y)
        )
        base_bid = travel_dist / max(self.max_speed, 0.1)

        # Workload penalty for tasks already claimed by this robot (enables fleet-wide bundle balancing)
        active_owned = sum(
            1 for t_id, c in self.winners.items()
            if c.winner_robot_id == self.robot_id and t_id != task.task_id
        )
        workload_penalty = active_owned * 120.0  # 120s penalty per existing assignment

        # Priority discount (higher priority = lower bid value / more attractive)
        priority_discount = (task.priority / 100.0) * 10.0

        return max(1.0, base_bid + workload_penalty - priority_discount)

    def run_round(self):
        if self.pose is None:
            return
        now = now_seconds(self)

        for task in list(self.tasks.values()):
            if stamp_seconds(task.expires_at) < now:
                self.tasks.pop(task.task_id, None)
                self.winners.pop(task.task_id, None)
                continue

            my_bid = self.bid(task)
            old = self.winners.get(task.task_id)

            if old is None:
                winner = self.robot_id
                winning_bid = my_bid
                epoch = 1
                winner_session = self.session_id
            else:
                lease_expired = stamp_seconds(old.lease_until) < now
                epoch = old.assignment_epoch + (1 if lease_expired else 0)
                if (my_bid, self.robot_id) < (old.winning_bid, old.winner_robot_id) or lease_expired:
                    winner = self.robot_id
                    winning_bid = my_bid
                    winner_session = self.session_id
                else:
                    winner = old.winner_robot_id
                    winning_bid = old.winning_bid
                    winner_session = old.winner_session_id

            self.sequence += 1
            consensus = TaskConsensus()
            consensus.fleet_header = header(self, self.robot_id, self.session_id, self.sequence, 2.5)
            consensus.task_id = task.task_id
            consensus.winner_robot_id = winner
            consensus.winner_session_id = winner_session
            consensus.winning_bid = float(winning_bid)
            consensus.assignment_epoch = epoch
            consensus.lease_until = consensus.fleet_header.valid_until
            consensus.event = TaskConsensus.CLAIM if winner == self.robot_id else TaskConsensus.BID

            self.winners[task.task_id] = consensus
            self.consensus_pub.publish(consensus)

            if winner == self.robot_id:
                assignment = TaskAssignment()
                assignment.fleet_header = consensus.fleet_header
                assignment.task = task
                assignment.owner_robot_id = self.robot_id
                assignment.owner_session_id = self.session_id
                assignment.assignment_epoch = epoch
                assignment.lease_until = consensus.lease_until
                assignment.active = True
                self.assignment_pub.publish(assignment)


def main(args=None):
    rclpy.init(args=args)
    node = CbbaNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
