import json
import math
import pathlib
import yaml
import rclpy
from rclpy.node import Node
from sih_amr_interfaces.msg import (
    FleetHeader, FleetHealth, RobotState, Task, TaskAnnouncement, TaskAssignment,
    TaskConsensus, TaskExecutionStatus,
)
from geometry_msgs.msg import Pose2D
from std_msgs.msg import String

from .algorithms import float32_wire_value, freeze_auction_value, static_grid_path_distance
from .common import FLEET_STATE_QOS, POSE_QOS, PROTOCOL_QOS, TASK_SOURCE_QOS, header, new_session_id, now_seconds, stamp_seconds
from .map_geometry import map_geometry_from_data

UNAVAILABLE_BID = 1.0e9


class CbbaNode(Node):
    """CBBA bidding with unanimous pre-execution ownership commitment."""

    def __init__(self):
        super().__init__('cbba_node')
        self.robot_id = self.declare_parameter('robot_id', 'robot_1').value
        self.map_file = self.declare_parameter('map_file', '').value
        self.resolution = self.declare_parameter('grid_resolution_m', 0.5).value
        self.width = 90
        self.height = 120
        self.origin_x = -22.5
        self.origin_y = -30.0
        self.static_blocked = set()
        # Gazebo baseline speed.  This is deliberately an explicit fleet
        # parameter: physical AMRs must use their measured safe limit instead.
        self.max_speed = self.declare_parameter('nominal_speed_mps', 6.0).value
        # A small collection window allows every robot's initial bid to arrive
        # before the two-phase ownership decision can commit.
        self.consensus_settle_s = self.declare_parameter('consensus_settle_s', 2.0).value
        self.expected_robot_ids = set(self.declare_parameter(
            'expected_robot_ids', ['robot_1', 'robot_2', 'robot_3', 'robot_4']).value)
        self.session_id = new_session_id()
        self.sequence = 0
        self.pose = None
        self._received_local_state = False
        self._received_fleet_state = False
        self._received_task = False

        if self.map_file:
            self.load_map(self.map_file)
        self.received_task_ids = set()
        self.consensus_participants = {}
        self._reported_missing_consensus = {}
        self._reported_claim_problems = {}
        self._received_consensus_wire_sources = set()
        self.tasks = {}
        self.task_seen_at = {}
        self.winners = {}  # task_id -> latest locally selected TaskConsensus
        # Assignment is a two-phase operation.  bid_views contains each
        # participant's own bid; claim_views contains acknowledgements of the
        # winner independently derived from the complete bid set.  Merely
        # hearing from every participant is not sufficient: every latest
        # acknowledgement must name the same winner before any executor can be
        # started.
        self.bid_views = {}  # task_id -> source_robot_id -> TaskConsensus(BID)
        self.claim_views = {}  # task_id -> source_robot_id -> TaskConsensus(CLAIM)
        # A participant signs one bid and one winner tuple per auction epoch.
        # Recomputing either from a moving pose (or from asynchronously learned
        # busy state) made healthy participants continuously revoke one
        # another's exact-match quorum.
        self.own_bids = {}  # task_id -> (bid, epoch)
        self.own_claims = {}  # task_id -> (owner, owner_session, bid, epoch)
        self.committed_claims = {}  # task_id -> (owner, session, bid, epoch)
        self.peer_health_until = {}
        # A unanimous CLAIM quorum is the commit boundary.  Execution status
        # confirms and refreshes that decision, but is deliberately not the
        # first lock: by then two local executors could already have accepted.
        self.executing_tasks = {}  # task_id -> (owner_robot_id, last_seen)
        self.execution_confirmed_tasks = set()
        self.busy_robots = {}  # robot_id -> (task_id, last_seen)
        self._reported_execution_conflicts = set()
        self.consensus_pub = self.create_publisher(TaskConsensus, '/fleet/task_consensus', PROTOCOL_QOS)
        # This fleet's large process graph has repeatedly shown asymmetric
        # delivery on one shared custom-message topic: an audit subscriber can
        # receive a writer while one CBBA reader does not.  Keep the public
        # typed topic, and independently fan the same source-identified content
        # into one reliable standard-message inbox per participant.
        self.consensus_wire_pubs = {
            robot_id: self.create_publisher(
                String, f'/{robot_id}/consensus_inbox', PROTOCOL_QOS)
            for robot_id in sorted(self.expected_robot_ids)
            if robot_id != self.robot_id
        }
        self.assignment_pub = self.create_publisher(TaskAssignment, 'task_assignment', FLEET_STATE_QOS)
        self.task_receipt_pub = self.create_publisher(String, '/fleet/task_receipt', FLEET_STATE_QOS)

        self.create_subscription(TaskAnnouncement, '/fleet/task_announcement', self.on_task, TASK_SOURCE_QOS)
        self.create_subscription(
            TaskAnnouncement, f'/{self.robot_id}/task_announcement',
            self.on_task, TASK_SOURCE_QOS)
        self.create_subscription(String, f'/{self.robot_id}/task_inbox', self.on_local_task_wire, FLEET_STATE_QOS)
        self.create_subscription(TaskConsensus, '/fleet/task_consensus', self.on_consensus, PROTOCOL_QOS)
        self.create_subscription(
            String, f'/{self.robot_id}/consensus_inbox',
            self.on_local_consensus_wire, PROTOCOL_QOS)
        self.create_subscription(RobotState, 'state', self.on_state, POSE_QOS)
        # The local stream is the low-latency path.  This independently
        # filtered fleet stream is a deliberate resilience path: it is already
        # required by the executor/safety stack and prevents a local DDS
        # discovery delay from silently suppressing all bidding.
        self.create_subscription(RobotState, '/fleet/robot_state', self.on_fleet_state, FLEET_STATE_QOS)
        self.create_subscription(TaskExecutionStatus, '/fleet/task_execution_status', self.on_execution, FLEET_STATE_QOS)
        self.create_subscription(FleetHealth, '/fleet/health', self.on_health, FLEET_STATE_QOS)
        self.create_timer(0.5, self.run_round)
        self.get_logger().info(f'CbbaNode initialized for {self.robot_id}')

    def on_state(self, msg):
        self.pose = msg.pose
        if not self._received_local_state:
            self._received_local_state = True
            self.get_logger().info('CBBA received first local RobotState sample')

    def on_fleet_state(self, msg):
        if msg.fleet_header.robot_id != self.robot_id or not msg.localization_valid:
            return
        self.pose = msg.pose
        if not self._received_fleet_state:
            self._received_fleet_state = True
            self.get_logger().info('CBBA received first fleet RobotState sample')

    def on_task(self, msg):
        if not msg.task.task_id:
            self.get_logger().warning('Ignoring task announcement without a task ID')
            return
        self.tasks[msg.task.task_id] = msg.task
        self.task_seen_at.setdefault(msg.task.task_id, now_seconds(self))
        if not self._received_task:
            self._received_task = True
            self.get_logger().info(f'CBBA received first task announcement: {msg.task.task_id}')

    def publish_task_receipt(self, task_id, source_robot_id, source_session_id, source_seq):
        receipt_key = (source_session_id, int(source_seq), task_id)
        self.task_receipt_pub.publish(String(data=json.dumps({
            'event': 'task_receipt', 'robot_id': self.robot_id,
            'task_id': task_id, 'source_robot_id': source_robot_id,
            'source_session_id': source_session_id, 'source_seq': int(source_seq),
        }, separators=(',', ':'), sort_keys=True)))
        if receipt_key not in self.received_task_ids:
            self.received_task_ids.add(receipt_key)
            self.get_logger().info(f'CBBA task receipt: {task_id}')

    def on_local_task_wire(self, msg):
        """Decode the local standard-message fallback after strict validation."""
        try:
            data = json.loads(msg.data)
            pickup, dropoff = data['pickup'], data['dropoff']
            values = [*pickup, *dropoff, data['pickup_wait_s'], data['dropoff_wait_s']]
            created_at_ns = int(data['created_at_ns'])
            expires_at_ns = int(data['expires_at_ns'])
            if (not isinstance(data['task_id'], str) or not data['task_id'] or
                    len(pickup) != 3 or len(dropoff) != 3 or
                    not all(math.isfinite(float(value)) for value in values) or
                    created_at_ns < 0 or expires_at_ns <= self.get_clock().now().nanoseconds):
                raise ValueError('invalid task fields')
            task = Task()
            task.task_id = data['task_id']
            task.pickup = Pose2D(x=float(pickup[0]), y=float(pickup[1]), theta=float(pickup[2]))
            task.dropoff = Pose2D(x=float(dropoff[0]), y=float(dropoff[1]), theta=float(dropoff[2]))
            task.priority = int(data['priority'])
            task.pickup_wait_s = float(data['pickup_wait_s'])
            task.dropoff_wait_s = float(data['dropoff_wait_s'])
            task.created_at.sec = created_at_ns // 1_000_000_000
            task.created_at.nanosec = created_at_ns % 1_000_000_000
            task.expires_at.sec = expires_at_ns // 1_000_000_000
            task.expires_at.nanosec = expires_at_ns % 1_000_000_000
            announcement = TaskAnnouncement()
            announcement.task = task
            self.on_task(announcement)
            self.publish_task_receipt(
                task.task_id, data.get('source_robot_id', 'task_generator'),
                data.get('source_session_id', ''), data.get('source_seq', 0))
        except (KeyError, OverflowError, TypeError, ValueError, json.JSONDecodeError) as exc:
            self.get_logger().warning(f'Ignoring invalid local task payload: {exc}')

    def consensus_wire_payload(self, consensus):
        """Encode one consensus packet without changing its logical source."""
        sent_at_ns = (
            int(consensus.fleet_header.sent_at.sec) * 1_000_000_000 +
            int(consensus.fleet_header.sent_at.nanosec))
        valid_until_ns = (
            int(consensus.fleet_header.valid_until.sec) * 1_000_000_000 +
            int(consensus.fleet_header.valid_until.nanosec))
        lease_until_ns = (
            int(consensus.lease_until.sec) * 1_000_000_000 +
            int(consensus.lease_until.nanosec))
        return json.dumps({
            'source_robot_id': consensus.fleet_header.robot_id,
            'source_session_id': consensus.fleet_header.session_id,
            'source_seq': int(consensus.fleet_header.sequence_no),
            'sent_at_ns': sent_at_ns,
            'valid_until_ns': valid_until_ns,
            'task_id': consensus.task_id,
            'winner_robot_id': consensus.winner_robot_id,
            'winner_session_id': consensus.winner_session_id,
            'winning_bid': float(consensus.winning_bid),
            'assignment_epoch': int(consensus.assignment_epoch),
            'lease_until_ns': lease_until_ns,
            'event': int(consensus.event),
        }, separators=(',', ':'), sort_keys=True)

    def publish_consensus(self, consensus):
        """Publish the typed audit packet and redundant targeted copies."""
        self.consensus_pub.publish(consensus)
        if not consensus.winner_session_id:
            return
        wire = String(data=self.consensus_wire_payload(consensus))
        for publisher in self.consensus_wire_pubs.values():
            publisher.publish(wire)

    def on_local_consensus_wire(self, msg):
        """Validate and reconstruct a targeted consensus transport copy."""
        try:
            data = json.loads(msg.data)
            source = data['source_robot_id']
            source_session = data['source_session_id']
            task_id = data['task_id']
            winner = data['winner_robot_id']
            winner_session = data['winner_session_id']
            source_seq = int(data['source_seq'])
            assignment_epoch = int(data['assignment_epoch'])
            event = int(data['event'])
            winning_bid = float(data['winning_bid'])
            sent_at_ns = int(data['sent_at_ns'])
            valid_until_ns = int(data['valid_until_ns'])
            lease_until_ns = int(data['lease_until_ns'])
            now_ns = self.get_clock().now().nanoseconds
            if (source not in self.expected_robot_ids or source == self.robot_id or
                    not isinstance(source_session, str) or not source_session or
                    not isinstance(task_id, str) or not task_id or
                    winner not in self.expected_robot_ids or
                    not isinstance(winner_session, str) or not winner_session or
                    source_seq <= 0 or assignment_epoch <= 0 or
                    event not in (TaskConsensus.BID, TaskConsensus.CLAIM) or
                    not math.isfinite(winning_bid) or winning_bid < 0.0 or
                    sent_at_ns < 0 or valid_until_ns <= now_ns or
                    lease_until_ns <= now_ns):
                raise ValueError('invalid or expired consensus fields')

            consensus = TaskConsensus()
            consensus.fleet_header = FleetHeader()
            consensus.fleet_header.robot_id = source
            consensus.fleet_header.session_id = source_session
            consensus.fleet_header.sequence_no = source_seq
            consensus.fleet_header.sent_at.sec = sent_at_ns // 1_000_000_000
            consensus.fleet_header.sent_at.nanosec = sent_at_ns % 1_000_000_000
            consensus.fleet_header.valid_until.sec = valid_until_ns // 1_000_000_000
            consensus.fleet_header.valid_until.nanosec = valid_until_ns % 1_000_000_000
            consensus.task_id = task_id
            consensus.winner_robot_id = winner
            consensus.winner_session_id = winner_session
            consensus.winning_bid = winning_bid
            consensus.assignment_epoch = assignment_epoch
            consensus.lease_until.sec = lease_until_ns // 1_000_000_000
            consensus.lease_until.nanosec = lease_until_ns % 1_000_000_000
            consensus.event = event
            wire_source = (source, source_session)
            if wire_source not in self._received_consensus_wire_sources:
                self._received_consensus_wire_sources.add(wire_source)
                self.get_logger().info(
                    f'CBBA consensus inbox received first packet from {source}')
            self.on_consensus(consensus)
        except (KeyError, OverflowError, TypeError, ValueError, json.JSONDecodeError) as exc:
            self.get_logger().warning(f'Ignoring invalid consensus inbox payload: {exc}')

    def on_execution(self, msg):
        if msg.phase == TaskExecutionStatus.COMPLETED:
            existing = self.executing_tasks.get(msg.task_id)
            if existing is not None and existing[0] != msg.owner_robot_id:
                self.get_logger().error(
                    f'Ignoring conflicting completion for {msg.task_id}: '
                    f'committed={existing[0]}, observed={msg.owner_robot_id}')
                return
            committed = self.executing_tasks.pop(msg.task_id, None)
            if committed and self.busy_robots.get(committed[0], (None,))[0] == msg.task_id:
                self.busy_robots.pop(committed[0], None)
            self.forget_task(msg.task_id)
            self._reported_execution_conflicts = {
                pair for pair in self._reported_execution_conflicts if pair[0] != msg.task_id
            }
            return

        if msg.phase not in (
                TaskExecutionStatus.EN_ROUTE_PICKUP,
                TaskExecutionStatus.PICKUP_WAIT,
                TaskExecutionStatus.EN_ROUTE_DROPOFF,
                TaskExecutionStatus.DROPOFF_WAIT):
            return

        now = now_seconds(self)
        committed = self.executing_tasks.get(msg.task_id)
        if committed is not None and committed[0] != msg.owner_robot_id:
            conflict = (msg.task_id, committed[0], msg.owner_robot_id)
            if conflict not in self._reported_execution_conflicts:
                self._reported_execution_conflicts.add(conflict)
                self.get_logger().error(
                    f'Ignoring conflicting executor owner for {msg.task_id}: '
                    f'committed={committed[0]}, observed={msg.owner_robot_id}')
            return
        self.executing_tasks[msg.task_id] = (msg.owner_robot_id, now)
        self.execution_confirmed_tasks.add(msg.task_id)
        self.busy_robots[msg.owner_robot_id] = (msg.task_id, now)
        self.committed_claims.setdefault(
            msg.task_id, (msg.owner_robot_id, '', UNAVAILABLE_BID, 1))

    def on_health(self, msg):
        if msg.fleet_header.robot_id != self.robot_id:
            self.peer_health_until[msg.fleet_header.robot_id] = stamp_seconds(msg.fleet_header.valid_until)

    def on_consensus(self, msg):
        if msg.fleet_header.robot_id == self.robot_id:
            return
        now = now_seconds(self)
        if (stamp_seconds(msg.fleet_header.valid_until) < now or
                stamp_seconds(msg.lease_until) < now):
            return

        source = msg.fleet_header.robot_id
        if source not in self.expected_robot_ids:
            return

        committed = self.executing_tasks.get(msg.task_id)
        if committed is not None:
            # A unanimous pre-execution claim (or matching execution status)
            # is stronger than any later auction packet.
            if msg.event != TaskConsensus.CLAIM or msg.winner_robot_id != committed[0]:
                return

        self.consensus_participants.setdefault(msg.task_id, set()).add(source)
        if msg.event == TaskConsensus.BID:
            # BID means "my own bid", not "the best winner I have heard".
            # This makes every replica calculate the same total ordering from
            # the same complete set instead of propagating racing local minima.
            if (msg.winner_robot_id != source or
                    msg.winner_session_id != msg.fleet_header.session_id):
                return
            old = self.bid_views.setdefault(msg.task_id, {}).get(source)
            if (old is not None and
                    old.fleet_header.session_id == msg.fleet_header.session_id and
                    old.fleet_header.sequence_no >= msg.fleet_header.sequence_no):
                return
            self.bid_views.setdefault(msg.task_id, {})[source] = msg
        elif msg.event == TaskConsensus.CLAIM:
            if (msg.winner_robot_id not in self.expected_robot_ids or
                    not math.isfinite(msg.winning_bid)):
                return
            old = self.claim_views.setdefault(msg.task_id, {}).get(source)
            if (old is not None and
                    old.fleet_header.session_id == msg.fleet_header.session_id and
                    old.fleet_header.sequence_no >= msg.fleet_header.sequence_no):
                return
            self.claim_views.setdefault(msg.task_id, {})[source] = msg

    def forget_task(self, task_id):
        """Remove all auction and commitment state for a terminal task."""
        self.tasks.pop(task_id, None)
        self.task_seen_at.pop(task_id, None)
        self.winners.pop(task_id, None)
        self.bid_views.pop(task_id, None)
        self.claim_views.pop(task_id, None)
        self.own_bids.pop(task_id, None)
        self.own_claims.pop(task_id, None)
        self.committed_claims.pop(task_id, None)
        self.execution_confirmed_tasks.discard(task_id)
        self.consensus_participants.pop(task_id, None)
        self._reported_missing_consensus.pop(task_id, None)
        self._reported_claim_problems.pop(task_id, None)

    def consensus_message(self, task_id, winner, winner_session, winning_bid, epoch, event):
        self.sequence += 1
        msg = TaskConsensus()
        msg.fleet_header = header(self, self.robot_id, self.session_id, self.sequence, 2.5)
        msg.task_id = task_id
        msg.winner_robot_id = winner
        msg.winner_session_id = winner_session
        # Python message objects retain the input double until serialization,
        # while DDS transports this field as float32.  Canonicalize before the
        # message enters any local BID/CLAIM view so local and remote replicas
        # sign the exact same wire-representable value.
        msg.winning_bid = float32_wire_value(winning_bid)
        msg.assignment_epoch = epoch
        msg.lease_until = msg.fleet_header.valid_until
        msg.event = event
        return msg

    @staticmethod
    def claim_key(msg):
        return (
            msg.winner_robot_id, msg.winner_session_id,
            msg.winning_bid, msg.assignment_epoch)

    def load_map(self, filename):
        try:
            data = yaml.safe_load(pathlib.Path(filename).read_text())
            (self.resolution, self.width, self.height,
             self.origin_x, self.origin_y,
             self.static_blocked) = map_geometry_from_data(
                data, default_resolution=self.resolution,
                default_width=self.width, default_height=self.height,
                default_origin=(self.origin_x, self.origin_y))
            self.get_logger().info(
                f'Loaded static map in CbbaNode: {self.width}x{self.height}, {len(self.static_blocked)} blocked cells')
        except Exception as e:
            self.get_logger().error(f'Failed loading map in CbbaNode: {e}')

    def to_cell(self, pose):
        cx = round((pose.x - self.origin_x) / self.resolution)
        cy = round((pose.y - self.origin_y) / self.resolution)
        return (max(0, min(cx, self.width - 1)), max(0, min(cy, self.height - 1)))

    def bid(self, task):
        if self.pose is None:
            return UNAVAILABLE_BID

        busy = self.busy_robots.get(self.robot_id)
        if busy is not None and busy[0] != task.task_id:
            return UNAVAILABLE_BID

        # Base travel distance using 2D static grid A* (respects shelf rows & walls)
        if self.static_blocked:
            amr_cell = self.to_cell(self.pose)
            pickup_cell = self.to_cell(task.pickup)
            dropoff_cell = self.to_cell(task.dropoff)

            dist_to_pickup = static_grid_path_distance(
                amr_cell, pickup_cell, self.static_blocked,
                self.width, self.height, self.resolution)
            dist_pickup_to_dropoff = static_grid_path_distance(
                pickup_cell, dropoff_cell, self.static_blocked,
                self.width, self.height, self.resolution)
            travel_dist = dist_to_pickup + dist_pickup_to_dropoff
        else:
            # Fallback to Euclidean distance if map not loaded
            travel_dist = (
                math.hypot(task.pickup.x - self.pose.x, task.pickup.y - self.pose.y) +
                math.hypot(task.dropoff.x - task.pickup.x, task.dropoff.y - task.pickup.y)
            )

        base_bid = travel_dist / max(self.max_speed, 0.1)

        # Priority discount (higher priority = lower bid value / more attractive)
        priority_discount = (task.priority / 100.0) * 10.0
        calculated_bid = max(1.0, base_bid - priority_discount)
        return calculated_bid

    def run_round(self):
        if self.pose is None:
            return
        now = now_seconds(self)

        for task in list(self.tasks.values()):
            # expires_at is the deadline for accepting queued work, not a
            # licence to strand an executor that already owns the delivery.
            # A committed task remains leased until an explicit COMPLETED
            # status closes it.
            if (stamp_seconds(task.expires_at) < now and
                    task.task_id not in self.executing_tasks):
                self.forget_task(task.task_id)
                continue

            # Freeze the phase-1 value on first participation. At 4 m/s even a
            # few centimetres of pose change altered float32 winning_bid and
            # caused four otherwise identical CLAIMs to compare unequal.
            own_bid_value, own_bid_epoch = freeze_auction_value(
                self.own_bids, task.task_id, (self.bid(task), 1))

            committed = self.executing_tasks.get(task.task_id)
            if committed is not None:
                owner, winner_session, winning_bid, epoch = self.committed_claims.get(
                    task.task_id, (committed[0], '', UNAVAILABLE_BID, 1))

                # A replica can observe the unanimous quorum just before the
                # winner does.  Keep refreshing this participant's frozen
                # phase-1 BID as well as its phase-2 CLAIM until execution is
                # visible.  Otherwise the early replicas stop sending BID,
                # the winner's bid cache expires, and the only robot allowed
                # to assign can never reconstruct the quorum.
                if (task.task_id not in self.execution_confirmed_tasks and
                        task.task_id in self.own_bids):
                    bid_refresh = self.consensus_message(
                        task.task_id, self.robot_id, self.session_id,
                        own_bid_value, own_bid_epoch,
                        TaskConsensus.BID)
                    self.bid_views[task.task_id][self.robot_id] = bid_refresh
                    self.publish_consensus(bid_refresh)

                consensus = self.consensus_message(
                    task.task_id, owner, winner_session, winning_bid,
                    epoch, TaskConsensus.CLAIM)
                self.winners[task.task_id] = consensus
                self.publish_consensus(consensus)

                if owner == self.robot_id:
                    assignment = TaskAssignment()
                    assignment.fleet_header = consensus.fleet_header
                    assignment.task = task
                    assignment.owner_robot_id = self.robot_id
                    assignment.owner_session_id = self.session_id
                    assignment.assignment_epoch = epoch
                    assignment.lease_until = consensus.lease_until
                    assignment.active = True
                    self.assignment_pub.publish(assignment)
                continue

            # Phase 1: publish only this robot's bid.  There are no autonomous
            # lease/health takeovers during an open auction; those were the
            # source of epoch divergence in the 2026-09-08 GPU validation.
            bid_msg = self.consensus_message(
                task.task_id, self.robot_id, self.session_id, own_bid_value,
                own_bid_epoch, TaskConsensus.BID)
            if task.task_id not in self.bid_views.get(task.task_id, {}):
                self.get_logger().info(
                    f'[{self.robot_id}:CBBA] Decision: SUBMIT_BID for task {task.task_id}. '
                    f'Actor=CBBA:{self.robot_id}. Info: bid={own_bid_value:.2f}, epoch={own_bid_epoch}, '
                    f'pose=({self.pose.x:.2f}, {self.pose.y:.2f}), pickup=({task.pickup.x:.2f}, {task.pickup.y:.2f}), '
                    f'priority={task.priority}.'
                )
            self.bid_views.setdefault(task.task_id, {})[self.robot_id] = bid_msg
            self.publish_consensus(bid_msg)
            participants = self.consensus_participants.setdefault(task.task_id, set())
            participants.add(self.robot_id)

            settled = now - self.task_seen_at.get(task.task_id, now) >= self.consensus_settle_s
            views = self.bid_views.setdefault(task.task_id, {})
            for source, view in list(views.items()):
                if stamp_seconds(view.lease_until) < now:
                    views.pop(source, None)
            missing = self.expected_robot_ids - set(views)
            if settled and missing:
                missing_tuple = tuple(sorted(missing))
                if self._reported_missing_consensus.get(task.task_id) != missing_tuple:
                    self._reported_missing_consensus[task.task_id] = missing_tuple
                    self.get_logger().warning(
                        f'[{self.robot_id}:CBBA] Decision: WITHHOLD_ASSIGNMENT for {task.task_id}. '
                        f'Actor=CBBA:{self.robot_id}. Reason=missing CBBA participants={list(missing_tuple)}, '
                        f'received_bids={list(views.keys())}.')
            # Do not sign a phase-2 value before the configured collection
            # window has elapsed, even if a transient complete view arrives
            # early. Once signed below, that value is immutable in epoch 1.
            if missing or not settled:
                continue

            candidates = [
                view for view in views.values()
                if view.winning_bid < UNAVAILABLE_BID
            ]
            if not candidates:
                continue
            winner_view = min(
                candidates, key=lambda view: (view.winning_bid, view.winner_robot_id))
            claim_key = freeze_auction_value(self.own_claims, task.task_id, (
                winner_view.winner_robot_id,
                winner_view.winner_session_id,
                winner_view.winning_bid,
                winner_view.assignment_epoch,
            ))
            claim = self.consensus_message(
                task.task_id, *claim_key, TaskConsensus.CLAIM)
            if task.task_id not in self.claim_views.get(task.task_id, {}):
                self.get_logger().info(
                    f'[{self.robot_id}:CBBA] Decision: DERIVED_WINNER for task {task.task_id}. '
                    f'Actor=CBBA:{self.robot_id}. Winner={claim_key[0]}, Bid={claim_key[2]:.2f}, epoch={claim_key[3]}. '
                    f'All candidate bids: {[(v.winner_robot_id, round(v.winning_bid, 2)) for v in candidates]}.'
                )
            self.claim_views.setdefault(task.task_id, {})[self.robot_id] = claim
            self.winners[task.task_id] = claim
            self.publish_consensus(claim)

            # Phase 2: every expected participant must acknowledge exactly the
            # same winner/session/bid/epoch tuple with a fresh CLAIM.  This is
            # the pre-execution commit that the old participant-count gate
            # lacked.
            claims = self.claim_views.setdefault(task.task_id, {})
            for source, peer_claim in list(claims.items()):
                if stamp_seconds(peer_claim.lease_until) < now:
                    claims.pop(source, None)
            claim_key = self.claim_key(claim)
            missing_claims = self.expected_robot_ids - set(claims)
            mismatched_claims = {
                source for source in self.expected_robot_ids - missing_claims
                if self.claim_key(claims[source]) != claim_key
            }
            wrong_source_sessions = {
                source for source in self.expected_robot_ids - missing_claims
                if source in views and
                claims[source].fleet_header.session_id !=
                views[source].fleet_header.session_id
            }
            winner, winner_session, winning_bid, epoch = claim_key
            unanimous = (
                not missing_claims and not mismatched_claims and
                not wrong_source_sessions and winner in views and
                winner_session == views[winner].fleet_header.session_id
            )
            if not (settled and unanimous):
                if settled:
                    problem = (
                        tuple(sorted(missing_claims)),
                        tuple(sorted(mismatched_claims)),
                        tuple(sorted(wrong_source_sessions)),
                        winner if winner in views else 'missing-winner-bid',
                    )
                    if self._reported_claim_problems.get(task.task_id) != problem:
                        self._reported_claim_problems[task.task_id] = problem
                        claim_values = {
                            source: self.claim_key(peer_claim)
                            for source, peer_claim in claims.items()
                        }
                        self.get_logger().warning(
                            f'[{self.robot_id}:CBBA] Decision: WITHHOLD_COMMIT for {task.task_id}. '
                            f'Actor=CBBA:{self.robot_id}. Info: missing_claims={list(problem[0])}, '
                            f'mismatched_claims={list(problem[1])}, '
                            f'wrong_source_sessions={list(problem[2])}, '
                            f'winner_bid_source={problem[3]}, '
                            f'claim_values={claim_values}')
                continue

            owner_work = self.busy_robots.get(winner)
            if owner_work is not None and owner_work[0] != task.task_id:
                continue
            self.executing_tasks[task.task_id] = (winner, now)
            self.busy_robots[winner] = (task.task_id, now)
            self.committed_claims[task.task_id] = claim_key
            self.get_logger().info(
                f'[{self.robot_id}:CBBA] Decision: UNANIMOUS_COMMIT for task {task.task_id} -> Winner={winner}, '
                f'Bid={winning_bid:.2f}, epoch={epoch}. Actor=CBBA:{self.robot_id}. Quorum={len(self.expected_robot_ids)}/{len(self.expected_robot_ids)} verified.'
            )

            if winner == self.robot_id and winning_bid < UNAVAILABLE_BID:
                assignment = TaskAssignment()
                assignment.fleet_header = claim.fleet_header
                assignment.task = task
                assignment.owner_robot_id = self.robot_id
                assignment.owner_session_id = winner_session
                assignment.assignment_epoch = epoch
                assignment.lease_until = claim.lease_until
                assignment.active = True
                self.get_logger().info(
                    f'[{self.robot_id}:CBBA] Decision: DISPATCH_LOCAL_ASSIGNMENT for task {task.task_id}. '
                    f'Actor=CBBA:{self.robot_id}. Info: target={self.robot_id}, epoch={epoch}.'
                )
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
