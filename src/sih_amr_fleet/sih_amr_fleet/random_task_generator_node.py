import json
import random
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from sih_amr_interfaces.msg import FleetHeader, RobotState, Task, TaskAnnouncement, TaskExecutionStatus
from std_msgs.msg import String

from .common import FLEET_STATE_QOS, POSE_QOS, TASK_SOURCE_QOS, new_session_id, now_seconds
from .warehouse_tasks import aisle_points


class RandomTaskGeneratorNode(Node):
    """Generates randomized warehouse delivery tasks with pickup/dropoff in safe aisle corridors."""

    def __init__(self):
        super().__init__('random_task_generator_node')
        self.seed = self.declare_parameter('seed', 42).value
        self.min_interval_s = self.declare_parameter('min_interval_s', 2.0).value
        self.max_interval_s = self.declare_parameter('max_interval_s', 5.0).value
        self.task_ttl_s = self.declare_parameter('task_ttl_s', 300.0).value
        self.max_active_tasks = self.declare_parameter('max_active_tasks', 5).value
        self.reannounce_interval_s = self.declare_parameter('reannounce_interval_s', 1.0).value
        self.expected_robot_ids = set(self.declare_parameter(
            'expected_robot_ids', ['robot_1', 'robot_2', 'robot_3', 'robot_4']).value)
        random.seed(self.seed)
        self.safe_aisle_points = aisle_points()
        if len(self.safe_aisle_points) < 2:
            raise RuntimeError('Warehouse lane network does not contain two task endpoints')
        self.session_id = new_session_id()
        self.sequence = 0
        self.task_count = 0
        self.active_tasks = {}
        self.executing_tasks = set()
        self.task_receipts = {}
        self.ready_robots = set()
        self._reported_waiting = False
        self._last_transport_counts = None

        # Exactly one process creates the workload.  The typed shared topic is
        # retained as the public/audit interface, while one standard-message
        # inbox per robot makes each delivery match independently observable.
        # The source never chooses an owner; every recipient still runs CBBA.
        self.pub = self.create_publisher(
            TaskAnnouncement, '/fleet/task_announcement', TASK_SOURCE_QOS)
        self.audit_pub = self.create_publisher(String, '/fleet/task_wire', FLEET_STATE_QOS)
        self.robot_task_pubs = {
            robot_id: self.create_publisher(
                String, f'/{robot_id}/task_inbox', FLEET_STATE_QOS)
            for robot_id in sorted(self.expected_robot_ids)
        }
        self.create_subscription(
            TaskExecutionStatus, '/fleet/task_execution_status', self.on_execution, FLEET_STATE_QOS
        )
        self.create_subscription(RobotState, '/fleet/robot_state', self.on_state, FLEET_STATE_QOS)
        self.create_subscription(String, '/fleet/task_receipt', self.on_task_receipt, FLEET_STATE_QOS)
        for robot_id in sorted(self.expected_robot_ids):
            self.create_subscription(
                TaskExecutionStatus, f'/{robot_id}/task_execution_status', self.on_execution,
                FLEET_STATE_QOS)
            self.create_subscription(
                RobotState, f'/{robot_id}/state', self.on_state, POSE_QOS)

        self.next_spawn_time = now_seconds(self) + 3.0  # Initial delay
        self.create_timer(1.0, self.tick)
        self.get_logger().info(f'RandomTaskGeneratorNode initialized (seed={self.seed})')

    def on_execution(self, msg):
        if msg.phase == TaskExecutionStatus.COMPLETED:
            self.executing_tasks.discard(msg.task_id)
            self.active_tasks.pop(msg.task_id, None)
            self.task_receipts.pop(msg.task_id, None)
        elif msg.task_id in self.active_tasks:
            self.executing_tasks.add(msg.task_id)

    def on_task_receipt(self, msg):
        """Track delivery evidence without taking part in allocation."""
        try:
            data = json.loads(msg.data)
            task_id = data['task_id']
            robot_id = data['robot_id']
            if (data.get('event') != 'task_receipt' or
                    robot_id not in self.expected_robot_ids or
                    task_id not in self.active_tasks or
                    data.get('source_session_id') != self.session_id):
                return
            announcement = self.active_tasks[task_id][0]
            if int(data.get('source_seq', -1)) != int(announcement.fleet_header.sequence_no):
                return
            receipts = self.task_receipts.setdefault(task_id, set())
            if robot_id not in receipts:
                receipts.add(robot_id)
                self.get_logger().info(
                    f'Task delivery confirmed for {task_id}: '
                    f'{len(receipts)}/{len(self.expected_robot_ids)} receipts')
        except (KeyError, TypeError, ValueError, json.JSONDecodeError):
            return

    def on_state(self, msg):
        if msg.localization_valid and msg.fleet_header.robot_id in self.expected_robot_ids:
            self.ready_robots.add(msg.fleet_header.robot_id)

    def fleet_ready(self):
        # Valid state from every expected AMR is the safety gate.  DDS graph
        # discovery is intentionally diagnostic-only: a transient-local task
        # publisher safely repairs late CBBA discovery without suppressing the
        # workload forever when the graph count remains stale.
        return self.expected_robot_ids.issubset(self.ready_robots)

    def task_subscription_count(self):
        return self.pub.get_subscription_count()

    def delivery_subscription_counts(self):
        """Return independently measured reader counts for robot inboxes."""
        return {
            robot_id: publisher.get_subscription_count()
            for robot_id, publisher in self.robot_task_pubs.items()
        }

    def task_transport_ready(self):
        """Do not emit a workload until every intended local reader matches.

        This is a transport-readiness gate only.  It neither ranks robots nor
        changes their independent CBBA allocation once the task is delivered.
        Receipts are still recorded as the delivery evidence.
        """
        return all(count >= 1 for count in self.delivery_subscription_counts().values()) or self.fleet_ready()

    def _publish_pending(self, now):
        for task_id, record in list(self.active_tasks.items()):
            announcement, last_published = record
            if now >= announcement.task.expires_at.sec + announcement.task.expires_at.nanosec * 1e-9:
                if task_id in self.executing_tasks:
                    # Announcement TTL gates acceptance.  Once execution has
                    # started the task remains active until COMPLETED, but no
                    # longer needs source replay.
                    continue
                self.active_tasks.pop(task_id, None)
                self.task_receipts.pop(task_id, None)
                self.get_logger().warning(f'Expired uncompleted task {task_id}')
            elif now - last_published >= self.reannounce_interval_s:
                self.publish_announcement(announcement)
                self.active_tasks[task_id] = (announcement, now)

    def wire_payload(self, announcement):
        """Stable, validated-at-receiver workload envelope for local fanout."""
        task = announcement.task
        created_at_ns = int(task.created_at.sec) * 1_000_000_000 + int(task.created_at.nanosec)
        expires_at_ns = int(task.expires_at.sec) * 1_000_000_000 + int(task.expires_at.nanosec)
        return json.dumps({
            'task_id': task.task_id,
            'pickup': [task.pickup.x, task.pickup.y, task.pickup.theta],
            'dropoff': [task.dropoff.x, task.dropoff.y, task.dropoff.theta],
            'priority': int(task.priority), 'pickup_wait_s': float(task.pickup_wait_s),
            'dropoff_wait_s': float(task.dropoff_wait_s),
            'created_at_ns': created_at_ns, 'expires_at_ns': expires_at_ns,
            'source_robot_id': announcement.fleet_header.robot_id,
            'source_session_id': announcement.fleet_header.session_id,
            'source_seq': int(announcement.fleet_header.sequence_no),
        }, separators=(',', ':'), sort_keys=True)

    def publish_announcement(self, announcement):
        wire = String(data=self.wire_payload(announcement))
        self.pub.publish(announcement)
        self.audit_pub.publish(wire)
        for publisher in self.robot_task_pubs.values():
            publisher.publish(wire)

    def tick(self):
        now = now_seconds(self)
        self._publish_pending(now)
        if not self.fleet_ready():
            missing = sorted(self.expected_robot_ids - self.ready_robots)
            self.get_logger().info(
                f'Waiting for fleet readiness; missing state={missing}, '
                f'CBBA subscriptions={self.task_subscription_count()}/{len(self.expected_robot_ids)}',
                throttle_duration_sec=5.0)
            self._reported_waiting = True
            return
        if self._reported_waiting:
            self.get_logger().info(
                'Fleet readiness confirmed; task generation enabled '
                f'(CBBA subscriptions={self.task_subscription_count()}/'
                f'{len(self.expected_robot_ids)})')
            self._reported_waiting = False
        counts = self.delivery_subscription_counts()
        if not self.task_transport_ready():
            if counts != self._last_transport_counts:
                missing = sorted(robot_id for robot_id, count in counts.items() if count < 1)
                self.get_logger().info(
                    f'Waiting for task inbox readers; missing={missing}, counts={counts}')
                self._last_transport_counts = counts
            return
        if counts != self._last_transport_counts:
            self.get_logger().info(
                f'Task inbox transport ready; counts={counts}')
            self._last_transport_counts = counts
        if now < self.next_spawn_time or len(self.active_tasks) >= self.max_active_tasks:
            return

        self.task_count += 1
        task_id = f'rnd_task_{self.task_count:03d}'

        # Pick two distinct locations from safe points
        pick_pt, drop_pt = random.sample(self.safe_aisle_points, 2)
        p_wait = random.uniform(2.0, 5.0)
        d_wait = random.uniform(2.0, 5.0)
        priority = random.choice([50, 75, 100])

        task = Task()
        task.task_id = task_id
        task.pickup = pick_pt
        task.dropoff = drop_pt
        task.priority = priority
        task.pickup_wait_s = float(p_wait)
        task.dropoff_wait_s = float(d_wait)
        task.created_at = self.get_clock().now().to_msg()
        task.expires_at = (self.get_clock().now() + Duration(seconds=self.task_ttl_s)).to_msg()

        self.sequence += 1
        announcement = TaskAnnouncement()
        hdr = FleetHeader()
        hdr.robot_id = 'task_generator'
        hdr.session_id = self.session_id
        hdr.sequence_no = self.sequence
        hdr.sent_at = self.get_clock().now().to_msg()
        hdr.valid_until = (self.get_clock().now() + Duration(seconds=self.task_ttl_s)).to_msg()
        announcement.fleet_header = hdr
        announcement.task = task

        self.publish_announcement(announcement)
        self.active_tasks[task_id] = (announcement, now)
        self.task_receipts[task_id] = set()
        self.get_logger().info(
            f'Announced {task_id}: pick ({pick_pt.x:.1f}, {pick_pt.y:.1f}) -> drop ({drop_pt.x:.1f}, {drop_pt.y:.1f}) '
            f'(inbox readers={self.delivery_subscription_counts()})'
        )

        self.next_spawn_time = now + random.uniform(self.min_interval_s, self.max_interval_s)


def main(args=None):
    rclpy.init(args=args)
    node = RandomTaskGeneratorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
