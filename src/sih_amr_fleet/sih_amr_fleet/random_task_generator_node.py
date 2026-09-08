import random
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from sih_amr_interfaces.msg import FleetHeader, RobotState, Task, TaskAnnouncement, TaskExecutionStatus

from .common import FLEET_STATE_QOS, TASK_SOURCE_QOS, new_session_id, now_seconds
from .warehouse_tasks import aisle_points


class RandomTaskGeneratorNode(Node):
    """Generates randomized warehouse delivery tasks with pickup/dropoff in safe aisle corridors."""

    def __init__(self):
        super().__init__('random_task_generator_node')
        self.seed = self.declare_parameter('seed', 42).value
        self.min_interval_s = self.declare_parameter('min_interval_s', 12.0).value
        self.max_interval_s = self.declare_parameter('max_interval_s', 25.0).value
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
        self.ready_robots = set()
        self._reported_waiting = False

        self.pub = self.create_publisher(TaskAnnouncement, '/fleet/task_announcement', TASK_SOURCE_QOS)
        self.create_subscription(
            TaskExecutionStatus, '/fleet/task_execution_status', self.on_execution, FLEET_STATE_QOS
        )
        self.create_subscription(RobotState, '/fleet/robot_state', self.on_state, FLEET_STATE_QOS)

        self.next_spawn_time = now_seconds(self) + 3.0  # Initial delay
        self.create_timer(1.0, self.tick)
        self.get_logger().info(f'RandomTaskGeneratorNode initialized (seed={self.seed})')

    def on_execution(self, msg):
        if msg.phase == TaskExecutionStatus.COMPLETED:
            self.active_tasks.pop(msg.task_id, None)

    def on_state(self, msg):
        if msg.localization_valid and msg.fleet_header.robot_id in self.expected_robot_ids:
            self.ready_robots.add(msg.fleet_header.robot_id)

    def fleet_ready(self):
        # Valid state from every expected AMR is the safety gate.  DDS graph
        # discovery is intentionally diagnostic-only: a transient-local task
        # publisher safely repairs late CBBA discovery without suppressing the
        # workload forever when the graph count remains stale.
        return self.expected_robot_ids.issubset(self.ready_robots)

    def _publish_pending(self, now):
        for task_id, record in list(self.active_tasks.items()):
            announcement, last_published = record
            if now >= announcement.task.expires_at.sec + announcement.task.expires_at.nanosec * 1e-9:
                self.active_tasks.pop(task_id, None)
                self.get_logger().warning(f'Expired uncompleted task {task_id}')
            elif now - last_published >= self.reannounce_interval_s:
                self.pub.publish(announcement)
                self.active_tasks[task_id] = (announcement, now)

    def tick(self):
        now = now_seconds(self)
        self._publish_pending(now)
        if not self.fleet_ready():
            if not self._reported_waiting:
                missing = sorted(self.expected_robot_ids - self.ready_robots)
                self.get_logger().info(
                    f'Waiting for fleet readiness; missing state={missing}, '
                    f'CBBA subscriptions={self.pub.get_subscription_count()}/{len(self.expected_robot_ids)}')
                self._reported_waiting = True
            return
        if self._reported_waiting:
            self.get_logger().info(
                'Fleet readiness confirmed; task generation enabled '
                f'(CBBA subscriptions={self.pub.get_subscription_count()}/'
                f'{len(self.expected_robot_ids)})')
            self._reported_waiting = False
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

        self.pub.publish(announcement)
        self.active_tasks[task_id] = (announcement, now)
        self.get_logger().info(
            f'Announced {task_id}: pick ({pick_pt.x:.1f}, {pick_pt.y:.1f}) -> drop ({drop_pt.x:.1f}, {drop_pt.y:.1f})'
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
