import random
import uuid
import rclpy
from geometry_msgs.msg import Pose2D
from rclpy.duration import Duration
from rclpy.node import Node
from sih_amr_interfaces.msg import FleetHeader, Task, TaskAnnouncement, TaskConsensus, TaskExecutionStatus

from .common import PROTOCOL_QOS, FLEET_STATE_QOS, new_session_id, now_seconds


class RandomTaskGeneratorNode(Node):
    """Generates randomized warehouse delivery tasks with pickup/dropoff in safe aisle corridors."""

    # Safe aisle centerline waypoints (between shelf rows in west, center, and east blocks)
    SAFE_AISLE_POINTS = [
        # West block aisles
        (-17.8, -27.7), (-17.8, -21.6), (-17.8, -15.5), (-17.8, -4.0), (-17.8, 4.0), (-17.8, 15.5), (-17.8, 25.0),
        (-13.5, -27.7), (-13.5, -21.6), (-13.5, -15.5), (-13.5, -4.0), (-13.5, 4.0), (-13.5, 15.5), (-13.5, 25.0),
        # Center block aisles (north and middle)
        (0.0, -5.0), (0.0, 0.0), (0.0, 5.0), (0.0, 15.5), (0.0, 25.0),
        # East block aisles
        (13.5, -27.7), (13.5, -21.6), (13.5, -15.5), (13.5, -4.0), (13.5, 4.0), (13.5, 15.5), (13.5, 25.0),
        (17.8, -27.7), (17.8, -21.6), (17.8, -15.5), (17.8, -4.0), (17.8, 4.0), (17.8, 15.5), (17.8, 25.0),
    ]

    def __init__(self):
        super().__init__('random_task_generator_node')
        self.seed = self.declare_parameter('seed', 42).value
        self.min_interval_s = self.declare_parameter('min_interval_s', 12.0).value
        self.max_interval_s = self.declare_parameter('max_interval_s', 25.0).value
        self.task_ttl_s = self.declare_parameter('task_ttl_s', 300.0).value
        self.max_active_tasks = self.declare_parameter('max_active_tasks', 5).value

        random.seed(self.seed)
        self.session_id = new_session_id()
        self.sequence = 0
        self.task_count = 0
        self.active_tasks = set()

        self.pub = self.create_publisher(TaskAnnouncement, '/fleet/task_announcement', PROTOCOL_QOS)
        self.create_subscription(
            TaskExecutionStatus, '/fleet/task_execution_status', self.on_execution, FLEET_STATE_QOS
        )

        self.next_spawn_time = now_seconds(self) + 3.0  # Initial delay
        self.create_timer(1.0, self.tick)
        self.get_logger().info(f'RandomTaskGeneratorNode initialized (seed={self.seed})')

    def on_execution(self, msg):
        if msg.phase == TaskExecutionStatus.COMPLETED:
            self.active_tasks.discard(msg.task_id)

    def tick(self):
        now = now_seconds(self)
        if now < self.next_spawn_time or len(self.active_tasks) >= self.max_active_tasks:
            return

        self.task_count += 1
        task_id = f'rnd_task_{self.task_count:03d}'

        # Pick two distinct locations from safe points
        pick_pt, drop_pt = random.sample(self.SAFE_AISLE_POINTS, 2)
        p_wait = random.uniform(2.0, 5.0)
        d_wait = random.uniform(2.0, 5.0)
        priority = random.choice([50, 75, 100])

        task = Task()
        task.task_id = task_id
        task.pickup = Pose2D(x=pick_pt[0], y=pick_pt[1], theta=0.0)
        task.dropoff = Pose2D(x=drop_pt[0], y=drop_pt[1], theta=0.0)
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
        self.active_tasks.add(task_id)
        self.get_logger().info(
            f'Announced {task_id}: pick ({pick_pt[0]:.1f}, {pick_pt[1]:.1f}) -> drop ({drop_pt[0]:.1f}, {drop_pt[1]:.1f})'
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
