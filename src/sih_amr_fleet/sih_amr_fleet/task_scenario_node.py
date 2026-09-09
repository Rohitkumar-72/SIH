import pathlib
import yaml
import rclpy
from geometry_msgs.msg import Pose2D
from rclpy.node import Node
from sih_amr_interfaces.msg import Task, TaskAnnouncement, TaskExecutionStatus

from .common import FLEET_STATE_QOS, TASK_SOURCE_QOS, header, new_session_id


class TaskScenarioNode(Node):
    """Publishes reproducible task arrivals; it is a scenario source, never an allocator."""
    def __init__(self):
        super().__init__('task_scenario_node')
        self.robot_id = 'scenario_source'; self.session_id, self.sequence = new_session_id(), 0
        file_name = self.declare_parameter('scenario_file', '').value
        self.tasks = yaml.safe_load(pathlib.Path(file_name).read_text()).get('tasks', []) if file_name else []
        self.started, self.sent = self.get_clock().now().nanoseconds * 1e-9, set()
        self.pending = {}
        self.executing = set()
        self.reannounce_interval_s = self.declare_parameter('reannounce_interval_s', 1.0).value
        self.robot_ids = self.declare_parameter(
            'robot_ids', ['robot_1', 'robot_2', 'robot_3', 'robot_4']).value
        self.pub = self.create_publisher(TaskAnnouncement, '/fleet/task_announcement', TASK_SOURCE_QOS)
        self.robot_task_pubs = {
            robot_id: self.create_publisher(
                TaskAnnouncement, f'/{robot_id}/task_announcement', TASK_SOURCE_QOS)
            for robot_id in self.robot_ids
        }
        self.create_subscription(TaskExecutionStatus, '/fleet/task_execution_status', self.on_execution, FLEET_STATE_QOS)
        self.create_timer(0.2, self.tick)

    def on_execution(self, msg):
        if msg.phase == TaskExecutionStatus.COMPLETED:
            self.executing.discard(msg.task_id)
            self.pending.pop(msg.task_id, None)
        elif msg.task_id in self.pending:
            self.executing.add(msg.task_id)

    def tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        elapsed = now - self.started
        for task_id, (msg, last_published) in list(self.pending.items()):
            expires_at = msg.task.expires_at.sec + msg.task.expires_at.nanosec * 1e-9
            if now >= expires_at:
                if task_id not in self.executing:
                    self.pending.pop(task_id, None)
            elif now - last_published >= self.reannounce_interval_s:
                self.publish_announcement(msg)
                self.pending[task_id] = (msg, now)
        for spec in self.tasks:
            if spec['id'] in self.sent or spec.get('at_s', 0.0) > elapsed: continue
            self.sequence += 1; msg = TaskAnnouncement(); msg.fleet_header = header(self, self.robot_id, self.session_id, self.sequence, spec.get('ttl_s', 120.0))
            task = Task(); task.task_id, task.priority = spec['id'], spec.get('priority', 100)
            task.pickup = Pose2D(x=spec['pickup'][0], y=spec['pickup'][1], theta=0.0)
            task.dropoff = Pose2D(x=spec['dropoff'][0], y=spec['dropoff'][1], theta=0.0)
            task.pickup_wait_s = float(spec.get('pickup_wait_s', 0.0))
            task.dropoff_wait_s = float(spec.get('dropoff_wait_s', 0.0))
            task.created_at, task.expires_at = msg.fleet_header.sent_at, msg.fleet_header.valid_until
            msg.task = task; self.publish_announcement(msg)
            self.pending[spec['id']] = (msg, now)
            self.sent.add(spec['id'])

    def publish_announcement(self, msg):
        self.pub.publish(msg)
        for publisher in self.robot_task_pubs.values():
            publisher.publish(msg)


def main():
    rclpy.init(); node = TaskScenarioNode()
    try: rclpy.spin(node)
    finally: node.destroy_node(); rclpy.shutdown()
