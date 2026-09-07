import pathlib
import yaml
import rclpy
from geometry_msgs.msg import Pose2D
from rclpy.node import Node
from sih_amr_interfaces.msg import Task, TaskAnnouncement

from .common import PROTOCOL_QOS, header, new_session_id


class TaskScenarioNode(Node):
    """Publishes reproducible task arrivals; it is a scenario source, never an allocator."""
    def __init__(self):
        super().__init__('task_scenario_node')
        self.robot_id = 'scenario_source'; self.session_id, self.sequence = new_session_id(), 0
        file_name = self.declare_parameter('scenario_file', '').value
        self.tasks = yaml.safe_load(pathlib.Path(file_name).read_text()).get('tasks', []) if file_name else []
        self.started, self.sent = self.get_clock().now().nanoseconds * 1e-9, set()
        self.pub = self.create_publisher(TaskAnnouncement, '/fleet/task_announcement', PROTOCOL_QOS)
        self.create_timer(0.2, self.tick)

    def tick(self):
        elapsed = self.get_clock().now().nanoseconds * 1e-9 - self.started
        for spec in self.tasks:
            if spec['id'] in self.sent or spec.get('at_s', 0.0) > elapsed: continue
            self.sequence += 1; msg = TaskAnnouncement(); msg.fleet_header = header(self, self.robot_id, self.session_id, self.sequence, spec.get('ttl_s', 120.0))
            task = Task(); task.task_id, task.priority = spec['id'], spec.get('priority', 100)
            task.pickup = Pose2D(x=spec['pickup'][0], y=spec['pickup'][1], theta=0.0)
            task.dropoff = Pose2D(x=spec['dropoff'][0], y=spec['dropoff'][1], theta=0.0)
            task.pickup_wait_s = float(spec.get('pickup_wait_s', 0.0))
            task.dropoff_wait_s = float(spec.get('dropoff_wait_s', 0.0))
            task.created_at, task.expires_at = msg.fleet_header.sent_at, msg.fleet_header.valid_until
            msg.task = task; self.pub.publish(msg); self.sent.add(spec['id'])


def main():
    rclpy.init(); node = TaskScenarioNode()
    try: rclpy.spin(node)
    finally: node.destroy_node(); rclpy.shutdown()
