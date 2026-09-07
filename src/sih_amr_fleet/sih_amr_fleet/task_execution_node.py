import math
import random
import rclpy
from geometry_msgs.msg import Pose2D
from rclpy.node import Node
from sih_amr_interfaces.msg import RobotState, TaskAssignment, TaskExecutionStatus

from .common import FLEET_STATE_QOS, header, new_session_id, now_seconds, stamp_seconds


class TaskExecutionNode(Node):
    """Tracks and executes the multi-stage delivery lifecycle for assigned tasks."""

    def __init__(self):
        super().__init__('task_execution_node')
        self.robot_id = self.declare_parameter('robot_id', 'robot_1').value
        self.arrival_tolerance_m = self.declare_parameter('arrival_tolerance_m', 0.45).value
        self.arrival_speed_threshold_mps = self.declare_parameter('arrival_speed_threshold_mps', 0.15).value

        self.session_id = new_session_id()
        self.sequence = 0
        self.current_assignment = None
        self.current_pose = None
        self.current_speed = 0.0
        self.phase = None
        self.wait_until = 0.0
        self.dwell_duration = 0.0
        self.completed_publish_count = 0

        self.status_pub = self.create_publisher(
            TaskExecutionStatus, '/fleet/task_execution_status', FLEET_STATE_QOS
        )
        self.create_subscription(TaskAssignment, 'task_assignment', self.on_assignment, FLEET_STATE_QOS)
        self.create_subscription(RobotState, 'state', self.on_state, FLEET_STATE_QOS)
        self.create_timer(0.1, self.tick)

    def on_assignment(self, msg):
        if msg.owner_robot_id != self.robot_id or not msg.active:
            if self.current_assignment and not msg.active and msg.task.task_id == self.current_assignment.task.task_id:
                self.current_assignment = None
                self.phase = None
            return

        now = now_seconds(self)
        if stamp_seconds(msg.lease_until) < now:
            return

        # New task assignment or epoch update
        if self.current_assignment is None or self.current_assignment.task.task_id != msg.task.task_id:
            self.current_assignment = msg
            self.phase = TaskExecutionStatus.EN_ROUTE_PICKUP
            self.wait_until = 0.0
            self.completed_publish_count = 0
            self.get_logger().info(
                f'[{self.robot_id}] Accepted new task {msg.task.task_id}: en route to pickup ({msg.task.pickup.x:.1f}, {msg.task.pickup.y:.1f})'
            )

    def on_state(self, msg):
        self.current_pose = msg.pose
        self.current_speed = math.hypot(msg.twist.linear.x, msg.twist.linear.y)

    def is_arrived(self, target_pose):
        if self.current_pose is None or target_pose is None:
            return False
        dist = math.hypot(target_pose.x - self.current_pose.x, target_pose.y - self.current_pose.y)
        return dist <= self.arrival_tolerance_m and self.current_speed <= self.arrival_speed_threshold_mps

    def tick(self):
        if self.current_assignment is None or self.phase is None or self.current_pose is None:
            return

        now = now_seconds(self)
        task = self.current_assignment.task
        status = TaskExecutionStatus()
        self.sequence += 1
        status.fleet_header = header(self, self.robot_id, self.session_id, self.sequence, 0.5)
        status.task_id = task.task_id
        status.owner_robot_id = self.robot_id
        status.wait_time_remaining_s = 0.0

        if self.phase == TaskExecutionStatus.EN_ROUTE_PICKUP:
            status.phase = TaskExecutionStatus.EN_ROUTE_PICKUP
            status.target = task.pickup
            if self.is_arrived(task.pickup):
                self.phase = TaskExecutionStatus.PICKUP_WAIT
                wait_time = task.pickup_wait_s if task.pickup_wait_s > 0.0 else random.uniform(2.0, 4.0)
                self.dwell_duration = wait_time
                self.wait_until = now + wait_time
                self.get_logger().info(f'[{self.robot_id}] Arrived at pickup for {task.task_id}; dwelling {wait_time:.1f}s')

        elif self.phase == TaskExecutionStatus.PICKUP_WAIT:
            status.phase = TaskExecutionStatus.PICKUP_WAIT
            status.target = task.pickup
            status.wait_time_remaining_s = max(0.0, float(self.wait_until - now))
            if now >= self.wait_until:
                self.phase = TaskExecutionStatus.EN_ROUTE_DROPOFF
                self.get_logger().info(
                    f'[{self.robot_id}] Pickup dwell complete for {task.task_id}; en route to dropoff ({task.dropoff.x:.1f}, {task.dropoff.y:.1f})'
                )

        elif self.phase == TaskExecutionStatus.EN_ROUTE_DROPOFF:
            status.phase = TaskExecutionStatus.EN_ROUTE_DROPOFF
            status.target = task.dropoff
            if self.is_arrived(task.dropoff):
                self.phase = TaskExecutionStatus.DROPOFF_WAIT
                wait_time = task.dropoff_wait_s if task.dropoff_wait_s > 0.0 else random.uniform(2.0, 4.0)
                self.dwell_duration = wait_time
                self.wait_until = now + wait_time
                self.get_logger().info(f'[{self.robot_id}] Arrived at dropoff for {task.task_id}; dwelling {wait_time:.1f}s')

        elif self.phase == TaskExecutionStatus.DROPOFF_WAIT:
            status.phase = TaskExecutionStatus.DROPOFF_WAIT
            status.target = task.dropoff
            status.wait_time_remaining_s = max(0.0, float(self.wait_until - now))
            if now >= self.wait_until:
                self.phase = TaskExecutionStatus.COMPLETED
                self.completed_publish_count = 0
                self.get_logger().info(f'[{self.robot_id}] Task {task.task_id} COMPLETED successfully!')

        elif self.phase == TaskExecutionStatus.COMPLETED:
            status.phase = TaskExecutionStatus.COMPLETED
            status.target = task.dropoff
            self.completed_publish_count += 1
            if self.completed_publish_count >= 10:  # Publish completed for 1.0s before resetting
                self.current_assignment = None
                self.phase = None

        self.status_pub.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = TaskExecutionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
