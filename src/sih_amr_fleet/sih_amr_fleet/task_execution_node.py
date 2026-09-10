import math
import random
import rclpy
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Bool
from rclpy.node import Node
from sih_amr_interfaces.msg import RobotState, TaskAssignment, TaskExecutionStatus

from .common import FLEET_STATE_QOS, POSE_QOS, header, new_session_id, now_seconds, stamp_seconds


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
        self._received_local_state = False

        self.is_low_battery = False
        self.status_pub = self.create_publisher(
            TaskExecutionStatus, '/fleet/task_execution_status', FLEET_STATE_QOS
        )
        self.local_status_pub = self.create_publisher(
            TaskExecutionStatus, 'task_execution_status', FLEET_STATE_QOS
        )
        self.need_dock_pub = self.create_publisher(
            Bool, 'docking/need_dock', FLEET_STATE_QOS
        )
        self.create_subscription(TaskAssignment, 'task_assignment', self.on_assignment, FLEET_STATE_QOS)
        self.create_subscription(RobotState, '/fleet/robot_state', self.on_state, FLEET_STATE_QOS)
        self.create_subscription(RobotState, 'state', self.on_local_state, POSE_QOS)
        self.create_subscription(Bool, 'charging/low_battery', self.on_low_battery, 10)
        self.create_timer(0.1, self.tick)

    def on_low_battery(self, msg):
        was_low = self.is_low_battery
        self.is_low_battery = bool(msg.data)
        if self.is_low_battery and not was_low:
            self.get_logger().warning(
                f'[{self.robot_id}:TaskExecutor] LOW BATTERY WARNING (<20%). Will seek dock after current task.'
            )
            if self.current_assignment is None:
                self.need_dock_pub.publish(Bool(data=True))
        elif not self.is_low_battery and was_low:
            self.get_logger().info(
                f'[{self.robot_id}:TaskExecutor] Battery recharged. Resuming regular fleet operations.'
            )
            self.need_dock_pub.publish(Bool(data=False))

    def on_assignment(self, msg):
        if msg.owner_robot_id != self.robot_id or not msg.active:
            if self.current_assignment and not msg.active and msg.task.task_id == self.current_assignment.task.task_id:
                self.get_logger().info(
                    f'[{self.robot_id}:TaskExecutor] Decision: WITHDRAW assignment for task {msg.task.task_id}. '
                    f'Actor=TaskExecutor:{self.robot_id}. Reason=active flag cleared by consensus.'
                )
                self.current_assignment = None
                self.phase = None
            return

        now = now_seconds(self)
        if stamp_seconds(msg.lease_until) < now:
            self.get_logger().warning(
                f'[{self.robot_id}:TaskExecutor] ERROR: Ignoring expired assignment for task {msg.task.task_id}. '
                f'Lease={stamp_seconds(msg.lease_until):.2f}s < now={now:.2f}s.'
            )
            return

        # A robot executes one delivery at a time. Keep its existing task
        # until completion (or an explicit inactive withdrawal); otherwise a
        # burst of consensus updates can make it oscillate between tasks.
        if (self.current_assignment is not None and
                self.current_assignment.task.task_id != msg.task.task_id):
            self.get_logger().warning(
                f'[{self.robot_id}:TaskExecutor] Decision: REJECT secondary assignment {msg.task.task_id}. '
                f'Actor=TaskExecutor:{self.robot_id}. Reason=already executing {self.current_assignment.task.task_id} (phase={self.phase}).'
            )
            return

        # New task assignment or a lease/epoch refresh of the current task.
        if self.current_assignment is None:
            if self.is_low_battery:
                self.get_logger().warning(
                    f'[{self.robot_id}:TaskExecutor] REJECT assignment {msg.task.task_id}: Low Battery (<20%). Seeking dock.'
                )
                self.need_dock_pub.publish(Bool(data=True))
                return
            self.current_assignment = msg
            self.phase = TaskExecutionStatus.EN_ROUTE_PICKUP
            self.wait_until = 0.0
            self.completed_publish_count = 0
            pose_str = f'({self.current_pose.x:.2f}, {self.current_pose.y:.2f})' if self.current_pose else 'unknown'
            self.get_logger().info(
                f'[{self.robot_id}:TaskExecutor] Decision: ACCEPT new task {msg.task.task_id}. '
                f'Actor=TaskExecutor:{self.robot_id}. Info: epoch={msg.assignment_epoch}, '
                f'robot_pose={pose_str}, pickup=({msg.task.pickup.x:.2f}, {msg.task.pickup.y:.2f}), '
                f'dropoff=({msg.task.dropoff.x:.2f}, {msg.task.dropoff.y:.2f}), priority={msg.task.priority}. '
                f'Action=Transition to EN_ROUTE_PICKUP.'
            )
        else:
            self.current_assignment = msg

    def on_state(self, msg):
        if msg.fleet_header.robot_id != self.robot_id:
            return
        self.current_pose = msg.pose
        self.current_speed = math.hypot(msg.twist.linear.x, msg.twist.linear.y)

    def on_local_state(self, msg):
        if not msg.localization_valid:
            return
        self.current_pose = msg.pose
        self.current_speed = math.hypot(msg.twist.linear.x, msg.twist.linear.y)
        if not self._received_local_state:
            self._received_local_state = True
            self.get_logger().info(
                f'[{self.robot_id}:TaskExecutor] Received first valid local RobotState sample at '
                f'({msg.pose.x:.2f}, {msg.pose.y:.2f}, theta={msg.pose.theta:.2f}).'
            )

    def is_arrived(self, target_pose):
        if self.current_pose is None or target_pose is None:
            return False
        dist = math.hypot(target_pose.x - self.current_pose.x, target_pose.y - self.current_pose.y)
        return dist <= self.arrival_tolerance_m and self.current_speed <= self.arrival_speed_threshold_mps

    def tick(self):
        if self.current_assignment is None or self.phase is None:
            return

        if self.current_pose is None:
            self.get_logger().warning(
                f'[{self.robot_id}:TaskExecutor] ERROR: No valid robot pose received while executing '
                f'task {self.current_assignment.task.task_id} in phase {self.phase}.',
                throttle_duration_sec=5.0
            )
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
            dist = math.hypot(task.pickup.x - self.current_pose.x, task.pickup.y - self.current_pose.y)
            if self.is_arrived(task.pickup):
                self.phase = TaskExecutionStatus.PICKUP_WAIT
                wait_time = task.pickup_wait_s if task.pickup_wait_s > 0.0 else random.uniform(2.0, 4.0)
                self.dwell_duration = wait_time
                self.wait_until = now + wait_time
                self.get_logger().info(
                    f'[{self.robot_id}:TaskExecutor] Decision: ARRIVED at pickup for task {task.task_id}. '
                    f'Actor=TaskExecutor:{self.robot_id}. Info: pose=({self.current_pose.x:.2f}, {self.current_pose.y:.2f}), '
                    f'pickup=({task.pickup.x:.2f}, {task.pickup.y:.2f}), dist={dist:.3f}m <= {self.arrival_tolerance_m}m, '
                    f'speed={self.current_speed:.3f}mps <= {self.arrival_speed_threshold_mps}mps. '
                    f'Action=Transition to PICKUP_WAIT, dwelling {wait_time:.1f}s.'
                )

        elif self.phase == TaskExecutionStatus.PICKUP_WAIT:
            status.phase = TaskExecutionStatus.PICKUP_WAIT
            status.target = task.pickup
            status.wait_time_remaining_s = max(0.0, float(self.wait_until - now))
            if now >= self.wait_until:
                self.phase = TaskExecutionStatus.EN_ROUTE_DROPOFF
                self.get_logger().info(
                    f'[{self.robot_id}:TaskExecutor] Decision: PICKUP_DWELL_COMPLETE for task {task.task_id}. '
                    f'Actor=TaskExecutor:{self.robot_id}. Info: dwell={self.dwell_duration:.1f}s finished. '
                    f'Action=Transition to EN_ROUTE_DROPOFF towards ({task.dropoff.x:.2f}, {task.dropoff.y:.2f}).'
                )

        elif self.phase == TaskExecutionStatus.EN_ROUTE_DROPOFF:
            status.phase = TaskExecutionStatus.EN_ROUTE_DROPOFF
            status.target = task.dropoff
            dist = math.hypot(task.dropoff.x - self.current_pose.x, task.dropoff.y - self.current_pose.y)
            if self.is_arrived(task.dropoff):
                self.phase = TaskExecutionStatus.DROPOFF_WAIT
                wait_time = task.dropoff_wait_s if task.dropoff_wait_s > 0.0 else random.uniform(2.0, 4.0)
                self.dwell_duration = wait_time
                self.wait_until = now + wait_time
                self.get_logger().info(
                    f'[{self.robot_id}:TaskExecutor] Decision: ARRIVED at dropoff for task {task.task_id}. '
                    f'Actor=TaskExecutor:{self.robot_id}. Info: pose=({self.current_pose.x:.2f}, {self.current_pose.y:.2f}), '
                    f'dropoff=({task.dropoff.x:.2f}, {task.dropoff.y:.2f}), dist={dist:.3f}m <= {self.arrival_tolerance_m}m, '
                    f'speed={self.current_speed:.3f}mps <= {self.arrival_speed_threshold_mps}mps. '
                    f'Action=Transition to DROPOFF_WAIT, dwelling {wait_time:.1f}s.'
                )

        elif self.phase == TaskExecutionStatus.DROPOFF_WAIT:
            status.phase = TaskExecutionStatus.DROPOFF_WAIT
            status.target = task.dropoff
            status.wait_time_remaining_s = max(0.0, float(self.wait_until - now))
            if now >= self.wait_until:
                self.phase = TaskExecutionStatus.COMPLETED
                self.completed_publish_count = 0
                self.get_logger().info(
                    f'[{self.robot_id}:TaskExecutor] Decision: DROPOFF_DWELL_COMPLETE for task {task.task_id}. '
                    f'Actor=TaskExecutor:{self.robot_id}. Action=Transition to COMPLETED! Task successfully fulfilled.'
                )

        elif self.phase == TaskExecutionStatus.COMPLETED:
            status.phase = TaskExecutionStatus.COMPLETED
            status.target = task.dropoff
            self.completed_publish_count += 1
            if self.completed_publish_count >= 10:  # Publish completed for 1.0s before resetting
                self.get_logger().info(
                    f'[{self.robot_id}:TaskExecutor] Decision: RESET executor after publishing completion for task {task.task_id}. '
                    f'Actor=TaskExecutor:{self.robot_id}. Robot is now IDLE and available for next task.'
                )
                self.current_assignment = None
                self.phase = None
                if self.is_low_battery:
                    self.get_logger().warning(
                        f'[{self.robot_id}:TaskExecutor] Low battery detected after task completion. Seeking charging dock.'
                    )
                    self.need_dock_pub.publish(Bool(data=True))

        self.status_pub.publish(status)
        self.local_status_pub.publish(status)


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
