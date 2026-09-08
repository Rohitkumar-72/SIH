import json
import os
import pathlib
import time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sih_amr_interfaces.msg import (
    BlockageObservation, CorridorProtocol, FleetHealth,
    DockProtocol, FleetEvent, RobotState, SafetyState, TaskAnnouncement, TaskConsensus, TaskExecutionStatus, TrajectoryIntent
)

from .common import FLEET_STATE_QOS, POSE_QOS, PROTOCOL_QOS, TASK_SOURCE_QOS, now_seconds, stamp_seconds


class DataCollectionNode(Node):
    """Passive JSONL telemetry recorder for fleet benchmarking and offline analysis."""

    def __init__(self):
        super().__init__('data_collection_node')
        self.output_file = self.declare_parameter('output_file', '/tmp/sih_amr_fleet_telemetry.jsonl').value
        self.run_id = f'run_{int(time.time())}'
        self.file_handle = None
        self.raw_odom = {}
        self.map_origin_x = self.declare_parameter('map_origin_x', -22.5).value
        self.map_origin_y = self.declare_parameter('map_origin_y', -30.0).value
        self.map_resolution_m = self.declare_parameter('map_resolution_m', 0.5).value
        self.robot_ids = self.declare_parameter('robot_ids', ['robot_1', 'robot_2', 'robot_3', 'robot_4']).value

        try:
            path = pathlib.Path(self.output_file)
            path.parent.mkdir(parents=True, exist_ok=True)
            self.file_handle = open(path, 'a', buffering=1, encoding='utf-8')
            manifest = {
                'event_type': 'run_manifest',
                'run_id': self.run_id,
                'start_time_iso': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
                'start_epoch_s': time.time(),
                'schema_version': '0.2.0',
                'record_fields': 'robot_state.map_pose is fleet map-relative state; robot_state.gazebo_odom is untransformed simulator wheel odometry.',
            }
            self.write_record(manifest)
            self.get_logger().info(f'DataCollectionNode logging to {self.output_file}')
        except Exception as e:
            self.get_logger().error(f'Failed to open telemetry log {self.output_file}: {e}')

        # Subscriptions across fleet coordination topics
        self.create_subscription(RobotState, '/fleet/robot_state', self.on_robot_state, FLEET_STATE_QOS)
        self.create_subscription(FleetHealth, '/fleet/health', self.on_health, FLEET_STATE_QOS)
        self.create_subscription(SafetyState, '/fleet/safety_state', self.on_safety, FLEET_STATE_QOS)
        self.create_subscription(TrajectoryIntent, '/fleet/trajectory_intent', self.on_intent, PROTOCOL_QOS)
        self.create_subscription(CorridorProtocol, '/fleet/corridor_protocol', self.on_corridor, PROTOCOL_QOS)
        self.create_subscription(TaskAnnouncement, '/fleet/task_announcement', self.on_task_announcement, TASK_SOURCE_QOS)
        self.create_subscription(TaskConsensus, '/fleet/task_consensus', self.on_consensus, PROTOCOL_QOS)
        self.create_subscription(TaskExecutionStatus, '/fleet/task_execution_status', self.on_execution, FLEET_STATE_QOS)
        self.create_subscription(BlockageObservation, '/fleet/blockage_observation', self.on_blockage, PROTOCOL_QOS)
        self.create_subscription(DockProtocol, '/fleet/dock_protocol', self.on_dock, PROTOCOL_QOS)
        self.create_subscription(FleetEvent, '/fleet/recovery_event', self.on_event, PROTOCOL_QOS)
        self.create_subscription(FleetEvent, '/fleet/collision_event', self.on_event, PROTOCOL_QOS)
        for robot_id in self.robot_ids:
            self.create_subscription(Odometry, f'/{robot_id}/odom',
                                     lambda msg, robot_id=robot_id: self.on_raw_odom(robot_id, msg), POSE_QOS)

    def on_raw_odom(self, robot_id, msg):
        self.raw_odom[robot_id] = {
            'pose': {'x': float(msg.pose.pose.position.x), 'y': float(msg.pose.pose.position.y),
                     'z': float(msg.pose.pose.position.z), 'qx': float(msg.pose.pose.orientation.x),
                     'qy': float(msg.pose.pose.orientation.y), 'qz': float(msg.pose.pose.orientation.z),
                     'qw': float(msg.pose.pose.orientation.w)},
            'twist': {'vx': float(msg.twist.twist.linear.x), 'vy': float(msg.twist.twist.linear.y),
                      'vz': float(msg.twist.twist.linear.z), 'wz': float(msg.twist.twist.angular.z)},
            'stamp_s': float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9,
        }

    def write_record(self, record):
        if not self.file_handle:
            return
        try:
            record['logged_at'] = now_seconds(self)
            self.file_handle.write(json.dumps(record) + '\n')
        except Exception:
            pass  # Strictly passive; never fail or impede control

    def on_robot_state(self, msg):
        self.write_record({
            'event_type': 'robot_state',
            'robot_id': msg.fleet_header.robot_id,
            'session_id': msg.fleet_header.session_id,
            'seq': msg.fleet_header.sequence_no,
            'x': float(msg.pose.x),
            'y': float(msg.pose.y),
            'theta': float(msg.pose.theta),
            'vx': float(msg.twist.linear.x),
            'vy': float(msg.twist.linear.y),
            'wz': float(msg.twist.angular.z),
            'map_pose': {'x': float(msg.pose.x), 'y': float(msg.pose.y), 'theta': float(msg.pose.theta),
                         'cell': [round((msg.pose.x-self.map_origin_x)/self.map_resolution_m), round((msg.pose.y-self.map_origin_y)/self.map_resolution_m)]},
            'map_twist': {'vx': float(msg.twist.linear.x), 'vy': float(msg.twist.linear.y), 'wz': float(msg.twist.angular.z)},
            'gazebo_odom': self.raw_odom.get(msg.fleet_header.robot_id),
            'localization_valid': msg.localization_valid,
        })

    def on_health(self, msg):
        self.write_record({
            'event_type': 'health',
            'robot_id': msg.fleet_header.robot_id,
            'battery_percent': float(msg.battery_percent),
            'comm_state': int(msg.communication_state),
            'task_feasible': msg.task_feasible,
            'safety_ok': msg.safety_ok,
            'active_task_id': msg.active_task_id,
        })

    def on_safety(self, msg):
        self.write_record({
            'event_type': 'safety_state',
            'robot_id': msg.fleet_header.robot_id,
            'level': int(msg.level),
            'nearest_obstacle_m': float(msg.nearest_obstacle_m),
            'ttc_s': float(msg.time_to_collision_s),
            'reason': msg.reason,
        })

    def on_intent(self, msg):
        self.write_record({
            'event_type': 'trajectory_intent',
            'robot_id': msg.fleet_header.robot_id,
            'plan_id': int(msg.plan_id),
            'cells_count': len(msg.reservations),
            'priority': int(msg.priority),
        })

    def on_corridor(self, msg):
        self.write_record({
            'event_type': 'corridor_protocol',
            'robot_id': msg.fleet_header.robot_id,
            'corridor_id': msg.corridor_id,
            'request_id': msg.request_id,
            'target_robot_id': msg.target_robot_id,
            'lamport_time': int(msg.lamport_time),
            'event': int(msg.event),
        })

    def on_consensus(self, msg):
        self.write_record({
            'event_type': 'task_consensus',
            'task_id': msg.task_id,
            'winner_robot_id': msg.winner_robot_id,
            'winning_bid': float(msg.winning_bid),
            'epoch': int(msg.assignment_epoch),
            'event': int(msg.event),
        })

    def on_task_announcement(self, msg):
        self.write_record({
            'event_type': 'task_announcement',
            'task_id': msg.task.task_id,
            'source_robot_id': msg.fleet_header.robot_id,
            'pickup_x': float(msg.task.pickup.x),
            'pickup_y': float(msg.task.pickup.y),
            'dropoff_x': float(msg.task.dropoff.x),
            'dropoff_y': float(msg.task.dropoff.y),
        })

    def on_execution(self, msg):
        self.write_record({
            'event_type': 'task_execution',
            'task_id': msg.task_id,
            'robot_id': msg.owner_robot_id,
            'phase': int(msg.phase),
            'target_x': float(msg.target.x),
            'target_y': float(msg.target.y),
            'wait_remaining_s': float(msg.wait_time_remaining_s),
        })

    def on_blockage(self, msg):
        self.write_record({
            'event_type': 'blockage_observation',
            'robot_id': msg.fleet_header.robot_id,
            'confidence': float(msg.confidence),
            'source': msg.source,
            'cells_count': len(msg.cells),
        })

    def on_dock(self, msg):
        self.write_record({'event_type': 'dock_protocol', 'robot_id': msg.fleet_header.robot_id,
                           'dock_id': msg.dock_id, 'request_id': msg.request_id,
                           'lamport_time': int(msg.lamport_time), 'event': int(msg.event)})

    def on_event(self, msg):
        self.write_record({'event_type': msg.event_type, 'robot_id': msg.fleet_header.robot_id,
                           'severity': int(msg.severity), 'detail': msg.detail,
                           'source': 'fleet_event'})

    def destroy_node(self):
        if self.file_handle:
            try:
                self.file_handle.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DataCollectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
