import json
import random
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from sih_amr_interfaces.msg import FleetEvent

from .common import FLEET_STATE_QOS, POSE_QOS, PROTOCOL_QOS, header, new_session_id, now_seconds


class FaultInjectorNode(Node):
    """Seedable, opt-in fault injection harness for multi-AMR fleet experiments."""

    def __init__(self):
        super().__init__('fault_injector_node')
        self.robot_id = self.declare_parameter('robot_id', 'robot_1').value
        self.seed = self.declare_parameter('random_seed', 42).value
        self.enabled = self.declare_parameter('fault_injection_enabled', False).value
        self.stall_prob = self.declare_parameter('robot_motor_stall_per_task', 0.01).value
        self.lidar_dropout_prob = self.declare_parameter('lidar_dropout_per_frame', 0.001).value
        self.packet_drop_rate = self.declare_parameter('network_packet_drop_rate', 0.05).value

        # Use deterministic seed offset per robot
        robot_idx = int(self.robot_id.rsplit('_', 1)[-1]) if '_' in self.robot_id else 1
        random.seed(self.seed + robot_idx * 100)

        self.session_id = new_session_id()
        self.sequence = 0
        self.active_motor_stall = False
        self.stall_until = 0.0
        self.active_lidar_dropout = False
        self.dropout_until = 0.0

        self.event_pub = self.create_publisher(FleetEvent, '/fleet/recovery_event', PROTOCOL_QOS)
        self.stall_pub = self.create_publisher(Bool, 'motor_stall_active', FLEET_STATE_QOS)

        if self.enabled:
            self.create_timer(1.0, self.fault_scheduler_tick)
            self.get_logger().info(f'FaultInjectorNode ACTIVE for {self.robot_id} with seed {self.seed}')

    def emit_fault_event(self, event_type, fault_name, duration_s):
        self.sequence += 1
        event = FleetEvent()
        event.fleet_header = header(self, self.robot_id, self.session_id, self.sequence, 5.0)
        event.event_type = event_type
        event.severity = 2 if 'start' in event_type else 1
        event.detail = json.dumps({
            'robot_id': self.robot_id,
            'fault': fault_name,
            'duration_s': round(duration_s, 2),
            'timestamp': now_seconds(self)
        })
        self.event_pub.publish(event)

    def fault_scheduler_tick(self):
        now = now_seconds(self)

        # Check motor stall recovery
        if self.active_motor_stall and now >= self.stall_until:
            self.active_motor_stall = False
            self.stall_pub.publish(Bool(data=False))
            self.emit_fault_event('fault_end', 'robot_motor_stall', 0.0)
            self.get_logger().info(f'[{self.robot_id}:Fault] RECOVERED from motor stall.')

        # Check LiDAR dropout recovery
        if self.active_lidar_dropout and now >= self.dropout_until:
            self.active_lidar_dropout = False
            self.emit_fault_event('fault_end', 'lidar_dropout', 0.0)

        # Probabilistic trigger for motor stall (evaluated periodically)
        if not self.active_motor_stall and random.random() < (self.stall_prob * 0.1):
            duration = random.uniform(5.0, 15.0)
            self.active_motor_stall = True
            self.stall_until = now + duration
            self.stall_pub.publish(Bool(data=True))
            self.emit_fault_event('fault_start', 'robot_motor_stall', duration)
            self.get_logger().warning(
                f'[{self.robot_id}:Fault] INJECTED motor stall for {duration:.1f}s.')


def main(args=None):
    rclpy.init(args=args)
    node = FaultInjectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
