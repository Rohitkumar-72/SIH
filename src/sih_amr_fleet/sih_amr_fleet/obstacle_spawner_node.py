import json
import math
import random
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sih_amr_interfaces.msg import FleetEvent

from .common import FLEET_STATE_QOS, PROTOCOL_QOS, header, new_session_id, now_seconds


class ObstacleSpawnerNode(Node):
    """Deterministic, seedable temporary obstacle spawner for Gazebo simulation."""

    def __init__(self):
        super().__init__('obstacle_spawner_node')
        self.seed = self.declare_parameter('random_seed', 42).value
        self.enabled = self.declare_parameter('spawning_enabled', True).value
        self.spawn_interval_s = self.declare_parameter('spawn_interval_s', 25.0).value
        self.obstacle_ttl_s = self.declare_parameter('obstacle_ttl_s', 20.0).value
        self.max_active_obstacles = self.declare_parameter('max_active_obstacles', 2).value

        random.seed(self.seed)
        self.session_id = new_session_id()
        self.sequence = 0
        self.obstacle_counter = 0
        self.active_obstacles = {}  # id -> {spawn_time, remove_time, x, y}

        # Safe free-space polygons (strictly outside dock pads, egress, and main junction choke points)
        # Format: (min_x, max_x, min_y, max_y)
        self.allowed_spawn_zones = [
            (-18.0, -12.0, -20.0, -15.0),
            (12.0, 18.0, -20.0, -15.0),
            (-18.0, -12.0, 15.0, 20.0),
            (12.0, 18.0, 15.0, 20.0),
            (-5.0, 5.0, -5.0, 5.0),
        ]

        self.event_pub = self.create_publisher(FleetEvent, '/fleet/collision_event', PROTOCOL_QOS)
        self.wire_pub = self.create_publisher(String, '/fleet/obstacle_wire', FLEET_STATE_QOS)

        if self.enabled:
            self.create_timer(self.spawn_interval_s, self.spawn_tick)
            self.create_timer(1.0, self.cleanup_expired_obstacles)
            self.get_logger().info(f'ObstacleSpawnerNode active with seed {self.seed}')

    def spawn_tick(self):
        if len(self.active_obstacles) >= self.max_active_obstacles:
            return

        zone = random.choice(self.allowed_spawn_zones)
        x = round(random.uniform(zone[0], zone[1]), 2)
        y = round(random.uniform(zone[2], zone[3]), 2)
        
        self.obstacle_counter += 1
        obs_id = f'obs_seed{self.seed}_{self.obstacle_counter:03d}'
        now = now_seconds(self)
        ttl = self.obstacle_ttl_s + random.uniform(-5.0, 5.0)
        remove_at = now + ttl

        self.active_obstacles[obs_id] = {
            'x': x, 'y': y,
            'spawn_time': now,
            'remove_time': remove_at,
            'type': 'tote_box'
        }

        # Emit structured event for telemetry recording
        self.sequence += 1
        event = FleetEvent()
        event.fleet_header = header(self, 'spawner', self.session_id, self.sequence, 5.0)
        event.event_type = 'obstacle_spawned'
        event.severity = 1
        event.detail = json.dumps({
            'obstacle_id': obs_id, 'x': x, 'y': y,
            'ttl_s': round(ttl, 2), 'type': 'tote_box'
        })
        self.event_pub.publish(event)
        self.wire_pub.publish(String(data=event.detail))
        self.get_logger().info(f'Spawned physical obstacle {obs_id} at ({x}, {y}), TTL={ttl:.1f}s')

    def cleanup_expired_obstacles(self):
        now = now_seconds(self)
        expired = [oid for oid, info in self.active_obstacles.items() if now >= info['remove_time']]
        for oid in expired:
            info = self.active_obstacles.pop(oid)
            self.sequence += 1
            event = FleetEvent()
            event.fleet_header = header(self, 'spawner', self.session_id, self.sequence, 5.0)
            event.event_type = 'obstacle_cleared'
            event.severity = 1
            event.detail = json.dumps({'obstacle_id': oid, 'x': info['x'], 'y': info['y']})
            self.event_pub.publish(event)
            self.wire_pub.publish(String(data=event.detail))
            self.get_logger().info(f'Cleared expired obstacle {oid} from ({info["x"]}, {info["y"]})')


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleSpawnerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
