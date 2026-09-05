"""Detect a correctly docked AMR and maintain its project battery simulation."""
import math
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, Float32, String

from .common import clamp, yaw_from_quaternion


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class ChargingPadNode(Node):
    """Charges only after the robot is aligned, stationary, and inside the pad zone.

    The TurtleBot simulator owns ``battery_state``.  This node deliberately publishes
    a separate project-owned battery estimate at ``charging/battery_state`` instead
    of competing with the simulator publisher.
    """

    def __init__(self):
        super().__init__('charging_pad_node')
        self.robot_id = self.declare_parameter('robot_id', 'robot_1').value
        self.pad_x = self.declare_parameter('pad_x', 0.513707).value
        self.pad_y = self.declare_parameter('pad_y', -9.859080).value
        self.pad_yaw = self.declare_parameter('pad_yaw', 1.5708).value
        # TurtleBot odometry starts at (0, 0, 0) at spawn.  These parameters map
        # that local odom frame back into the warehouse/world frame.
        self.odom_origin_x = self.declare_parameter('odom_origin_x', 2.0).value
        self.odom_origin_y = self.declare_parameter('odom_origin_y', 2.0).value
        self.odom_origin_yaw = self.declare_parameter('odom_origin_yaw', 0.0).value
        self.dock_x_min = self.declare_parameter('dock_x_min', -0.42).value
        self.dock_x_max = self.declare_parameter('dock_x_max', 0.05).value
        self.dock_half_width = self.declare_parameter('dock_half_width', 0.18).value
        self.max_heading_error_rad = self.declare_parameter('max_heading_error_rad', 0.35).value
        self.max_speed_mps = self.declare_parameter('max_speed_mps', 0.03).value
        self.settle_time_s = self.declare_parameter('settle_time_s', 2.0).value
        self.charge_rate_percent_per_min = self.declare_parameter(
            'charge_rate_percent_per_min', 10.0).value
        self.battery_percent = self.declare_parameter('initial_battery_percent', 50.0).value
        self.odom_timeout_s = self.declare_parameter('odom_timeout_s', 0.5).value

        self.odom = None
        self.last_odom_wall_time = None
        self.stable_since_ns = None
        self.is_docked = False
        self.last_tick_ns = self.get_clock().now().nanoseconds
        self.docked_pub = self.create_publisher(Bool, 'charging/is_docked', 10)
        self.percent_pub = self.create_publisher(Float32, 'charging/battery_percent', 10)
        self.battery_pub = self.create_publisher(BatteryState, 'charging/battery_state', 10)
        self.status_pub = self.create_publisher(String, 'charging/docking_status', 10)
        self.docking_status = 'waiting for odometry'
        self.create_subscription(Odometry, 'odom', self.on_odom, 10)
        self.create_timer(0.2, self.update)

    def on_odom(self, msg):
        self.odom = msg
        self.last_odom_wall_time = time.monotonic()

    def dock_conditions_met(self):
        if (self.odom is None or self.last_odom_wall_time is None
                or time.monotonic() - self.last_odom_wall_time > self.odom_timeout_s):
            self.docking_status = 'not docked: waiting for fresh odometry'
            return False
        pose = self.odom.pose.pose
        origin_cos, origin_sin = math.cos(self.odom_origin_yaw), math.sin(self.odom_origin_yaw)
        world_x = self.odom_origin_x + origin_cos * pose.position.x - origin_sin * pose.position.y
        world_y = self.odom_origin_y + origin_sin * pose.position.x + origin_cos * pose.position.y
        world_yaw = wrap_angle(self.odom_origin_yaw + yaw_from_quaternion(pose.orientation))
        dx, dy = world_x - self.pad_x, world_y - self.pad_y
        cos_yaw, sin_yaw = math.cos(self.pad_yaw), math.sin(self.pad_yaw)
        local_x = cos_yaw * dx + sin_yaw * dy
        local_y = -sin_yaw * dx + cos_yaw * dy
        expected_yaw = wrap_angle(self.pad_yaw + math.pi)
        heading_error = abs(wrap_angle(world_yaw - expected_yaw))
        twist = self.odom.twist.twist
        speed = math.hypot(twist.linear.x, twist.linear.y)
        x_ok = self.dock_x_min <= local_x <= self.dock_x_max
        lateral_ok = abs(local_y) <= self.dock_half_width
        heading_ok = heading_error <= self.max_heading_error_rad
        speed_ok = speed <= self.max_speed_mps
        self.docking_status = (
            f'world=({world_x:.3f}, {world_y:.3f}, {world_yaw:.3f}); '
            f'pad_local=({local_x:.3f}, {local_y:.3f}); '
            f'heading_error={heading_error:.3f}; speed={speed:.3f}; '
            f'x={x_ok}, lateral={lateral_ok}, heading={heading_ok}, speed_ok={speed_ok}')
        return x_ok and lateral_ok and heading_ok and speed_ok

    def update(self):
        now_ns = self.get_clock().now().nanoseconds
        elapsed_s = max(0.0, min((now_ns - self.last_tick_ns) * 1e-9, 1.0))
        self.last_tick_ns = now_ns
        if self.dock_conditions_met():
            if self.stable_since_ns is None:
                self.stable_since_ns = now_ns
            self.is_docked = (now_ns - self.stable_since_ns) * 1e-9 >= self.settle_time_s
        else:
            self.stable_since_ns = None
            self.is_docked = False
        if self.is_docked:
            self.battery_percent = clamp(
                self.battery_percent + self.charge_rate_percent_per_min * elapsed_s / 60.0,
                0.0, 100.0)
        self.publish_state(now_ns)

    def publish_state(self, now_ns):
        self.docked_pub.publish(Bool(data=self.is_docked))
        self.percent_pub.publish(Float32(data=float(self.battery_percent)))
        self.status_pub.publish(String(data=self.docking_status))
        battery = BatteryState()
        battery.header.stamp.sec = now_ns // 1_000_000_000
        battery.header.stamp.nanosec = now_ns % 1_000_000_000
        battery.present = True
        battery.percentage = float(self.battery_percent / 100.0)
        battery.power_supply_status = (
            BatteryState.POWER_SUPPLY_STATUS_CHARGING if self.is_docked
            else BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING)
        self.battery_pub.publish(battery)


def main():
    rclpy.init()
    node = ChargingPadNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
