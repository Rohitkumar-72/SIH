"""Drive one AMR through the two widened centre lanes and every main corridor."""
import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from .warehouse_tasks import narrow_lanes


def clamp(value, minimum, maximum):
    return max(minimum, min(value, maximum))


class CorridorSweepNode(Node):
    """Closed-loop, LiDAR-protected visual inspection route for the warehouse."""
    def __init__(self):
        super().__init__('corridor_sweep_node')
        self.robot_name = self.declare_parameter('robot_name', 'corridor_sweep').value
        self.origin_x = self.declare_parameter('spawn_x', -7.5).value
        self.origin_y = self.declare_parameter('spawn_y', -10.0).value
        self.origin_yaw = self.declare_parameter('spawn_yaw', 0.0).value
        self.main_speed = self.declare_parameter('speed_mps', 6.0).value
        self.narrow_speed = self.declare_parameter('narrow_speed_mps', 0.85).value
        self.robot_radius = self.declare_parameter('robot_radius_m', 0.35).value
        self.tolerance = self.declare_parameter('waypoint_tolerance_m', 0.18).value
        self.stop_distance = self.declare_parameter('emergency_stop_distance_m', 0.32).value
        self.lidar_yaw = self.declare_parameter('lidar_yaw_in_base_rad', math.pi / 2.0).value
        self.pose, self.front_range, self.index, self.finished = None, math.inf, 0, False
        self.emergency_reported = False
        self.route = self.build_route(self.robot_radius)
        self.publisher = self.create_publisher(Twist, f'/{self.robot_name}/cmd_vel', 10)
        self.create_subscription(Odometry, f'/{self.robot_name}/odom', self.on_odom, 10)
        self.create_subscription(LaserScan, f'/{self.robot_name}/scan', self.on_scan, 10)
        self.create_timer(0.05, self.control)
        self.get_logger().info(f'loaded {len(self.route)} inspection waypoints')

    @staticmethod
    def build_route(robot_radius_m=0.35):
        """Cover the main lanes and every predefined storage-lane centreline."""
        route = [('J-SW', -7.5, -10.0, 'main')]
        lanes = narrow_lanes(robot_radius_m)

        def traverse(lane):
            route.extend((
                (f'{lane.lane_id} entrance', *lane.start, 'main'),
                (f'{lane.lane_id} far end', *lane.end, 'narrow'),
                (f'{lane.lane_id} exit', *lane.start, 'narrow'),
            ))

        # Each horizontal storage aisle is entered from its adjacent main
        # corridor, traversed on its centreline, and exited by the same line.
        west = sorted((lane for lane in lanes if '-WEST-' in lane.lane_id),
                      key=lambda lane: lane.start[1])
        east = sorted((lane for lane in lanes if '-EAST-' in lane.lane_id),
                      key=lambda lane: lane.start[1], reverse=True)
        for lane in west:
            traverse(lane)
        route.extend((
            ('J-NW', -7.5, 10.0, 'main'),
            ('MC-EW-N east', 7.5, 10.0, 'main'),
        ))
        for lane in east:
            traverse(lane)

        middle, north = (lane for lane in lanes if '-CENTRE' in lane.lane_id)
        route.extend((
            ('MC-EW-S centre', 0.0, -10.0, 'main'),
            (f'{middle.lane_id} entrance', *middle.start, 'main'),
            (f'{middle.lane_id} far end', *middle.end, 'narrow'),
            (f'{middle.lane_id} exit', *middle.start, 'narrow'),
            ('MC-EW-N centre', 0.0, 10.0, 'main'),
            (f'{north.lane_id} entrance', *north.start, 'main'),
            (f'{north.lane_id} far end', *north.end, 'narrow'),
            (f'{north.lane_id} exit', *north.start, 'narrow'),
            ('MC-EW-N finish', 0.0, 10.0, 'main'),
        ))
        return route

    def on_odom(self, message):
        local = message.pose.pose.position
        quaternion = message.pose.pose.orientation
        local_yaw = math.atan2(
            2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
            1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z))
        cosine, sine = math.cos(self.origin_yaw), math.sin(self.origin_yaw)
        self.pose = (
            self.origin_x + cosine * local.x - sine * local.y,
            self.origin_y + sine * local.x + cosine * local.y,
            self.origin_yaw + local_yaw,
        )

    def on_scan(self, message):
        front = []
        for index, value in enumerate(message.ranges):
            angle = message.angle_min + index * message.angle_increment
            if (math.isfinite(value) and message.range_min <= value <= message.range_max
                    # Side shelves are only 0.5777 m from a centred narrow
                    # lane.  A tight forward cone detects an obstacle on the
                    # line ahead without mistaking those shelf corners for it.
                    and abs(math.atan2(
                        math.sin(angle + self.lidar_yaw),
                        math.cos(angle + self.lidar_yaw))) <= math.radians(8.0)):
                front.append(value)
        self.front_range = min(front, default=math.inf)

    def stop(self):
        self.publisher.publish(Twist())

    def control(self):
        if self.pose is None:
            self.stop()
            return
        if self.front_range <= self.stop_distance:
            self.stop()
            if not self.emergency_reported:
                self.get_logger().error(
                    f'emergency stop: forward LiDAR range {self.front_range:.2f} m is below '
                    f'{self.stop_distance:.2f} m')
                self.emergency_reported = True
            return
        self.emergency_reported = False
        if self.index >= len(self.route):
            self.stop()
            if not self.finished:
                self.finished = True
                self.get_logger().info('corridor sweep complete; stopping the inspection AMR')
                self.create_timer(1.0, self.finish)
            return
        label, target_x, target_y, lane_kind = self.route[self.index]
        x, y, yaw = self.pose
        dx, dy = target_x - x, target_y - y
        distance = math.hypot(dx, dy)
        if distance <= self.tolerance:
            self.get_logger().info(f'completed {label} ({self.index + 1}/{len(self.route)})')
            self.index += 1
            self.stop()
            return
        desired_yaw = math.atan2(dy, dx)
        error = math.atan2(math.sin(desired_yaw - yaw), math.cos(desired_yaw - yaw))
        command = Twist()
        command.angular.z = clamp(1.8 * error, -1.2, 1.2)
        if abs(error) < 0.35:
            speed_limit = self.narrow_speed if lane_kind == 'narrow' else self.main_speed
            command.linear.x = min(speed_limit, 1.5 * distance) * max(0.0, math.cos(error))
        self.publisher.publish(command)

    def finish(self):
        if rclpy.ok():
            rclpy.shutdown()

    def destroy_node(self):
        self.stop()
        return super().destroy_node()


def main():
    rclpy.init()
    node = CorridorSweepNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
