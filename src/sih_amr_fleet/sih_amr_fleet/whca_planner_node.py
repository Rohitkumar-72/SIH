import math
import pathlib
import yaml
import rclpy
from geometry_msgs.msg import Pose2D, PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from sih_amr_interfaces.msg import (
    BlockageObservation, GridCell, RobotState, RoutePlan, TaskAssignment,
    TaskExecutionStatus, TrajectoryIntent
)

from .algorithms import whca_star
from .common import FLEET_STATE_QOS, POSE_QOS, PROTOCOL_QOS, header, new_session_id, now_seconds, stamp_seconds


class WhcaPlannerNode(Node):
    """Rolling cooperative A* planner over a configured warehouse occupancy grid."""

    def __init__(self):
        super().__init__('whca_planner_node')
        self.robot_id = self.declare_parameter('robot_id', 'robot_1').value
        map_file = self.declare_parameter('map_file', '').value
        self.resolution = self.declare_parameter('grid_resolution_m', 0.5).value
        self.horizon = self.declare_parameter('horizon_steps', 12).value

        self.session_id = new_session_id()
        self.sequence = 0
        self.plan_id = 0
        self.pose = None
        self.assignment = None
        self.peer_intents = {}
        # cell -> observation validity deadline.  A moved obstacle must not
        # leave the warehouse permanently blocked in this robot's replica.
        self.blockages = {}
        self.width = 90
        self.height = 120
        self.origin_x = -22.5
        self.origin_y = -30.0
        self.static_blocked = set()
        self.execution_target = None
        self.execution_waiting = False

        if map_file:
            self.load_map(map_file)

        self.pub = self.create_publisher(RoutePlan, 'planned_route', FLEET_STATE_QOS)
        self.path_pub = self.create_publisher(Path, 'path', FLEET_STATE_QOS)

        self.create_subscription(RobotState, '/fleet/robot_state', self.on_state, FLEET_STATE_QOS)
        self.create_subscription(TaskAssignment, 'task_assignment', self.on_assignment, FLEET_STATE_QOS)
        self.create_subscription(TaskExecutionStatus, '/fleet/task_execution_status', self.on_execution, FLEET_STATE_QOS)
        self.create_subscription(TrajectoryIntent, '/fleet/trajectory_intent', self.on_intent, PROTOCOL_QOS)
        self.create_subscription(BlockageObservation, '/fleet/blockage_observation', self.on_blockage, PROTOCOL_QOS)
        self.create_timer(1.0, self.plan)

    def load_map(self, filename):
        try:
            data = yaml.safe_load(pathlib.Path(filename).read_text())
            self.width = int(data.get('width', self.width))
            self.height = int(data.get('height', self.height))
            origin = data.get('origin', [self.origin_x, self.origin_y])
            self.origin_x, self.origin_y = float(origin[0]), float(origin[1])
            self.resolution = float(data.get('resolution_m', self.resolution))
            self.static_blocked = {tuple(cell) for cell in data.get('blocked_cells', [])}

            layout = data.get('shelf_layout')
            if layout:
                footprint = layout.get('footprint_m', [3.92, 0.90])
                half_x = math.ceil(footprint[0] / self.resolution / 2.0)
                half_y = math.ceil(footprint[1] / self.resolution / 2.0)
                excluded = {tuple(pair) for pair in layout.get('excluded_zones', [])}

                for y_zone, rows in layout.get('y_zones', {}).items():
                    for x_zone, columns in layout.get('x_zones', {}).items():
                        if (y_zone, x_zone) in excluded:
                            continue
                        for x in columns:
                            for y in rows:
                                centre = self.to_cell(Pose2D(x=x, y=y, theta=0.0))
                                for cell_x in range(centre[0] - half_x, centre[0] + half_x + 1):
                                    for cell_y in range(centre[1] - half_y, centre[1] + half_y + 1):
                                        if 0 <= cell_x < self.width and 0 <= cell_y < self.height:
                                            self.static_blocked.add((cell_x, cell_y))
            self.get_logger().info(f'Loaded map in WhcaPlannerNode: {self.width}x{self.height}, {len(self.static_blocked)} blocked cells')
        except Exception as e:
            self.get_logger().error(f'Failed loading map in WhcaPlannerNode: {e}')

    def on_state(self, msg):
        if msg.fleet_header.robot_id != self.robot_id:
            return
        self.pose = msg.pose

    def on_assignment(self, msg):
        if msg.owner_robot_id == self.robot_id and msg.active:
            self.assignment = msg

    def on_execution(self, msg):
        if msg.owner_robot_id != self.robot_id:
            return
        self.execution_waiting = msg.phase in (
            TaskExecutionStatus.PICKUP_WAIT,
            TaskExecutionStatus.DROPOFF_WAIT,
            TaskExecutionStatus.COMPLETED
        )
        if self.execution_waiting:
            self.execution_target = None
        else:
            self.execution_target = msg.target

    def on_blockage(self, msg):
        valid_until = stamp_seconds(msg.fleet_header.valid_until)
        if valid_until >= now_seconds(self):
            for cell in msg.cells:
                key = (cell.x, cell.y)
                self.blockages[key] = max(self.blockages.get(key, 0.0), valid_until)

    def on_intent(self, msg):
        if msg.fleet_header.robot_id != self.robot_id and stamp_seconds(msg.fleet_header.valid_until) >= now_seconds(self):
            self.peer_intents[msg.fleet_header.robot_id] = msg

    def to_cell(self, pose):
        cx = round((pose.x - self.origin_x) / self.resolution)
        cy = round((pose.y - self.origin_y) / self.resolution)
        return (max(0, min(cx, self.width - 1)), max(0, min(cy, self.height - 1)))

    def to_pose(self, cell):
        return Pose2D(
            x=self.origin_x + cell[0] * self.resolution,
            y=self.origin_y + cell[1] * self.resolution,
            theta=0.0
        )

    def plan(self):
        if self.pose is None or self.assignment is None or stamp_seconds(self.assignment.lease_until) < now_seconds(self):
            return

        if self.execution_waiting:
            # Publish single stationary waypoint while holding at dwell location
            self.sequence += 1
            self.plan_id += 1
            start = self.to_cell(self.pose)
            msg = RoutePlan()
            msg.fleet_header = header(self, self.robot_id, self.session_id, self.sequence, 1.5)
            msg.plan_id = self.plan_id
            msg.task_id = self.assignment.task.task_id
            msg.route_feasible = True
            msg.failure_reason = 'dwelling at task station'
            msg.cells = [GridCell(x=start[0], y=start[1], time_slot=t) for t in range(self.horizon)]
            msg.waypoints = [self.to_pose(start)]
            self.pub.publish(msg)
            return

        target = self.execution_target or self.assignment.task.pickup
        start = self.to_cell(self.pose)
        goal = self.to_cell(target)

        now = now_seconds(self)
        self.blockages = {
            cell: valid_until for cell, valid_until in self.blockages.items()
            if valid_until >= now
        }
        reservations = set()
        for intent in list(self.peer_intents.values()):
            if stamp_seconds(intent.fleet_header.valid_until) >= now:
                reservations.update((cell.x, cell.y, cell.time_slot) for cell in intent.reservations)

        path = whca_star(
            start, goal, self.static_blocked | set(self.blockages), reservations,
            self.width, self.height, self.horizon
        )

        self.sequence += 1
        self.plan_id += 1
        msg = RoutePlan()
        msg.fleet_header = header(self, self.robot_id, self.session_id, self.sequence, 1.5)
        msg.plan_id = self.plan_id
        msg.task_id = self.assignment.task.task_id
        msg.route_feasible = bool(path)
        msg.failure_reason = '' if path else 'no conflict-free route in current WHCA* window'
        msg.cells = [GridCell(x=x, y=y, time_slot=t) for x, y, t in path]
        msg.waypoints = [self.to_pose((x, y)) for x, y, _ in path]
        self.pub.publish(msg)

        nav_path = Path()
        nav_path.header.stamp = self.get_clock().now().to_msg()
        nav_path.header.frame_id = 'map'
        for waypoint in msg.waypoints:
            pose = PoseStamped()
            pose.header = nav_path.header
            pose.pose.position.x = waypoint.x
            pose.pose.position.y = waypoint.y
            pose.pose.orientation.w = 1.0
            nav_path.poses.append(pose)
        self.path_pub.publish(nav_path)


def main(args=None):
    rclpy.init(args=args)
    node = WhcaPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
