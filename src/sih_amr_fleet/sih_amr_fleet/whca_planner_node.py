import pathlib
import yaml
import rclpy
from geometry_msgs.msg import Pose2D
from rclpy.node import Node
from sih_amr_interfaces.msg import BlockageObservation, GridCell, RobotState, RoutePlan, TaskAssignment, TrajectoryIntent

from .algorithms import whca_star
from .common import FLEET_STATE_QOS, PROTOCOL_QOS, header, new_session_id, now_seconds, stamp_seconds


class WhcaPlannerNode(Node):
    """Rolling cooperative A* planner over a configured warehouse occupancy grid."""
    def __init__(self):
        super().__init__('whca_planner_node')
        self.robot_id = self.declare_parameter('robot_id', 'robot_1').value
        map_file = self.declare_parameter('map_file', '').value
        self.resolution = self.declare_parameter('grid_resolution_m', 0.5).value
        self.horizon = self.declare_parameter('horizon_steps', 12).value
        self.session_id, self.sequence, self.plan_id = new_session_id(), 0, 0
        self.pose, self.assignment, self.peer_intents, self.blockages = None, None, {}, set()
        self.width, self.height, self.static_blocked = 30, 24, set()
        if map_file: self.load_map(map_file)
        self.pub = self.create_publisher(RoutePlan, 'planned_route', FLEET_STATE_QOS)
        self.create_subscription(RobotState, 'state', self.on_state, FLEET_STATE_QOS)
        self.create_subscription(TaskAssignment, 'task_assignment', self.on_assignment, FLEET_STATE_QOS)
        self.create_subscription(TrajectoryIntent, '/fleet/trajectory_intent', self.on_intent, PROTOCOL_QOS)
        self.create_subscription(BlockageObservation, '/fleet/blockage_observation', self.on_blockage, PROTOCOL_QOS)
        self.create_timer(1.0, self.plan)

    def load_map(self, filename):
        data = yaml.safe_load(pathlib.Path(filename).read_text())
        self.width, self.height = data['width'], data['height']
        self.static_blocked = {tuple(cell) for cell in data.get('blocked_cells', [])}

    def on_state(self, msg): self.pose = msg.pose
    def on_assignment(self, msg):
        if msg.owner_robot_id == self.robot_id and msg.active: self.assignment = msg
    def on_blockage(self, msg):
        if stamp_seconds(msg.fleet_header.valid_until) >= now_seconds(self): self.blockages.update((cell.x, cell.y) for cell in msg.cells)
    def on_intent(self, msg):
        if msg.fleet_header.robot_id != self.robot_id and stamp_seconds(msg.fleet_header.valid_until) >= now_seconds(self):
            self.peer_intents[msg.fleet_header.robot_id] = msg

    def to_cell(self, pose): return (round(pose.x / self.resolution), round(pose.y / self.resolution))
    def to_pose(self, cell): return Pose2D(x=cell[0] * self.resolution, y=cell[1] * self.resolution, theta=0.0)

    def plan(self):
        if self.pose is None or self.assignment is None or stamp_seconds(self.assignment.lease_until) < now_seconds(self): return
        start, goal = self.to_cell(self.pose), self.to_cell(self.assignment.task.pickup)
        reservations = set()
        for intent in self.peer_intents.values():
            if stamp_seconds(intent.fleet_header.valid_until) >= now_seconds(self):
                reservations.update((cell.x, cell.y, cell.time_slot) for cell in intent.reservations)
        path = whca_star(start, goal, self.static_blocked | self.blockages, reservations, self.width, self.height, self.horizon)
        self.sequence += 1; self.plan_id += 1; msg = RoutePlan()
        msg.fleet_header = header(self, self.robot_id, self.session_id, self.sequence, 1.5)
        msg.plan_id, msg.task_id = self.plan_id, self.assignment.task.task_id
        msg.route_feasible, msg.failure_reason = bool(path), '' if path else 'no route in current WHCA* window'
        msg.cells = [GridCell(x=x, y=y, time_slot=t) for x, y, t in path]
        msg.waypoints = [self.to_pose((x, y)) for x, y, _ in path]
        self.pub.publish(msg)


def main():
    rclpy.init(); node = WhcaPlannerNode()
    try: rclpy.spin(node)
    finally: node.destroy_node(); rclpy.shutdown()
