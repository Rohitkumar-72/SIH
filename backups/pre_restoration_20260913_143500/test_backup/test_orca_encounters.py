"""Deterministic encounter unit tests for resource-aware ORCA local avoidance and corridor safety."""

import math
import pytest
from geometry_msgs.msg import Pose2D, Twist
from sih_amr_interfaces.msg import (
    CorridorProtocol, GridCell, PeerTrack, PeerTrackArray, RobotState, RoutePlan, SafetyState
)

from sih_amr_fleet.algorithms import corridor_axial_holding_cell
from sih_amr_fleet.corridor_mutex_node import CorridorMutexNode
from sih_amr_fleet.orca_node import OrcaNode
from sih_amr_fleet.path_follower_node import PathFollowerNode


class MockClock:
    def __init__(self, start_s=100.0):
        self._s = start_s

    def advance(self, dt):
        self._s += dt

    @property
    def now_seconds(self):
        return self._s


def make_track(robot_id, x, y, vx=0.0, vy=0.0):
    track = PeerTrack()
    track.robot_id = robot_id
    track.pose.x = float(x)
    track.pose.y = float(y)
    track.twist.linear.x = float(vx)
    track.twist.linear.y = float(vy)
    track.covariance_trace = 0.01
    return track


@pytest.fixture
def rclpy_init():
    import rclpy
    if not rclpy.ok():
        rclpy.init()
    yield


def test_lower_id_waiter_vs_higher_id_corridor_owner(rclpy_init, monkeypatch):
    """Verify that a higher-ID corridor owner outranks a lower-ID outside waiter.
    
    Robot 1 (outside waiter) must yield to Robot 4 (corridor owner).
    Robot 4 (corridor owner) must NOT yield to Robot 1.
    """
    clock = MockClock(100.0)
    monkeypatch.setattr('sih_amr_fleet.orca_node.now_seconds', lambda node: clock.now_seconds)

    # Robot 1: outside waiter, lower ID
    r1 = OrcaNode()
    r1.robot_id = 'robot_1'
    r1.corridor_protected = False
    r1.peer_corridors['robot_4'] = 'NC-MIDDLE-EAST-01'
    r1.pose = Pose2D(x=5.5, y=-6.0, theta=0.0)
    r1.desired = Twist()
    r1.desired.linear.x = 0.46

    # Peer Robot 4: inside corridor, higher ID, exiting westward
    track_r4 = make_track('robot_4', 7.0, -6.0, vx=-0.46, vy=0.0)
    r1.tracks = [track_r4]

    published_r1 = []
    r1.pub = type('MockPub', (), {'publish': lambda self, msg: published_r1.append(msg)})()

    r1.control()
    assert r1.peer_outranks_self('robot_4') is True, "Corridor owner R4 must outrank outside waiter R1"
    assert r1.yielding_to == 'robot_4', "Outside waiter R1 must yield to corridor owner R4"
    # Waiter yields safely by holding at 0.0m/s; reverse recovery is handled by supervisor/PathFollower
    assert published_r1[-1].linear.x == 0.0, "Outside waiter must hold stop to yield right-of-way"

    # Now verify Robot 4's perspective (corridor owner)
    r4 = OrcaNode()
    r4.robot_id = 'robot_4'
    r4.corridor_protected = True
    r4.peer_corridors = {}
    r4.pose = Pose2D(x=7.0, y=-6.0, theta=math.pi)
    r4.desired = Twist()
    r4.desired.linear.x = 0.46

    track_r1 = make_track('robot_1', 5.5, -6.0, vx=0.0, vy=0.0)
    r4.tracks = [track_r1]

    published_r4 = []
    r4.pub = type('MockPub', (), {'publish': lambda self, msg: published_r4.append(msg)})()

    r4.control()
    assert r4.peer_outranks_self('robot_1') is False, "Corridor owner R4 must NOT yield to outside waiter R1"
    assert r4.yielding_to is None, "Corridor owner R4 must not enter yield latch against outside waiter R1"

    r1.destroy_node()
    r4.destroy_node()


def test_higher_id_waiter_vs_lower_id_corridor_owner(rclpy_init, monkeypatch):
    """Verify that a lower-ID corridor owner outranks a higher-ID outside waiter.
    
    Robot 4 (outside waiter) must yield to Robot 1 (corridor owner).
    Robot 1 (corridor owner) must NOT yield to Robot 4.
    """
    clock = MockClock(150.0)
    monkeypatch.setattr('sih_amr_fleet.orca_node.now_seconds', lambda node: clock.now_seconds)

    # Robot 4: outside waiter, higher ID
    r4 = OrcaNode()
    r4.robot_id = 'robot_4'
    r4.corridor_protected = False
    r4.peer_corridors['robot_1'] = 'NC-MIDDLE-EAST-01'
    r4.pose = Pose2D(x=5.5, y=-6.0, theta=0.0)
    r4.desired = Twist()
    r4.desired.linear.x = 0.46

    # Peer Robot 1: inside corridor, lower ID
    track_r1 = make_track('robot_1', 7.0, -6.0, vx=-0.46, vy=0.0)
    r4.tracks = [track_r1]

    published_r4 = []
    r4.pub = type('MockPub', (), {'publish': lambda self, msg: published_r4.append(msg)})()

    r4.control()
    assert r4.peer_outranks_self('robot_1') is True, "Corridor owner R1 must outrank outside waiter R4"
    assert r4.yielding_to == 'robot_1', "Outside waiter R4 must yield to corridor owner R1"
    assert published_r4[-1].linear.x == 0.0, "Outside waiter must hold stop to yield right-of-way"

    # Robot 1's perspective: owner
    r1 = OrcaNode()
    r1.robot_id = 'robot_1'
    r1.corridor_protected = True
    r1.peer_corridors = {}
    r1.pose = Pose2D(x=7.0, y=-6.0, theta=math.pi)
    r1.desired = Twist()
    r1.desired.linear.x = 0.46

    track_r4 = make_track('robot_4', 5.5, -6.0, vx=0.0, vy=0.0)
    r1.tracks = [track_r4]

    published_r1 = []
    r1.pub = type('MockPub', (), {'publish': lambda self, msg: published_r1.append(msg)})()

    r1.control()
    assert r1.peer_outranks_self('robot_4') is False, "Corridor owner R1 must not yield to outside waiter R4"
    assert r1.yielding_to is None, "Corridor owner R1 must not enter yield latch"

    r1.destroy_node()
    r4.destroy_node()


def test_open_space_tie_breaker_uses_id_priority(rclpy_init, monkeypatch):
    """Verify that open-space encounters (neither has corridor protection) use ID priority."""
    clock = MockClock(200.0)
    monkeypatch.setattr('sih_amr_fleet.orca_node.now_seconds', lambda node: clock.now_seconds)

    # R2 in open space encountering R1
    r2 = OrcaNode()
    r2.robot_id = 'robot_2'
    r2.corridor_protected = False
    r2.peer_corridors = {}
    r2.pose = Pose2D(x=0.0, y=0.0, theta=0.0)
    r2.desired = Twist()
    r2.desired.linear.x = 0.46

    track_r1 = make_track('robot_1', 1.8, 0.0, vx=-0.46, vy=0.0)
    r2.tracks = [track_r1]

    published_cmds = []
    r2.pub = type('MockPub', (), {'publish': lambda self, msg: published_cmds.append(msg)})()

    r2.control()
    assert r2.peer_outranks_self('robot_1') is True, "Open-space: R1 outranks R2 by static ID tie-breaker"
    assert r2.yielding_to == 'robot_1'
    assert published_cmds[-1].linear.x == 0.0

    # R1 in open space encountering R2
    r1 = OrcaNode()
    r1.robot_id = 'robot_1'
    r1.corridor_protected = False
    r1.peer_corridors = {}
    r1.pose = Pose2D(x=1.8, y=0.0, theta=math.pi)
    r1.desired = Twist()
    r1.desired.linear.x = 0.46

    track_r2 = make_track('robot_2', 0.0, 0.0, vx=0.46, vy=0.0)
    r1.tracks = [track_r2]

    published_r1 = []
    r1.pub = type('MockPub', (), {'publish': lambda self, msg: published_r1.append(msg)})()

    r1.control()
    assert r1.peer_outranks_self('robot_2') is False, "Open-space: R1 does not yield to lower-priority R2"
    assert r1.yielding_to is None

    r1.destroy_node()
    r2.destroy_node()


def test_open_space_parallel_noncollision(rclpy_init, monkeypatch):
    """Verify that two robots on parallel offset tracks execute a passing manoeuvre without deadlock."""
    clock = MockClock(300.0)
    monkeypatch.setattr('sih_amr_fleet.orca_node.now_seconds', lambda node: clock.now_seconds)

    # Robot 1 traveling East along y = 0.0
    r1 = OrcaNode()
    r1.robot_id = 'robot_1'
    r1.radius = 0.28
    r1.pose = Pose2D(x=0.0, y=0.0, theta=0.0)
    r1.desired = Twist()
    r1.desired.linear.x = 0.46

    # Robot 2 traveling West along y = 0.70m (> 2 * radius = 0.56m)
    r2 = OrcaNode()
    r2.robot_id = 'robot_2'
    r2.radius = 0.28
    r2.pose = Pose2D(x=2.0, y=0.70, theta=math.pi)
    r2.desired = Twist()
    r2.desired.linear.x = 0.46

    pub_r1 = []
    r1.pub = type('MockPub', (), {'publish': lambda self, msg: pub_r1.append(msg)})()
    pub_r2 = []
    r2.pub = type('MockPub', (), {'publish': lambda self, msg: pub_r2.append(msg)})()

    # Simulate passing manoeuvre over 4 seconds
    dt = 0.5
    for _ in range(8):
        clock.advance(dt)
        r1.tracks = [make_track('robot_2', r2.pose.x, r2.pose.y, vx=-0.46, vy=0.0)]
        r2.tracks = [make_track('robot_1', r1.pose.x, r1.pose.y, vx=0.46, vy=0.0)]

        r1.control()
        r2.control()

        # Check separation distance: must strictly exceed physical collision threshold 2 * 0.17m = 0.34m
        sep = math.hypot(r1.pose.x - r2.pose.x, r1.pose.y - r2.pose.y)
        assert sep >= 0.70, f"Separation violated safe lateral envelope: {sep:.2f}m"

        # Neither robot should enter yield deadlock
        assert r1.yielding_to is None
        assert r2.yielding_to is None

        # Both maintain forward motion
        assert pub_r1[-1].linear.x > 0.30
        assert pub_r2[-1].linear.x > 0.30

        # Advance poses
        r1.pose.x += pub_r1[-1].linear.x * dt
        r2.pose.x -= pub_r2[-1].linear.x * dt

    r1.destroy_node()
    r2.destroy_node()


def test_axial_gate_holding_setback_both_directions():
    """Verify axial gate selection selects >= 2.0m setback outside corridor footprint for X and Y axes."""
    width, height = 100, 100
    res = 0.5
    static_blocked = set()

    # 1. X-axis corridor (horizontal aisle along y = 48, from x = 60 to 88)
    corridor_x = {(x, 48) for x in range(60, 89)}
    # Approach from West: start at (50, 48)
    start_west = (50, 48)
    goal_inside_x = (70, 48)

    holding_x = corridor_axial_holding_cell(
        corridor_x, width, height, static_blocked, set(),
        start_west, resolution_m=res, approach_dist_m=2.0
    )
    assert holding_x is not None
    # Far edge of 2.0m approach zone: 2.0m / 0.5m = 4 cells. Mouth is x = 60 -> holding cell x = 56
    assert holding_x == (56, 48), f"Expected (56, 48), got {holding_x}"
    assert holding_x not in corridor_x, "Holding cell must be outside corridor footprint"
    # Physical setback distance from corridor mouth (cell 60):
    setback_dist_x = (60 - holding_x[0]) * res
    assert setback_dist_x >= 2.0, f"Physical setback must be >= 2.0m, got {setback_dist_x}m"

    # 2. Y-axis corridor (vertical aisle along x = 45, from y = 10 to 30)
    corridor_y = {(45, y) for y in range(10, 31)}
    # Approach from South: start at (45, 2)
    start_south = (45, 2)
    goal_inside_y = (45, 20)

    holding_y = corridor_axial_holding_cell(
        corridor_y, width, height, static_blocked, set(),
        start_south, resolution_m=res, approach_dist_m=2.0
    )
    assert holding_y is not None
    # Mouth is y = 10 -> holding cell y = 6 (4 cells setback = 2.0m)
    assert holding_y == (45, 6), f"Expected (45, 6), got {holding_y}"
    assert holding_y not in corridor_y, "Holding cell must be outside corridor footprint"
    setback_dist_y = (10 - holding_y[1]) * res
    assert setback_dist_y >= 2.0, f"Physical setback must be >= 2.0m, got {setback_dist_y}m"


def test_entered_owner_never_releases_on_timeout(rclpy_init, monkeypatch):
    """Verify that an AMR physically inside a corridor NEVER times out or releases its token."""
    clock = MockClock(100.0)
    monkeypatch.setattr('sih_amr_fleet.corridor_mutex_node.now_seconds', lambda node: clock.now_seconds)

    node = CorridorMutexNode()
    node.robot_id = 'robot_2'
    node.corridors = {'NC-MIDDLE-EAST-01': {(x, 48) for x in range(60, 89)}}
    node.request = {
        'id': 'req-1',
        'corridor': 'NC-MIDDLE-EAST-01',
        'grants': {'robot_1', 'robot_3', 'robot_4'},
        'entered': True,
        'entered_at': 100.0,
        'was_inside': True,
    }

    released_events = []
    node.send = lambda event, c_id, r_id, peer=None: released_events.append(event)

    # Robot remains inside corridor cell (70, 48)
    node.cell = (70, 48)

    # Advance time by 25.0 simulated seconds (far past 10s unentered timeout)
    clock.advance(25.0)
    node.on_state(type('MockMsg', (), {
        'fleet_header': type('MockHeader', (), {'robot_id': 'robot_2'})(),
        'pose': Pose2D(x=35.0, y=24.0, theta=0.0)
    })())

    # Token must NOT be cancelled or released
    assert CorridorProtocol.CANCEL not in released_events
    assert CorridorProtocol.RELEASE not in released_events
    assert node.request is not None, "Entered owner request must never be dropped"
    assert node.request['was_inside'] is True

    node.destroy_node()


def test_unentered_owner_cancellation_on_timeout(rclpy_init, monkeypatch):
    """Verify that an unentered owner holding a token cancels after timeout to unblock peers."""
    clock = MockClock(100.0)
    monkeypatch.setattr('sih_amr_fleet.corridor_mutex_node.now_seconds', lambda node: clock.now_seconds)

    node = CorridorMutexNode()
    node.robot_id = 'robot_3'
    node.corridors = {'NC-MIDDLE-WEST-01': {(x, 48) for x in range(2, 31)}}
    node.request = {
        'id': 'req-3',
        'corridor': 'NC-MIDDLE-WEST-01',
        'grants': {'robot_1', 'robot_2', 'robot_4'},
        'entered': True,
        'entered_at': 100.0,
        'was_inside': False,
    }

    released_events = []
    node.send = lambda event, c_id, r_id, peer=None: released_events.append(event)

    # Robot is stuck outside corridor at entrance cell (35, 48)
    node.cell = (35, 48)

    # Advance 11.0 simulated seconds (> 10s timeout)
    clock.advance(11.0)
    node.on_state(type('MockMsg', (), {
        'fleet_header': type('MockHeader', (), {'robot_id': 'robot_3'})(),
        'pose': Pose2D(x=17.5, y=24.0, theta=0.0)
    })())

    # CANCEL must be emitted to release peers
    assert CorridorProtocol.CANCEL in released_events, "Unentered owner must CANCEL on timeout"
    assert node.request is None, "Request must be cleared after cancellation"

    node.destroy_node()


def test_owner_exits_and_releases_within_15_simulated_seconds(rclpy_init, monkeypatch):
    """Verify that an owner moving through a corridor exits and emits EXIT within 15 simulated seconds."""
    clock = MockClock(100.0)
    monkeypatch.setattr('sih_amr_fleet.corridor_mutex_node.now_seconds', lambda node: clock.now_seconds)

    node = CorridorMutexNode()
    node.robot_id = 'robot_2'
    # Corridor length: 10 cells = 5.0m
    node.corridors = {'NC-TEST': {(x, 10) for x in range(20, 31)}}
    node.request = {
        'id': 'req-test',
        'corridor': 'NC-TEST',
        'grants': {'robot_1'},
        'entered': True,
        'entered_at': 100.0,
        'was_inside': True,
    }

    released_events = []
    node.send = lambda event, c_id, r_id, peer=None: released_events.append(event)

    # Robot traverses from cell 20 to cell 30 at 0.46 m/s (5.0m / 0.46 m/s = 10.87s)
    speed = 0.46
    x_pos = 10.0  # cell 20
    t = 0.0
    dt = 0.5

    while t < 15.0:
        clock.advance(dt)
        t += dt
        x_pos += speed * dt
        cell_x = round(x_pos / 0.5)
        node.cell = (cell_x, 10)
        node.on_state(type('MockMsg', (), {
            'fleet_header': type('MockHeader', (), {'robot_id': 'robot_2'})(),
            'pose': Pose2D(x=x_pos, y=5.0, theta=0.0)
        })())
        if CorridorProtocol.EXIT in released_events:
            break

    assert CorridorProtocol.EXIT in released_events, f"Owner failed to exit and release within 15s (took {t:.1f}s)"
    assert t < 15.0, f"Exit took {t:.1f}s >= 15.0s limit"

    node.destroy_node()


def test_guarded_route_retention_and_overrides(rclpy_init, monkeypatch):
    """Verify that last valid route is retained <= 3.0s, but strictly overridden by corridor holds and safety stops."""
    clock = MockClock(500.0)
    monkeypatch.setattr('sih_amr_fleet.path_follower_node.now_seconds', lambda node: clock.now_seconds)

    node = PathFollowerNode()
    node.robot_id = 'robot_1'
    node.pose = Pose2D(x=0.0, y=0.0, theta=0.0)

    # Initial valid route
    valid_route = RoutePlan()
    valid_route.route_feasible = True
    valid_route.task_id = 'task_001'
    valid_route.waypoints = [Pose2D(x=0.0, y=0.0, theta=0.0), Pose2D(x=5.0, y=0.0, theta=0.0)]

    node.on_route(valid_route)
    assert node.route is not None
    assert node.route_infeasible_since is None

    # Step 1: Transient route infeasibility arrives at t = 500.0s
    infeasible_route = RoutePlan()
    infeasible_route.route_feasible = False
    infeasible_route.task_id = 'task_001'
    node.on_route(infeasible_route)

    # Route must be retained at t = 501.0s (1.0s <= 3.0s limit)
    clock.advance(1.0)
    cmd1 = node.control()
    assert node.route is not None, "Valid route must be retained during transient infeasibility"

    # Step 2: Corridor hold override triggers: corridor_motion_allowed becomes False
    node.clear = False
    published = []
    node.pub = type('MockPub', (), {'publish': lambda self, msg: published.append(msg)})()
    node.control()
    assert published[-1].linear.x == 0.0, "Corridor hold must strictly override retained route to 0 speed"

    # Step 3: Safety stop override triggers: safety supervisor STOP active
    node.clear = True
    node.safety_stop = True
    published.clear()
    node.control()
    assert published[-1].linear.x == 0.0, "Safety stop must strictly override retained route to 0 speed"

    # Step 4: Expiration after 3.0s (advance to t = 504.5s > 500 + 3.0)
    node.safety_stop = False
    clock.advance(3.5)
    node.control()
    assert node.route is None, "Retained route must expire after 3.0 simulation seconds"

    node.destroy_node()


def test_no_manufactured_motion_when_desired_speed_is_zero(rclpy_init, monkeypatch):
    """Verify that when rotating in place or waiting (desired vx=0), ORCA never commands forward translation."""
    clock = MockClock(600.0)
    monkeypatch.setattr('sih_amr_fleet.orca_node.now_seconds', lambda node: clock.now_seconds)

    r1 = OrcaNode()
    r1.robot_id = 'robot_1'
    r1.radius = 0.28
    r1.pose = Pose2D(x=0.0, y=0.0, theta=0.0)
    # Robot is rotating in place: desired vx is 0.0, wz is 1.2
    r1.desired = Twist()
    r1.desired.linear.x = 0.0
    r1.desired.angular.z = 1.2

    # Peer is nearby closing in front
    track_r2 = make_track('robot_2', 0.5, 0.0, vx=-0.2, vy=0.0)
    r1.tracks = [track_r2]

    published_cmds = []
    r1.pub = type('MockPub', (), {'publish': lambda self, msg: published_cmds.append(msg)})()

    r1.control()
    assert published_cmds[-1].linear.x == 0.0, "ORCA must never command forward motion when desired speed is 0"
    assert published_cmds[-1].angular.z == 1.2, "Angular turn velocity must be preserved"

    r1.destroy_node()
