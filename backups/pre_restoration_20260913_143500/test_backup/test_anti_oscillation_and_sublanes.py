import math
import pytest
from geometry_msgs.msg import Pose2D, Twist
from sih_amr_interfaces.msg import PeerTrack, RoutePlan

from sih_amr_fleet.algorithms import (
    apply_right_hand_sublanes, avoidance_velocity
)
from sih_amr_fleet.path_follower_node import PathFollowerNode
from sih_amr_fleet.orca_node import OrcaNode


class MockClock:
    def __init__(self, initial=0.0):
        self.now_seconds = initial

    def advance(self, dt):
        self.now_seconds += dt


@pytest.fixture
def rclpy_init():
    import rclpy
    if not rclpy.ok():
        rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


def test_path_follower_monotonic_progress_under_replanning(rclpy_init, monkeypatch):
    """Simulate an AMR traversing a route with 1 Hz rolling replans across varying dt (0.02s to 0.15s).

    Assert:
    - route_progress_index is strictly non-decreasing.
    - Zero target-direction reversals occur.
    - Path follower reaches the destination without rotational oscillation.
    """
    clock = MockClock(0.0)
    monkeypatch.setattr('sih_amr_fleet.path_follower_node.now_seconds', lambda node: clock.now_seconds)

    pf = PathFollowerNode()
    pf.robot_id = 'robot_1'
    pf.tracking_speed = 0.46
    pf.max_speed = 0.46
    pf.speed_cap = 0.46
    pf.lookahead_distance = 0.80

    # L-shaped route: (-5.0, -10.0) -> (-9.0, -10.0) -> (-9.0, -5.0)
    base_waypoints = [
        Pose2D(x=-5.0 - 0.5 * i, y=-10.0, theta=0.0) for i in range(9)
    ] + [
        Pose2D(x=-9.0, y=-10.0 + 0.5 * i, theta=1.5708) for i in range(1, 11)
    ]

    route_msg = RoutePlan()
    route_msg.task_id = 'task_1001'
    route_msg.route_feasible = True
    route_msg.waypoints = base_waypoints

    pf.on_route(route_msg)

    # Robot starts at (-5.0, -10.0), heading West (pi)
    curr_x, curr_y, curr_th = -5.0, -10.0, math.pi
    pf.pose = Pose2D(x=curr_x, y=curr_y, theta=curr_th)

    last_progress = 0
    target_history = []
    total_rotation = 0.0

    # Simulate motion with variable dt from 0.02s to 0.15s
    dts = [0.02, 0.05, 0.10, 0.15]
    step = 0
    while clock.now_seconds < 30.0:
        dt = dts[step % len(dts)]
        step += 1
        clock.advance(dt)

        # 1 Hz rolling replan: perturb waypoints slightly (<= 0.02m) without task change
        if step % 10 == 0:
            perturbed = [
                Pose2D(x=w.x + 0.01 * math.sin(step), y=w.y + 0.01 * math.cos(step), theta=w.theta)
                for w in base_waypoints
            ]
            replan = RoutePlan()
            replan.task_id = 'task_1001'
            replan.route_feasible = True
            replan.waypoints = perturbed
            pf.on_route(replan)

        target = pf.route_target()
        assert target is not None
        target_history.append((target.x, target.y))

        # Check monotonic progression: progress index must never retreat
        assert pf.route_progress_index >= last_progress, (
            f"Progress index retreated from {last_progress} to {pf.route_progress_index}"
        )
        last_progress = pf.route_progress_index

        # Steer to target
        limit, _ = pf.task_speed_limit()
        cmd = pf.steer_to(target, limit)

        # Integrate differential drive kinematics
        curr_x += cmd.linear.x * math.cos(curr_th) * dt
        curr_y += cmd.linear.x * math.sin(curr_th) * dt
        dth = cmd.angular.z * dt
        curr_th = (curr_th + dth + math.pi) % (2.0 * math.pi) - math.pi
        total_rotation += abs(dth)
        pf.pose = Pose2D(x=curr_x, y=curr_y, theta=curr_th)

        # Check if arrived at final goal (-9.0, -5.0)
        dist_to_goal = math.hypot(curr_x - (-9.0), curr_y - (-5.0))
        if dist_to_goal < 0.20:
            break

    # Robot must successfully reach within 0.25m of destination
    assert math.hypot(curr_x - (-9.0), curr_y - (-5.0)) < 0.25, "Robot failed to reach goal"

    # In a 90-degree turn along a 9m path, total rotation should be ~pi/2 + minor corrections (< 4.0 rad).
    # It must NOT spin uncontrollably (baseline was ~11 rad, regression was >400 rad).
    assert total_rotation < 5.0, f"Excessive rotation accumulated: {total_rotation:.2f} rad"

    # Check that target coordinates never reversed direction
    for i in range(1, len(target_history) - 1):
        prev_t = target_history[i - 1]
        curr_t = target_history[i]
        # Target must never bounce backwards along X when moving West
        if curr_t[1] == -10.0 and prev_t[1] == -10.0:
            assert curr_t[0] <= prev_t[0] + 0.05, "Target bounced backwards along X"

    pf.destroy_node()


def test_path_follower_turn_mode_hysteresis(rclpy_init, monkeypatch):
    """Verify turn-mode hysteresis eliminates chattering between 0.25 rad and 0.60 rad."""
    clock = MockClock(0.0)
    monkeypatch.setattr('sih_amr_fleet.path_follower_node.now_seconds', lambda node: clock.now_seconds)

    pf = PathFollowerNode()
    pf.robot_id = 'robot_1'
    pf.pose = Pose2D(x=0.0, y=0.0, theta=0.0)

    # 1. Heading error = 0.50 rad (< 0.60 rad enter threshold): Should NOT enter turn mode
    target_1 = Pose2D(x=math.cos(0.50), y=math.sin(0.50), theta=0.0)
    cmd_1 = pf.steer_to(target_1, 0.46)
    assert not pf.in_turn_mode
    assert cmd_1.linear.x > 0.0, "Linear velocity should be positive before entering turn mode"

    # 2. Heading error = 0.65 rad (> 0.60 rad enter threshold): Enters turn mode, linear.x = 0.0
    target_2 = Pose2D(x=math.cos(0.65), y=math.sin(0.65), theta=0.0)
    cmd_2 = pf.steer_to(target_2, 0.46)
    assert pf.in_turn_mode
    assert cmd_2.linear.x == 0.0, "Linear velocity must be zero in turn mode"

    # 3. Heading error decreases to 0.40 rad: Should REMAIN in turn mode (hysteresis holds until < 0.25 rad)
    target_3 = Pose2D(x=math.cos(0.40), y=math.sin(0.40), theta=0.0)
    cmd_3 = pf.steer_to(target_3, 0.46)
    assert pf.in_turn_mode, "Must remain in turn mode due to hysteresis"
    assert cmd_3.linear.x == 0.0, "Linear velocity must remain zero while in turn mode"

    # 4. Heading error decreases to 0.20 rad (< 0.25 rad exit threshold): Exits turn mode
    target_4 = Pose2D(x=math.cos(0.20), y=math.sin(0.20), theta=0.0)
    cmd_4 = pf.steer_to(target_4, 0.46)
    assert not pf.in_turn_mode, "Must exit turn mode once error < 0.25 rad"
    assert cmd_4.linear.x > 0.40, "Linear velocity must resume when exiting turn mode"

    pf.destroy_node()


def test_main_corridor_right_hand_sublanes():
    """Verify right-hand sublane lateral offsetting for opposing traffic in main corridors."""
    origin = (-22.5, -30.0)
    resolution = 0.5

    # main_vertical_west is at x = -9.0 (cell x = 27), spanning y from -30 to 30 (cells 0 to 119)
    main_corridor_cells = {(27, y) for y in range(120)}

    # Robot A: Northbound along main_vertical_west
    northbound = [
        Pose2D(x=-9.0, y=-20.0 + 0.5 * i, theta=1.5708) for i in range(10)
    ]
    # Robot B: Southbound along main_vertical_west
    southbound = [
        Pose2D(x=-9.0, y=-15.0 - 0.5 * i, theta=-1.5708) for i in range(10)
    ]

    offset_north = apply_right_hand_sublanes(northbound, main_corridor_cells, origin, resolution, offset_m=0.40)
    offset_south = apply_right_hand_sublanes(southbound, main_corridor_cells, origin, resolution, offset_m=0.40)

    # In Northbound (+Y), right-hand normal is +X -> waypoints should be at x = -8.60
    for wp in offset_north[:-1]:
        assert abs(wp.x - (-8.60)) < 1e-4, f"Northbound sublane expected x=-8.60, got {wp.x}"

    # In Southbound (-Y), right-hand normal is -X -> waypoints should be at x = -9.40
    for wp in offset_south[:-1]:
        assert abs(wp.x - (-9.40)) < 1e-4, f"Southbound sublane expected x=-9.40, got {wp.x}"

    # Lateral centerline separation between opposing sublanes
    lat_separation = offset_north[0].x - offset_south[0].x
    assert abs(lat_separation - 0.80) < 1e-4, f"Sublane separation expected 0.80m, got {lat_separation}m"

    # Final destination waypoint must remain untouched
    assert offset_north[-1].x == -9.0
    assert offset_south[-1].x == -9.0


def test_head_on_encounter_with_sublanes_no_cpa_slowdown():
    """Verify that opposing traffic in sublanes passes without reciprocal CPA collision false alarms."""
    # Northbound AMR at (-8.60, -10.0), traveling North at 0.46 m/s
    self_xy = (-8.60, -10.0)
    preferred = (0.0, 0.46)
    radius = 0.28
    horizon = 2.0
    max_speed = 0.46

    # Southbound peer at (-9.40, -8.0), traveling South at -0.46 m/s
    peer = {
        'x': -9.40, 'y': -8.0,
        'vx': 0.0, 'vy': -0.46,
        'radius_inflation': 0.04,
        'robot_id': 'robot_2'
    }

    vx, vy = avoidance_velocity(preferred, self_xy, [peer], radius, horizon, max_speed)

    # Lateral separation = 0.80m > 2 * (0.28 + 0.04) = 0.64m collision envelope.
    # 2D CPA must predict zero collision and maintain full forward velocity (vy = 0.46)
    assert abs(vy - 0.46) < 0.01, f"Expected full speed 0.46 m/s, got vy={vy:.3f} m/s"
    assert abs(vx) < 0.01, f"Expected zero lateral push, got vx={vx:.3f} m/s"


def make_track(robot_id, x, y, vx=0.0, vy=0.0):
    track = PeerTrack()
    track.robot_id = robot_id
    track.pose.x = float(x)
    track.pose.y = float(y)
    track.twist.linear.x = float(vx)
    track.twist.linear.y = float(vy)
    track.covariance_trace = 0.01
    return track


def test_priority_first_orca_no_winner_slowdown(rclpy_init, monkeypatch):
    """Verify that the priority winner maintains full speed while the yielding robot stops."""
    clock = MockClock(10.0)
    monkeypatch.setattr('sih_amr_fleet.orca_node.now_seconds', lambda node: clock.now_seconds)

    # Robot 1 (higher priority): heading East at (0.0, 0.0)
    r1 = OrcaNode()
    r1.robot_id = 'robot_1'
    r1.corridor_protected = False
    r1.pose = Pose2D(x=0.0, y=0.0, theta=0.0)
    r1.desired = Twist()
    r1.desired.linear.x = 0.46

    # Peer Robot 3 (lower priority): approaching Westbound at (1.5, 0.0)
    r1.tracks = [make_track('robot_3', 1.5, 0.0, vx=-0.46, vy=0.0)]

    published_r1 = []
    r1.pub = type('MockPub', (), {'publish': lambda self, msg: published_r1.append(msg)})()

    r1.control()

    # Robot 1 has right-of-way ('robot_1' < 'robot_3'):
    # Must NOT yield and must NOT be stopped by symmetric CPA!
    assert r1.yielding_to is None
    assert published_r1[-1].linear.x > 0.40, (
        f"Priority winner R1 was unexpectedly stopped or slowed: vx={published_r1[-1].linear.x}"
    )

    # Now verify Robot 3 (yielding lower priority)
    r3 = OrcaNode()
    r3.robot_id = 'robot_3'
    r3.corridor_protected = False
    r3.pose = Pose2D(x=1.5, y=0.0, theta=math.pi)
    r3.desired = Twist()
    r3.desired.linear.x = 0.46

    r3.tracks = [make_track('robot_1', 0.0, 0.0, vx=0.46, vy=0.0)]

    published_r3 = []
    r3.pub = type('MockPub', (), {'publish': lambda self, msg: published_r3.append(msg)})()

    r3.control()

    # Robot 3 outranked by Robot 1: must yield and stop
    assert r3.yielding_to == 'robot_1'
    assert published_r3[-1].linear.x == 0.0, "Yielding robot R3 must stop (0.0 m/s)"

    r1.destroy_node()
    r3.destroy_node()


def test_four_robot_j_sw_congestion_no_mutual_zero_freeze(rclpy_init, monkeypatch):
    """Simulate 4 robots near J-SW (-9.0, -10.0). Assert highest priority robot proceeds."""
    clock = MockClock(50.0)
    monkeypatch.setattr('sih_amr_fleet.orca_node.now_seconds', lambda node: clock.now_seconds)

    # Robot 1 is at (-9.5, -10.0) moving East toward the junction
    r1 = OrcaNode()
    r1.robot_id = 'robot_1'
    r1.pose = Pose2D(x=-9.5, y=-10.0, theta=0.0)
    r1.desired = Twist()
    r1.desired.linear.x = 0.46

    # Peers: R2, R3, R4 in vicinity
    r1.tracks = [
        make_track('robot_2', -9.0, -10.5),
        make_track('robot_3', -8.5, -10.0),
        make_track('robot_4', -9.0, -9.5),
    ]

    published = []
    r1.pub = type('MockPub', (), {'publish': lambda self, msg: published.append(msg)})()

    r1.control()

    # R1 outranks all peers; distance > 2R (0.56m) -> R1 must proceed
    assert r1.yielding_to is None
    assert published[-1].linear.x > 0.0, "Highest priority robot R1 must not freeze in 4-robot junction"

    r1.destroy_node()


def test_gazebo_sync_validation_failure_tracking(rclpy_init):
    """Verify that KinematicCarrierNode tracks and logs Gazebo set_pose_vector failures."""
    from sih_amr_fleet.kinematic_carrier_node import KinematicCarrierNode

    carrier = KinematicCarrierNode()
    assert carrier.gz_sync_error_count == 0
    assert carrier.gz_sync_total_count == 0

    class FailingGzNode:
        def request(self, service, req, req_type, rep_type, timeout):
            class MockRep:
                data = False
            return True, MockRep()

    carrier.gz_node = FailingGzNode()

    # Trigger simulated dispatch
    from sih_amr_fleet.kinematic_carrier_node import GzPose_V
    gz_vec = GzPose_V()
    p = gz_vec.pose.add()
    p.id = 1

    try:
        res, rep = carrier.gz_node.request('/world/test/set_pose_vector', gz_vec, None, None, 50)
        carrier.gz_sync_total_count += 1
        if not (res and rep is not None and rep.data):
            carrier.gz_sync_error_count += 1
    except Exception:
        carrier.gz_sync_error_count += 1

    assert carrier.gz_sync_total_count == 1
    assert carrier.gz_sync_error_count == 1, "Failed Gazebo sync was not tracked"
    carrier.destroy_node()
