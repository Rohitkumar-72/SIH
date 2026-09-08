import math
import pathlib
import pytest
import yaml
from sih_amr_fleet.algorithms import (
    ConstantVelocityTrack, DockLeaseTable, approach_policy, avoidance_velocity,
    directional_scan_minimum, finite_command, map_transform_for_anchor,
    reverse_recovery_allowed, whca_star,
)
from sih_amr_fleet.warehouse_tasks import aisle_points, narrow_lanes


def test_whca_basic_path():
    path = whca_star(start=(0, 0), goal=(3, 0), blocked=set(), reservations=set(), width=5, height=5, horizon=8)
    assert len(path) == 4
    assert path[0] == (0, 0, 0)
    assert path[-1] == (3, 0, 3)


def test_whca_avoids_reserved_cell():
    # Cell (3, 2) is reserved at time slot 1.
    path = whca_star(start=(2, 2), goal=(5, 2), blocked=set(), reservations={(3, 2, 1)}, width=7, height=5, horizon=10)
    assert path
    assert (3, 2, 1) not in path
    # Robot will wait or route around
    assert path[-1][0] == 5 and path[-1][1] == 2


def test_whca_avoids_one_cell_reservation_buffer():
    # The peer owns (1, 1) at t=1.  The direct first step (1, 2) is adjacent,
    # so the default one-cell safety buffer must rule it out too.
    path = whca_star(
        start=(0, 2), goal=(3, 2), blocked=set(), reservations={(1, 1, 1)},
        width=5, height=5, horizon=10,
    )
    assert path
    assert (1, 2, 1) not in path


def test_whca_prevents_edge_swap():
    # Peer moving from (1, 0) at t=0 to (0, 0) at t=1.
    # Self moving from (0, 0) at t=0 to (1, 0) at t=1 would be an edge swap.
    peer_reservations = {(3, 2, 0), (2, 2, 1)}
    path = whca_star(start=(2, 2), goal=(4, 2), blocked=set(), reservations=peer_reservations, width=6, height=6, horizon=8, reservation_buffer_cells=0)
    assert path
    # First step should NOT be (1, 0, 1) because that would head-on swap with peer
    assert path[1] != (3, 2, 1)


def test_whca_respects_static_obstacles():
    blocked = {(1, 0), (1, 1), (1, 2)}
    path = whca_star(start=(0, 0), goal=(2, 0), blocked=blocked, reservations=set(), width=5, height=5, horizon=10)
    assert path
    for x, y, t in path:
        assert (x, y) not in blocked


def test_whca_horizon_limit():
    # Goal is far away, horizon is only 3 steps
    path = whca_star(start=(0, 0), goal=(10, 10), blocked=set(), reservations=set(), width=20, height=20, horizon=3)
    assert len(path) == 4  # t=0, t=1, t=2, t=3
    assert path[-1][2] == 3


def test_track_prediction_increases_uncertainty():
    track = ConstantVelocityTrack(0.0, 0.0, 1.0, 0.0)
    before = track.variance
    track.predict(1.0)
    assert track.x == 1.0
    assert track.variance > before


def test_track_update_reduces_uncertainty():
    track = ConstantVelocityTrack(0.0, 0.0, 1.0, 0.0)
    track.predict(2.0)
    cov_after_predict = track.variance
    track.update(2.1, 0.05, 1.0, 0.0)
    assert track.variance < cov_after_predict


def test_avoidance_reduces_head_on_speed():
    safe = avoidance_velocity(
        preferred=(0.4, 0.0),
        self_xy=(0.0, 0.0),
        peers=[{'x': 0.4, 'y': 0.0, 'vx': -0.2, 'vy': 0.0, 'radius_inflation': 0.0}],
        radius=0.28,
        horizon=1.5,
        max_speed=0.45
    )
    assert safe[0] < 0.4


def test_avoidance_uncertainty_inflation():
    # With larger uncertainty inflation, repulsive force is stronger (lower forward velocity and stronger push)
    safe_small_cov = avoidance_velocity(
        preferred=(0.4, 0.0),
        self_xy=(0.0, 0.0),
        peers=[{'x': 0.8, 'y': 0.1, 'vx': -0.1, 'vy': 0.0, 'radius_inflation': 0.0}],
        radius=0.28,
        horizon=1.5,
        max_speed=0.45
    )
    safe_large_cov = avoidance_velocity(
        preferred=(0.4, 0.0),
        self_xy=(0.0, 0.0),
        peers=[{'x': 0.8, 'y': 0.1, 'vx': -0.1, 'vy': 0.0, 'radius_inflation': 0.3}],
        radius=0.28,
        horizon=1.5,
        max_speed=0.45
    )
    # Larger inflation produces a stronger repulsive push (forward velocity is reduced or reversed)
    assert safe_large_cov[0] < safe_small_cov[0]
    # Lateral push is also stronger
    assert abs(safe_large_cov[1]) > abs(safe_small_cov[1])


def test_avoidance_clips_max_speed():
    safe = avoidance_velocity(
        preferred=(0.8, 0.8),
        self_xy=(0.0, 0.0),
        peers=[],
        radius=0.28,
        horizon=1.5,
        max_speed=0.45
    )
    speed = math.hypot(safe[0], safe[1])
    assert speed <= 0.45 + 1e-6


def test_predefined_narrow_lanes_cover_every_storage_aisle():
    lanes = narrow_lanes()
    assert len(lanes) == 48
    assert all(lane.kind == 'narrow' for lane in lanes)
    assert any(lane.lane_id == 'NC-MIDDLE-CENTRE' for lane in lanes)
    assert any(point.x == 0.0 for point in aisle_points())


def test_dock_simultaneous_requests_have_one_lamport_owner_and_alternate():
    table = DockLeaseTable()
    table.observe('charging_pad_1', 'robot_2', 'r2', 10, 5.0, 'REQUEST', 0.0)
    table.observe('charging_pad_1', 'robot_1', 'r1', 10, 5.0, 'REQUEST', 0.0)
    assert table.owner('charging_pad_1', 0.1) == ('robot_1', 'r1')
    assert table.choose(['charging_pad_1', 'charging_pad_2'], 'robot_2', 0.1) == 'charging_pad_2'


def test_dock_lease_expiry_and_release_make_pad_available():
    table = DockLeaseTable()
    table.observe('charging_pad_1', 'robot_1', 'r1', 1, 2.0, 'CLAIM', 0.0)
    assert table.owner('charging_pad_1', 1.0) == ('robot_1', 'r1')
    table.observe('charging_pad_1', 'robot_1', 'r1', 2, 0.0, 'RELEASE', 1.1)
    assert table.owner('charging_pad_1', 1.2) is None
    table.observe('charging_pad_1', 'robot_1', 'r1', 3, 2.0, 'CLAIM', 0.0)
    assert table.owner('charging_pad_1', 2.1) is None


def test_narrow_approach_stops_without_permit_or_when_network_degraded():
    assert approach_policy(True, False, False, 0.4) == 0.0
    assert approach_policy(True, True, True, 0.4) == 0.0
    assert approach_policy(True, True, False, 0.4) == 0.4
    assert math.isinf(approach_policy(False, False, True, 0.4))


def test_recovery_reverse_rejects_protected_or_obstructed_retreat():
    assert reverse_recovery_allowed(3.0, 2.0, 0.5, False, False)
    assert not reverse_recovery_allowed(2.5, 2.0, 0.5, False, False)
    assert not reverse_recovery_allowed(5.0, 2.0, 0.5, True, False)
    assert not reverse_recovery_allowed(5.0, 2.0, 0.5, False, True)


def test_map_declares_dock_resources_and_strip_references():
    data = yaml.safe_load(pathlib.Path(__file__).parents[1].joinpath('maps/demo_warehouse.yaml').read_text())
    assert set(data['dock_resources']) == set(data['anchors'])
    assert all('centre_strip' in resource for resource in data['dock_resources'].values())


def test_confirmed_dock_anchor_transform_corrects_current_raw_odom_sample():
    origin_x, origin_y, origin_yaw = map_transform_for_anchor((3.6, -29.55, -1.57), (1.0, 0.5, 0.2))
    actual_x = origin_x + math.cos(origin_yaw) * 1.0 - math.sin(origin_yaw) * 0.5
    actual_y = origin_y + math.sin(origin_yaw) * 1.0 + math.cos(origin_yaw) * 0.5
    assert actual_x == pytest.approx(3.6)
    assert actual_y == pytest.approx(-29.55)
    assert origin_yaw + 0.2 == pytest.approx(-1.57)


def test_telemetry_schema_explicitly_keeps_map_and_raw_simulator_odom_distinct():
    source = pathlib.Path(__file__).parents[1].joinpath('sih_amr_fleet/data_collection_node.py').read_text()
    assert "'map_pose'" in source
    assert "'gazebo_odom'" in source
    assert "'schema_version': '0.2.0'" in source


def test_live_qos_matches_dock_protocol_and_nearest_obstacle_publishers():
    package_root = pathlib.Path(__file__).parents[1].joinpath('sih_amr_fleet')
    localization = package_root.joinpath('localization_node.py').read_text()
    follower = package_root.joinpath('path_follower_node.py').read_text()
    assert "DockProtocol, '/fleet/dock_protocol', self.on_dock_protocol, PROTOCOL_QOS" in localization
    assert "Float32, 'nearest_obstacle_m', lambda msg: setattr(self, 'nearest', msg.data), POSE_QOS" in follower


def test_safety_rejects_nonfinite_velocity_components():
    assert finite_command(0.0, 0.0)
    assert not finite_command(float('nan'), 0.0)
    assert not finite_command(0.0, float('inf'))


def test_directional_scan_braking_ignores_side_and_rear_returns():
    # Four beams: front, left, rear, right.  A parked AMR may be close to a
    # dock or wall at its side/rear, but only the travel sector can veto a
    # forward command.
    ranges = [2.0, 0.34, 0.20, 0.34]
    assert directional_scan_minimum(ranges, 0.0, math.pi / 2.0, 0.0, math.pi / 4.0) == pytest.approx(2.0)
    assert directional_scan_minimum(ranges, 0.0, math.pi / 2.0, math.pi, math.pi / 4.0) == pytest.approx(0.20)
    assert math.isinf(directional_scan_minimum([float('nan')], 0.0, 1.0, 0.0, 1.0))


def test_cbba_has_a_verified_fleet_state_fallback_and_exact_local_state_qos():
    package_root = pathlib.Path(__file__).parents[1].joinpath('sih_amr_fleet')
    cbba = package_root.joinpath('cbba_node.py').read_text()
    localization = package_root.joinpath('localization_node.py').read_text()
    assert "RobotState, '/fleet/robot_state', self.on_fleet_state, FLEET_STATE_QOS" in cbba
    assert "create_publisher(RobotState, 'state', POSE_QOS)" in localization


def test_task_sources_replay_pending_work_and_generator_waits_for_fleet_readiness():
    package_root = pathlib.Path(__file__).parents[1].joinpath('sih_amr_fleet')
    generator = package_root.joinpath('random_task_generator_node.py').read_text()
    scenario = package_root.joinpath('task_scenario_node.py').read_text()
    assert 'def fleet_ready(self):' in generator
    assert 'return self.expected_robot_ids.issubset(self.ready_robots)' in generator
    assert "TASK_SOURCE_QOS" in generator
    assert 'def _publish_pending(self, now):' in generator
    assert "self.active_tasks[task_id] = (announcement, now)" in generator
    assert 'self.pending[spec[\'id\']] = (msg, now)' in scenario
    assert 'TASK_SOURCE_QOS' in scenario


def test_controller_contract_is_stamped_and_bridge_rejects_nonfinite_commands():
    package_root = pathlib.Path(__file__).parents[1]
    bridge = package_root.joinpath('sih_amr_fleet/twist_stamper_node.py').read_text()
    control = package_root.joinpath('config/fleet_fast_control.yaml').read_text()
    assert 'use_stamped_vel: true' in control
    assert "TwistStamped, 'diffdrive_controller/cmd_vel'" in bridge
    assert 'Rejected nonfinite cmd_vel before controller bridge' in bridge
    assert 'def publish_idle_stop(self):' in bridge
    assert 'durability=DurabilityPolicy.TRANSIENT_LOCAL' in bridge
    assert 'self.create_timer(0.05, self.publish_idle_stop)' in bridge


def test_local_state_fallback_covers_every_motion_critical_node():
    package_root = pathlib.Path(__file__).parents[1].joinpath('sih_amr_fleet')
    for name in ('task_execution_node.py', 'whca_planner_node.py', 'path_follower_node.py',
                 'orca_node.py', 'safety_supervisor_node.py'):
        source = package_root.joinpath(name).read_text()
        assert "RobotState, 'state', self.on_local_state, POSE_QOS" in source
    safety = package_root.joinpath('safety_supervisor_node.py').read_text()
    assert 'directional_scan_minimum' in safety
    assert "'scan stale'" in safety
