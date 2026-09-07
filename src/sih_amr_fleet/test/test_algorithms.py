import math
from sih_amr_fleet.algorithms import ConstantVelocityTrack, avoidance_velocity, whca_star
from sih_amr_fleet.warehouse_tasks import aisle_points, narrow_lanes


def test_whca_basic_path():
    path = whca_star(start=(0, 0), goal=(3, 0), blocked=set(), reservations=set(), width=5, height=5, horizon=8)
    assert len(path) == 4
    assert path[0] == (0, 0, 0)
    assert path[-1] == (3, 0, 3)


def test_whca_avoids_reserved_cell():
    # Cell (1, 0) is reserved at time slot 1
    path = whca_star(start=(0, 0), goal=(3, 0), blocked=set(), reservations={(1, 0, 1)}, width=5, height=5, horizon=8)
    assert path
    assert (1, 0, 1) not in path
    # Robot will wait or route around
    assert path[-1][0] == 3 and path[-1][1] == 0


def test_whca_prevents_edge_swap():
    # Peer moving from (1, 0) at t=0 to (0, 0) at t=1.
    # Self moving from (0, 0) at t=0 to (1, 0) at t=1 would be an edge swap.
    peer_reservations = {(1, 0, 0), (0, 0, 1)}
    path = whca_star(start=(0, 0), goal=(2, 0), blocked=set(), reservations=peer_reservations, width=5, height=5, horizon=8)
    assert path
    # First step should NOT be (1, 0, 1) because that would head-on swap with peer
    assert path[1] != (1, 0, 1)


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
