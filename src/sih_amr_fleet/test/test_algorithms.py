from sih_amr_fleet.algorithms import ConstantVelocityTrack, avoidance_velocity, whca_star


def test_whca_avoids_reserved_cell():
    path = whca_star((0, 0), (3, 0), set(), {(1, 0, 1)}, 5, 5, 8)
    assert path
    assert (1, 0, 1) not in path


def test_track_prediction_increases_uncertainty():
    track = ConstantVelocityTrack(0.0, 0.0, 1.0, 0.0)
    before = track.variance
    track.predict(1.0)
    assert track.x == 1.0
    assert track.variance > before


def test_avoidance_reduces_head_on_speed():
    safe = avoidance_velocity((0.4, 0.0), (0.0, 0.0), [
        {'x': 0.4, 'y': 0.0, 'vx': -0.2, 'vy': 0.0, 'radius_inflation': 0.0}
    ], radius=0.28, horizon=1.5, max_speed=0.45)
    assert safe[0] < 0.4
