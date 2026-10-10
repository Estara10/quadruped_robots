from types import SimpleNamespace

from s1_run_diagnostic import standing_hold_reached, standing_physics_sample_failure


def sample(**overrides):
    values = {
        "sim_time": 10.0,
        "physics_coverage_complete": 1,
        "unknown_contacts": 0,
        "fall_candidate": 0,
        "fall_confirmed": 0,
        "robot_obstacle_contacts": 0,
        "base_height_m": 0.36,
        "base_roll_rad": 0.02,
        "base_pitch_rad": -0.03,
        "foot_ground_contacts": 4,
    }
    values.update(overrides)
    return SimpleNamespace(**values)


def test_standing_physics_sample_requires_live_coverage_and_support():
    assert standing_physics_sample_failure(sample()) is None
    for overrides in (
        {"sim_time": float("nan")},
        {"physics_coverage_complete": 0},
        {"unknown_contacts": 1},
        {"fall_candidate": 1},
        {"fall_confirmed": 1},
        {"robot_obstacle_contacts": 1},
        {"base_height_m": 0.24},
        {"base_roll_rad": 0.31},
        {"base_pitch_rad": -0.31},
        {"foot_ground_contacts": 1},
    ):
        assert standing_physics_sample_failure(sample(**overrides)) is not None


def test_standing_hold_uses_sim_time_and_rejects_clock_reversal():
    assert not standing_hold_reached(10.0, 14.99, 5.0)
    assert standing_hold_reached(10.0, 15.0, 5.0)
    assert not standing_hold_reached(10.0, 9.99, 5.0)
    assert not standing_hold_reached(10.0, float("nan"), 5.0)
