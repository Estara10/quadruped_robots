#!/usr/bin/env python3
"""Targeted tests for saved-cycle C hold analysis; synthetic rows only."""

from s1_analyze_run import audit_hold_episodes


def row(cycle, sim_time, before, after, ra, status, elapsed, valid=True):
    return {"switching_mode": 3, "mode_before": before, "mode_after": after,
            "session_id": 9, "rl_step": cycle, "sim_time_s": sim_time,
            "sim_clock_valid": valid, "sim_clock_segment_id": 2,
            "hold_enabled": True, "recovery_min_hold_s": 0.30,
            "hold_enter_sim_time_s": 1.0,
            "hold_elapsed_sim_s": elapsed, "hold_status": status,
            "ra_value": ra, "entry_threshold": -0.05}


def test_closed_episode_passes_at_h_and_exits_only_below_e():
    rows = [row(1, 1.0, 0, 1, -0.01, 2, 0.0),
            row(2, 1.1, 1, 1, -0.06, 2, 0.1),
            row(3, 1.3, 1, 0, -0.06, 3, 0.3)]
    result = audit_hold_episodes(rows)
    assert result["status"] == "pass", result
    episode = result["episodes"][0]
    assert episode["exit_cycle"] == 3 and abs(episode["observed_exit_elapsed_s"] - 0.3) < 1e-9


def test_early_exit_fails_and_unclosed_episode_is_censored():
    early = [row(1, 1.0, 0, 1, -0.01, 2, 0.0),
             row(2, 1.2, 1, 0, -0.06, 2, 0.2)]
    assert audit_hold_episodes(early)["status"] == "fail"
    censored = [row(1, 1.0, 0, 1, -0.01, 2, 0.0),
                row(2, 1.1, 1, 1, -0.06, 2, 0.1)]
    result = audit_hold_episodes(censored)
    assert result["status"] == "incomplete" and result["episodes"][0]["censored"]


def test_invalid_clock_marks_episode_coverage_incomplete():
    rows = [row(1, 1.0, 0, 1, -0.01, 2, 0.0),
            row(2, None, 1, 1, -0.06, 4, None, valid=False),
            row(3, 1.4, 1, 0, -0.06, 3, 0.4)]
    result = audit_hold_episodes(rows)
    assert result["status"] == "incomplete", result
    assert result["episodes"][0]["clock_coverage"] == "incomplete"


def test_d_combines_hysteresis_with_hold_and_exits_below_x():
    rows = [
        row(1, 1.0, 0, 1, -0.04, 2, 0.0),
        row(2, 1.2, 1, 1, -0.06, 2, 0.2),
        row(3, 1.3, 1, 1, -0.06, 3, 0.3),
        row(4, 1.35, 1, 1, -0.04, 3, 0.35),
        row(5, 1.4, 1, 0, -0.081, 3, 0.4),
    ]
    for item in rows:
        item["switching_mode"] = 4
        item["exit_threshold"] = -0.08
    result = audit_hold_episodes(rows)
    assert result["status"] == "pass", result
    episode = result["episodes"][0]
    assert episode["exit_cycle"] == 5
    assert abs(episode["observed_exit_elapsed_s"] - 0.4) < 1e-9


def test_d_float_boundary_uses_producer_expiry_status():
    enter_time = 24.833999999997115
    rows = [row(69, enter_time, 0, 1, -0.026, 2, 0.0)]
    for cycle in range(70, 84):
        sim_time = enter_time + (cycle - 69) * 0.02
        rows.append(row(cycle, sim_time, 1, 1, -0.1, 2, sim_time - enter_time))
    rows.extend([
        row(84, 25.13399999999695, 1, 1, -0.239, 2, 0.29999999999983373),
        row(85, 25.153999999996937, 1, 0, -0.248, 3, 0.31999999999982265),
    ])
    for item in rows:
        item["switching_mode"] = 4
        item["exit_threshold"] = -0.08
        item["hold_enter_sim_time_s"] = enter_time
    result = audit_hold_episodes(rows)
    assert result["status"] == "pass", result
    assert result["episodes"][0]["exit_cycle"] == 85


def test_d_expired_low_risk_cycle_must_exit_that_cycle():
    enter_time = 1.0
    rows = [
        row(1, enter_time, 0, 1, -0.026, 2, 0.0),
        row(2, 1.3, 1, 1, -0.239, 3, 0.30000000000000004),
    ]
    for item in rows:
        item["switching_mode"] = 4
        item["exit_threshold"] = -0.08
        item["hold_enter_sim_time_s"] = enter_time
    result = audit_hold_episodes(rows)
    assert result["status"] == "fail", result
    assert any(v["reason"] == "D_did_not_exit_when_H_expired_and_RA_below_X"
               for v in result["violations"])


if __name__ == "__main__":
    test_closed_episode_passes_at_h_and_exits_only_below_e()
    test_early_exit_fails_and_unclosed_episode_is_censored()
    test_invalid_clock_marks_episode_coverage_incomplete()
    test_d_combines_hysteresis_with_hold_and_exits_below_x()
    test_d_float_boundary_uses_producer_expiry_status()
    test_d_expired_low_risk_cycle_must_exit_that_cycle()
    print("S4-01/S5-01 hold analysis checks PASS")
