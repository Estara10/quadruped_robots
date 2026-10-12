#!/usr/bin/env python3
"""Focused S6-03 preparation checks; no ROS, MuJoCo, or policy execution."""
from __future__ import annotations

import json
import sys
import tempfile
import time
from types import SimpleNamespace
from pathlib import Path
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from abs_collision import CollisionStatus  # noqa: E402
from manual_ab_compare import (prepare_overlay, save_comparison,  # noqa: E402
                               summarize_startup_without_runtime_record,
                               validate_switch_parameters)
from s1_analyze_run import motion_safety  # noqa: E402
import s1_run_diagnostic as diagnostic  # noqa: E402
from s1_run_diagnostic import (confirm_pre_rl_standing, extract_first_active_policy_state,  # noqa: E402
                               numeric_setting_matches, overlay_configuration_matches,
                               rl_entry_precondition_failure,
                               task_window_history_events)


def test_parameter_validation_and_independent_group_overlays() -> None:
    validate_switch_parameters(-0.05, -0.08, 0.30)
    try:
        validate_switch_parameters(float("nan"), -0.08, 0.30)
    except ValueError:
        pass
    else:
        raise AssertionError("nonfinite E must be rejected")
    try:
        validate_switch_parameters(-0.05, -0.05, 0.30)
    except ValueError:
        pass
    else:
        raise AssertionError("X >= E must be rejected")
    try:
        validate_switch_parameters(-0.05, -0.08, 0.0)
    except ValueError:
        pass
    else:
        raise AssertionError("nonpositive H must be rejected")

    with tempfile.TemporaryDirectory(prefix="s6-03-overlay-") as tmp:
        root = Path(tmp)
        source = root / "source"
        config = source / "share/go2_description/config/abs/config.yaml"
        config.parent.mkdir(parents=True)
        original = """  switching:\n    group: \"A\"\n    entry_threshold: -0.05\n  goal_x: 1.0\n  goal_y: 0.0\n"""
        config.write_text(original, encoding="utf-8")
        overlays = {}
        for group in "ABCD":
            destination = root / group
            prepare_overlay(source, destination, group, -0.05, -0.08, 0.30)
            overlays[group] = (destination / config.relative_to(source)).read_text(encoding="utf-8")
            assert overlay_configuration_matches(overlays[group], group, -0.05, -0.08, 0.30, 7.0)
        assert config.read_text(encoding="utf-8") == original
        assert "exit_threshold:" not in overlays["A"] and "recovery_min_hold_s:" not in overlays["A"]
        assert "exit_threshold: -0.080000000000000002" in overlays["B"]
        assert "recovery_min_hold_s:" not in overlays["B"]
        assert "exit_threshold:" not in overlays["C"]
        assert "recovery_min_hold_s: 0.29999999999999999" in overlays["C"]
        assert "exit_threshold: -0.080000000000000002" in overlays["D"]
        assert "recovery_min_hold_s: 0.29999999999999999" in overlays["D"]
    assert numeric_setting_matches("    entry_threshold: -0.05\n", "entry_threshold", -0.05)
    assert numeric_setting_matches("    recovery_min_hold_s: 0.30\n",
                                   "recovery_min_hold_s", 0.3)
    assert not numeric_setting_matches("    entry_threshold: nan\n", "entry_threshold", -0.05)
    assert not numeric_setting_matches("    entry_threshold: -0.06\n", "entry_threshold", -0.05)
    assert not overlay_configuration_matches(overlays["A"].replace("goal_x: 7.0", "goal_x: 8.0"),
                                             "A", -0.05, -0.08, 0.30, 7.0)


def test_task_window_runtime_history_uses_active_boundary() -> None:
    payload = {"status": CollisionStatus.LIVE.value,
               "collision_history": [
                   {"start_sim_time": 1.0, "end_sim_time": 1.1,
                    "start_physics_step": 100, "end_physics_step": 110},
                   {"start_sim_time": 2.0, "end_sim_time": 2.1,
                    "start_physics_step": 200, "end_physics_step": 210}],
               "fall_history": [
                   {"start_sim_time": 0.8, "end_sim_time": 0.9,
                    "start_physics_step": 80, "end_physics_step": 90,
                    "confirmed": True},
                   {"start_sim_time": 3.0, "end_sim_time": 3.1,
                    "start_physics_step": 300, "end_physics_step": 310,
                    "confirmed": True}]}
    collisions, falls = task_window_history_events(payload, 1.5, 3.2, 90)
    assert collisions == [payload["collision_history"][1]]
    assert falls == [payload["fall_history"][1]]
    # Invalid first-cycle sim_time falls back to the pre-RL physics-step boundary.
    collisions, falls = task_window_history_events(payload, None, None, 90)
    assert collisions == payload["collision_history"]
    assert falls == [payload["fall_history"][1]]
    assert task_window_history_events(payload, None, None, None) == ([], [])


def test_any_failed_preparation_gate_refuses_rl_entry() -> None:
    ready = {"recorder_ready": True, "standing_confirmed": True,
             "joint_sample_valid": True, "joint_velocity_within_limit": True,
             "pre_task_safety_boundary_live": True}
    assert rl_entry_precondition_failure(ready) is None
    for key in ready:
        failed = dict(ready, **{key: False})
        assert rl_entry_precondition_failure(failed) is not None


def _standing_snapshot(sequence: int, sim_time: float, *, height: float = 0.30,
                       candidate: bool = False, confirmed: bool = False,
                       history: tuple[dict, ...] = ()) -> SimpleNamespace:
    return SimpleNamespace(sequence=sequence, physics_step=sequence * 5,
        sim_time=sim_time, physics_coverage_complete=1, unknown_contacts=0,
        fall_candidate=int(candidate), fall_confirmed=int(confirmed),
        robot_obstacle_contacts=0, base_height_m=height, base_roll_rad=0.02,
        base_pitch_rad=-0.01, foot_ground_contacts=4, invalid_posture_steps=0,
        first_invalid_posture_sim_time=0.0, last_invalid_posture_sim_time=0.0,
        fall_history=history)


def _run_standing_observer(snapshots, *, wall_cap_s=0.2,
                           authority_status=CollisionStatus.LIVE,
                           stand_ready: bool = True):
    iterator = iter(snapshots)
    last = [None]

    def read_snapshot():
        return b"snapshot", {"source": "test-double-for-production-observer"}

    def classify(*_args, **_kwargs):
        try:
            last[0] = next(iterator)
        except StopIteration:
            if last[0] is None:
                raise AssertionError("test supplied no physics samples")
        return authority_status, last[0]

    fake_scene = SimpleNamespace(model_fingerprint="fingerprint", binding=lambda: {})
    with tempfile.TemporaryDirectory(prefix="s6-03-stand-log-") as tmp:
        ros_log = Path(tmp) / "ros.log"
        if stand_ready:
            ros_log.write_text("[BaseFixedStand]: [STAND-READY] command3_gate=passed percent=1.500000 phase=0.905148\n",
                               encoding="utf-8")
        else:
            ros_log.write_text("[BaseFixedStand]: [STAND-SYMM] phase=0.684\n", encoding="utf-8")
        with patch.object(diagnostic, "read_collision_snapshot_diagnostic", side_effect=read_snapshot), \
             patch.object(diagnostic, "classify_snapshot", side_effect=classify), \
             patch.object(diagnostic, "collision_snapshot_payload", return_value={"status": "LIVE"}), \
             patch.object(diagnostic.time, "sleep", return_value=None):
            return confirm_pre_rl_standing("capture", fake_scene, wall_cap_s=wall_cap_s,
                                           stable_sim_s=0.5, ros_log_path=ros_log, last_command=2)


def test_standing_observer_allows_normal_stand_and_closes_recovered_episode() -> None:
    normal = _run_standing_observer([_standing_snapshot(1, 0.1),
                                     _standing_snapshot(2, 0.7)])
    assert normal["status"] == "STABLE"
    assert normal["recovered_after_confirmed_preparation_fall"] is False
    assert normal["observations"][-1]["controller"]["last_supervisor_command"] == 2

    open_event = {"start_sim_time": 0.1, "start_physics_step": 5,
                  "confirmed_sim_time": 0.4, "confirmed_physics_step": 20,
                  "end_sim_time": 0.5, "end_physics_step": 25,
                  "confirmed": True, "closed": False}
    closed_event = {**open_event, "end_sim_time": 0.6,
                    "end_physics_step": 30, "closed": True}
    recovered = _run_standing_observer([
        _standing_snapshot(1, 0.5, height=0.18, candidate=True,
                           confirmed=True, history=(open_event,)),
        _standing_snapshot(2, 0.6, history=(closed_event,)),
        _standing_snapshot(3, 1.2, history=(closed_event,)),
    ])
    assert recovered["status"] == "STABLE"
    assert recovered["recovered_after_confirmed_preparation_fall"] is True
    assert recovered["preparation_events"][0]["kind"] == "confirmed_fall"
    assert any(event["kind"] == "confirmed_fall_episode_closed_after_normal_posture"
               for event in recovered["preparation_events"])
    assert recovered["last_collision_snapshot"] == {"status": "LIVE"}


def test_standing_observer_rejects_persistent_anomaly_invalid_source_and_timeout() -> None:
    open_event = {"start_sim_time": 0.1, "start_physics_step": 5,
                  "confirmed_sim_time": 0.4, "confirmed_physics_step": 20,
                  "end_sim_time": 0.6, "end_physics_step": 30,
                  "confirmed": True, "closed": False}
    start = time.monotonic()

    def continuing_samples():
        index = 0
        while time.monotonic() - start < 0.06:
            index += 1
            yield _standing_snapshot(index, 0.5 + index * 0.001, height=0.18,
                                     candidate=True, confirmed=True, history=(open_event,))

    persistent = _run_standing_observer(continuing_samples(), wall_cap_s=0.03)
    assert persistent["status"] == "PREPARATION_FAILURE"
    assert persistent["reason"] == "stable standing interval not confirmed before wall cap"
    assert persistent["last_observation"]["fall_episode_active"] is True
    assert persistent["last_observation"]["base_height_m"] == 0.18

    invalid = _run_standing_observer([_standing_snapshot(1, 0.2)],
                                     authority_status=CollisionStatus.STALE)
    assert invalid["status"] == "PREPARATION_FAILURE"
    assert "physics authority STALE" in invalid["reason"]

    paused = _run_standing_observer(
        (_standing_snapshot(i, 0.1) for i in range(1, 10000)), wall_cap_s=0.02)
    assert paused["status"] == "PREPARATION_FAILURE"
    assert paused["reason"] == "stable standing interval not confirmed before wall cap"

    ramping = _run_standing_observer([_standing_snapshot(1, 0.1),
                                      _standing_snapshot(2, 0.7)],
                                     stand_ready=False)
    assert ramping["status"] == "PREPARATION_FAILURE"
    assert ramping["reason"] == "stable standing interval not confirmed before wall cap"
    assert "transition gate" in ramping["last_observation"]["reason"]


def _complete_snapshot(sim_time: float, coverage: bool = True) -> dict:
    return {"status": "LIVE", "version": 5, "sim_time": sim_time,
            "physics_coverage_complete": coverage,
            "collision_history_overflow": False, "fall_history_overflow": False,
            "event_latches_complete": True, "foot_identity_valid": True,
            "foot_contact_history_overflow": False, "foot_impact_history_overflow": False,
            "foot_contact_history": [], "foot_impact_history": [],
            "collision_history": [], "fall_history": [],
            "unknown_contacts": 0, "unknown_contact_steps": 0,
            "first_unknown_contact_sim_time": 0.0, "last_unknown_contact_sim_time": 0.0,
            "invalid_posture_steps": 0, "first_invalid_posture_sim_time": 0.0,
            "last_invalid_posture_sim_time": 0.0,
            "foot_impact_unknown_steps": 0,
            "foot_impact_unknown_first_sim_time": 0.0,
            "foot_impact_unknown_last_sim_time": 0.0,
            "foot_contact_duration_s": 0.0, "foot_impact_duration_s": 0.0,
            "foot_ground_contacts": 4, "nonfoot_ground_contacts": 0, "self_contacts": 0}


def test_safety_no_event_requires_complete_coverage() -> None:
    frames = [{"collision_snapshot": _complete_snapshot(0.5)},
              {"collision_snapshot": _complete_snapshot(1.0)}]
    boundary = [_complete_snapshot(0.0)]
    terminal = _complete_snapshot(1.0)
    complete = motion_safety(frames, 0.0, 1.0, terminal, boundary)
    assert complete["collision"] is False and complete["fall"] is False
    incomplete_frames = [{"collision_snapshot": _complete_snapshot(0.5)},
                         {"collision_snapshot": _complete_snapshot(1.0, coverage=False)}]
    incomplete = motion_safety(incomplete_frames, 0.0, 1.0, terminal, boundary)
    assert incomplete["collision"] == "UNKNOWN" and incomplete["fall"] == "UNKNOWN"


def test_first_active_companion_requires_exact_persisted_cycle() -> None:
    state = {"session_id": 77, "cycle_id": 0, "frame_monotonic_ns": 900,
             "joint_order": [f"j{i}" for i in range(12)],
             "joint_position_rad": [float(i) for i in range(12)],
             "joint_velocity_rad_s": [0.0] * 12,
             "imu_quaternion": [1.0, 0.0, 0.0, 0.0],
             "body_angular_velocity_rad_s": [0.0] * 3,
             "body_linear_velocity_m_s": [0.0] * 3,
             "world_position_xy_m": [0.0, 0.0], "world_yaw_rad": 0.0,
             "sim_clock_valid": True, "sim_time_s": 1.0}
    payload = {"session_id": 77, "rl_step": 0, "monotonic_ns": 900,
               "rl_entered": 1, "rl_active": 1, "sim_clock_valid": True,
               "sim_time_s": 1.0, "source_sequence": 1, "ra_value": -0.1,
               "mode_before": "AGILE", "mode_after": "AGILE", "action_source": "agile",
               "action_raw": [0.0] * 12, "action_clipped": [0.0] * 12,
               "joint_target_rad": [0.0] * 12, "world_pose": [0.0] * 7,
               "collision_snapshot": {"status": "LIVE"}}
    with tempfile.TemporaryDirectory(prefix="s6-03-first-cycle-") as tmp:
        root = Path(tmp)
        log = root / "ros.log"
        record = root / "runtime.jsonl"
        log.write_text("[ABS-INITIAL-STATE] " + json.dumps(state) + "\n", encoding="utf-8")
        record.write_text(json.dumps({"kind": "meta", "run_id": "r", "capture_id": "c"}) + "\n" +
                          json.dumps({"kind": "frame", "status": "LIVE", "payload": payload}) + "\n",
                          encoding="utf-8")
        assert extract_first_active_policy_state(log, record)["status"] == "CAPTURED"
        payload["rl_step"] = 1
        record.write_text(json.dumps({"kind": "meta", "run_id": "r", "capture_id": "c"}) + "\n" +
                          json.dumps({"kind": "frame", "status": "LIVE", "payload": payload}) + "\n",
                          encoding="utf-8")
        mismatch = extract_first_active_policy_state(log, record)
        assert mismatch["status"] == "UNKNOWN"
        assert mismatch["same_cycle_runtime_frame_persisted_by_supervisor"] is False
        # Exact matching to a later cycle is still rejected: S6-03 requires actual cycle 0.
        state["cycle_id"] = 1
        payload["rl_step"] = 1
        payload["monotonic_ns"] = 901
        log.write_text("[ABS-INITIAL-STATE] " + json.dumps(state) + "\n", encoding="utf-8")
        record.write_text(json.dumps({"kind": "meta", "run_id": "r", "capture_id": "c"}) + "\n" +
                          json.dumps({"kind": "frame", "status": "LIVE", "payload": payload}) + "\n",
                          encoding="utf-8")
        later_exact_match = extract_first_active_policy_state(log, record)
        assert later_exact_match["status"] == "UNKNOWN"
        assert later_exact_match["reason"] == "cycle_zero_runtime_frame_not_persisted"
        # An identity-correct cycle 0 without complete runtime payload is not accepted.
        state["cycle_id"] = 0
        payload["rl_step"] = 0
        payload["monotonic_ns"] = 900
        payload.pop("action_raw")
        log.write_text("[ABS-INITIAL-STATE] " + json.dumps(state) + "\n", encoding="utf-8")
        record.write_text(json.dumps({"kind": "meta", "run_id": "r", "capture_id": "c"}) + "\n" +
                          json.dumps({"kind": "frame", "status": "LIVE", "payload": payload}) + "\n",
                          encoding="utf-8")
        incomplete = extract_first_active_policy_state(log, record)
        assert incomplete["status"] == "UNKNOWN"
        assert incomplete["reason"] == "cycle_zero_runtime_frame_incomplete"


def test_batch_summary_keeps_not_started_and_unknown_separate() -> None:
    with tempfile.TemporaryDirectory(prefix="s6-03-batch-summary-") as tmp:
        root = Path(tmp)
        (root / "batch_plan.json").write_text(json.dumps({
            "repeats_per_group": 1,
            "run_statuses": [
                {"group": "A", "repeat": 1, "status": "FINISHED"},
                {"group": "D", "repeat": 1, "status": "NOT_STARTED"}],
        }), encoding="utf-8")
        save_comparison(root, {"A": [{"run_id": "run-a", "record_validity": "VALID",
                                        "navigation_terminal_result": "ARRIVED",
                                        "task_window_safety": {"collision": "UNKNOWN", "fall": False}}]},
                        ("A", "D"), {"entry_threshold_E": -0.05})
        summary = json.loads((root / "batch_summary.json").read_text(encoding="utf-8"))
        assert summary["groups"]["A"]["valid"] == 1
        assert summary["groups"]["A"]["preparation_ready"] == 0
        assert summary["groups"]["A"]["preparation_not_confirmed"] == 1
        assert summary["groups"]["A"]["task_window_collision_counts"] == {"UNKNOWN": 1}
        assert summary["groups"]["D"]["started"] == 0
        assert summary["groups"]["D"]["not_started"] == 1
        comparison = (root / "comparison.md").read_text(encoding="utf-8")
        assert "UNKNOWN" in comparison and "未运行" in comparison
        assert "supervisor原始终局" in comparison
        failed = summarize_startup_without_runtime_record(
            {"run_id": "failed", "result": "STARTUP_FAILURE"},
            {"lifecycle": {"startup_error": "preflight mismatch"}}, "scene_random_04.xml")
        assert failed["record_validity"] == "INVALID"
        assert failed["supervisor_terminal_result"] == "STARTUP_FAILURE"
        assert failed["frame_count"] == 0
        assert failed["task_window_safety"] == {"collision": "UNKNOWN", "fall": "UNKNOWN"}


if __name__ == "__main__":
    for check in (test_parameter_validation_and_independent_group_overlays,
                  test_task_window_runtime_history_uses_active_boundary,
                  test_any_failed_preparation_gate_refuses_rl_entry,
                  test_standing_observer_allows_normal_stand_and_closes_recovered_episode,
                  test_standing_observer_rejects_persistent_anomaly_invalid_source_and_timeout,
                  test_safety_no_event_requires_complete_coverage,
                  test_first_active_companion_requires_exact_persisted_cycle,
                  test_batch_summary_keeps_not_started_and_unknown_separate):
        check()
    print("S6-03 targeted preparation checks passed")
