#!/usr/bin/env python3
"""Focused runtime source binding checks against preserved real launch logs."""
from __future__ import annotations

import json
import re
from pathlib import Path

from s1_runtime_source import verify_config_consumption, verify_source_binding


ROOT = Path(__file__).resolve().parents[1]
RUN = ROOT / "docs/thesis_project/evidence/S2-08/diagnostic_runs_20261011/scene_random_03/e4a9f73ae2894432b65b5de251f861db"


def fixture() -> tuple[str, dict, dict, int]:
    return fixture_for(RUN)


def fixture_for(run_dir: Path) -> tuple[str, dict, dict, int]:
    context = json.loads((run_dir / "run_context.json").read_text(encoding="utf-8"))
    rows = [json.loads(line) for line in (run_dir / "runtime_record.jsonl").read_text(encoding="utf-8").splitlines()]
    frame = rows[1]["payload"]
    managers = context["preflight"]["launched_manager_process_evidence"]
    assert len(managers) == 1
    settings = context["settings"]
    overlay = settings["overlay"]
    expected = {
        "package_share": str(Path(settings["controller_config"]).parents[2]),
        "config": settings["controller_config"],
        "launch_config": str(Path(overlay) / "share/go2_description/config/robot_control.yaml"),
        "controller_cmdline": managers[0]["cmdline"],
        "agile_model": settings["model_paths"][0],
        "ra_model": settings["model_paths"][1],
        "recovery_model": settings["model_paths"][2],
        "candidate": "paper_faithful_switch", "candidate_mode": 1,
        "group": "A", "hysteresis": "false", "hold": "false",
        "entry_threshold": -0.05, "exit_threshold": -0.05, "hold_s": 0.0,
        "goal_x": settings["goal_world_m"][0], "goal_y": settings["goal_world_m"][1],
    }
    return ((run_dir / "ros_launch.log").read_text(encoding="utf-8", errors="replace"),
            expected, frame, int(managers[0]["pid"]))


def test_previous_real_log_binds_candidate_models_config_and_writer() -> None:
    log, expected, frame, pid = fixture()
    result = verify_source_binding(log, expected, frame, pid)
    assert result["status"] == "confirmed", result
    evidence = result["evidence"]
    assert evidence["candidate"] == "paper_faithful_switch"
    assert evidence["package_share"] == expected["package_share"]
    assert evidence["agile_model"] == expected["agile_model"]
    assert evidence["ra_model"] == expected["ra_model"]
    assert evidence["recovery_model"] == expected["recovery_model"]
    assert evidence["config"] == expected["config"]
    assert evidence["goal_world_m"] == [expected["goal_x"], expected["goal_y"]]
    assert evidence["runtime_path_goal_world_m"] == [expected["goal_x"], expected["goal_y"]]
    assert evidence["controller_launch_config"] == expected["launch_config"]
    assert evidence["session_id"] == frame["session_id"]
    assert evidence["controller_pid"] == evidence["writer_pid"] == pid


def test_candidate_and_goal_mismatch_are_rejected() -> None:
    log, expected, frame, pid = fixture()
    candidate_log = log.replace("candidate=paper_faithful_switch", "candidate=stabilized_switch", 1)
    assert verify_source_binding(candidate_log, expected, frame, pid)["status"] == "conflict"
    goal_log = log.replace("goal=(7.000,0.000)", "goal=(7.500,0.000)", 1)
    assert verify_source_binding(goal_log, expected, frame, pid)["reason"] == "goal_mismatch"
    path_log = log.replace("goal=(7.00,0.00)", "goal=(7.50,0.00)", 1)
    assert verify_source_binding(path_log, expected, frame, pid)["reason"] == "runtime_path_goal_mismatch"


def test_full_model_path_and_writer_session_mismatch_are_rejected() -> None:
    log, expected, frame, pid = fixture()
    wrong_launch = dict(expected, controller_cmdline=expected["controller_cmdline"].replace(
        expected["launch_config"], "/tmp/wrong/robot_control.yaml"))
    assert verify_source_binding(log, wrong_launch, frame, pid)["reason"] == "controller_launch_config_mismatch"
    model_log = log.replace(expected["agile_model"], "/tmp/wrong/config/abs/policy.pt", 1)
    assert verify_source_binding(model_log, expected, frame, pid)["reason"] == "agile_model_mismatch"
    writer_log = log.replace(f"writer_pid={pid}", "writer_pid=999999", 1)
    assert verify_source_binding(writer_log, expected, frame, pid)["reason"] == "writer_controller_mismatch"
    wrong_session = dict(frame, session_id=int(frame["session_id"]) + 1)
    assert verify_source_binding(log, expected, wrong_session, pid)["reason"] == "writer_session_mismatch"


def test_multiple_controller_sources_and_missing_evidence_are_not_accepted() -> None:
    log, expected, frame, pid = fixture()
    source_line = next(line for line in log.splitlines() if "[S1-RUN-SOURCE]" in line)
    mixed_log = log + "\n" + source_line.replace("ros2_control_node-3", "ros2_control_node-4").replace(
        f"pid={pid}", "pid=999998", 1) + "\n"
    assert verify_source_binding(mixed_log, expected, frame, pid)["reason"] == "multiple_controller_source_events"
    missing_recovery = "\n".join(line for line in log.splitlines() if "[REC] Recovery policy loaded:" not in line)
    pending = verify_source_binding(missing_recovery, expected, frame, pid)
    assert pending["status"] == "pending" and pending["reason"] == "recovery_model_event_missing"
    missing_source = "\n".join(line for line in log.splitlines() if "[S1-RUN-SOURCE]" not in line)
    assert verify_source_binding(missing_source, expected, frame, pid)["status"] == "pending"


def test_both_completed_diagnostic_launches_pass_current_matcher() -> None:
    root = ROOT / "docs/thesis_project/evidence/S2-08/diagnostic_runs_20261011"
    runs = [root / "scene_random_01/4af238aeb65845a6967eb23dafac6c51",
            root / "scene_random_02/cea73ebffa3246438bc1ebf501334630",
            root / "scene_random_03/e4a9f73ae2894432b65b5de251f861db"]
    for run_dir in runs:
        log, expected, frame, pid = fixture_for(run_dir)
        result = verify_source_binding(log, expected, frame, pid)
        assert result["status"] == "confirmed", (run_dir, result)


def test_b_group_source_and_frame_are_bound_without_legacy_aliasing() -> None:
    log, expected, frame, pid = fixture()
    b_log = (log.replace("candidate=paper_faithful_switch", "candidate=hysteresis_switch")
             .replace("group=A", "group=B")
             .replace("effective_exit_threshold=-0.05", "effective_exit_threshold=-0.08")
             .replace("hysteresis=false", "hysteresis=true"))
    b_expected = dict(expected, candidate="hysteresis_switch", candidate_mode=2,
                      group="B", hysteresis="true", exit_threshold=-0.08)
    b_frame = dict(frame, switching_mode=2, exit_threshold=-0.08)
    result = verify_source_binding(b_log, b_expected, b_frame, pid)
    assert result["status"] == "confirmed", result
    assert result["evidence"]["group"] == "B"
    assert result["evidence"]["hysteresis"] == "true"
    wrong_frame = dict(b_frame, switching_mode=1)
    assert verify_source_binding(b_log, b_expected, wrong_frame, pid)["reason"] == "frame_candidate_mismatch"


def test_c_group_requires_actual_hold_duration_in_source_and_frame() -> None:
    log, expected, frame, pid = fixture()
    c_log = (log.replace("candidate=paper_faithful_switch", "candidate=minimum_hold_switch")
             .replace("group=A", "group=C")
             .replace("hold=false goal=", "hold=true hold_s=0.3 goal=")
             .replace("hold=false\n", "hold=true hold_s=0.3\n"))
    c_expected = dict(expected, candidate="minimum_hold_switch", candidate_mode=3,
                      group="C", hysteresis="false", hold="true", hold_s=0.30)
    c_frame = dict(frame, switching_mode=3, hold_enabled=True, recovery_min_hold_s=0.30)
    assert verify_source_binding(c_log, c_expected, c_frame, pid)["status"] == "confirmed"
    missing = c_log.replace("hold_s=0.3", "hold_s=0.0")
    assert verify_source_binding(missing, c_expected, c_frame, pid)["reason"] == "hold_duration_mismatch"


def test_d_group_binds_combined_threshold_and_hold_source_to_frame() -> None:
    log, expected, frame, pid = fixture()
    d_log = (log.replace("candidate=paper_faithful_switch", "candidate=combined_switch")
             .replace("group=A", "group=D")
             .replace("effective_exit_threshold=-0.05", "effective_exit_threshold=-0.08")
             .replace("hysteresis=false", "hysteresis=true")
             .replace("hold=false goal=", "hold=true hold_s=0.3 goal=")
             .replace("hold=false\n", "hold=true hold_s=0.3\n"))
    d_expected = dict(expected, candidate="combined_switch", candidate_mode=4,
                      group="D", hysteresis="true", hold="true",
                      exit_threshold=-0.08, hold_s=0.30)
    d_frame = dict(frame, switching_mode=4, exit_threshold=-0.08,
                   hold_enabled=True, recovery_min_hold_s=0.30)
    result = verify_source_binding(d_log, d_expected, d_frame, pid)
    assert result["status"] == "confirmed", result
    assert result["evidence"]["group"] == "D"
    assert result["evidence"]["hysteresis"] == "true"
    assert result["evidence"]["hold"] == "true"
    assert result["evidence"]["recovery_min_hold_s"] == 0.3


def test_current_c_initialization_log_binds_config_models_and_controller_pid() -> None:
    run_dir = ROOT / "logs/manual_ab_compare/20261011_150533_31b9d4/C/e42ac1001777404a941f2708cefd01ad"
    context = json.loads((run_dir / "run_context.json").read_text(encoding="utf-8"))
    log = (run_dir / "ros_launch.log").read_text(encoding="utf-8", errors="replace")
    pid_match = re.search(r"\[S1-RUN-SOURCE\] pid=(\d+)", log)
    command_match = re.search(r"process has died \[pid \d+, exit code -?\d+, cmd '([^']+)'\]", log)
    assert pid_match and command_match
    settings = context["settings"]
    overlay = Path(settings["overlay"])
    expected = {
        "controller_cmdline": command_match.group(1),
        "launch_config": str(overlay / "share/go2_description/config/robot_control.yaml"),
        "package_share": str(overlay / "share/go2_description"),
        "config": settings["controller_config"],
        "agile_model": settings["model_paths"][0],
        "ra_model": settings["model_paths"][1],
        "recovery_model": settings["model_paths"][2],
        "candidate": settings["candidate"], "group": settings["group"],
        "entry_threshold": settings["entry_threshold"], "exit_threshold": settings["exit_threshold"],
        "hysteresis": settings["hysteresis"], "hold": settings["hold"],
        "hold_s": settings["recovery_min_hold_s"],
        "goal_x": settings["goal_world_m"][0], "goal_y": settings["goal_world_m"][1],
    }
    result = verify_config_consumption(log, expected, int(pid_match.group(1)))
    assert result["status"] == "confirmed", result
    assert result["evidence"]["group"] == "C"
    assert result["evidence"]["hold_s"] == 0.3


if __name__ == "__main__":
    test_previous_real_log_binds_candidate_models_config_and_writer()
    test_candidate_and_goal_mismatch_are_rejected()
    test_full_model_path_and_writer_session_mismatch_are_rejected()
    test_multiple_controller_sources_and_missing_evidence_are_not_accepted()
    test_both_completed_diagnostic_launches_pass_current_matcher()
    test_b_group_source_and_frame_are_bound_without_legacy_aliasing()
    test_c_group_requires_actual_hold_duration_in_source_and_frame()
    test_current_c_initialization_log_binds_config_models_and_controller_pid()
    print("S1 runtime source binding checks PASS")
