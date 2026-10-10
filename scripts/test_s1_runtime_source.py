#!/usr/bin/env python3
"""Focused S1-04 source binding regression against a preserved real launch log."""
from __future__ import annotations

import json
from pathlib import Path

from s1_runtime_source import verify_source_binding


ROOT = Path(__file__).resolve().parents[1]
RUN = ROOT / "docs/thesis_project/evidence/S1-04/diagnostic_runs_ray_clock_fix_20261008/6ff88d266e174f71a74b8f2a64f1c651"


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
        "entry_threshold": -0.05, "exit_threshold": -0.05,
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
    assert evidence["goal_world_m"] == [1.0, 0.0]
    assert evidence["runtime_path_goal_world_m"] == [1.0, 0.0]
    assert evidence["controller_launch_config"] == expected["launch_config"]
    assert evidence["session_id"] == frame["session_id"]
    assert evidence["controller_pid"] == evidence["writer_pid"] == pid


def test_candidate_and_goal_mismatch_are_rejected() -> None:
    log, expected, frame, pid = fixture()
    candidate_log = log.replace("candidate=paper_faithful_switch", "candidate=stabilized_switch", 1)
    assert verify_source_binding(candidate_log, expected, frame, pid)["status"] == "conflict"
    goal_log = log.replace("goal=(1.000,0.000)", "goal=(1.500,0.000)", 1)
    assert verify_source_binding(goal_log, expected, frame, pid)["reason"] == "goal_mismatch"
    path_log = log.replace("goal=(1.00,0.00)", "goal=(1.50,0.00)", 1)
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
    root = ROOT / "docs/thesis_project/evidence/S1-04/diagnostic_runs_source_matcher_20261008"
    runs = [root / "cb47356b98104e40b258b81a8246db17",
            root / "22e64352d7c042708b96608a16a89221"]
    for run_dir in runs:
        log, expected, frame, pid = fixture_for(run_dir)
        result = verify_source_binding(log, expected, frame, pid)
        assert result["status"] == "confirmed", (run_dir, result)


if __name__ == "__main__":
    test_previous_real_log_binds_candidate_models_config_and_writer()
    test_candidate_and_goal_mismatch_are_rejected()
    test_full_model_path_and_writer_session_mismatch_are_rejected()
    test_multiple_controller_sources_and_missing_evidence_are_not_accepted()
    test_both_completed_diagnostic_launches_pass_current_matcher()
    print("S1 runtime source binding checks PASS")
