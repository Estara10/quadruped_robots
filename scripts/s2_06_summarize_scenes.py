#!/usr/bin/env python3
"""Build the S2-06 descriptive comparison from production run artifacts.

Motion elapsed time is the boundary endpoint difference. Effective duration,
path length, and mean speed are limited to valid contiguous sim-time segments.
Mode transitions are counted by policy cycle within the inclusive motion-cycle
boundary; a separate count identifies transitions with valid sim timestamps.
"""
from __future__ import annotations

import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
RUNS = ROOT / "docs/thesis_project/evidence/S2-06/diagnostic_runs_20261010"
LAYOUTS = ROOT / "docs/thesis_project/designs/random_static_scenes/layouts.json"


def read_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def summarize(index: int, layout: dict) -> dict:
    scene_dir = RUNS / f"scene_random_{index:02d}"
    candidates = [path for path in scene_dir.iterdir()
                  if path.is_dir() and path.name != "analysis"]
    if len(candidates) != 1:
        raise ValueError(f"scene {index}: expected one diagnostic run, found {len(candidates)}")
    run = candidates[0]
    metrics = read_json(run / "metrics.json")
    context = read_json(run / "run_context.json")
    terminal = read_json(run / "terminal.json")
    process_facts = read_json(run / "process_facts.json")
    settings = context.get("settings", {})
    observed = context.get("runtime_config_observed", {}).get("source_binding", {})
    expected_scene = f"scene_random_{index:02d}.xml"
    if metrics.get("record_validity") != "VALID":
        raise ValueError(f"{expected_scene}: invalid record {metrics.get('record_validity_reasons')}")
    if settings.get("scene_filename") != expected_scene or settings.get("scene_id") != f"random_{index:02d}":
        raise ValueError(f"{expected_scene}: run context scene identity mismatch")
    if (observed.get("group") != "A" or observed.get("candidate") != "paper_faithful_switch" or
            abs(float(observed.get("entry_threshold", math.inf)) + 0.05) > 1e-5 or
            abs(float(observed.get("exit_threshold", math.inf)) + 0.05) > 1e-5):
        raise ValueError(f"{expected_scene}: consumed config differs from fixed A settings")
    if settings.get("goal_world_m") != [7.0, 0.0] or settings.get("arrival_threshold_m") != 0.5:
        raise ValueError(f"{expected_scene}: goal/arrival settings mismatch")
    if (settings.get("diagnostic_sim_limit_after_motion_s") != 20.0 or
            process_facts.get("closeout_complete") is not True or
            process_facts.get("shutdown_complete") is not True):
        raise ValueError(f"{expected_scene}: run limit or closeout facts mismatch")

    motion = metrics.get("motion_metric_time_boundary", {})
    start_s, end_s = motion.get("start_sim_time_s"), motion.get("end_sim_time_s")
    start_cycle, end_cycle = motion.get("start_cycle"), motion.get("end_cycle")
    if not isinstance(start_s, (int, float)) or not isinstance(end_s, (int, float)):
        raise ValueError(f"{expected_scene}: missing bounded motion interval")
    if not isinstance(start_cycle, int) or not isinstance(end_cycle, int) or start_cycle > end_cycle:
        raise ValueError(f"{expected_scene}: missing bounded motion cycle interval")
    records = [json.loads(line) for line in (run / "runtime_record.jsonl").read_text(encoding="utf-8").splitlines()
               if line.strip()]
    frames = [item["payload"] for item in records if item.get("status") == "LIVE" and
              isinstance(item.get("payload"), dict)]
    motion_frames = [frame for frame in frames if frame.get("sim_clock_valid") is True and
                     isinstance(frame.get("sim_time_s"), (int, float)) and
                     math.isfinite(frame["sim_time_s"]) and
                     start_s <= frame["sim_time_s"] <= end_s]
    # Use cycle identity for all mode changes, including changes whose timestamp
    # is invalid. Invalid timestamps exclude a transition from time-based metrics,
    # but do not erase the observed mode change.
    motion_cycle_frames = [frame for frame in frames if isinstance(frame.get("rl_step"), int) and
                           start_cycle <= frame["rl_step"] <= end_cycle]
    if not motion_frames or not motion.get("terminal_frame_payload_complete"):
        raise ValueError(f"{expected_scene}: missing complete in-motion terminal payload")
    sessions = {frame.get("session_id") for frame in frames}
    if sessions != {metrics.get("session_id")}:
        raise ValueError(f"{expected_scene}: mixed or mismatched sessions {sessions}")
    ray_valid_frames = sum(frame.get("ray_valid") == 1 for frame in frames)
    scene_bound_collision_frames = sum(
        isinstance(frame.get("collision_snapshot"), dict) and
        frame["collision_snapshot"].get("scenario_id") == settings["scene_id"] and
        frame["collision_snapshot"].get("runtime_model_fingerprint") == settings.get("runtime_model_fingerprint")
        for frame in frames)
    if ray_valid_frames != len(frames) or scene_bound_collision_frames != len(frames):
        raise ValueError(f"{expected_scene}: ray/collision scene binding is incomplete "
                         f"(rays={ray_valid_frames}/{len(frames)}, "
                         f"collision={scene_bound_collision_frames}/{len(frames)})")

    agile_cycles = sum(frame.get("mode_after") == 0 for frame in motion_frames)
    recovery_cycles = sum(frame.get("mode_after") == 1 for frame in motion_frames)
    transitions_all = [frame for frame in motion_cycle_frames
                   if frame.get("mode_before") in (0, 1) and frame.get("mode_after") in (0, 1) and
                   frame["mode_before"] != frame["mode_after"]]
    transitions_valid_time = [frame for frame in transitions_all
                              if frame.get("sim_clock_valid") is True and
                              isinstance(frame.get("sim_time_s"), (int, float)) and
                              math.isfinite(frame["sim_time_s"])]
    ras = [float(frame["ra_value"]) for frame in motion_frames
           if isinstance(frame.get("ra_value"), (int, float)) and math.isfinite(frame["ra_value"])]
    safety = metrics.get("motion_safety", {})
    cleanup = terminal.get("cleanup_events", [])
    passive_confirmed = any(event.get("event") == "STRATEGY_STOP_CONFIRMATION" and
                            event.get("confirmed") is True and
                            event.get("evidence", {}).get("state") == "PASSIVE"
                            for event in cleanup)
    binding = context.get("preflight", {}).get("loaded_mujoco_scene_authority", {})
    initial_pose = frames[0].get("world_pose") if frames else None
    return {
        "scene": expected_scene,
        "scene_id": settings["scene_id"],
        "seed": layout["seed"],
        "obstacle_count": len(layout["obstacles"]),
        "run_id": context["run_id"],
        "record_validity": metrics["record_validity"],
        "scene_authority": binding.get("status"),
        "session_id": metrics["session_id"],
        "ray_valid_policy_frames": ray_valid_frames,
        "collision_scene_bound_policy_frames": scene_bound_collision_frames,
        "initial_policy_xy_yaw_raw": initial_pose,
        "terminal": metrics.get("navigation_terminal_result"),
        "goal_distance_final_m": metrics.get("target_distance_final_m"),
        "motion_elapsed_start_to_terminal_s": end_s - start_s,
        "motion_effective_valid_contiguous_time_s": metrics.get("valid_motion_duration_s"),
        "valid_contiguous_segment_path_length_m": metrics.get("path_length_m"),
        "valid_contiguous_segment_mean_speed_mps": metrics.get("mean_path_speed_mps"),
        "agile_valid_motion_cycles": agile_cycles,
        "recovery_valid_motion_cycles": recovery_cycles,
        "mode_switches_motion_all_cycles": len(transitions_all),
        "agile_to_recovery_motion_all_cycles": sum(frame["mode_before"] == 0 for frame in transitions_all),
        "recovery_to_agile_motion_all_cycles": sum(frame["mode_before"] == 1 for frame in transitions_all),
        "mode_switches_motion_with_valid_sim_time": len(transitions_valid_time),
        "agile_to_recovery_motion_with_valid_sim_time": sum(frame["mode_before"] == 0 for frame in transitions_valid_time),
        "recovery_to_agile_motion_with_valid_sim_time": sum(frame["mode_before"] == 1 for frame in transitions_valid_time),
        "mode_switches_motion_invalid_sim_time": len(transitions_all) - len(transitions_valid_time),
        "recovery_time_s_contiguous": metrics.get("recovery_sim_duration_s"),
        "ra_min_motion": min(ras) if ras else None,
        "ra_max_motion": max(ras) if ras else None,
        "foot_contact_count": safety.get("foot_contact_count"),
        "foot_impact_count": safety.get("foot_impact_count"),
        "nonfoot_collision": safety.get("nonfoot_collision_failure"),
        "fall": safety.get("fall"),
        "policy_frames_full_capture": metrics.get("frame_count"),
        "policy_frequency_hz_full_capture": metrics.get("policy_frequency_hz_overall"),
        "policy_interval_ms_median": metrics.get("policy_interval_ms_median"),
        "policy_interval_ms_p95": metrics.get("policy_interval_ms_p95"),
        "policy_interval_ms_median_p95_min_max": [metrics.get("policy_interval_ms_median"),
            metrics.get("policy_interval_ms_p95"), metrics.get("policy_interval_ms_min"),
            metrics.get("policy_interval_ms_max")],
        "invalid_clock_frames_full_capture": metrics.get("invalid_clock_frames"),
        "short_reverse_switch_count_existing_metric": metrics.get("short_reverse_switch_count"),
        "short_reverse_window_s": metrics.get("short_reverse_window_s"),
        "policy_cycle_gaps": metrics.get("rl_step_gaps"),
        "motion_clock_coverage_complete": safety.get("coverage_complete_for_motion_interval"),
        "passive_controller_confirmed": passive_confirmed,
        "process_shutdown_complete": process_facts.get("shutdown_complete"),
        "closeout_complete": process_facts.get("closeout_complete"),
        "closeout_errors": process_facts.get("closeout_errors", []),
        "artifacts": {"runtime_record": str(run / "runtime_record.jsonl"),
                      "context": str(run / "run_context.json"),
                      "terminal": str(run / "terminal.json"),
                      "metrics": str(run / "metrics.json"),
                      "timeline": str(run / "timeline.png")},
    }


def main() -> int:
    layouts = read_json(LAYOUTS)["scenes"]
    if len(layouts) != 5:
        raise ValueError("expected five frozen layouts")
    rows = [summarize(i, layout) for i, layout in enumerate(layouts, 1)]
    json_path = RUNS / "scene_comparison.json"
    md_path = RUNS / "scene_comparison.md"
    json_path.write_text(json.dumps(rows, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    columns = ["scene", "seed", "obstacle_count", "terminal", "goal_distance_final_m",
               "motion_elapsed_start_to_terminal_s", "motion_effective_valid_contiguous_time_s",
               "valid_contiguous_segment_path_length_m", "valid_contiguous_segment_mean_speed_mps",
               "agile_valid_motion_cycles", "recovery_valid_motion_cycles",
               "mode_switches_motion_all_cycles", "agile_to_recovery_motion_all_cycles",
               "recovery_to_agile_motion_all_cycles", "mode_switches_motion_with_valid_sim_time",
               "mode_switches_motion_invalid_sim_time", "short_reverse_switch_count_existing_metric",
               "recovery_time_s_contiguous",
               "ra_min_motion", "ra_max_motion", "foot_contact_count", "foot_impact_count",
               "nonfoot_collision", "fall", "policy_frequency_hz_full_capture",
               "policy_interval_ms_median", "policy_interval_ms_p95",
               "invalid_clock_frames_full_capture", "policy_cycle_gaps", "passive_controller_confirmed",
               "process_shutdown_complete"]
    lines = [
        "| " + " | ".join(columns) + " |",
        "|" + "|".join(["---"] * len(columns)) + "|",
    ]
    for row in rows:
        values = []
        for column in columns:
            value = row.get(column)
            values.append("UNKNOWN" if value is None else
                          (f"{value:.4f}" if isinstance(value, float) else str(value)))
        lines.append("| " + " | ".join(values) + " |")
    definitions = [
        "",
        "时间与路径口径：`motion_elapsed_start_to_terminal_s` 是运动窗口起止边界仿真时间之差；",
        "`motion_effective_valid_contiguous_time_s` 是分析器累计的有效连续仿真间隔，缺失/无效时钟处不桥接。",
        "路径长度与平均速度仅覆盖有效连续片段，不能解释为完整起终点路径或全程平均速度。",
        "模式变化按 `start_cycle..end_cycle`（含边界）统计，保留无效时间周期中已经记录的变化；",
        "`mode_switches_motion_with_valid_sim_time` 只统计具有有效仿真时间的变化，不能用于补计无效区间的精确驻留时间。",
        "`short_reverse_switch_count_existing_metric` 是分析器在有效时钟运动窗口中，对相邻且方向相反、间隔不超过 0.5 s 的转换对计数；",
        "它不包含时间无效的转换，也不代表全部模式变化数。",
        "",
    ]
    md_path.write_text("\n".join(lines + definitions), encoding="utf-8")
    print(f"scenes={len(rows)} comparison={md_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
