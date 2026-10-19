#!/usr/bin/env python3
"""Summarize exact invalid-clock signatures and PhysicsLoop fall-window evidence."""
from __future__ import annotations

import json
from pathlib import Path


def audit(root: Path) -> list[dict]:
    results = []
    for path in sorted((root / "logs/b1/runs").glob("*/**/runtime_record.jsonl")):
        frames = []
        for raw in path.read_text(encoding="utf-8").splitlines():
            line = json.loads(raw)
            if line.get("kind") == "frame" and isinstance(line.get("payload"), dict):
                frames.append(line["payload"])
        if not frames:
            continue
        objects = [json.loads(raw) for raw in path.read_text(encoding="utf-8").splitlines()]
        terminal = next((item for item in objects if item.get("kind") == "terminal"), {})
        context_path = path.with_name("run_context.json")
        context = json.loads(context_path.read_text(encoding="utf-8")) if context_path.exists() else {}
        task = (context.get("lifecycle") or {}).get("task_start") or {}
        end_ref = (context.get("lifecycle") or {}).get("terminal_frame_ref") or {}
        start_s = task.get("sim_time_s") if task.get("sim_clock_valid") else None
        end_s = end_ref.get("sim_time_s")
        stale = [item for item in frames if item.get("sim_clock_status") == 2]
        last = ((terminal.get("collision_coverage") or {}).get("last_snapshot") or {})
        events = last.get("fall_history", [])
        startup = [event for event in events if isinstance(event, dict) and start_s is not None and
                   isinstance(event.get("end_sim_time"), (int, float)) and event["end_sim_time"] < start_s]
        task_candidates = [event for event in events if isinstance(event, dict) and
                           event.get("confirmed") is False and
                           isinstance(event.get("start_sim_time"), (int, float)) and
                           isinstance(event.get("end_sim_time"), (int, float)) and
                           start_s is not None and end_s is not None and
                           event["end_sim_time"] >= start_s and event["start_sim_time"] <= end_s]
        results.append({
            "run": path.parent.parent.parent.parent.name,
            "frames": len(frames),
            "clock_status_stale": len(stale),
            "stale_age_values_ns": sorted({item.get("sim_clock_age_ns") for item in stale}),
            "stale_nonzero_sequence": all(item.get("sim_clock_sequence", 0) > 0 for item in stale),
            "task_start_sim_s": start_s,
            "task_end_sim_s": end_s,
            "producer_physics_coverage_complete_at_terminal": last.get("physics_coverage_complete"),
            "unknown_contact_steps": last.get("unknown_contact_steps"),
            "invalid_posture_steps": last.get("invalid_posture_steps"),
            "fall_history_overflow": last.get("fall_history_overflow"),
            "startup_confirmed_falls_excluded_from_task": len([event for event in startup
                if event.get("confirmed") is True]),
            "unconfirmed_fall_candidates_overlapping_task": len(task_candidates),
            "base_bounds_authority_present_in_old_record": any(
                "base_bounds_snapshot" in item for item in frames),
        })
    return results


def main() -> int:
    root = Path(__file__).resolve().parents[1]
    results = audit(root)
    output = root / "docs/thesis_project/evidence/B1/B1_CLOCK_SAFETY_AUDIT.json"
    output.write_text(json.dumps(results, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps(results, indent=2, ensure_ascii=False))
    print(f"wrote {output.relative_to(root)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
