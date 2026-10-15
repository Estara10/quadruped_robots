#!/usr/bin/env python3
"""Run the frozen S6-12 six-map, eight-exit-threshold coarse matrix.

Each plan item delegates exactly one scene run to the existing production
manual_ab_compare / s1_run_diagnostic chain. A non-VALID record, unexplained
safety coverage, or unconfirmed cleanup stops the batch without retry.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
import random
import signal
import subprocess
import sys
import statistics
import csv as csv_module
from datetime import datetime
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
import manual_ab_compare as manual  # noqa: E402
from abs_scene import resolve_scene  # noqa: E402
from s1_run_diagnostic import overlay_configuration_matches  # noqa: E402

INDEX = ROOT / "docs/thesis_project/designs/M1/library_v1/scene_index.csv"
SOURCE = ROOT / "quadruped_ros2_control_humble/install/go2_description"
DEFAULT_PARENT = ROOT / "logs/s6_12/b_exit_coarse"
SCHEDULE_SEED = 61220261015
SCENES = (
    ("dev03.xml", "m1ssd01", "sparse", "gentle_s_bypass"),
    ("dev09.xml", "m1sfd01", "sparse", "forward_offset_obstacles"),
    ("dev11.xml", "m1mbd01", "medium", "single_side_bypass"),
    ("dev15.xml", "m1mod01", "medium", "staggered_channel"),
    ("dev27.xml", "m1dld01", "dense", "short_local_bypass"),
    ("dev29.xml", "m1dfd01", "dense", "forward_offset_obstacles"),
)
THRESHOLDS = (-0.05, -0.07, -0.09, -0.11, -0.13, -0.15, -0.17, -0.19)
E = -0.05
X_A_PLACEHOLDER = -0.07  # Required by the shared CLI parser; ignored by legal A config.


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write_json(path: Path, value: dict) -> None:
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")


def scene_catalog() -> dict[str, dict[str, str]]:
    with INDEX.open(newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    found = {row["filename"]: row for row in rows}
    out = {}
    for filename, scene_id, density, route in SCENES:
        row = found.get(filename)
        if not row:
            raise RuntimeError(f"scene missing from frozen M1 index: {filename}")
        expected = (scene_id, density, route, "development")
        actual = (row["scene_id"], row["density"], row["route_structure"], row["split"])
        if actual != expected:
            raise RuntimeError(f"M1 scene identity/classification mismatch for {filename}: {actual} != {expected}")
        resolve_scene(filename)
        out[filename] = {"scene_id": scene_id, "density": density, "route_structure": route,
                         "split": "development", "generation_seed": row["seed"]}
    return out


def make_plan(catalog: dict[str, dict[str, str]]) -> dict:
    rng = random.Random(SCHEDULE_SEED)
    runs = []
    seq = 0
    for filename, scene_id, _, _ in SCENES:
        scene = catalog[filename]
        for repeat in (1, 2):
            # Each map/repeat block gets an independently shuffled eight-arm order.
            order = list(THRESHOLDS)
            rng.shuffle(order)
            for x in order:
                seq += 1
                group = "A" if x == -0.05 else "B"
                x_tag = f"x{abs(round(x * 100)):02d}"
                item_id = f"{seq:03d}_{scene_id}_{x_tag}_r{repeat}"
                runs.append({
                    "sequence": seq, "plan_item_id": item_id,
                    "scene": filename, "scene_id": scene_id,
                    "density": scene["density"], "route_structure": scene["route_structure"],
                    "generation_seed": scene["generation_seed"], "schedule_repeat": repeat,
                    "entry_threshold_E": E,
                    "exit_threshold_setting": x,
                    "group": group,
                    "effective_exit_threshold": E if group == "A" else x,
                    "hysteresis": group == "B", "minimum_hold": False,
                    "status": "NOT_STARTED", "attempts": [],
                })
    return {
        "schema": "s6-12-b-exit-coarse-v1",
        "classification": "S6-12 development-set B exit-threshold coarse pre-experiment; nonformal; not for paper results",
        "created_at": datetime.now().astimezone().isoformat(timespec="seconds"),
        "library_version": "m1-mapset/1.0",
        "scene_index_sha256": digest(INDEX),
        "schedule_seed": SCHEDULE_SEED,
        "schedule_seed_scope": "only the randomized threshold order within each scene x repeat block; not a simulator/noise seed",
        "schedule": "fixed scene order; for each of two blocks per scene, shuffle all eight thresholds once",
        "planned_count": 96, "A_reference_count": 12, "B_candidate_count": 84,
        "fixed_conditions": {
            "entry_threshold_E": E,
            "exit_thresholds": list(THRESHOLDS),
            "threshold_-0.05": "legal A reference; exit RA<E, no hysteresis, no hold",
            "other_thresholds": "B; exit RA<X, equality holds Recovery, no hold",
            "goal_world_m": [7.5, 0.0], "arrival_radius_m": 0.5,
            "sim_limit_s": 20.0, "wall_cap_s": 120.0,
            "standing_observation_sim_s": 5.0, "standing_wall_cap_s": 30.0,
            "startup_gain_transition_s_control_runtime": 0.30,
            "startup_recovery_observation_enabled": False,
            "scenes": [{"filename": f, **catalog[f]} for f, *_ in SCENES],
            "models_and_control_chain": "current installed production controller and models; shared S6-05 startup gain transition and current safety/termination chain",
        },
        "run_budget": "at most one launch per item; no automatic retry; stop on infrastructure, invalid-record, unexplained task safety coverage, or cleanup failure",
        "runs": runs,
    }


def validate_plan(plan: dict, catalog: dict[str, dict[str, str]]) -> None:
    rows = plan.get("runs", [])
    if len(rows) != 96 or len({r.get("plan_item_id") for r in rows}) != 96:
        raise RuntimeError("plan must contain 96 unique item IDs")
    if sum(r["group"] == "A" for r in rows) != 12 or sum(r["group"] == "B" for r in rows) != 84:
        raise RuntimeError("plan group counts must be A=12 and B=84")
    allowed = set(THRESHOLDS)
    for f, *_ in SCENES:
        for repeat in (1, 2):
            block = [r for r in rows if r["scene"] == f and r["schedule_repeat"] == repeat]
            if len(block) != 8 or {r["exit_threshold_setting"] for r in block} != allowed:
                raise RuntimeError(f"incomplete/duplicate threshold block: {f} repeat {repeat}")
    for r in rows:
        if r["scene"] not in catalog or r["entry_threshold_E"] != E or r["minimum_hold"]:
            raise RuntimeError(f"invalid plan row: {r['plan_item_id']}")
        expected_group = "A" if r["exit_threshold_setting"] == -0.05 else "B"
        if r["group"] != expected_group or r["hysteresis"] != (expected_group == "B"):
            raise RuntimeError(f"illegal threshold/group mapping: {r['plan_item_id']}")
        if expected_group == "B" and not r["exit_threshold_setting"] < E:
            raise RuntimeError(f"B exit threshold must remain below E: {r['plan_item_id']}")
    # Rebuild with the frozen seed and compare sequence/IDs to prove schedule determinism.
    replay = make_plan(catalog)
    keys = ("plan_item_id", "scene", "scene_id", "schedule_repeat", "group", "exit_threshold_setting")
    if [[r[k] for k in keys] for r in rows] != [[r[k] for k in keys] for r in replay["runs"]]:
        raise RuntimeError("schedule is not reproducible from its frozen seed")


def validate_authorized_first_retry(plan: dict) -> dict:
    """Accept only the explicitly authorized preserved-INVALID first-item retry state."""
    rows = plan.get("runs", [])
    first = rows[0] if rows else {}
    attempts = first.get("attempts", [])
    exact_first = (first.get("sequence") == 1 and
                   first.get("plan_item_id") == "001_m1ssd01_x15_r1" and
                   first.get("scene") == "dev03.xml" and first.get("group") == "B" and
                   first.get("entry_threshold_E") == E and
                   first.get("exit_threshold_setting") == -0.15)
    original_failure = (first.get("status") == "INFRASTRUCTURE_OR_RECORD_FAILURE_STOPPED" and
                        len(attempts) == 1 and
                        attempts[0].get("terminal_result") == "STARTUP_FAILURE" and
                        attempts[0].get("record_validity") == "INVALID" and
                        attempts[0].get("process_shutdown_complete") is True and
                        attempts[0].get("closeout_complete") is True)
    later_unstarted = all(row.get("status") == "NOT_STARTED" and not row.get("attempts")
                          for row in rows[1:])
    if (plan.get("status") != "STOPPED" or len(rows) != 96 or not exact_first or
            not original_failure or not later_unstarted):
        raise RuntimeError("authorized first-item retry rejected: frozen STOPPED plan must contain exactly one preserved INVALID first attempt and 95 untouched items")
    return first


def validate_authorized_second_retry(plan: dict) -> dict:
    """Accept only the Director-authorized one-time retry of frozen row 2."""
    rows = plan.get("runs", [])
    first = rows[0] if len(rows) > 0 else {}
    second = rows[1] if len(rows) > 1 else {}
    first_attempts = first.get("attempts", [])
    second_attempts = second.get("attempts", [])
    exact_first = (first.get("sequence") == 1 and
                   first.get("plan_item_id") == "001_m1ssd01_x15_r1" and
                   first.get("status") == "VALID_TASK_RESULT" and
                   len(first_attempts) == 2 and
                   first_attempts[0].get("record_validity") == "INVALID" and
                   first_attempts[0].get("process_shutdown_complete") is True and
                   first_attempts[0].get("closeout_complete") is True and
                   first_attempts[1].get("record_validity") == "VALID" and
                   first_attempts[1].get("process_shutdown_complete") is True and
                   first_attempts[1].get("closeout_complete") is True)
    exact_second = (second.get("sequence") == 2 and
                    second.get("plan_item_id") == "002_m1ssd01_x17_r1" and
                    second.get("scene") == "dev03.xml" and
                    second.get("scene_id") == "m1ssd01" and
                    second.get("group") == "B" and
                    second.get("entry_threshold_E") == E and
                    second.get("exit_threshold_setting") == -0.17 and
                    second.get("status") == "INFRASTRUCTURE_OR_RECORD_FAILURE_STOPPED" and
                    len(second_attempts) == 1 and
                    second_attempts[0].get("terminal_result") == "PREPARATION_FAILURE" and
                    second_attempts[0].get("record_validity") == "INVALID" and
                    second_attempts[0].get("process_shutdown_complete") is True and
                    second_attempts[0].get("closeout_complete") is True)
    later_unstarted = all(row.get("status") == "NOT_STARTED" and not row.get("attempts")
                          for row in rows[2:])
    if (plan.get("status") != "STOPPED" or len(rows) != 96 or
            not exact_first or not exact_second or not later_unstarted):
        raise RuntimeError("authorized second-item retry rejected: frozen STOPPED plan must preserve valid row 1, exactly one cleaned INVALID PREPARATION_FAILURE row 2, and 94 untouched rows")
    return second


def set_b_threshold(config_path: Path, x: float) -> None:
    text = config_path.read_text(encoding="utf-8")
    import re
    text, n = re.subn(r"(?m)^(    exit_threshold:)\s*[^\n]+$", rf"\g<1> {x:.17g}", text)
    if n != 1:
        raise RuntimeError(f"expected exactly one B exit_threshold in {config_path}, found {n}")
    config_path.write_text(text, encoding="utf-8")
    if not overlay_configuration_matches(text, "B", E, x, 0.30, 7.5, 0.0):
        raise RuntimeError(f"production parser rejected numeric B overlay at X={x}: {config_path}")


def prepare_overlays(batch_root: Path) -> dict[str, str]:
    if not SOURCE.is_dir():
        raise RuntimeError(f"installed package share missing: {SOURCE}")
    overlays = {}
    for group, x in (("A", -0.07), *[("B", t) for t in THRESHOLDS[1:]]):
        key = "A_ref" if group == "A" else f"B_{x:.2f}"
        target = batch_root / "overlays" / key
        manual.prepare_overlay(SOURCE, target, group, E, x, 0.30)
        config = target / "share/go2_description/config/abs/config.yaml"
        manual.set_overlay_goal(config, 7.5, 0.0)
        if group == "B":
            set_b_threshold(config, x)
        detached = manual.verify_overlay_detached(SOURCE, target)
        config_text = config.read_text(encoding="utf-8")
        if not overlay_configuration_matches(config_text, group, E, x, 0.30, 7.5, 0.0):
            raise RuntimeError(f"production config validator rejected overlay {key}")
        overlays[key] = {"path": str(target), "group": group,
                         "exit_threshold_setting": x,
                         "effective_exit_threshold": E if group == "A" else x,
                         "independent_files": detached,
                         "config_sha256": digest(config)}
    return overlays


def safety_unknown_reason(metrics: dict) -> str | None:
    safety = metrics.get("task_window_safety") or {}
    collision = safety.get("collision")
    fall = safety.get("fall")
    if collision in (True, False) and fall in (True, False):
        return None
    reasons = []
    for label, value in (("collision", collision), ("fall", fall)):
        if value not in (True, False):
            reasons.append(f"{label}={value!r}")
    coverage = safety.get("coverage_reason") or metrics.get("collision_coverage", {}).get("reason")
    return "; ".join(reasons) + (f"; coverage_reason={coverage}" if coverage else "; no explicit coverage reason")


def extract_ra(run_dir: Path) -> dict:
    values = []
    record = run_dir / "runtime_record.jsonl"
    if not record.is_file():
        return {"sample_count": 0, "min": None, "max": None, "median": None,
                "risk_sample_count_at_E": None}
    with record.open(encoding="utf-8") as f:
        for line in f:
            try:
                row = json.loads(line)
            except json.JSONDecodeError:
                continue
            value = (row.get("payload") or {}).get("ra_value")
            if row.get("kind") == "frame" and isinstance(value, (int, float)) and math.isfinite(value):
                values.append(float(value))
    return {"sample_count": len(values), "min": min(values) if values else None,
            "max": max(values) if values else None,
            "median": statistics.median(values) if values else None,
            "risk_sample_count_at_E": sum(v >= E for v in values)}


def write_partial_summary(root: Path, plan: dict, plot_python: str) -> None:
    """Materialize a compact run table even when the batch stops early."""
    rows = [r for r in plan.get("runs", []) if r.get("attempts")]
    attempt_fields = ["sequence", "plan_item_id", "attempt_number", "scene", "scene_id", "group",
                      "entry_threshold_E", "exit_threshold_setting", "run_id", "terminal_result",
                      "record_validity", "child_exit_code", "run_directory",
                      "process_shutdown_complete", "closeout_complete", "runtime_source_validation",
                      "actual_group", "actual_entry_threshold", "actual_exit_threshold"]
    attempt_output = root / "coarse_attempt_results.csv"
    with attempt_output.open("w", newline="", encoding="utf-8") as f:
        writer = csv_module.DictWriter(f, fieldnames=attempt_fields, extrasaction="ignore")
        writer.writeheader()
        for item in plan.get("runs", []):
            for attempt_index, attempt in enumerate(item.get("attempts", []), 1):
                attempt_row = {**item, **attempt,
                               "attempt_number": attempt.get("attempt_number", attempt_index)}
                run_directory = attempt.get("run_directory")
                if run_directory:
                    run_path = Path(run_directory)
                    context_path = run_path / "run_context.json"
                    if context_path.is_file():
                        context = json.loads(context_path.read_text(encoding="utf-8"))
                        source = context.get("runtime_source_validation") or {}
                        evidence = source.get("evidence") or {}
                        attempt_row.update({"runtime_source_validation": source.get("status"),
                                            "actual_group": evidence.get("group"),
                                            "actual_entry_threshold": evidence.get("entry_threshold"),
                                            "actual_exit_threshold": evidence.get("exit_threshold")})
                writer.writerow(attempt_row)
    fields = ["sequence", "plan_item_id", "scene", "density", "route_structure", "schedule_repeat",
              "group", "entry_threshold_E", "exit_threshold_setting", "effective_exit_threshold",
              "status", "run_id", "terminal_result", "record_validity", "runtime_source_validation",
              "target_distance_final_m", "elapsed_task_duration_s", "valid_motion_duration_s",
              "task_valid_time_coverage_fraction", "path_length_m", "mean_path_speed_mps",
              "mode_transitions", "short_reverse_switch_count", "short_reverse_switch_coverage",
              "recovery_sim_duration_s", "recovery_share_valid_motion_time", "recovery_hold_audit_status",
              "motion_fall_result", "foot_contact_count", "foot_impact_count", "nonfoot_collision_failure",
              "rl_step_gaps", "invalid_clock_frames", "standing_observation_status",
              "process_shutdown_complete", "closeout_complete", "ra_sample_count", "ra_min", "ra_max", "ra_median"]
    output = root / "coarse_run_results.csv"
    with output.open("w", newline="", encoding="utf-8") as f:
        writer = csv_module.DictWriter(f, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        for item in rows:
            m = item.get("metrics") or {}
            safety = m.get("task_window_safety") or {}
            attempt = (item.get("attempts") or [{}])[-1]
            row = {**item, **m,
                   "target_distance_final_m": m.get("target_distance_final_m"),
                   "elapsed_task_duration_s": m.get("elapsed_task_duration_s"),
                   "valid_motion_duration_s": m.get("valid_motion_duration_s"),
                   "task_valid_time_coverage_fraction": m.get("task_valid_time_coverage_fraction"),
                   "mode_transitions": m.get("mode_transitions"),
                   "motion_fall_result": m.get("motion_fall_result"),
                   "foot_contact_count": safety.get("foot_contact_count"),
                   "foot_impact_count": safety.get("foot_impact_count"),
                   "nonfoot_collision_failure": safety.get("nonfoot_collision_failure"),
                   "process_shutdown_complete": attempt.get("process_shutdown_complete"),
                   "closeout_complete": attempt.get("closeout_complete"),
                   "ra_sample_count": (item.get("ra_summary") or {}).get("sample_count"),
                   "ra_min": (item.get("ra_summary") or {}).get("min"),
                   "ra_max": (item.get("ra_summary") or {}).get("max"),
                   "ra_median": (item.get("ra_summary") or {}).get("median")}
            writer.writerow(row)
    outcomes = {}
    by_x = {}
    by_scene_x = {}
    valid_rows = [item for item in rows
                  if item.get("status") == "VALID_TASK_RESULT" and item.get("record_validity") == "VALID"]
    status_counts = {
        "valid_task_result": len(valid_rows),
        "attempted_invalid_or_stopped": sum(bool(i.get("attempts")) and i.get("status") != "VALID_TASK_RESULT" for i in plan["runs"]),
        "not_started": sum(i.get("status") == "NOT_STARTED" for i in plan["runs"]),
    }
    for item in valid_rows:
        outcome = item.get("terminal_result") or item.get("status")
        outcomes[outcome] = outcomes.get(outcome, 0) + 1
        x = item.get("exit_threshold_setting")
        bucket = by_x.setdefault(x, {"attempted": 0, "outcomes": {}, "arrived_elapsed": [],
                                     "short_reverse_rates": [], "recovery_shares": []})
        bucket["attempted"] += 1
        bucket["outcomes"][outcome] = bucket["outcomes"].get(outcome, 0) + 1
        m = item.get("metrics") or {}
        if outcome == "ARRIVED" and isinstance(m.get("elapsed_task_duration_s"), (int, float)):
            bucket["arrived_elapsed"].append(m["elapsed_task_duration_s"])
        vtime = m.get("valid_motion_duration_s")
        reverse = m.get("short_reverse_switch_count")
        if isinstance(vtime, (int, float)) and vtime > 0 and isinstance(reverse, (int, float)):
            bucket["short_reverse_rates"].append(reverse / (vtime / 60.0))
        share = m.get("recovery_share_valid_motion_time")
        if isinstance(share, (int, float)) and math.isfinite(share):
            bucket["recovery_shares"].append(share)
        key = (item.get("scene_id"), x)
        by_scene_x[key] = by_scene_x.get(key, 0) + (outcome == "ARRIVED")
    write_json(root / "coarse_partial_summary.json", {
        "status": plan.get("status"), "planned": 96,
        "attempted": sum(bool(i.get("attempts")) for i in plan["runs"]),
        "attempted_plan_items": sum(bool(i.get("attempts")) for i in plan["runs"]),
        "attempt_count_including_preserved_retries": sum(len(i.get("attempts", [])) for i in plan.get("runs", [])),
        "valid_task_result_plan_items": sum(i.get("status") == "VALID_TASK_RESULT" for i in plan.get("runs", [])),
        "valid_task_results": sum(i.get("status") == "VALID_TASK_RESULT" for i in plan["runs"]),
        "not_started": sum(i.get("status") == "NOT_STARTED" for i in plan["runs"]),
        "plan_item_status_counts": status_counts,
        "all_attempt_terminal_results": {
            result: sum(a.get("terminal_result") == result for i in plan["runs"] for a in i.get("attempts", []))
            for result in sorted({a.get("terminal_result") for i in plan["runs"] for a in i.get("attempts", []) if a.get("terminal_result")})
        },
        "outcomes": outcomes,
        "by_exit_threshold": by_x,
        "threshold_heatmap_arrived_count_by_scene": {
            f"{scene}/{x:.2f}": by_scene_x.get((scene, x), 0)
            for scene in [entry[1] for entry in SCENES] for x in THRESHOLDS
        },
        "semantics": "UNKNOWN and INVALID remain separate; this is development-only coarse exploration, not a formal result.",
    })
    # Fail before the first simulation if the required chart stack is unavailable.
    plot_env = os.environ.copy()
    plot_env["MPLCONFIGDIR"] = "/tmp/s6_12_mpl"
    Path(plot_env["MPLCONFIGDIR"]).mkdir(parents=True, exist_ok=True)
    check = subprocess.run([plot_python, "-c", "import matplotlib, numpy; import matplotlib.pyplot as plt"],
                          cwd=ROOT, env=plot_env, capture_output=True, text=True)
    if check.returncode != 0:
        raise RuntimeError(f"plotting environment unavailable: {check.stderr[-1200:]}")
    script = r'''import csv,json,sys,math
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
p=Path(sys.argv[1]); d=json.loads((p/'coarse_partial_summary.json').read_text()); rows=list(csv.DictReader((p/'coarse_run_results.csv').open()))
xs=[-0.05,-0.07,-0.09,-0.11,-0.13,-0.15,-0.17,-0.19]
labels=[f'{x:.2f}' for x in xs]
# Performance plots only include VALID task-result plan items; invalid startup/preparation
# attempts stay in the audit ledgers and the separate execution-status plot.
rows=[r for r in rows if r.get('status')=='VALID_TASK_RESULT' and r.get('record_validity')=='VALID']
terms=sorted(d['outcomes'])
fig,ax=plt.subplots(figsize=(11,5)); bottom=np.zeros(len(xs))
for t in terms:
 vals=np.array([d['by_exit_threshold'].get(str(x),{}).get('outcomes',{}).get(t,0) for x in xs])
 if vals.any(): ax.bar(labels,vals,bottom=bottom,label=t); bottom+=vals
ax.set(title=f"Valid task-result terminals by exit setting (n={len(rows)} valid plan items; coarse development only)",xlabel='Exit threshold (−0.05 is A reference)',ylabel='Runs'); ax.legend(fontsize=8);fig.tight_layout();fig.savefig(p/'terminal_composition.png',dpi=150);plt.close(fig)
def series(key, title, ylabel, predicate=lambda r:True, transform=lambda r:float(r[key]) if r.get(key) not in ('',None) else np.nan):
 vals=[]; counts=[]
 for x in xs:
  rs=[r for r in rows if abs(float(r['exit_threshold_setting'])-x)<1e-8 and predicate(r)]
  a=[transform(r) for r in rs]; a=[v for v in a if math.isfinite(v)]
  vals.append(float(np.mean(a)) if a else np.nan);counts.append(len(a))
 fig,ax=plt.subplots(figsize=(9,4));ax.plot(labels,vals,'o-');ax.set(title=f'{title} (valid n per point: {counts})',xlabel='Exit threshold (−0.05 is A reference)',ylabel=ylabel);ax.grid(alpha=.3);fig.tight_layout();fig.savefig(p/(key+'.png'),dpi=150);plt.close(fig)
series('short_reverse_switch_count','0.5 s short-reverse switches per minute of valid task time','count/min',transform=lambda r:(float(r['short_reverse_switch_count'])/(float(r['valid_motion_duration_s'])/60) if r.get('short_reverse_switch_count') not in ('',None) and float(r.get('valid_motion_duration_s') or 0)>0 and r.get('short_reverse_switch_coverage')=='complete' else np.nan))
series('recovery_share_valid_motion_time','Recovery share of valid continuous task time','fraction',transform=lambda r:(float(r['recovery_share_valid_motion_time']) if r.get('recovery_share_valid_motion_time') not in ('',None) else np.nan))
series('elapsed_task_duration_s','Arrival elapsed time (ARRIVED only)','seconds',predicate=lambda r:r.get('terminal_result')=='ARRIVED',transform=lambda r:(float(r['elapsed_task_duration_s']) if r.get('elapsed_task_duration_s') not in ('',None) else np.nan))
# Heat map: ARRIVED count out of at most two runs per map/setting.
scenes=['m1ssd01','m1sfd01','m1mbd01','m1mod01','m1dld01','m1dfd01']; arr=np.zeros((len(scenes),len(xs)))
for i,s in enumerate(scenes):
 for j,x in enumerate(xs): arr[i,j]=sum(1 for r in rows if r.get('scene_id')==s and abs(float(r['exit_threshold_setting'])-x)<1e-8 and r.get('terminal_result')=='ARRIVED')
fig,ax=plt.subplots(figsize=(10,4));im=ax.imshow(arr,aspect='auto',vmin=0,vmax=2,cmap='viridis');ax.set_xticks(range(len(xs)),labels);ax.set_yticks(range(len(scenes)),scenes);ax.set(title=f'ARRIVED count by M1 development map and threshold (maximum n≤2; valid plan items={len(rows)})',xlabel='Exit threshold (−0.05 is A reference)');
for i in range(len(scenes)):
 for j in range(len(xs)): ax.text(j,i,f'{int(arr[i,j])}/2',ha='center',va='center',color='white' if arr[i,j]<1 else 'black',fontsize=8)
fig.colorbar(im,ax=ax,label='ARRIVED count');fig.tight_layout();fig.savefig(p/'map_threshold_heatmap.png',dpi=150);plt.close(fig)
fig,ax=plt.subplots(figsize=(8,4)); names=['Valid task result','Attempted invalid/stopped','Not started']; vals=[d['plan_item_status_counts'][k] for k in ('valid_task_result','attempted_invalid_or_stopped','not_started')]; bars=ax.bar(names,vals,color=['#4c9f70','#d67b62','#aab2bd']); ax.bar_label(bars); ax.set(title=f"Frozen coarse plan status (96 plan items; {d['attempt_count_including_preserved_retries']} total attempts including preserved retry)",ylabel='Plan items');fig.tight_layout();fig.savefig(p/'execution_status.png',dpi=150);plt.close(fig)
'''
    plot = subprocess.run([plot_python, "-c", script, str(root)], cwd=ROOT, env=plot_env,
                          capture_output=True, text=True)
    if plot.returncode != 0:
        raise RuntimeError(f"plot generation failed: {plot.stderr[-1600:]}")


def choose_timelines(root: Path, plan: dict) -> list[dict]:
    rows = [i for i in plan["runs"] if i.get("status") == "VALID_TASK_RESULT"]
    selections = []
    for x in THRESHOLDS:
        group = [r for r in rows if r["exit_threshold_setting"] == x]
        arrivals = [r for r in group if r.get("terminal_result") == "ARRIVED" and
                    isinstance((r.get("metrics") or {}).get("elapsed_task_duration_s"), (int, float))]
        if arrivals:
            med = statistics.median((r["metrics"]["elapsed_task_duration_s"] for r in arrivals))
            mid = min(arrivals, key=lambda r: abs(r["metrics"]["elapsed_task_duration_s"] - med))
            selections.append({"role": "ARRIVED elapsed nearest within-setting median", "plan_item_id": mid["plan_item_id"],
                               "run_directory": mid.get("run_directory"), "threshold": x})
        failures = [r for r in group if r.get("terminal_result") not in (None, "ARRIVED")]
        if failures:
            f = failures[0]
            selections.append({"role": "first non-ARRIVED case in frozen order", "plan_item_id": f["plan_item_id"],
                               "run_directory": f.get("run_directory"), "threshold": x})
    write_json(root / "representative_timelines.json", {"selection_rule": "predeclared: within-threshold ARRIVED elapsed nearest median plus first non-ARRIVED item; not best-case selection", "items": selections})
    return selections


def locate_child(root: Path, item: dict) -> tuple[Path | None, dict]:
    child = root / "runs" / item["plan_item_id"]
    bp = child / "batch_plan.json"
    if not bp.is_file():
        return None, {}
    plan = json.loads(bp.read_text(encoding="utf-8"))
    status = next((x for x in plan.get("run_statuses", []) if x.get("group") == item["group"]), {})
    out = Path(status.get("output_directory", ""))
    if not out.is_absolute():
        # The one-run script records an absolute result root in normal operation.
        out = (child / out).resolve()
    return out, status


def update_plan(root: Path, plan: dict) -> None:
    write_json(root / "s6_12_coarse_plan.json", plan)


def run_item(root: Path, item: dict, plan: dict, args: argparse.Namespace) -> bool:
    overlay_key = "A_ref" if item["group"] == "A" else f"B_{item['exit_threshold_setting']:.2f}"
    overlay = Path(plan["overlays"][overlay_key]["path"])
    if item["group"] == "B":
        set_b_threshold(overlay / "share/go2_description/config/abs/config.yaml",
                        item["exit_threshold_setting"])
    attempts = item.setdefault("attempts", [])
    attempt_number = len(attempts) + 1
    item_root = root / "runs" / item["plan_item_id"]
    if attempt_number == 1:
        child = item_root
    else:
        item_root.mkdir(parents=True, exist_ok=True)
        child = item_root / f"attempt_{attempt_number:02d}"
    child.mkdir(parents=True, exist_ok=False)
    result_root = child / "result"
    item["status"] = "RUNNING"
    item["started_at"] = datetime.now().astimezone().isoformat(timespec="seconds")
    item["output_directory"] = str(result_root)
    update_plan(root, plan)
    x_cli = X_A_PLACEHOLDER if item["group"] == "A" else item["exit_threshold_setting"]
    command = [
        "/usr/bin/python3", "-u", str(ROOT / "scripts/manual_ab_compare.py"),
        "--groups", item["group"], "--repeats", "1", "--scene", item["scene"],
        "--goal-x", "7.5", "--goal-y", "0", "--sim-limit-s", "20", "--wall-cap-s", "120",
        "--entry-threshold", "-0.05", "--exit-threshold", f"{x_cli:.17g}",
        "--hold-s", "0.30", "--classification", plan["classification"],
        "--display", args.display, "--xauthority", args.xauthority,
        "--plot-python", args.plot_python, "--result-root", str(result_root),
        "--prepared-overlay", str(overlay),
    ]
    log_path = child / "console.log"
    try:
        with log_path.open("w", encoding="utf-8") as log:
            proc = subprocess.Popen(command, cwd=ROOT, stdout=subprocess.PIPE,
                                    stderr=subprocess.STDOUT, text=True, bufsize=1)
            try:
                assert proc.stdout is not None
                for line in proc.stdout:
                    print(line, end="", flush=True)
                    log.write(line)
                    log.flush()
                code = proc.wait()
            except KeyboardInterrupt:
                if proc.poll() is None:
                    proc.send_signal(signal.SIGINT)
                try:
                    proc.wait(timeout=90)
                except subprocess.TimeoutExpired:
                    item["status"] = "STOPPED_CLEANUP_UNCONFIRMED"
                    item["stop_reason"] = "child failed to exit after SIGINT within 90 seconds"
                    update_plan(root, plan)
                    raise RuntimeError("child cleanup unconfirmed after Ctrl+C")
                raise
    except KeyboardInterrupt:
        item["status"] = "OPERATOR_ABORT_STOPPED"
        item["finished_at"] = datetime.now().astimezone().isoformat(timespec="seconds")
        update_plan(root, plan)
        raise
    item["finished_at"] = datetime.now().astimezone().isoformat(timespec="seconds")
    item["exit_code"] = code
    batch_plan = result_root / "batch_plan.json"
    run_status = {}
    if batch_plan.is_file():
        bp = json.loads(batch_plan.read_text(encoding="utf-8"))
        run_status = next((r for r in bp.get("run_statuses", []) if r.get("group") == item["group"]), {})
    run_id = run_status.get("run_id")
    run_dir = result_root / item["group"] / "rep_01" / str(run_id) if run_id else None
    metrics = {}
    context = {}
    facts = {}
    if run_dir and (run_dir / "metrics.json").is_file():
        metrics = json.loads((run_dir / "metrics.json").read_text(encoding="utf-8"))
    if run_dir and (run_dir / "run_context.json").is_file():
        context = json.loads((run_dir / "run_context.json").read_text(encoding="utf-8"))
    if run_dir and (run_dir / "process_facts.json").is_file():
        facts = json.loads((run_dir / "process_facts.json").read_text(encoding="utf-8"))
    item["attempts"].append({"attempt_number": attempt_number,
                             "run_id": run_id, "child_exit_code": code,
                             "child_status": run_status.get("status"),
                             "terminal_result": (metrics.get("supervisor_terminal_result") or
                                                 context.get("terminal_result")),
                             "record_validity": metrics.get("record_validity"),
                             "run_directory": str(run_dir) if run_dir else None,
                             "process_shutdown_complete": facts.get("shutdown_complete", metrics.get("process_shutdown_complete")),
                             "closeout_complete": facts.get("closeout_complete", metrics.get("closeout_complete"))})
    item["run_id"] = run_id
    item["run_directory"] = str(run_dir) if run_dir else None
    item["terminal_result"] = item["attempts"][-1]["terminal_result"]
    item["record_validity"] = metrics.get("record_validity")
    source_validation = context.get("runtime_source_validation") or {}
    source_evidence = source_validation.get("evidence") or {}
    item["runtime_source_validation"] = source_validation.get("status")
    item["actual_group"] = source_evidence.get("group")
    item["actual_entry_threshold"] = source_evidence.get("entry_threshold")
    item["actual_exit_threshold"] = source_evidence.get("exit_threshold")
    item["metrics"] = metrics
    item["ra_summary"] = extract_ra(run_dir) if run_dir else {"sample_count": 0}
    if code != 0 or metrics.get("record_validity") != "VALID" or not facts.get("shutdown_complete", metrics.get("process_shutdown_complete")) or not facts.get("closeout_complete", metrics.get("closeout_complete")):
        item["status"] = "INFRASTRUCTURE_OR_RECORD_FAILURE_STOPPED"
        item["stop_reason"] = "runner nonzero, record not VALID, or shutdown/closeout not confirmed; no retry"
        update_plan(root, plan)
        return False
    expected_x = E if item["group"] == "A" else item["exit_threshold_setting"]
    try:
        actual_e = float(item["actual_entry_threshold"])
        actual_x = float(item["actual_exit_threshold"])
    except (TypeError, ValueError):
        actual_e = actual_x = math.nan
    if item["actual_group"] != item["group"] or abs(actual_e - E) > 1e-5 or abs(actual_x - expected_x) > 1e-5:
        item["status"] = "ACTUAL_CONFIG_MISMATCH_STOPPED"
        item["stop_reason"] = f"consumed group/threshold mismatch: group={item['actual_group']}, E={actual_e}, X={actual_x}"
        update_plan(root, plan)
        return False
    unknown_reason = safety_unknown_reason(metrics)
    if unknown_reason:
        item["status"] = "TASK_SAFETY_UNKNOWN_STOPPED"
        item["stop_reason"] = unknown_reason
        update_plan(root, plan)
        return False
    item["status"] = "VALID_TASK_RESULT"
    update_plan(root, plan)
    print(f"S6-12 {item['sequence']}/96: {item['scene']} {item['group']} X={item['exit_threshold_setting']:.2f} rep={item['schedule_repeat']} -> {item['terminal_result']}; record VALID; cleaned.", flush=True)
    return True


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--batch-root", type=Path, default=None)
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--authorized-retry-first", action="store_true",
                        help="perform the single explicitly authorized retry of the preserved first infrastructure-failed item")
    parser.add_argument("--authorized-retry-second", action="store_true",
                        help="perform the single explicitly authorized retry of frozen plan item 002, then continue untouched rows")
    parser.add_argument("--preflight-only", action="store_true")
    parser.add_argument("--display", default=os.environ.get("DISPLAY", ":0"))
    parser.add_argument("--xauthority", default=os.environ.get("XAUTHORITY", "/run/user/1000/gdm/Xauthority"))
    parser.add_argument("--plot-python", default="/home/lidio/anaconda3/bin/python3")
    args = parser.parse_args()
    if (args.authorized_retry_first or args.authorized_retry_second) and not args.batch_root:
        parser.error("authorized retry requires the original frozen --batch-root")
    if args.authorized_retry_first and args.authorized_retry_second:
        parser.error("choose only one authorized retry mode")
    if not Path(args.plot_python).is_file():
        parser.error(f"plot Python not found: {args.plot_python}")
    catalog = scene_catalog()
    if args.batch_root:
        root = args.batch_root.resolve()
        plan_path = root / "s6_12_coarse_plan.json"
        if not args.resume or not plan_path.is_file():
            parser.error("existing batch root requires --resume and its frozen plan")
        plan = json.loads(plan_path.read_text(encoding="utf-8"))
        if plan.get("scene_index_sha256") != digest(INDEX):
            raise RuntimeError("M1 scene index changed after freeze; refusing to run")
        if args.authorized_retry_first and not args.resume:
            parser.error("--authorized-retry-first requires --resume")
    else:
        if args.resume:
            parser.error("--resume requires --batch-root")
        parent = DEFAULT_PARENT
        parent.mkdir(parents=True, exist_ok=True)
        root = parent / (datetime.now().strftime("%Y%m%d_%H%M%S") + "_" + str(os.getpid()))
        root.mkdir(parents=False, exist_ok=False)
        plan = make_plan(catalog)
        validate_plan(plan, catalog)
        plan["overlays"] = prepare_overlays(root)
        plan["code_commit"] = subprocess.run(["git", "rev-parse", "HEAD"], cwd=ROOT,
                                               check=True, capture_output=True, text=True).stdout.strip()
        plan["controller_executable"] = str((ROOT / "unitree_mujoco/simulate/build2/unitree_mujoco").resolve())
        plan["installed_ros_package_share"] = str(SOURCE.resolve())
        plan["controller_library_build_id"] = "recorded per run by runtime source validator; see child contexts"
        plan["initial_process_inventory"] = "No project MuJoCo/controller/recorder process found in preflight process listing."
        update_plan(root, plan)
        if args.preflight_only:
            print(f"Frozen S6-12 plan ready: {root / 's6_12_coarse_plan.json'}; 96 unique rows, A=12, B=84; overlays detached and production config validation passed. No scene launched.")
            return 0
    validate_plan(plan, catalog)
    authorized_retry = False
    authorized_second_retry = False
    if args.authorized_retry_first:
        first = validate_authorized_first_retry(plan)
        attempts = first["attempts"]
        plan["authorized_resume"] = {
            "authorization": "User authorized preserving the original GLFW failure, one additional attempt of plan item 001, then continuation of the original 95 untouched items.",
            "authorized_at": datetime.now().astimezone().isoformat(timespec="seconds"),
            "original_attempt_preserved": attempts[0].get("run_directory"),
            "additional_attempt_limit_for_first_item": 1,
        }
        authorized_retry = True
    if args.authorized_retry_second:
        second = validate_authorized_second_retry(plan)
        attempts = second["attempts"]
        plan["authorized_second_item_resume"] = {
            "authorization": "User authorized preserving the first valid result, one additional attempt of plan item 002 after the offline reader fix, then continuation of the original 94 untouched items if VALID and cleanup are confirmed.",
            "authorized_at": datetime.now().astimezone().isoformat(timespec="seconds"),
            "original_attempt_preserved": attempts[0].get("run_directory"),
            "additional_attempt_limit_for_second_item": 1,
        }
        authorized_second_retry = True
    if args.resume and plan.get("status") in ("STOPPED", "COMPLETE") and not authorized_retry and not authorized_second_retry:
        if plan["status"] == "COMPLETE":
            print("S6-12 coarse matrix already complete; no new scene launched.")
            return 0
        print("S6-12 batch was stopped by an earlier failure; this executor does not auto-resume or retry it.")
        return 2
    plan["status"] = "RUNNING"
    update_plan(root, plan)
    if authorized_retry:
        first = plan["runs"][0]
        first["status"] = "AUTHORIZED_RETRY_RUNNING"
        update_plan(root, plan)
        if not run_item(root, first, plan, args):
            plan["status"] = "STOPPED"
            plan["stop_sequence"] = first["sequence"]
            update_plan(root, plan)
            print(f"S6-12 authorized retry stopped at {first['plan_item_id']}: {first.get('stop_reason')}; no second retry or later run started.", file=sys.stderr)
            return 1
        remaining_items = plan["runs"][1:]
    elif authorized_second_retry:
        second = plan["runs"][1]
        second["status"] = "AUTHORIZED_RETRY_RUNNING"
        update_plan(root, plan)
        if not run_item(root, second, plan, args):
            plan["status"] = "STOPPED"
            plan["stop_sequence"] = second["sequence"]
            update_plan(root, plan)
            print(f"S6-12 authorized retry stopped at {second['plan_item_id']}: {second.get('stop_reason')}; no second retry or later run started.", file=sys.stderr)
            return 1
        remaining_items = plan["runs"][2:]
    else:
        remaining_items = plan["runs"]
    for item in remaining_items:
        if item["status"] == "VALID_TASK_RESULT":
            continue
        if item["status"] != "NOT_STARTED":
            plan["status"] = "STOPPED"
            plan["stop_sequence"] = item["sequence"]
            update_plan(root, plan)
            raise RuntimeError(f"plan item {item['plan_item_id']} has state {item['status']}; no automatic restart")
        if not run_item(root, item, plan, args):
            plan["status"] = "STOPPED"
            plan["stop_sequence"] = item["sequence"]
            update_plan(root, plan)
            print(f"S6-12 stopped at {item['plan_item_id']}: {item.get('stop_reason')}; no retry or later run started.", file=sys.stderr)
            return 1
    plan["status"] = "COMPLETE"
    plan["completed_at"] = datetime.now().astimezone().isoformat(timespec="seconds")
    update_plan(root, plan)
    write_partial_summary(root, plan, args.plot_python)
    choose_timelines(root, plan)
    print(f"S6-12 coarse matrix complete: {root / 's6_12_coarse_plan.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
