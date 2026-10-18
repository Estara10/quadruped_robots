#!/usr/bin/env python3
"""T1 new-M2 18-map B-group single-pass diagnostic (hidden GLFW, no RViz).

This runner is deliberately small. It freezes the 18 map identities and the
B / E=-0.05 / X=-0.07 contract, then drives the existing production chain
manual_ab_compare.py -> s1_run_diagnostic.py once per map. It is resumable per
plan item and never reruns a completed scene without an explicit operator
retry authorization.
"""
from __future__ import annotations

import argparse
import csv
import fcntl
import hashlib
import json
import math
import os
import shutil
import signal
import subprocess
import sys
import time
import uuid
from datetime import datetime
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
import manual_ab_compare as manual  # noqa: E402
from abs_scene import resolve_scene  # noqa: E402
from s1_run_diagnostic import resolve_map_manifest  # noqa: E402

PARENT = ROOT / "logs/t1/m2_map_smoke_18"
LAYOUTS = ROOT / "docs/thesis_project/designs/M2/selected_v2/layouts.json"
SCENE_INDEX = ROOT / "docs/thesis_project/designs/M2/selected_v2/scene_index.csv"
SOURCE = ROOT / "quadruped_ros2_control_humble/install/go2_description"
PLAN_FILE = "m2_map_smoke_plan.json"
MAP_ORDER = ("e01.xml", "m01.xml", "d01.xml", "e02.xml", "m02.xml", "d02.xml",
             "e03.xml", "m03.xml", "d03.xml", "m04.xml", "d04.xml", "m05.xml",
             "d05.xml", "m06.xml", "d06.xml", "d07.xml", "d08.xml", "d09.xml")
GROUP = "B"
E = -0.05
X = -0.07
GOAL = (7.5, 0.0)
SIM_LIMIT_S = 20.0
WALL_CAP_S = 120.0
STANDING_SIM_S = 2.0
STANDING_WALL_CAP_S = 30.0
ARRIVAL_RADIUS_M = 0.5
CLASSIFICATION = "新版M2 18图B组单次地图诊断，非正式实验"
LEGITIMATE_TERMINALS = {"ARRIVED", "COLLISION_TERMINATION", "FALL_TERMINATION",
                        "DIAGNOSTIC_SIM_TIME_LIMIT", "WALL_CLOCK_GUARD_INCOMPLETE"}
BASE_SCENE_STARTS = 18
MAX_SCENE_STARTS = 20
MAX_STARTS_PER_ITEM = 2
MAX_ENGINEERING_REPAIRS = 2
MAX_BATCH_BYTES = 1 * 1024**3
MIN_FREE_BYTES = 15 * 1024**3
RELATED_PATTERNS = ("unitree_mujoco", "rl_quadruped_controller", "ros2 launch",
                    "rviz2", "manual_ab_compare.py", "s1_run_diagnostic.py")


def now() -> str:
    return datetime.now().astimezone().isoformat(timespec="seconds")


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def atomic_json(path: Path, value: object) -> None:
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(value, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    os.replace(tmp, path)


def load_json(path: Path) -> dict:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return {}


def write_status(root: Path, status: str, **fields: object) -> None:
    payload = load_json(root / "runner_status.json")
    payload.update({"status": status, "runner_pid": payload.get("runner_pid") or os.getpid(),
                    "updated_at": now(), "batch_root": str(root.resolve()),
                    "render_mode": "hidden GLFW", "rviz": "disabled", **fields})
    atomic_json(root / "runner_status.json", payload)


def related_processes() -> list[dict]:
    hits = []
    own = {os.getpid(), os.getppid()}
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit() or int(entry.name) in own:
            continue
        try:
            cmd = (entry / "cmdline").read_bytes().replace(b"\0", b" ").decode("utf-8", "replace").strip()
        except OSError:
            continue
        if not cmd:
            continue
        if any(pattern in cmd for pattern in RELATED_PATTERNS):
            if "manual_t1_m2_smoke_18.py" in cmd:
                continue
            hits.append({"pid": int(entry.name), "cmd": cmd[:300]})
    return hits


def load_contract() -> list[dict]:
    library = load_json(LAYOUTS)
    if library.get("mapset_version") != "m2-selected-development/2.1":
        raise RuntimeError("selected_v2 layouts.json is not the approved mapset revision")
    by_name = {row["filename"]: row for row in library.get("scenes", [])}
    index = {}
    with SCENE_INDEX.open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            index[row["filename"]] = row
    if set(by_name) != set(MAP_ORDER) or set(index) != set(MAP_ORDER):
        raise RuntimeError("selected_v2 index does not contain exactly the frozen 18 maps")
    items = []
    seen_ids = set()
    seen_sources = set()
    for sequence, filename in enumerate(MAP_ORDER, start=1):
        record = by_name[filename]
        row = index[filename]
        spec = resolve_scene(filename)
        if spec.scene_id != record["scene_id"] or spec.scene_id != row["scene_id"]:
            raise RuntimeError(f"scene identity mismatch: {filename}")
        if spec.scene_id in seen_ids:
            raise RuntimeError(f"duplicate scene identity: {spec.scene_id}")
        seen_ids.add(spec.scene_id)
        if row["difficulty_label"] != record["difficulty_label"]:
            raise RuntimeError(f"difficulty label mismatch: {filename}")
        source_key = (record["source_set"], record["source_filename"])
        if source_key in seen_sources:
            raise RuntimeError(f"source map duplicated as an independent item: {filename}")
        seen_sources.add(source_key)
        items.append({
            "sequence": sequence, "plan_item_id": f"{sequence:02d}_{record['scene_id']}",
            "map": filename, "scene_id": spec.scene_id,
            "difficulty_label": record["difficulty_label"],
            "source_set": record["source_set"], "source_filename": record["source_filename"],
            "previous_m2_filename": record.get("previous_m2_filename"),
            "layout_revision": record.get("layout_revision"),
            "field_bounds_m": record["field_bounds_m"],
            "xml_sha256": sha(ROOT / "unitree_mujoco/unitree_robots/go2" / filename),
            "group": GROUP, "entry_threshold": E, "exit_threshold": X, "hold": False,
            "goal_world_m": list(GOAL), "sim_limit_s": SIM_LIMIT_S, "wall_cap_s": WALL_CAP_S,
            "standing_observation_sim_s": STANDING_SIM_S,
            "standing_observation_wall_cap_s": STANDING_WALL_CAP_S,
            "status": "NOT_STARTED", "attempts": []})
    return items


def record_versions() -> dict:
    def digest(path: Path) -> dict:
        return {"path": str(path.resolve()), "sha256": sha(path)} if path.is_file() else {"path": str(path), "missing": True}
    models = ROOT / "quadruped_ros2_control_humble/install/go2_description/share/go2_description/config"
    try:
        commit = subprocess.run(["git", "-C", str(ROOT), "rev-parse", "HEAD"], capture_output=True,
                                text=True, timeout=10).stdout.strip() or None
    except (OSError, subprocess.SubprocessError):
        commit = None
    return {
        "git_commit": commit,
        "catalog_sha256": sha(ROOT / "common/abs_scene_catalog.def"),
        "mujoco_binary": digest(ROOT / "unitree_mujoco/simulate/build2/unitree_mujoco"),
        "controller_library": digest(
            ROOT / "quadruped_ros2_control_humble/install/rl_quadruped_controller/lib/"
                   "rl_quadruped_controller/librl_quadruped_controller.so"),
        "models": {name: digest(models / name) for name in
                   ("abs/policy.pt", "abs/ra_value.pt", "rec/policy.pt")},
        "runtime_mujoco_version": "3.3.3",
        "recorded_at": now(),
    }


def create_plan(root: Path) -> dict:
    items = load_contract()
    plan = {
        "schema": "t1-m2-18-map-smoke-v1",
        "batch_id": root.name,
        "classification": CLASSIFICATION,
        "authority": "docs/thesis_project/T1_M2_18_MAP_SMOKE_PROMPT.md",
        "created_at": now(),
        "mapset": {"path": str(LAYOUTS.resolve()), "version": "m2-selected-development/2.1",
                   "sha256": sha(LAYOUTS), "scene_index_sha256": sha(SCENE_INDEX)},
        "versions": record_versions(),
        "contract": {"group": GROUP, "entry_threshold_E": E, "exit_threshold_X": X,
                     "hysteresis": True, "hold_enabled": False,
                     "goal_world_m": list(GOAL), "arrival_radius_m": ARRIVAL_RADIUS_M,
                     "sim_limit_s": SIM_LIMIT_S, "wall_cap_s": WALL_CAP_S,
                     "arrival_standing_observation_sim_s": STANDING_SIM_S,
                     "arrival_standing_wall_cap_s": STANDING_WALL_CAP_S},
        "rendering": {"backend": "GLFW", "requested_mode": "hidden",
                      "env": "ABS_MUJOCO_NO_WINDOW=1", "rviz": "start_rviz:=false",
                      "framebuffer_capture": False,
                      "note": "hidden GLFW still requires the approved X11/OpenGL context; not true headless"},
        "budget": {"base_scene_starts": BASE_SCENE_STARTS,
                   "max_scene_starts": MAX_SCENE_STARTS,
                   "max_starts_per_item": MAX_STARTS_PER_ITEM,
                   "max_engineering_repairs": MAX_ENGINEERING_REPAIRS,
                   "max_batch_bytes": MAX_BATCH_BYTES,
                   "min_free_bytes": MIN_FREE_BYTES},
        "counters": {"confirmed_scene_starts": 0, "engineering_repairs": 0},
        "items": items,
    }
    atomic_json(root / PLAN_FILE, plan)
    return plan


def prepare_overlay(root: Path) -> Path:
    overlay = root / "overlays" / "B"
    if not (overlay / "share/go2_description/config/abs/config.yaml").is_file():
        overlay.parent.mkdir(parents=True, exist_ok=True)
        manual.prepare_overlay(SOURCE, overlay, GROUP, E, X, 0.30)
    manual.set_overlay_goal(overlay / "share/go2_description/config/abs/config.yaml",
                            GOAL[0], GOAL[1])
    config = overlay / "share/go2_description/config/abs/config.yaml"
    if not manual.overlay_configuration_matches(config.read_text(encoding="utf-8"), GROUP, E,
                                               X, 0.30, GOAL[0], GOAL[1]):
        raise RuntimeError("prepared B overlay failed production configuration validation")
    return overlay


def preflight(root: Path, args: argparse.Namespace) -> dict:
    env = os.environ.copy()
    env.update({"DISPLAY": args.display, "XAUTHORITY": args.xauthority,
                "MUJOCO_GL": "glfw", "PYTHONNOUSERSITE": "1"})
    path = root / "glfw_preflight.json"
    result = subprocess.run(["/usr/bin/python3", str(ROOT / "scripts/m2_glfw_preflight.py"), str(path)],
                            cwd=ROOT, env=env, capture_output=True, text=True, timeout=30)
    facts = load_json(path)
    ok = (result.returncode == 0 and facts.get("status") == "PASS"
          and (facts.get("environment") or {}).get("DISPLAY") == args.display
          and (facts.get("environment") or {}).get("XAUTHORITY") == args.xauthority)
    facts["same_execution_context_as_scene_runs"] = ok
    facts["returncode"] = result.returncode
    facts["stderr_tail"] = result.stderr[-1000:]
    return facts


def item_console_path(item_root: Path) -> Path:
    return item_root / "console.log"


def build_command(item: dict, overlay: Path, result_root: Path, args: argparse.Namespace) -> list[str]:
    return ["/usr/bin/python3", "-u", str(ROOT / "scripts/manual_ab_compare.py"),
            "--groups", "B", "--repeats", "1", "--scene", item["map"],
            "--goal-x", f"{GOAL[0]:.17g}", "--goal-y", f"{GOAL[1]:.17g}",
            "--entry-threshold", f"{E:g}", "--exit-threshold", f"{X:g}",
            "--sim-limit-s", f"{SIM_LIMIT_S:.17g}", "--wall-cap-s", f"{WALL_CAP_S:.17g}",
            "--standing-observation-sim-s", f"{STANDING_SIM_S:.17g}",
            "--result-root", str(result_root), "--prepared-overlay", str(overlay),
            "--display", args.display, "--xauthority", args.xauthority,
            "--plot-python", args.plot_python,
            "--classification", CLASSIFICATION,
            "--no-window", "--disable-framebuffer-capture"]


def read_run(root: Path, result_root: Path) -> dict:
    plan = load_json(result_root / "batch_plan.json")
    status = next((row for row in plan.get("run_statuses", []) if row.get("group") == GROUP), {})
    run_id = status.get("run_id")
    run_dir = Path(status["output_directory"]) if status.get("output_directory") else None
    if run_dir is not None and not run_dir.is_absolute():
        run_dir = (result_root / run_dir).resolve()
    found = {"batch_plan": plan, "status": status, "run_id": run_id,
             "run_directory": run_dir, "metrics": {}, "context": {}, "terminal": {},
             "process_facts": {}}
    if run_dir is None or not run_dir.is_dir():
        candidates = sorted(result_root.glob("B/rep_01/*"))
        run_dir = candidates[0] if candidates else None
        found["run_directory"] = run_dir
    for name, key in (("metrics.json", "metrics"), ("run_context.json", "context"),
                      ("terminal.json", "terminal"), ("process_facts.json", "process_facts")):
        if run_dir is not None:
            found[key] = load_json(run_dir / name)
    mujoco_log = (run_dir / "mujoco.log") if run_dir is not None else None
    found["mujoco_started"] = bool(mujoco_log and mujoco_log.is_file()
                                   and "MuJoCo version" in mujoco_log.read_text(encoding="utf-8", errors="replace"))
    found["runtime_record"] = bool(run_dir and (run_dir / "runtime_record.jsonl").is_file())
    return found


def classify(item: dict, found: dict, child_code: int | None) -> tuple[str, list[str], str | None]:
    """Return (status, reasons, stop_class).

    status: VALID_TASK_RESULT | ENGINEERING_FAILURE | STOPPED_FAILURE
    stop_class: None | 'safety' | 'record' | 'cleanup' | 'infrastructure'
    """
    metrics = found["metrics"]
    context = found["context"]
    facts = found["process_facts"]
    source = ((context.get("runtime_source_validation") or {}).get("evidence") or {})
    supervisor = metrics.get("supervisor_terminal_result")
    reasons = []
    if (found["run_directory"] is None and not found["mujoco_started"]
            and not metrics and not found["terminal"]):
        # Argument/orchestration rejection happens before s1 creates any run
        # directory or MuJoCo process. Account separately from scene starts.
        return "ORCHESTRATION_FAILURE", reasons + [f"runner_exit={child_code}"], "orchestration"
    if child_code != 0:
        reasons.append(f"runner_exit={child_code}")
    if not found["mujoco_started"] and not supervisor:
        return "ENGINEERING_FAILURE", reasons + ["no_scene_or_terminal_evidence"], "infrastructure"
    if supervisor == "SYSTEM_SAFETY_ABORT":
        return "STOPPED_FAILURE", reasons + ["system_safety_abort"], "safety"
    if metrics.get("record_validity") != "VALID":
        reasons.append(f"record_validity={metrics.get('record_validity')}")
    if (context.get("runtime_source_validation") or {}).get("status") != "confirmed":
        reasons.append("runtime_source_not_confirmed")
    if source.get("group") != GROUP:
        reasons.append(f"actual_group={source.get('group')}")
    try:
        if abs(float(source.get("entry_threshold")) - E) > 1e-5:
            reasons.append("entry_threshold_mismatch")
        if abs(float(source.get("exit_threshold")) - X) > 1e-5:
            reasons.append("exit_threshold_mismatch")
    except (TypeError, ValueError):
        reasons.append("consumed_threshold_missing")
    if metrics.get("s6_preparation_readiness", {}).get("first_active_policy_frame_exactly_persisted") is not True:
        reasons.append("first_active_policy_frame_not_exactly_persisted")
    if facts.get("shutdown_complete") is not True or facts.get("closeout_complete") is not True:
        reasons.append("shutdown_or_closeout_unconfirmed")
    if not found["runtime_record"] or not found["terminal"]:
        reasons.append("raw_runtime_or_terminal_missing")
    if supervisor not in LEGITIMATE_TERMINALS:
        reasons.append(f"terminal={supervisor}")
    if reasons:
        stop = "cleanup" if any("shutdown_or_closeout" in r for r in reasons) else (
            "record" if any(("record_validity" in r or "raw_runtime" in r or "source" in r or
                             "threshold" in r) for r in reasons) else "infrastructure")
        return "STOPPED_FAILURE", reasons, stop
    return "VALID_TASK_RESULT", [], None


def run_item(root: Path, plan: dict, item: dict, args: argparse.Namespace) -> dict:
    attempt_number = len(item["attempts"]) + 1
    if attempt_number > MAX_STARTS_PER_ITEM:
        raise RuntimeError("per-item scene-start cap exhausted")
    if plan["counters"]["confirmed_scene_starts"] >= MAX_SCENE_STARTS:
        raise RuntimeError("total scene-start cap exhausted")
    overlay = prepare_overlay(root)
    item_root = root / "runs" / item["plan_item_id"]
    child = item_root if attempt_number == 1 else item_root / f"attempt_{attempt_number:02d}"
    child.mkdir(parents=True, exist_ok=True)
    result_root = child / "result"
    if result_root.exists():
        raise RuntimeError(f"result root already exists; refusing overwrite: {result_root}")
    attempt = {"attempt_number": attempt_number, "status": "RUNNING", "started_at": now(),
               "result_root": str(result_root), "classification": CLASSIFICATION,
               "render_mode": "hidden GLFW", "rviz": "disabled",
               "command": build_command(item, overlay, result_root, args)}
    item["attempts"].append(attempt)
    item["status"] = "RUNNING"
    attempt["planned_progress"] = f"{item['sequence']}/18"
    atomic_json(root / PLAN_FILE, plan)
    console = item_console_path(item_root) if attempt_number == 1 else child / "console.log"
    code = None
    with console.open("w", encoding="utf-8") as stream:
        proc = subprocess.Popen(build_command(item, overlay, result_root, args), cwd=ROOT,
                                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
        try:
            assert proc.stdout is not None
            for line in proc.stdout:
                print(line, end="", flush=True)
                stream.write(line)
                stream.flush()
            code = proc.wait()
        except KeyboardInterrupt:
            if proc.poll() is None:
                proc.send_signal(signal.SIGINT)
            try:
                proc.wait(timeout=120)
            except subprocess.TimeoutExpired as error:
                raise RuntimeError("child cleanup unconfirmed after Ctrl+C") from error
            raise
    found = read_run(root, result_root)
    status, reasons, stop_class = classify(item, found, code)
    if status == "ORCHESTRATION_FAILURE":
        plan.setdefault("orchestration_events", []).append({
            "sequence": item["sequence"], "map": item["map"], "at": now(),
            "child_exit_code": code, "reasons": reasons,
            "classification": "pure orchestration error; no scene created; not counted as a scene start"})
        plan["counters"]["orchestration_corrections"] = (
            plan["counters"].get("orchestration_corrections", 0) + 1)
        if plan["counters"]["orchestration_corrections"] > 2:
            raise RuntimeError("orchestration correction budget exhausted")
        item["attempts"].pop()
        archive_index = len([e for e in plan.get("orchestration_events", [])
                             if e.get("sequence") == item["sequence"]])
        if result_root.exists():
            result_root.rename(child / f"orchestration_result_{archive_index:02d}")
        if console.exists():
            console.rename(child / f"orchestration_console_{archive_index:02d}.log")
        item["status"] = "ORCHESTRATION_FAILURE"
        item["failure_reasons"] = reasons
        item["stop_class"] = stop_class
        atomic_json(root / PLAN_FILE, plan)
        write_progress(root, plan)
        print(f"[{item['sequence']}/18] {item['map']} -> ORCHESTRATION_FAILURE "
              f"({'; '.join(reasons)}); no scene created", file=sys.stderr, flush=True)
        return {"attempt": None, "status": status, "stop_class": stop_class, "reasons": reasons}
    metrics, context, facts = found["metrics"], found["context"], found["process_facts"]
    source = ((context.get("runtime_source_validation") or {}).get("evidence") or {})
    cycle0 = (context.get("initial_state") or {}).get("first_active_policy_cycle") or {}
    attempt.update({
        "finished_at": now(), "child_exit_code": code,
        "run_id": found["run_id"], "run_directory": str(found["run_directory"]) if found["run_directory"] else None,
        "terminal_result": metrics.get("supervisor_terminal_result"),
        "record_validity": metrics.get("record_validity"),
        "runtime_source_validation": (context.get("runtime_source_validation") or {}).get("status"),
        "actual_group": source.get("group"), "actual_entry_threshold": source.get("entry_threshold"),
        "actual_exit_threshold": source.get("exit_threshold"),
        "actual_hold": source.get("hold"),
        "process_shutdown_complete": facts.get("shutdown_complete"),
        "closeout_complete": facts.get("closeout_complete"),
        "mujoco_started": found["mujoco_started"],
        "muJoCo_window_log": ((context.get("preflight") or {}).get("mujoco_window_observation") or {}),
        "initial_state": {"status": cycle0.get("status"), "sim_time_s": cycle0.get("sim_time_s"),
                          "world_position_xy_m": cycle0.get("world_position_xy_m"),
                          "world_yaw_rad": cycle0.get("world_yaw_rad"),
                          "same_cycle_runtime_frame_persisted_by_supervisor":
                              cycle0.get("same_cycle_runtime_frame_persisted_by_supervisor")},
        "status": status, "failure_reasons": reasons})
    if found["mujoco_started"]:
        plan["counters"]["confirmed_scene_starts"] += 1
    if status == "VALID_TASK_RESULT":
        item["status"] = status
        item["finished_at"] = attempt["finished_at"]
    else:
        item["status"] = status
        item["failure_reasons"] = reasons
    item["stop_class"] = stop_class
    atomic_json(root / PLAN_FILE, plan)
    write_progress(root, plan)
    print(f"[{item['sequence']}/18] {item['map']} -> {attempt['terminal_result']} "
          f"({status}); record={attempt['record_validity']}; cleanup="
          f"{attempt['process_shutdown_complete']}/{attempt['closeout_complete']}", flush=True)
    return {"attempt": attempt, "status": status, "stop_class": stop_class, "reasons": reasons}


def write_progress(root: Path, plan: dict) -> None:
    rows = []
    for item in plan["items"]:
        last = item["attempts"][-1] if item["attempts"] else {}
        rows.append([item["sequence"], item["map"], item["difficulty_label"], item["source_set"],
                     item["source_filename"], item.get("status"), last.get("run_id"),
                     last.get("terminal_result"), last.get("record_validity"),
                     "; ".join(item.get("failure_reasons") or [])])
    with (root / "attempt_ledger.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["sequence", "map", "difficulty", "source_set", "source_map",
                         "item_status", "run_id", "terminal_result", "record_validity",
                         "failure_reasons"])
        writer.writerows(rows)
    atomic_json(root / "progress.json", {
        "updated_at": now(), "planned": len(plan["items"]),
        "completed": sum(item.get("status") == "VALID_TASK_RESULT" for item in plan["items"]),
        "confirmed_scene_starts": plan["counters"]["confirmed_scene_starts"],
        "items": [{"sequence": item["sequence"], "map": item["map"], "status": item.get("status")}
                  for item in plan["items"]]})


def extract_row(item: dict, attempt: dict) -> dict:
    metrics = load_json(Path(attempt["run_directory"]) / "metrics.json") if attempt.get("run_directory") else {}
    context = load_json(Path(attempt["run_directory"]) / "run_context.json") if attempt.get("run_directory") else {}
    safety = metrics.get("task_window_safety") or {}
    bounds = metrics.get("declared_boundary_physics_observation") or {}
    return {
        "sequence": item["sequence"], "map": item["map"], "scene_id": item["scene_id"],
        "difficulty": item["difficulty_label"], "source_set": item["source_set"],
        "source_map": item["source_filename"], "previous_m2_name": item.get("previous_m2_filename"),
        "layout_revision": item.get("layout_revision"), "run_id": attempt.get("run_id"),
        "attempt": attempt.get("attempt_number"),
        "actual_config": f"{attempt.get('actual_group')}/E={attempt.get('actual_entry_threshold')}/"
                         f"X={attempt.get('actual_exit_threshold')}/hold={attempt.get('actual_hold')}",
        "terminal": attempt.get("terminal_result"),
        "record_validity": attempt.get("record_validity"),
        "config_consumption": (context.get("runtime_source_validation") or {}).get("status"),
        "arrival_distance_m": metrics.get("target_distance_final_m"),
        "task_duration_s": metrics.get("elapsed_task_duration_s"),
        "path_length_m": metrics.get("path_length_m"),
        "policy_frequency_hz": metrics.get("policy_frequency_hz_overall"),
        "cycle_gaps": metrics.get("motion_policy_cycle_gaps"),
        "invalid_clock_frames": metrics.get("invalid_clock_frames"),
        "recovery_entries": metrics.get("agile_to_recovery"),
        "recovery_exits": metrics.get("recovery_to_agile"),
        "recovery_share": metrics.get("recovery_share_valid_motion_time"),
        "short_reversals": metrics.get("short_reverse_switch_count"),
        "foot_impact_count": metrics.get("foot_impact_count"),
        "nonfoot_collision": safety.get("nonfoot_collision_failure"),
        "collision": safety.get("collision"),
        "fall": safety.get("fall"),
        "safety_coverage_complete": safety.get("coverage_complete_for_motion_interval"),
        "outside_physics_steps": bounds.get("outside_physics_step_delta"),
        "bounds_status": bounds.get("status"),
        "standing_observation": metrics.get("standing_observation_status"),
        "standing_sim_s_requested": item["standing_observation_sim_s"],
        "shutdown_complete": attempt.get("process_shutdown_complete"),
        "closeout_complete": attempt.get("closeout_complete"),
        "mujoco_started": attempt.get("mujoco_started"),
        "window_log": json.dumps(attempt.get("muJoCo_window_log") or {}, ensure_ascii=False),
        "initial_xy": json.dumps((attempt.get("initial_state") or {}).get("world_position_xy_m")),
        "initial_yaw": (attempt.get("initial_state") or {}).get("world_yaw_rad"),
        "config_initial_sim_time_s": (attempt.get("initial_state") or {}).get("sim_time_s"),
        "status": item.get("status"),
        "failure_reasons": "; ".join(item.get("failure_reasons") or []),
    }


def analyze(root: Path, plan: dict, plot_python: str) -> dict:
    rows = [extract_row(item, item["attempts"][-1]) for item in plan["items"] if item["attempts"]]
    fields = list(rows[0].keys()) if rows else []
    with (root / "run_results.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    completed = sum(row["status"] == "VALID_TASK_RESULT" for row in rows)
    terminals = {}
    for row in rows:
        if row["status"] == "VALID_TASK_RESULT":
            terminals[row["terminal"]] = terminals.get(row["terminal"], 0) + 1
    lines = ["# 新版M2 18图B组单次诊断（非正式实验）", "",
             f"- 批次：`{root.name}`",
             f"- 分类：{CLASSIFICATION}",
             f"- 固定参数：B / E={E} / X={X} / 保持关闭；目标 {GOAL}，到达半径 {ARRIVAL_RADIUS_M} m",
             f"- 时限：{SIM_LIMIT_S:g} 仿真秒 / {WALL_CAP_S:g} 墙钟秒；到达后站立观察 {STANDING_SIM_S:g} 仿真秒",
             f"- 计划/有效任务结果：{len(plan['items'])}/{completed}",
             f"- 终局：{json.dumps(terminals, ensure_ascii=False)}",
             f"- 确认场景启动：{plan['counters']['confirmed_scene_starts']}（上限 {MAX_SCENE_STARTS}）",
             "", "| 序号 | 地图 | 难度 | 来源 | run_id | 终局 | 记录 | 到达距离m | 任务s | 路径m | 频率Hz | 周期缺口 | Recovery进/出 | 短反向 | 足端撞击 | 非足端碰撞 | 跌倒 | 越界步 | 站立2s | 清理 |",
             "|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|"]
    for row in rows:
        lines.append("| " + " | ".join(str(value) if value is not None else "UNKNOWN" for value in (
            row["sequence"], row["map"], row["difficulty"], f"{row['source_set']}:{row['source_map']}",
            row["run_id"], row["terminal"], row["record_validity"], row["arrival_distance_m"],
            row["task_duration_s"], row["path_length_m"], row["policy_frequency_hz"], row["cycle_gaps"],
            f"{row['recovery_entries']}/{row['recovery_exits']}", row["short_reversals"],
            row["foot_impact_count"], row["nonfoot_collision"], row["fall"], row["outside_physics_steps"],
            row["standing_observation"], f"{row['shutdown_complete']}/{row['closeout_complete']}")) + " |")
    lines.append("")
    lines.append("次数、覆盖率、终局与安全UNKNOWN分开保留；单次地图诊断不估计成功率、不比较阈值、不自动删图。")
    (root / "RESULTS_SUMMARY.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    plotter = ROOT / "scripts/plot_t1_m2_smoke_18.py"
    if plotter.is_file() and rows:
        result = subprocess.run([plot_python, str(plotter), str(root / "run_results.csv"),
                                 str(root / "charts")], capture_output=True, text=True, timeout=180)
        (root / "plot.log").write_text(result.stdout + result.stderr, encoding="utf-8")
    return {"run_results_csv": str(root / "run_results.csv"),
            "summary": str(root / "RESULTS_SUMMARY.md"), "completed": completed,
            "terminals": terminals, "rows": len(rows)}


def storage_guard(root: Path, plan: dict) -> None:
    usage = sum(path.stat().st_size for path in root.rglob("*") if path.is_file())
    free = shutil.disk_usage(root).free
    plan.setdefault("storage_checkpoints", []).append(
        {"at": now(), "allocated_bytes": usage, "filesystem_free_bytes": free})
    if usage > MAX_BATCH_BYTES or free < MIN_FREE_BYTES:
        raise RuntimeError(f"storage guard tripped: batch={usage} free={free}")


def pending_items(root: Path, plan: dict, retry_sequence: int | None, retry_reason: str | None) -> list[dict]:
    # Reclassify pre-existing pre-scene failures created before this runner
    # distinguished orchestration errors from infrastructure failures.
    for item in plan["items"]:
        if item.get("status") != "ENGINEERING_FAILURE" or not item.get("attempts"):
            continue
        last = item["attempts"][-1]
        if last.get("run_id") or last.get("mujoco_started") or last.get("run_directory"):
            continue
        plan.setdefault("orchestration_events", []).append({
            "sequence": item["sequence"], "map": item["map"], "at": now(),
            "child_exit_code": last.get("child_exit_code"),
            "reasons": last.get("failure_reasons"),
            "reclassified_from": "ENGINEERING_FAILURE",
            "classification": "pre-scene orchestration error; no scene created; not a scene start"})
        item["attempts"].pop()
        child = root / "runs" / item["plan_item_id"]
        failed_result = Path(last["result_root"]) if last.get("result_root") else child / "result"
        archive_index = len([e for e in plan["orchestration_events"]
                             if e.get("sequence") == item["sequence"]]) - 1
        if failed_result.exists():
            failed_result.rename(failed_result.parent / f"orchestration_result_{archive_index:02d}")
        console = child / "console.log"
        if console.exists():
            console.rename(child / f"orchestration_console_{archive_index:02d}.log")
        item["status"] = "NOT_STARTED"
        item["failure_reasons"] = []
        plan["counters"]["orchestration_corrections"] = (
            plan["counters"].get("orchestration_corrections", 0) + 1)
    if retry_sequence is not None:
        matches = [item for item in plan["items"] if item["sequence"] == retry_sequence]
        if len(matches) != 1:
            raise RuntimeError("retry sequence does not identify exactly one item")
        item = matches[0]
        if item.get("status") not in {"ENGINEERING_FAILURE", "STOPPED_FAILURE"}:
            raise RuntimeError("retry requires a previously stopped engineering failure")
        if len(item["attempts"]) >= MAX_STARTS_PER_ITEM:
            raise RuntimeError("per-item scene-start cap exhausted")
        if not retry_reason:
            raise RuntimeError("retry requires an explicit repair reason")
        item["attempts"][-1]["status"] = "ENGINEERING_FAILURE_REPAIRED"
        item["attempts"][-1]["repair"] = {"reason": retry_reason, "authorized_at": now()}
        plan["counters"]["engineering_repairs"] += 1
        if plan["counters"]["engineering_repairs"] > MAX_ENGINEERING_REPAIRS:
            raise RuntimeError("engineering repair budget exhausted")
        item["status"] = "NOT_STARTED"
        return [item]
    done = {item["sequence"] for item in plan["items"] if item.get("status") == "VALID_TASK_RESULT"}
    pending = [item for item in plan["items"] if item["sequence"] not in done]
    stopped = [item for item in pending if item.get("status") in {"ENGINEERING_FAILURE", "STOPPED_FAILURE"}]
    if stopped:
        raise RuntimeError("batch contains a stopped item; explicit retry authorization is required: "
                           + ", ".join(str(item["sequence"]) for item in stopped))
    if [item["sequence"] for item in pending] != sorted(item["sequence"] for item in pending):
        raise RuntimeError("resume would reorder the frozen sequence")
    return pending


def run_batch(root: Path, args: argparse.Namespace) -> int:
    lock_stream = (root / "runner.lock").open("a+")
    try:
        fcntl.flock(lock_stream.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError as error:
        lock_stream.close()
        raise RuntimeError("another 18-map smoke runner owns the batch lock") from error
    try:
        return _run_locked(root, args)
    except KeyboardInterrupt:
        write_status(root, "STOPPED_OPERATOR", reason="keyboard interrupt; child cleanup requested")
        raise
    except Exception as error:
        write_status(root, "FAILED", error=f"{type(error).__name__}: {error}")
        raise
    finally:
        fcntl.flock(lock_stream.fileno(), fcntl.LOCK_UN)
        lock_stream.close()


def _run_locked(root: Path, args: argparse.Namespace) -> int:
    plan_path = root / PLAN_FILE
    plan = load_json(plan_path) or create_plan(root)
    plan["classification"] = CLASSIFICATION
    pending = pending_items(root, plan, args.retry_sequence, args.retry_reason)
    if not pending:
        write_status(root, "NO_PENDING_ITEMS")
        return 0
    existing = related_processes()
    if existing:
        plan["preflight_process_inventory_before_scene"] = existing
        atomic_json(plan_path, plan)
        raise RuntimeError("related project process already running; refusing first scene")
    facts = preflight(root, args)
    plan["preflight"] = facts
    if not facts.get("same_execution_context_as_scene_runs"):
        plan["status"] = "STOPPED_PREFLIGHT"
        atomic_json(plan_path, plan)
        write_status(root, "STOPPED_PREFLIGHT", preflight=facts)
        raise RuntimeError("same-context hidden GLFW preflight failed; no scene started")
    plan["status"] = "RUNNING"
    plan["runner_pid"] = os.getpid()
    atomic_json(plan_path, plan)
    (root / "runner.pid").write_text(f"{os.getpid()}\n", encoding="ascii")
    write_status(root, "RUNNING", preflight_status="PASS", renderer=facts.get("renderer"),
                 planned=len(plan["items"]), pending=len(pending))
    for item in pending:
        if (root / "STOP_AFTER_CURRENT_RUN").exists():
            plan["status"] = "STOPPED_USER_REQUEST"
            plan["next_sequence"] = item["sequence"]
            atomic_json(plan_path, plan)
            write_status(root, "STOPPED_USER_REQUEST", next_sequence=item["sequence"])
            return 0
        if related_processes():
            raise RuntimeError("identified project process remained after prior cleanup")
        storage_guard(root, plan)
        atomic_json(plan_path, plan)
        print(f"开始第 {item['sequence']}/18 项：{item['map']} "
              f"({item['difficulty_label']}, {item['source_set']}:{item['source_filename']})", flush=True)
        outcome = run_item(root, plan, item, args)
        if outcome["status"] != "VALID_TASK_RESULT":
            plan["status"] = "STOPPED"
            plan["stop_sequence"] = item["sequence"]
            plan["stop_reason"] = "; ".join(outcome["reasons"])
            plan["stop_class"] = outcome["stop_class"]
            atomic_json(plan_path, plan)
            write_progress(root, plan)
            print(f"STOP at {item['map']}: {plan['stop_reason']}", file=sys.stderr, flush=True)
            return 1
    plan["status"] = "RUNS_COMPLETE_PENDING_ANALYSIS"
    atomic_json(plan_path, plan)
    try:
        plan["analysis"] = analyze(root, plan, args.plot_python)
    except Exception as error:
        plan["status"] = "ANALYSIS_FAILED_RAW_DATA_PRESERVED"
        plan["analysis_error"] = f"{type(error).__name__}: {error}"
        atomic_json(plan_path, plan)
        raise
    plan["status"] = "COMPLETE"
    plan["finished_at"] = now()
    atomic_json(plan_path, plan)
    write_progress(root, plan)
    write_status(root, "COMPLETE", completed=plan["analysis"]["completed"],
                 confirmed_scene_starts=plan["counters"]["confirmed_scene_starts"])
    return 0


class Tee:
    def __init__(self, *streams):
        self.streams = streams

    def write(self, data):
        for stream in self.streams:
            stream.write(data)
            stream.flush()

    def flush(self):
        for stream in self.streams:
            stream.flush()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--batch-id", default=None,
                        help="existing batch id to resume; default creates a new batch")
    parser.add_argument("--root", type=Path, default=None, help="explicit batch root (default under logs/t1/m2_map_smoke_18)")
    parser.add_argument("--retry-sequence", type=int, default=None,
                        help="authorize one engineering retry of the stopped item sequence")
    parser.add_argument("--retry-reason", default=None)
    parser.add_argument("--plot-python", default="/home/lidio/anaconda3/bin/python3")
    parser.add_argument("--display", default=":0")
    parser.add_argument("--xauthority", default="/run/user/1000/gdm/Xauthority")
    args = parser.parse_args()
    if args.root is not None and args.batch_id is not None:
        parser.error("use either --root or --batch-id")
    if not Path(args.plot_python).is_file():
        parser.error(f"plot python not found: {args.plot_python}")
    if args.root is not None:
        root = args.root.resolve()
    elif args.batch_id:
        root = (PARENT / args.batch_id).resolve()
    else:
        root = PARENT / (datetime.now().strftime("%Y%m%d_%H%M%S") + "_" + uuid.uuid4().hex[:6])
    if root.exists() and not (root / PLAN_FILE).is_file() and any(root.iterdir()):
        parser.error(f"existing non-empty directory without a plan: {root}")
    root.mkdir(parents=True, exist_ok=True)
    log_stream = (root / "runner.log").open("a", encoding="utf-8", buffering=1)
    sys.stdout = Tee(sys.__stdout__, log_stream)
    sys.stderr = Tee(sys.__stderr__, log_stream)
    print(f"批次目录：{root}", flush=True)
    return run_batch(root, args)


if __name__ == "__main__":
    sys.exit(main())
