#!/usr/bin/env python3
"""Frozen T1 two-exit-threshold comparison on the 36 M1/M2 development maps."""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
import random
import shutil
import signal
import subprocess
import sys
from collections import Counter, defaultdict
from datetime import datetime
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
import manual_ab_compare as manual  # noqa: E402
import manual_t1_expanded_400 as expanded  # noqa: E402
from abs_scene import resolve_scene  # noqa: E402
from s1_run_diagnostic import overlay_configuration_matches  # noqa: E402

M1_INDEX = ROOT / "docs/thesis_project/designs/M1/library_v1/scene_index.csv"
M2_INDEX = ROOT / "docs/thesis_project/designs/M2/candidate_v1/scene_index.csv"
M2_LAYOUTS = ROOT / "docs/thesis_project/designs/M2/candidate_v1/layouts.json"
SOURCE = ROOT / "quadruped_ros2_control_humble/install/go2_description"
PARENT = ROOT / "logs/t1/two_thresholds_36_maps"
E = -0.05
XS = (-0.07, -0.12)
SCHEDULE_SEED = 2026101604
BOOTSTRAP_SEED = 2026101605
BOOTSTRAP_N = 10000
MAX_BATCH_BYTES = 4 * 1024**3
MIN_FREE_BYTES = 15 * 1024**3
PLAN_FILE = "expanded_400_plan.json"  # consumed by the already-reviewed production run helper
MAP_NAMES = tuple([f"dev{i:02d}.xml" for i in range(1, 31)] +
                  [f"m2dev{i:02d}.xml" for i in range(1, 7)])


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def atomic_json(path: Path, value: object) -> None:
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(value, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    os.replace(tmp, path)


def load_catalog() -> list[dict]:
    rows = {}
    for path in (M1_INDEX, M2_INDEX):
        with path.open(newline="", encoding="utf-8") as stream:
            rows.update({row["filename"]: row for row in csv.DictReader(stream)})
    layouts = json.loads(M2_LAYOUTS.read_text(encoding="utf-8"))
    m2_layouts = {row["filename"]: row for row in layouts.get("scenes", [])}
    catalog = []
    for filename in MAP_NAMES:
        row = rows.get(filename)
        if not row:
            raise RuntimeError(f"missing scene-index entry: {filename}")
        m2 = m2_layouts.get(filename, {})
        expected_split = "development" if filename.startswith("dev") else "development_candidate"
        actual_split = row.get("split") or m2.get("split")
        if actual_split != expected_split:
            raise RuntimeError(f"non-development scene rejected: {filename} split={actual_split}")
        scene = resolve_scene(filename)
        scene_id = row.get("scene_id")
        if scene.scene_id != scene_id:
            raise RuntimeError(f"production scene identity mismatch: {filename}")
        if len(scene_id.encode("ascii")) >= 32:
            raise RuntimeError(f"scene identity exceeds authority field: {filename}")
        xml = ROOT / "unitree_mujoco/unitree_robots/go2" / filename
        if not xml.is_file():
            raise RuntimeError(f"scene XML missing: {xml}")
        goal_x = float(row.get("goal_x", 7.5))
        goal_y = float(row.get("goal_y", 0.0))
        sim_limit = float(row.get("sim_limit_s", 20))
        wall_cap = float(row.get("wall_cap_s", 120))
        if (not math.isclose(goal_x, 7.5, abs_tol=1e-9) or not math.isclose(goal_y, 0.0, abs_tol=1e-9)
                or sim_limit != 20 or wall_cap != 120):
            raise RuntimeError(f"frozen goal/limit mismatch: {filename}")
        if filename.startswith("m2") and m2.get("scene_id") != scene_id:
            raise RuntimeError(f"M2 layout identity mismatch: {filename}")
        catalog.append({"filename": filename, "scene_id": scene_id,
            "dataset": "M1" if filename.startswith("dev") else "M2",
            "density": row.get("density") or m2.get("density", "UNKNOWN"),
            "route_structure": row.get("route_structure") or row.get("route_structure_label_zh") or "UNKNOWN",
            "split": expected_split, "generation_seed": row.get("seed", row.get("generation_seed")),
            "goal_world_m": [goal_x, goal_y], "sim_limit_s": sim_limit,
            "wall_cap_s": wall_cap, "xml_sha256": sha(xml)})
    return catalog


def make_schedule(catalog: list[dict]) -> list[dict]:
    rng = random.Random(SCHEDULE_SEED)
    first_x07 = set(rng.sample([m["filename"] for m in catalog], 18))
    runs = []
    sequence = 0
    per_round_order = {}
    for repeat in (1, 2):
        order = list(catalog)
        rng.shuffle(order)
        for map_index, scene in enumerate(order):
            if repeat == 1:
                xs = [-0.07, -0.12] if scene["filename"] in first_x07 else [-0.12, -0.07]
            else:
                first = per_round_order[scene["filename"]]
                xs = list(reversed(first))
            if repeat == 1:
                per_round_order[scene["filename"]] = list(xs)
            for x in xs:
                sequence += 1
                item_id = f"{sequence:03d}_r{repeat}_{scene['scene_id']}_x{abs(round(x*100)):02d}"
                runs.append({"sequence": sequence, "plan_item_id": item_id,
                    "map": scene["filename"], "scene_id": scene["scene_id"],
                    "dataset": scene["dataset"], "density": scene["density"],
                    "route_structure": scene["route_structure"], "repeat": repeat,
                    "group": "B", "entry_threshold_E": E, "exit_threshold_X": x,
                    "effective_exit_threshold": x, "hysteresis": True, "hold_enabled": False,
                    "first_threshold_in_round": xs[0],
                    "screenshot_preselected": repeat == 1 and map_index < 4,
                    "status": "NOT_STARTED", "attempts": []})
    return runs


def validate_schedule(runs: list[dict], catalog: list[dict]) -> None:
    if len(runs) != 144 or len({r["plan_item_id"] for r in runs}) != 144:
        raise RuntimeError("plan must contain 144 unique item IDs")
    if {r["map"] for r in runs} != set(MAP_NAMES) or {r["exit_threshold_X"] for r in runs} != set(XS):
        raise RuntimeError("map/threshold set differs from frozen authorization")
    for repeat in (1, 2):
        block = [r for r in runs if r["repeat"] == repeat]
        if len(block) != 72:
            raise RuntimeError(f"round {repeat} must contain 72 items")
        pairs = [block[i:i+2] for i in range(0, 72, 2)]
        if any(len(pair) != 2 or pair[0]["map"] != pair[1]["map"] or
               {pair[0]["exit_threshold_X"], pair[1]["exit_threshold_X"]} != set(XS) for pair in pairs):
            raise RuntimeError(f"round {repeat} does not keep two thresholds adjacent by map")
        if len({pair[0]["map"] for pair in pairs}) != 36:
            raise RuntimeError(f"round {repeat} has duplicate or missing map blocks")
    round1 = {r["map"]: [x["exit_threshold_X"] for x in runs if x["repeat"] == 1 and x["map"] == r["map"]]
              for r in runs if r["repeat"] == 1}
    first_counts = Counter(values[0] for values in round1.values())
    if first_counts != {-0.07: 18, -0.12: 18}:
        raise RuntimeError(f"round one must balance first threshold 18/18: {first_counts}")
    for scene in MAP_NAMES:
        a = [r["exit_threshold_X"] for r in runs if r["map"] == scene and r["repeat"] == 1]
        b = [r["exit_threshold_X"] for r in runs if r["map"] == scene and r["repeat"] == 2]
        if b != list(reversed(a)):
            raise RuntimeError(f"round two failed to reverse paired order: {scene}")
    regenerated = make_schedule(catalog)
    keys = ("plan_item_id", "map", "scene_id", "repeat", "group", "exit_threshold_X")
    if [[r[k] for k in keys] for r in runs] != [[r[k] for k in keys] for r in regenerated]:
        raise RuntimeError("schedule is not reproducible from frozen seed")


def prepare_overlays(root: Path) -> dict:
    overlays = {}
    for x in XS:
        key = f"B_{x:.2f}"
        target = root / "overlays" / key
        manual.prepare_overlay(SOURCE, target, "B", E, x, 0.30)
        config = target / "share/go2_description/config/abs/config.yaml"
        manual.set_overlay_goal(config, 7.5, 0.0)
        detached = manual.verify_overlay_detached(SOURCE, target)
        text = config.read_text(encoding="utf-8")
        if not overlay_configuration_matches(text, "B", E, x, 0.30, 7.5, 0.0):
            raise RuntimeError(f"production parser rejected B overlay X={x}")
        if "recovery_min_hold_s:" in text:
            raise RuntimeError(f"B overlay unexpectedly enables hold: {key}")
        overlays[key] = {"path": str(target.resolve()), "group": "B",
            "entry_threshold_E": E, "exit_threshold_X": x, "hysteresis": True,
            "hold_enabled": False, "goal_world_m": [7.5, 0.0],
            "config_sha256": sha(config), "detached_files": detached}
    return overlays


def current_related_processes() -> list[dict]:
    matches = []
    self_pid = os.getpid()
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit() or int(entry.name) == self_pid:
            continue
        try:
            exe = os.readlink(entry / "exe")
            cmd = (entry / "cmdline").read_bytes().replace(b"\0", b" ").decode("utf-8", "replace").strip()
        except OSError:
            continue
        base = Path(exe).name
        if base in {"unitree_mujoco", "ros2_control_node", "rviz2"} or (
                base == "ros2" and "launch rl_quadruped_controller" in cmd) or (
                base.startswith("python") and any(n in cmd for n in
                    ("scripts/manual_ab_compare.py", "scripts/s1_run_diagnostic.py",
                     "scripts/manual_t1_two_thresholds_36.py"))):
            matches.append({"pid": int(entry.name), "exe": exe, "cmdline": cmd[:1000]})
    return matches


def storage_reference() -> dict:
    # Use only already completed M1 development runs to estimate raw allocation.
    candidates = [ROOT / "logs/t1/b_exit_validation_400/20261016_131943_26/expanded_400_plan.json",
                  ROOT / "logs/s6_12/b_exit_coarse/20261015_192706_3/s6_12_coarse_plan.json"]
    sizes = []
    source = None
    for plan_path in candidates:
        if not plan_path.is_file():
            continue
        try:
            previous = json.loads(plan_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            continue
        for item in previous.get("runs", []):
            for attempt in item.get("attempts", []):
                if attempt.get("record_validity") != "VALID":
                    continue
                run_dir = attempt.get("run_directory") or attempt.get("child_root")
                path = Path(run_dir) if run_dir else None
                if path and path.is_dir():
                    sizes.append(expanded.allocated_bytes(path))
        if sizes:
            source = str(plan_path)
            break
    if not sizes:
        raise RuntimeError("cannot estimate storage from prior valid production runs")
    sizes.sort()
    return {"reference_plan": source, "valid_run_samples": len(sizes),
        "median_allocated_bytes_per_run": sizes[len(sizes)//2],
        "p90_allocated_bytes_per_run": sizes[int(.9*(len(sizes)-1))]}


def code_snapshot(catalog: list[dict]) -> dict:
    snap = expanded.code_snapshot({"code_commit": subprocess.run(
        ["git", "rev-parse", "HEAD"], cwd=ROOT, check=True, capture_output=True, text=True).stdout.strip()})
    snap.update({"m1_scene_index_sha256": sha(M1_INDEX), "m2_scene_index_sha256": sha(M2_INDEX),
        "m2_layouts_sha256": sha(M2_LAYOUTS), "map_xml_sha256": {m["filename"]: m["xml_sha256"] for m in catalog},
        "batch_runner_sha256": sha(Path(__file__).resolve())})
    return snap


def verify_frozen(plan: dict, catalog: list[dict]) -> None:
    snap = plan["code_snapshot"]
    expanded.assert_frozen_code_unchanged(plan)
    now = code_snapshot(catalog)
    for key in ("m1_scene_index_sha256", "m2_scene_index_sha256", "m2_layouts_sha256", "map_xml_sha256", "batch_runner_sha256"):
        if snap.get(key) != now.get(key):
            raise RuntimeError(f"frozen batch input changed: {key}")
    if plan.get("maps") != catalog:
        raise RuntimeError("frozen map catalog changed")
    for x in XS:
        overlay = Path(plan["overlays"][f"B_{x:.2f}"]["path"])
        if not manual.verify_overlay_detached(SOURCE, overlay):
            raise RuntimeError(f"overlay detached-file check failed: {overlay}")
        config = overlay / "share/go2_description/config/abs/config.yaml"
        if not overlay_configuration_matches(config.read_text(encoding="utf-8"), "B", E, x, 0.30, 7.5, 0.0):
            raise RuntimeError(f"frozen config mismatch: {x}")


def prepare_plan(root: Path, display: str, xauthority: str) -> dict:
    if root.exists():
        raise RuntimeError(f"refusing to overwrite existing batch: {root}")
    root.mkdir(parents=True, exist_ok=False)
    (root / "runs").mkdir()
    (root / "overlays").mkdir()
    catalog = load_catalog()
    runs = make_schedule(catalog)
    validate_schedule(runs, catalog)
    overlays = prepare_overlays(root)
    code = code_snapshot(catalog)
    reference = storage_reference()
    free = shutil.disk_usage(root).free
    if free < MIN_FREE_BYTES:
        raise RuntimeError(f"only {free} free bytes; need at least {MIN_FREE_BYTES}")
    if current_related_processes():
        raise RuntimeError("identified project runtime processes already active; inspect before batch")
    plan = {"schema": "t1-two-thresholds-36-v1", "status": "FROZEN",
        "created_at": datetime.now().astimezone().isoformat(timespec="seconds"),
        "batch_id": root.name, "classification": "T1 B-group development parameter pre-experiment; nonformal",
        "maps": catalog, "runs": runs, "planned_count": 144, "group": "B",
        "entry_threshold_E": E, "exit_thresholds_X": list(XS), "hold_enabled": False,
        "goal_world_m": [7.5, 0.0], "arrival_radius_m": 0.5,
        "sim_limit_s": 20.0, "wall_cap_s": 120.0,
        "standing_observation_sim_s": 5.0, "standing_wall_cap_s": 30.0,
        "schedule_seed": SCHEDULE_SEED,
        "schedule_rule": "two rounds of 36 shuffled adjacent map blocks; thresholds are paired; first-round order balanced 18/18 and second round reverses each map; schedule frozen before launch",
        "simulator_random_seed": {"controllable": False,
            "evidence": "manual_ab_compare and s1_run_diagnostic CLI expose no simulator/noise seed argument; no seed was injected"},
        "overlays": overlays, "code_snapshot": code, "storage_reference": reference,
        "storage_budget_bytes": MAX_BATCH_BYTES, "minimum_filesystem_free_bytes": MIN_FREE_BYTES,
        "attempt_limits": {"planned_scene_starts": 144, "additional_repaired_infrastructure_starts": 4,
            "total_actual_scene_start_cap": 148, "max_starts_per_plan_item": 2,
            "pre_scene_orchestration_reentries": 2, "failure_policy": "stop on first unresolved failure; legal task terminal never retried"},
        "preflight": {"required_environment": {"DISPLAY": display, "XAUTHORITY": xauthority, "MUJOCO_GL": "glfw"},
            "status": "NOT_RUN", "record": str(root / "glfw_preflight.json")},
        "preflight_process_inventory": {"checked_at": datetime.now().astimezone().isoformat(timespec="seconds"),
            "matches": current_related_processes()},
        "scoring": {"weights": {"arrival": 0.4, "no_terminal_collision": 0.3,
            "efficiency": 0.2, "stability": 0.1}, "time_cap_s": 20.0,
            "short_reverse_reference_per_valid_min": 257.8,
            "formula": "100*(.40*arrival + .30*no_terminal_collision + .20*efficiency + .10/(1+f/257.8))",
            "bootstrap_replicates": BOOTSTRAP_N, "bootstrap_seed": BOOTSTRAP_SEED,
            "fixed_before_scene_runs": True},
        "status_counts": {"planned": 144, "per_threshold": 72, "per_map_threshold": 2}}
    plan["storage_estimate"] = {"free_before_run_bytes": free,
        "projected_final_bytes": reference["p90_allocated_bytes_per_run"] * 144 + 8 * 20 * 1024**2,
        "must_stay_below_bytes": MAX_BATCH_BYTES}
    atomic_json(root / PLAN_FILE, plan)
    return plan


def resume_gate(plan: dict, root: Path, *, retry_sequence: int | None = None, repair_reason: str | None = None) -> list[dict]:
    catalog = load_catalog()
    validate_schedule(plan["runs"], catalog)
    verify_frozen(plan, catalog)
    if plan.get("status") == "COMPLETE":
        return []
    runs = plan["runs"]
    if retry_sequence is None:
        if plan.get("status") not in {"FROZEN", "RUNNING", "STOPPED_OPERATOR"}:
            raise RuntimeError(f"batch status is not resumable: {plan.get('status')}")
        if any(r.get("status") not in {"NOT_STARTED", "VALID_TASK_RESULT"} for r in runs):
            raise RuntimeError("unresolved failure; requires explicit repaired-failure retry gate")
        states = [r.get("status") for r in runs]
        first_pending = next((i for i, state in enumerate(states) if state == "NOT_STARTED"), len(states))
        if states[:first_pending] != ["VALID_TASK_RESULT"] * first_pending or states[first_pending:] != ["NOT_STARTED"] * (len(states)-first_pending):
            raise RuntimeError("resume would skip a completed item or reorder the frozen schedule")
    else:
        if not repair_reason or plan.get("status") != "STOPPED":
            raise RuntimeError("retry requires a stopped batch and a specific repair reason")
        item = runs[retry_sequence - 1]
        attempts = item.get("attempts", [])
        if (len(attempts) != 1 or item.get("status") != "STOPPED_FAILURE" or
                attempts[0].get("terminal_result") in {"ARRIVED", "COLLISION_TERMINATION", "FALL_TERMINATION",
                  "DIAGNOSTIC_SIM_TIME_LIMIT", "SYSTEM_SAFETY_ABORT"} or
                attempts[0].get("process_shutdown_complete") is not True or
                attempts[0].get("closeout_complete") is not True):
            raise RuntimeError("only one repaired, cleaned infrastructure/record failure may be retried")
        if any(r.get("status") != "VALID_TASK_RESULT" for r in runs[:retry_sequence-1]) or any(
                r.get("status") != "NOT_STARTED" or r.get("attempts") for r in runs[retry_sequence:]):
            raise RuntimeError("repair retry must resume at the single stopped row without skipping/reordering later rows")
        prior_retries = int(plan.get("repaired_infrastructure_retry_count", 0))
        if prior_retries >= 4:
            raise RuntimeError("four repaired infrastructure retry allowance exhausted")
        if any(r.get("status") not in {"NOT_STARTED", "VALID_TASK_RESULT", "STOPPED_FAILURE"} for r in runs):
            raise RuntimeError("batch contains conflicting in-progress items")
        item["status"] = "RETRY_AUTHORIZED_ENGINEERING"
        attempts[0]["status"] = "ENGINEERING_FAILURE_REPAIRED"
        attempts[0]["repair"] = {"reason": repair_reason,
            "authorized_at": datetime.now().astimezone().isoformat(timespec="seconds")}
        plan["repaired_infrastructure_retry_count"] = prior_retries + 1
        plan["status"] = "RUNNING"
        atomic_json(root / PLAN_FILE, plan)
    pending = [r for r in runs if r.get("status") in {"NOT_STARTED", "RETRY_AUTHORIZED_ENGINEERING"}]
    if pending:
        first = pending[0]["sequence"]
        if any(r["sequence"] > first and r.get("status") == "RETRY_AUTHORIZED_ENGINEERING" for r in pending):
            raise RuntimeError("cannot skip or reorder a previously authorized retry")
    return pending


def run_batch(root: Path, args: argparse.Namespace) -> int:
    plan_path = root / PLAN_FILE
    plan = json.loads(plan_path.read_text(encoding="utf-8"))
    pending = resume_gate(plan, root, retry_sequence=args.retry_sequence, repair_reason=args.repair_reason)
    if not pending:
        return 0
    env = os.environ.copy()
    env.update({"DISPLAY": args.display, "XAUTHORITY": args.xauthority,
        "MUJOCO_GL": "glfw", "PYTHONNOUSERSITE": "1"})
    # A single bounded GLFW context check and the run loop share this process environment.
    preflight = subprocess.run(["/usr/bin/python3", str(ROOT / "scripts/m2_glfw_preflight.py"),
        str(root / "glfw_preflight.json")], cwd=ROOT, env=env, capture_output=True, text=True, timeout=20)
    try:
        facts = json.loads((root / "glfw_preflight.json").read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        facts = {"status": "FAIL", "stderr": preflight.stderr[-2000:]}
    expected = {"DISPLAY": args.display, "XAUTHORITY": args.xauthority, "MUJOCO_GL": "glfw"}
    if preflight.returncode != 0 or facts.get("status") != "PASS" or any((facts.get("environment") or {}).get(k) != v for k, v in expected.items()):
        plan["preflight"] = {"status": "FAIL", "returncode": preflight.returncode,
            "stdout": preflight.stdout[-2000:], "stderr": preflight.stderr[-2000:], "facts": facts}
        plan["status"] = "STOPPED_PREFLIGHT"
        atomic_json(plan_path, plan)
        raise RuntimeError("same-context GLFW preflight failed; no scene started")
    inventory = current_related_processes()
    if inventory:
        plan["status"] = "STOPPED_PREFLIGHT"
        plan["preflight_process_inventory_after_glfw"] = {"matches": inventory}
        atomic_json(plan_path, plan)
        raise RuntimeError("related project process appeared during preflight; refusing first scene")
    plan["preflight"] = {"status": "PASS", "returncode": preflight.returncode,
        "facts": facts, "environment": facts.get("environment"),
        "same_execution_context_as_scene_runs": True,
        "window_closed": facts.get("window_closed") is True}
    plan["preflight_process_inventory_after_glfw"] = {"checked_at": datetime.now().astimezone().isoformat(timespec="seconds"), "matches": []}
    plan["status"] = "RUNNING"
    atomic_json(plan_path, plan)
    expanded.MAX_BATCH_BYTES = MAX_BATCH_BYTES
    expanded.MIN_FREE_BYTES = MIN_FREE_BYTES
    for item in pending:
        if current_related_processes():
            raise RuntimeError("identified project process remained after prior item cleanup; stopping")
        confirmed_starts = sum(bool(a.get("run_id") or a.get("scene_started") is True)
            for row in plan["runs"] for a in row.get("attempts", []))
        if confirmed_starts >= 148:
            plan["status"] = "STOPPED_SCENE_START_CAP"
            plan["stop_sequence"] = item["sequence"]
            atomic_json(plan_path, plan)
            return 2
        if item.get("attempts") and len(item["attempts"]) >= 2:
            raise RuntimeError("plan item scene-start cap (2) exhausted")
        if sum(len(r.get("attempts", [])) > 1 for r in plan["runs"]) >= 4 and not item.get("attempts"):
            raise RuntimeError("four infrastructure repair starts already consumed")
        try:
            expanded.run_one(root, plan, item, args)
            validate_completed_item(root, plan, item)
        except KeyboardInterrupt:
            plan["status"] = "STOPPED_OPERATOR"
            atomic_json(plan_path, plan)
            raise
        except Exception as error:
            if item.get("attempts"):
                item["attempts"][-1].update({"status": "STOPPED_FAILURE",
                    "post_orchestration_error": f"{type(error).__name__}: {error}"})
            item["status"] = "STOPPED_FAILURE"
            item["failure_reasons"] = [f"{type(error).__name__}: {error}"]
            plan["status"] = "STOPPED"
            plan["stop_sequence"] = item["sequence"]
            plan["stop_reason"] = item["failure_reasons"][0]
            atomic_json(plan_path, plan)
            write_progress(root, plan)
            print(f"STOP at {item['plan_item_id']}: {error}", file=sys.stderr, flush=True)
            return 1
        item["status"] = "VALID_TASK_RESULT"
        item["completed_at"] = datetime.now().astimezone().isoformat(timespec="seconds")
        atomic_json(plan_path, plan)
        if item["sequence"] % 8 == 0:
            usage = expanded.allocated_bytes(root)
            free = shutil.disk_usage(root).free
            plan.setdefault("storage_checkpoints", []).append({"sequence": item["sequence"],
                "allocated_bytes": usage, "filesystem_free_bytes": free,
                "checked_at": datetime.now().astimezone().isoformat(timespec="seconds")})
            atomic_json(plan_path, plan)
            write_progress(root, plan)
            print(f"Progress {item['sequence']}/144; batch={usage/1024**3:.2f} GiB; free={free/1024**3:.1f} GiB", flush=True)
            if usage > MAX_BATCH_BYTES or free < MIN_FREE_BYTES:
                plan["status"] = "STOPPED_DISK_GUARD"
                plan["stop_sequence"] = item["sequence"]
                atomic_json(plan_path, plan)
                return 2
    plan["status"] = "RUNS_COMPLETE_PENDING_ANALYSIS"
    plan["finished_at"] = datetime.now().astimezone().isoformat(timespec="seconds")
    atomic_json(plan_path, plan)
    write_progress(root, plan)
    try:
        plan["analysis"] = analyze(root, plan, args.plot_python)
    except Exception as error:
        plan["status"] = "ANALYSIS_FAILED_RAW_DATA_PRESERVED"
        plan["analysis_error"] = f"{type(error).__name__}: {error}"
        atomic_json(plan_path, plan)
        raise
    plan["status"] = "COMPLETE"
    plan["analysis_completed_at"] = datetime.now().astimezone().isoformat(timespec="seconds")
    atomic_json(plan_path, plan)
    write_progress(root, plan)
    return 0


def write_progress(root: Path, plan: dict) -> None:
    attempts = [a for r in plan["runs"] for a in r.get("attempts", [])]
    summary = {"status": plan.get("status"), "planned": 144,
        "plan_items_with_valid_tasks": sum(r.get("status") == "VALID_TASK_RESULT" and r.get("record_validity") == "VALID" for r in plan["runs"]),
        "plan_items_with_attempt": sum(bool(r.get("attempts")) for r in plan["runs"]),
        "attempt_records": len(attempts), "not_started": sum(r.get("status") == "NOT_STARTED" for r in plan["runs"]),
        "terminal_counts_valid": dict(Counter((r.get("terminal_result") or "UNKNOWN") for r in plan["runs"]
            if r.get("status") == "VALID_TASK_RESULT")),
        "filesystem_free_bytes": shutil.disk_usage(root).free,
        "batch_allocated_bytes": expanded.allocated_bytes(root)}
    atomic_json(root / "progress_summary.json", summary)


def validate_completed_item(root: Path, plan: dict, item: dict) -> None:
    attempt = item["attempts"][-1]
    run_dir = Path(attempt["run_directory"])
    context = load_json(run_dir / "run_context.json")
    metrics = load_json(run_dir / "metrics.json")
    facts = load_json(run_dir / "process_facts.json")
    source = context.get("runtime_source_validation") or {}
    evidence = source.get("evidence") or {}
    settings = context.get("settings") or {}
    overlay = Path(plan["overlays"][f"B_{item['exit_threshold_X']:.2f}"]["path"]).resolve()
    issues = []
    checks = {
        "source confirmed": source.get("status") == "confirmed",
        "group B consumed": evidence.get("group") == "B" and settings.get("group") == "B",
        "entry E consumed": abs(float(evidence.get("entry_threshold", math.nan)) - E) < 1e-5,
        "exit X consumed": abs(float(evidence.get("exit_threshold", math.nan)) - item["exit_threshold_X"]) < 1e-5,
        "no minimum hold": evidence.get("hold") in ("false", False) and float(evidence.get("recovery_min_hold_s", math.nan)) == 0.0,
        "correct scene filename": settings.get("scene_filename") == item["map"],
        "correct scene identity": settings.get("scene_id") == item["scene_id"],
        "correct target": settings.get("goal_world_m") == [7.5, 0.0] and evidence.get("goal_world_m") == [7.5, 0.0],
        "correct detached overlay": Path(settings.get("overlay", "")).resolve() == overlay,
        "one writer/controller": evidence.get("writer_pid") is not None and evidence.get("writer_pid") == evidence.get("controller_pid"),
        "same session": evidence.get("session_id") == metrics.get("session_id") and context.get("session_id") == metrics.get("session_id"),
        "cycle0 persisted": (metrics.get("s6_preparation_readiness") or {}).get("first_active_policy_frame_exactly_persisted") is True,
        "clean closeout": facts.get("shutdown_complete") is True and facts.get("closeout_complete") is True,
        "terminal and raw frame records": (run_dir / "terminal.json").is_file() and (run_dir / "runtime_record.jsonl").is_file(),
    }
    issues = [name for name, passed in checks.items() if not passed]
    item["post_run_source_and_boundary_checks"] = checks
    if issues:
        attempt["status"] = "STOPPED_FAILURE"
        attempt["failure_reasons"] = issues
        item["status"] = "STOPPED_FAILURE"
        item["failure_reasons"] = issues
        atomic_json(root / PLAN_FILE, plan)
        raise RuntimeError("post-run source/boundary check failed: " + ", ".join(issues))


def load_json(path: Path) -> dict:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
        return value if isinstance(value, dict) else {}
    except (OSError, json.JSONDecodeError):
        return {}


def metrics_for_batch(plan: dict) -> list[dict]:
    rows = []
    for item in plan["runs"]:
        attempts = item.get("attempts", [])
        attempt = attempts[-1] if attempts else {}
        run_dir = Path(attempt["run_directory"]) if attempt.get("run_directory") else None
        metrics = load_json(run_dir / "metrics.json") if run_dir else {}
        context = load_json(run_dir / "run_context.json") if run_dir else {}
        facts = load_json(run_dir / "process_facts.json") if run_dir else {}
        source = context.get("runtime_source_validation") or {}
        source_evidence = source.get("evidence") or {}
        safety = metrics.get("task_window_safety") or {}
        lifecycle = context.get("lifecycle") or {}
        state = context.get("initial_state") or {}
        row = {k: item.get(k) for k in ("sequence", "plan_item_id", "map", "scene_id", "dataset",
            "density", "route_structure", "repeat", "group", "entry_threshold_E", "exit_threshold_X",
            "effective_exit_threshold", "screenshot_preselected", "status")}
        row.update({"attempts": len(attempts), "run_id": attempt.get("run_id"),
            "record_validity": attempt.get("record_validity"), "run_directory": str(run_dir) if run_dir else None,
            "terminal_result": attempt.get("terminal_result"), "source_validation": source.get("status"),
            "actual_group": source_evidence.get("group"), "actual_entry_threshold": source_evidence.get("entry_threshold"),
            "actual_exit_threshold": source_evidence.get("exit_threshold"),
            "actual_hold_enabled": source_evidence.get("hold", source_evidence.get("hold_enabled")),
            "actual_scene_id": source_evidence.get("scene_id"),
            "actual_goal_world_m": json.dumps(source_evidence.get("goal_world_m"), ensure_ascii=False),
            "target_distance_final_m": metrics.get("target_distance_final_m"),
            "elapsed_task_duration_s": metrics.get("elapsed_task_duration_s"),
            "valid_motion_duration_s": metrics.get("valid_motion_duration_s"),
            "task_valid_time_coverage_fraction": metrics.get("task_valid_time_coverage_fraction"),
            "path_length_m": metrics.get("path_length_m"), "mean_path_speed_mps": metrics.get("mean_path_speed_mps"),
            "peak_interval_path_speed_mps": metrics.get("peak_interval_path_speed_mps"),
            "policy_frequency_hz": metrics.get("policy_frequency_hz_overall"),
            "mode_transitions": metrics.get("mode_transitions"),
            "transitions_with_valid_sim_time": metrics.get("transitions_with_valid_sim_time"),
            "short_reverse_switch_count": metrics.get("short_reverse_switch_count"),
            "short_reverse_switch_coverage": metrics.get("short_reverse_switch_coverage"),
            "short_reverse_per_valid_min": (metrics.get("short_reverse_switch_count") /
                (metrics.get("valid_motion_duration_s") / 60.0)
                if metrics.get("short_reverse_switch_coverage") == "complete" and
                isinstance(metrics.get("short_reverse_switch_count"), (int, float)) and
                isinstance(metrics.get("valid_motion_duration_s"), (int, float)) and
                metrics["valid_motion_duration_s"] > 0 else None),
            "recovery_duration_s": metrics.get("recovery_sim_duration_s"),
            "recovery_share_valid_motion_time": metrics.get("recovery_share_valid_motion_time"),
            "recovery_cycles": metrics.get("recovery_cycles"), "agile_cycles": metrics.get("agile_cycles"),
            "collision": safety.get("collision", "UNKNOWN"), "fall": metrics.get("motion_fall_result", "UNKNOWN"),
            "collision_coverage_complete": safety.get("coverage_complete_for_motion_interval"),
            "foot_contact_count": safety.get("foot_contact_count"), "foot_impact_count": safety.get("foot_impact_count"),
            "nonfoot_collision_failure": safety.get("nonfoot_collision_failure"),
            "rl_step_gaps": metrics.get("rl_step_gaps"), "invalid_clock_frames": metrics.get("invalid_clock_frames"),
            "cycle0_saved": (metrics.get("s6_preparation_readiness") or {}).get("first_active_policy_frame_exactly_persisted"),
            "standing_observation_status": metrics.get("standing_observation_status") or
                ("COMPLETE" if (lifecycle.get("standing_observation") or {}).get("complete") is True else
                 "INCOMPLETE" if lifecycle.get("standing_observation") else "NOT_REACHED_OR_UNKNOWN"),
            "passive_transition_confirmed": (lifecycle.get("post_terminal_strategy_stop") or {}).get("controller_transition_confirmed"),
            "process_shutdown_complete": facts.get("shutdown_complete", attempt.get("process_shutdown_complete")),
            "closeout_complete": facts.get("closeout_complete", attempt.get("closeout_complete")),
            "cycle0_initial_state_json": json.dumps(state, ensure_ascii=False, sort_keys=True),
            "source_context_path": str(run_dir / "run_context.json") if run_dir else None,
            "terminal_path": str(run_dir / "terminal.json") if run_dir else None,
            "timeline_path": str(run_dir / "timeline.json") if run_dir and (run_dir / "timeline.json").is_file() else None})
        score = score_one(row)
        row.update(score)
        rows.append(row)
    return rows


def score_one(row: dict) -> dict:
    def num(value):
        try:
            v = float(value)
            return v if math.isfinite(v) else None
        except (TypeError, ValueError):
            return None
    collision = row.get("collision")
    if collision == "True": collision = True
    if collision == "False": collision = False
    elapsed = num(row.get("elapsed_task_duration_s"))
    frequency = num(row.get("short_reverse_per_valid_min"))
    reasons = []
    if row.get("status") != "VALID_TASK_RESULT" or row.get("record_validity") != "VALID": reasons.append("not_valid_task_record")
    if collision not in (True, False): reasons.append("task_window_collision_UNKNOWN")
    if elapsed is None: reasons.append("task_duration_missing")
    if row.get("short_reverse_switch_coverage") != "complete" or frequency is None: reasons.append("short_reverse_incomplete")
    if reasons:
        return {"score_status": "UNKNOWN", "score_missing_reasons": ";".join(reasons), "total_score_100": None}
    arrived = row.get("terminal_result") == "ARRIVED"
    arrival = 1.0 if arrived else 0.0
    safe = 1.0 if collision is False else 0.0
    efficiency = max(0.0, 1.0 - elapsed / 20.0) if arrived else 0.0
    stability = 1.0 / (1.0 + frequency / 257.8)
    total = 100.0 * (0.4 * arrival + 0.3 * safe + 0.2 * efficiency + 0.1 * stability)
    return {"score_status": "COMPLETE", "score_missing_reasons": "", "arrival_score_0_1": arrival,
        "no_terminal_collision_score_0_1": safe, "efficiency_score_0_1": efficiency,
        "stability_score_0_1": stability, "total_score_100": total}


def analyze(root: Path, plan: dict, plot_python: str) -> dict:
    rows = metrics_for_batch(plan)
    if len(rows) != 144:
        raise RuntimeError(f"analysis expected 144 rows, got {len(rows)}")
    fields = sorted({key for row in rows for key in row})
    with (root / "per_run_results_144.csv").open("w", newline="", encoding="utf-8-sig") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="ignore")
        writer.writeheader(); writer.writerows(rows)
    attempt_rows = []
    for item in plan["runs"]:
        for attempt in item.get("attempts", []):
            attempt_rows.append({"sequence": item["sequence"], "plan_item_id": item["plan_item_id"],
                "map": item["map"], "scene_id": item["scene_id"], "dataset": item["dataset"],
                "repeat": item["repeat"], "group": item["group"], "entry_threshold_E": E,
                "exit_threshold_X": item["exit_threshold_X"], **attempt})
    if attempt_rows:
        attempt_fields = sorted({k for row in attempt_rows for k in row})
        with (root / "attempt_ledger.csv").open("w", newline="", encoding="utf-8-sig") as stream:
            writer = csv.DictWriter(stream, fieldnames=attempt_fields, extrasaction="ignore")
            writer.writeheader(); writer.writerows(attempt_rows)
    with (root / "initial_state_and_coverage_144.csv").open("w", newline="", encoding="utf-8-sig") as stream:
        state_fields = ["sequence", "plan_item_id", "map", "scene_id", "repeat", "exit_threshold_X",
            "run_id", "cycle0_saved", "cycle0_initial_state_json", "source_context_path", "terminal_path",
            "rl_step_gaps", "invalid_clock_frames", "task_valid_time_coverage_fraction",
            "collision", "fall", "collision_coverage_complete"]
        writer = csv.DictWriter(stream, fieldnames=state_fields, extrasaction="ignore")
        writer.writeheader(); writer.writerows(rows)
    map_rows = []
    for scene in plan["maps"]:
        arms = {}
        for x in XS:
            cell = [r for r in rows if r["map"] == scene["filename"] and r["exit_threshold_X"] == x]
            valid = [r for r in cell if r["status"] == "VALID_TASK_RESULT" and r["record_validity"] == "VALID"]
            scored = [r for r in cell if r["score_status"] == "COMPLETE"]
            both = []
            for rep in (1, 2):
                pair = [r for r in rows if r["map"] == scene["filename"] and r["repeat"] == rep]
                left = next((r for r in pair if r["exit_threshold_X"] == -0.07), None)
                right = next((r for r in pair if r["exit_threshold_X"] == -0.12), None)
                if left and right and left.get("terminal_result") == right.get("terminal_result") == "ARRIVED":
                    both.append((left, right))
            arms[f"{x:.2f}"] = {"planned_n": 2, "attempt_n": sum(r["attempts"] for r in cell),
                "valid_n": len(valid), "terminal_counts": dict(Counter(r.get("terminal_result") or "UNKNOWN" for r in cell)),
                "arrived_n": sum(r.get("terminal_result") == "ARRIVED" for r in valid),
                "collision_true_n": sum(r.get("collision") is True or r.get("collision") == "True" for r in valid),
                "collision_false_n": sum(r.get("collision") is False or r.get("collision") == "False" for r in valid),
                "collision_unknown_n": sum(r.get("collision") not in (True, False, "True", "False") for r in valid),
                "fall_true_n": sum(r.get("fall") is True or r.get("fall") == "True" for r in valid),
                "fall_unknown_n": sum(r.get("fall") not in (True, False, "True", "False") for r in valid),
                "mean_arrival_time_s": mean([r.get("elapsed_task_duration_s") for r in valid if r.get("terminal_result") == "ARRIVED"]),
                "mean_short_reverse_per_valid_min": mean([r.get("short_reverse_per_valid_min") for r in valid]),
                "mean_recovery_share": mean([r.get("recovery_share_valid_motion_time") for r in valid]),
                "score_complete_n": len(scored),
                "mean_score_complete_only": mean([r.get("total_score_100") for r in scored]) if len(scored) == 2 else None,
                "mean_score_partial_available_only": mean([r.get("total_score_100") for r in scored]),
                "short_reverse_complete_n": sum(r.get("short_reverse_switch_coverage") == "complete" and isinstance(r.get("short_reverse_per_valid_min"), (int, float)) for r in valid),
                "safety_unknown_n": sum(r.get("collision") not in (True, False, "True", "False") or r.get("fall") not in (True, False, "True", "False") for r in valid),
                "standing_statuses": ";".join(str(r.get("standing_observation_status")) for r in cell),
                "all_run_ids": ";".join(str(r.get("run_id")) for r in cell)}
        a, b = arms["-0.07"], arms["-0.12"]
        map_rows.append({"map": scene["filename"], "scene_id": scene["scene_id"], "dataset": scene["dataset"],
            "density": scene["density"], "route_structure": scene["route_structure"],
            "x_-0.07": json.dumps(a, ensure_ascii=False, sort_keys=True),
            "x_-0.12": json.dumps(b, ensure_ascii=False, sort_keys=True),
            "delta_score_-0.12_minus_-0.07": diff(b["mean_score_complete_only"], a["mean_score_complete_only"]),
            "delta_arrival_time_both_arrived_subset_s": diff(
                mean([p[1].get("elapsed_task_duration_s") for p in both]),
                mean([p[0].get("elapsed_task_duration_s") for p in both])),
            "both_arrived_same_map_repeat_n": len(both),
            "both_arrived_subset_note": "仅同图同重复两阈值均ARRIVED的选择性子集；不作因果解释"})
    with (root / "map_comparison_36.csv").open("w", newline="", encoding="utf-8-sig") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(map_rows[0]))
        writer.writeheader(); writer.writerows(map_rows)
    summaries = []
    for dataset, subset_maps in (("M1", [m["filename"] for m in plan["maps"] if m["dataset"] == "M1"]),
                                 ("M2", [m["filename"] for m in plan["maps"] if m["dataset"] == "M2"]),
                                 ("ALL36_MAP_WEIGHTED", [m["filename"] for m in plan["maps"]])):
        per_x = {}
        for x in XS:
            mr = [next(z for z in map_rows if z["map"] == m) for m in subset_maps]
            arm = [json.loads(z[f"x_{x:.2f}"]) for z in mr]
            per_x[f"{x:.2f}"] = {"map_n": len(subset_maps),
                "valid_runs": sum(z["valid_n"] for z in arm), "planned_runs": 2 * len(subset_maps),
                "arrived_runs": sum(z["arrived_n"] for z in arm),
                "terminal_counts": dict(sum_counter(z["terminal_counts"] for z in arm)),
                "mean_score_map_equal_complete_only": mean([z["mean_score_complete_only"] for z in arm]),
                "maps_with_complete_score": sum(z["score_complete_n"] == 2 for z in arm),
                "mean_arrival_time_map_equal": mean([z["mean_arrival_time_s"] for z in arm]),
                "mean_short_reverse_map_equal": mean([z["mean_short_reverse_per_valid_min"] for z in arm]),
                "mean_recovery_share_map_equal": mean([z["mean_recovery_share"] for z in arm]),
                "collision_unknown_runs": sum(z["collision_unknown_n"] for z in arm),
                "fall_unknown_runs": sum(z["fall_unknown_n"] for z in arm)}
        summaries.append({"dataset": dataset, "maps": len(subset_maps), "thresholds": per_x})
    summary_by_map = {z["map"]: z for z in map_rows}
    paired_map_delta = {m: diff(json.loads(summary_by_map[m]["x_-0.12"])["mean_score_complete_only"],
                               json.loads(summary_by_map[m]["x_-0.07"])["mean_score_complete_only"]) for m in MAP_NAMES}
    signs = Counter("positive" if v > 0 else "negative" if v < 0 else "same" for v in paired_map_delta.values() if v is not None)
    m1_maps = [m for m in MAP_NAMES if m.startswith("dev")]
    m2_maps = [m for m in MAP_NAMES if m.startswith("m2")]
    delta_summary = {"complete_score_map_n": sum(paired_map_delta[m] is not None for m in MAP_NAMES),
        "positive_negative_same_map_counts": dict(signs), "mean_delta_equal_complete_map": mean(list(paired_map_delta.values())),
        "bootstrap": paired_bootstrap(paired_map_delta, m1_maps, m2_maps)}
    m1_strata = []
    m1_scenes = [m for m in plan["maps"] if m["dataset"] == "M1"]
    for field in ("density", "route_structure"):
        for category in sorted({m[field] for m in m1_scenes}):
            names = [m["filename"] for m in m1_scenes if m[field] == category]
            for x in XS:
                arms = [json.loads(summary_by_map[name][f"x_{x:.2f}"]) for name in names]
                m1_strata.append({"stratifier": field, "category": category, "maps": len(names),
                    "exit_threshold_X": x, "valid_runs": sum(a["valid_n"] for a in arms),
                    "arrived_runs": sum(a["arrived_n"] for a in arms),
                    "mean_score_complete_cells_only": mean([a["mean_score_complete_only"] for a in arms]),
                    "mean_arrival_time_map_equal": mean([a["mean_arrival_time_s"] for a in arms]),
                    "mean_short_reverse_map_equal": mean([a["mean_short_reverse_per_valid_min"] for a in arms]),
                    "mean_recovery_share_map_equal": mean([a["mean_recovery_share"] for a in arms])})
    with (root / "M1_density_structure_descriptive.csv").open("w", newline="", encoding="utf-8-sig") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(m1_strata[0]))
        writer.writeheader(); writer.writerows(m1_strata)
    atomic_json(root / "threshold_summary_M1_M2_ALL.json", {"summaries": summaries,
        "paired_map_score_delta_-0.12_minus_-0.07": delta_summary,
        "M1_density_and_route_structure_descriptive_only": m1_strata,
        "interpretation": "descriptive development-set comparison; map is the cluster; no winner or parameter optimum claimed"})
    flat_summary = []
    for s in summaries:
        for x in XS:
            flat_summary.append({"dataset": s["dataset"], "maps": s["maps"], "exit_threshold_X": x,
                **s["thresholds"][f"{x:.2f}"]})
    with (root / "threshold_summary_M1_M2_ALL.csv").open("w", newline="", encoding="utf-8-sig") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(flat_summary[0]))
        writer.writeheader(); writer.writerows(flat_summary)
    write_summary_markdown(root, plan, summaries, map_rows, delta_summary)
    make_plots(root, rows, map_rows, plot_python)
    return {"per_run_rows": len(rows), "map_rows": len(map_rows), "summary_groups": [s["dataset"] for s in summaries],
        "paired_map_score_delta": delta_summary}


def mean(values):
    vals = []
    for value in values:
        try:
            n = float(value)
            if math.isfinite(n): vals.append(n)
        except (TypeError, ValueError):
            pass
    return sum(vals) / len(vals) if vals else None


def diff(a, b):
    return a - b if isinstance(a, (int, float)) and isinstance(b, (int, float)) else None


def sum_counter(counters):
    result = Counter()
    for value in counters:
        result.update(value)
    return result


def paired_bootstrap(deltas: dict, m1: list[str], m2: list[str]) -> dict:
    # Resample maps as clusters, stratified to retain the fixed 30/6 dataset composition.
    rng = random.Random(BOOTSTRAP_SEED)
    vals = []
    for _ in range(BOOTSTRAP_N):
        sample = [rng.choice(m1) for _ in m1] + [rng.choice(m2) for _ in m2]
        observed = [deltas[m] for m in sample if deltas.get(m) is not None]
        if observed: vals.append(sum(observed) / len(observed))
    vals.sort()
    def q(p): return vals[int(p * (len(vals)-1))] if vals else None
    return {"metric": "map equal-weight mean score delta; X=-0.12 minus X=-0.07",
        "mean_delta": mean(list(deltas.values())), "ci95_low": q(.025), "ci95_high": q(.975),
        "complete_map_clusters": sum(v is not None for v in deltas.values()), "bootstrap_n": BOOTSTRAP_N,
        "seed": BOOTSTRAP_SEED, "resampling": "paired map clusters, M1 30 and M2 6 stratified with replacement; repeats kept within map"}


def write_summary_markdown(root: Path, plan: dict, summaries: list[dict], map_rows: list[dict], delta: dict) -> None:
    lines = ["# T1 双退出阈值36张开发图批次摘要", "",
        "开发预实验，不是正式实验；不使用留出集，不据此宣布最优参数。每图每阈值两次；同图同重复用于配对设计，但初态不保证一致。", "",
        "分数按固定权重逐次计算后再按地图平均；碰撞、短反向频率等缺失保持 UNKNOWN，不能补零。有效记录不代表安全成功。", "",
        "| 数据集 | 地图数 | X | 有效运行 | ARRIVED | 终局计数 | 地图等权平均分（完整地图项） | 分数地图数 | 平均到达时间（地图等权） | 短反向/有效分钟 | Recovery占比 | 碰撞UNKNOWN | 跌倒UNKNOWN |", "|---|---:|---:|---:|---:|---|---:|---:|---:|---:|---:|---:|---:|"]
    for s in summaries:
        for x in XS:
            a = s["thresholds"][f"{x:.2f}"]
            lines.append(f"| {s['dataset']} | {s['maps']} | {x:.2f} | {a['valid_runs']}/{a['planned_runs']} | {a['arrived_runs']} | `{json.dumps(a['terminal_counts'], ensure_ascii=False)}` | {fmt(a['mean_score_map_equal_complete_only'])} | {a['maps_with_complete_score']} | {fmt(a['mean_arrival_time_map_equal'])} | {fmt(a['mean_short_reverse_map_equal'])} | {fmt(a['mean_recovery_share_map_equal'])} | {a['collision_unknown_runs']} | {a['fall_unknown_runs']} |")
    lines.extend(["", f"配对地图分数差（−0.12 减 −0.07）：正/负/相同 `{delta['positive_negative_same_map_counts']}`；完整分数地图数 `{delta['complete_score_map_n']}/36`；地图簇 bootstrap 95%区间 `{delta['bootstrap']['ci95_low']}` 至 `{delta['bootstrap']['ci95_high']}`。该区间为开发集描述性统计。", "",
        "结果按M1（30图）、M2（6图）、全36图分别列出。全36图按地图等权，保留M1/M2比例30/6。M2仅6个地图簇，区间较宽，不作强泛化。", "",
        "主要产物：`per_run_results_144.csv`、`attempt_ledger.csv`、`initial_state_and_coverage_144.csv`、`map_comparison_36.csv`、`threshold_summary_M1_M2_ALL.csv/.json`、`M1_density_structure_descriptive.csv`、`charts/`。逐图阈值对象包含终局、有效样本、ARRIVED、到达时间、短反向、Recovery占比、分数、安全UNKNOWN及站立收尾。", ""])
    (root / "RESULTS_SUMMARY.md").write_text("\n".join(lines), encoding="utf-8")


def fmt(value):
    return "UNKNOWN" if value is None else f"{value:.4f}" if isinstance(value, float) else str(value)


def make_plots(root: Path, rows: list[dict], map_rows: list[dict], plot_python: str) -> None:
    payload = {"rows": rows, "maps": map_rows}
    data = root / "plot_data.json"
    atomic_json(data, payload)
    charts = root / "charts"
    charts.mkdir(exist_ok=True)
    code = r'''import json,sys,math
from pathlib import Path
from collections import Counter
import numpy as np
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
p=Path(sys.argv[1]); d=json.load(open(p/'plot_data.json')); rows=d['rows']; maps=d['maps']; xs=[-0.07,-0.12]; labels=['X=-0.07','X=-0.12']; out=p/'charts'
valid=[r for r in rows if r.get('status')=='VALID_TASK_RESULT' and r.get('record_validity')=='VALID']
terms=sorted(set(r.get('terminal_result') or 'UNKNOWN' for r in valid)); fig,ax=plt.subplots(figsize=(8,5)); bottom=np.zeros(2)
for t in terms:
 vals=np.array([sum(r.get('terminal_result')==t and abs(float(r['exit_threshold_X'])-x)<1e-8 for r in valid) for x in xs]); ax.bar(labels,vals,bottom=bottom,label=t); bottom+=vals
ax.set(title='Valid run terminal outcomes (development set)',ylabel='runs');ax.legend(fontsize=8);fig.tight_layout();fig.savefig(out/'terminal_outcomes.png',dpi=140);plt.close(fig)
def arm_metric(x,key):
 vals=[]
 for r in maps:
  a=json.loads(r['x_'+format(x,'.2f')]); v=a.get(key)
  if isinstance(v,(int,float)) and math.isfinite(v): vals.append(v)
 return np.mean(vals) if vals else np.nan
score=[]; arrival=[]; arrival_time=[]; reverse=[]; rec=[]
for x in xs:
 score.append(arm_metric(x,'mean_score_complete_only'))
 arrival.append(arm_metric(x,'arrived_n')/2)
 vals=[r.get('elapsed_task_duration_s') for r in valid if r.get('terminal_result')=='ARRIVED' and abs(float(r['exit_threshold_X'])-x)<1e-8]
 vals=[v for v in vals if isinstance(v,(int,float)) and math.isfinite(v)]
 arrival_time.append(float(np.mean(vals)) if vals else np.nan)
 reverse.append(arm_metric(x,'mean_short_reverse_per_valid_min'))
 rec.append(arm_metric(x,'mean_recovery_share'))
for values,title,ylabel,name in [(score,'Score (complete map cells)','score','score_by_threshold.png'),(arrival,'Arrival fraction per planned item','ARRIVED / 2','arrival_fraction_by_threshold.png'),(arrival_time,'Arrival time among ARRIVED runs','seconds','arrival_time_by_threshold.png'),(reverse,'Short reverse switch frequency','per valid task minute','short_reverse_by_threshold.png'),(rec,'Recovery share','fraction of valid motion time','recovery_share_by_threshold.png')]:
 fig,ax=plt.subplots(figsize=(7,4));ax.bar(labels,values,color=['#3572A5','#D9822B']);ax.set(title=title,ylabel=ylabel);ax.grid(axis='y',alpha=.25);fig.tight_layout();fig.savefig(out/name,dpi=140);plt.close(fig)
diff=[]; labels2=[]
for r in maps:
 a=json.loads(r['x_-0.07']);b=json.loads(r['x_-0.12']);diff.append(b.get('mean_score_complete_only')-a.get('mean_score_complete_only') if isinstance(a.get('mean_score_complete_only'),(int,float)) and isinstance(b.get('mean_score_complete_only'),(int,float)) else np.nan);labels2.append(r['map'].replace('.xml',''))
fig,ax=plt.subplots(figsize=(12,5));colors=['#2c7' if v>0 else '#d55' if v<0 else '#999' for v in diff];ax.bar(range(len(diff)),diff,color=colors);ax.axhline(0,color='black',lw=.8);ax.set_xticks(range(len(labels2)),labels2,rotation=70);ax.set(title='Per-map score delta: X=-0.12 minus X=-0.07',ylabel='score points');fig.tight_layout();fig.savefig(out/'map_score_delta.png',dpi=140);plt.close(fig)
mat=np.full((36,2),np.nan)
for i,r in enumerate(maps):
 for j,x in enumerate(xs):
  a=json.loads(r['x_'+format(x,'.2f')]);mat[i,j]=a.get('arrived_n')/2
fig,ax=plt.subplots(figsize=(6,12));im=ax.imshow(mat,vmin=0,vmax=1,aspect='auto',cmap='viridis');ax.set_xticks(range(2),labels);ax.set_yticks(range(36),[r['map'].replace('.xml','') for r in maps],fontsize=6);ax.set(title='Arrival fraction by development map and threshold');fig.colorbar(im,ax=ax,label='ARRIVED / 2');fig.tight_layout();fig.savefig(out/'arrival_heatmap_36.png',dpi=140);plt.close(fig)
'''
    env = os.environ.copy()
    env["MPLCONFIGDIR"] = "/tmp/t1_two_thresholds_mpl"
    env["PYTHONNOUSERSITE"] = "1"
    Path(env["MPLCONFIGDIR"]).mkdir(parents=True, exist_ok=True)
    result = subprocess.run([plot_python, "-c", code, str(root)], cwd=ROOT, env=env, capture_output=True, text=True)
    if result.returncode:
        raise RuntimeError(f"plot generation failed: {result.stderr[-2000:]}")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--prepare-only", action="store_true")
    ap.add_argument("--batch-root", type=Path)
    ap.add_argument("--resume", action="store_true")
    ap.add_argument("--retry-sequence", type=int)
    ap.add_argument("--repair-reason")
    ap.add_argument("--display", default=os.environ.get("DISPLAY", ":0"))
    ap.add_argument("--xauthority", default=os.environ.get("XAUTHORITY", "/run/user/1000/gdm/Xauthority"))
    ap.add_argument("--plot-python", default="/home/lidio/anaconda3/bin/python3")
    ap.add_argument("--self-check", action="store_true")
    ap.add_argument("--summarize-only", action="store_true",
        help="regenerate metrics, comparisons, and plots from the frozen raw runs without launching a scene")
    ap.add_argument("--refresh-freeze", action="store_true",
        help="refresh binary/source/map freeze only while the saved plan has no attempts")
    args = ap.parse_args()
    catalog = load_catalog()
    if args.self_check:
        runs = make_schedule(catalog)
        validate_schedule(runs, catalog)
        assert len([r for r in runs if r["exit_threshold_X"] == -0.07]) == 72
        assert len([r for r in runs if r["exit_threshold_X"] == -0.12]) == 72
        # Verify dispatch-to-overlay mapping with the production parser and legal B schema.
        for x in XS:
            temp = Path("/tmp") / f"t1-36-overlay-selfcheck-{os.getpid()}-{abs(round(x*100))}"
            manual.prepare_overlay(SOURCE, temp, "B", E, x, 0.30)
            cfg = temp / "share/go2_description/config/abs/config.yaml"
            manual.set_overlay_goal(cfg, 7.5, 0.0)
            text = cfg.read_text(encoding="utf-8")
            if not overlay_configuration_matches(text, "B", E, x, 0.30, 7.5, 0.0) or "recovery_min_hold_s:" in text:
                raise RuntimeError(f"offline production config check failed for X={x}")
            shutil.rmtree(temp)
        assert current_related_processes() == []
        print("PASS: 144 unique rows, 72 per X, 36 maps × 2 repeats, adjacent pairs, 18/18 first-order balance, round-2 reversal, reproducible schedule, no hold, production B config parser accepts both overlays; retry gate requires explicit sequence and repair reason.")
        return 0
    if args.batch_root:
        if not args.resume:
            ap.error("existing batch root requires --resume")
        root = args.batch_root.resolve()
        plan = json.loads((root / PLAN_FILE).read_text(encoding="utf-8"))
        if args.summarize_only:
            if plan.get("status") not in {"RUNS_COMPLETE_PENDING_ANALYSIS", "ANALYSIS_FAILED_RAW_DATA_PRESERVED", "COMPLETE"}:
                raise RuntimeError("summarize-only requires all planned items to have reached the analysis boundary")
            if any(r.get("status") != "VALID_TASK_RESULT" for r in plan.get("runs", [])):
                raise RuntimeError("cannot create complete analysis for an incomplete batch")
            plan["analysis"] = analyze(root, plan, args.plot_python)
            plan["status"] = "COMPLETE"
            plan["analysis_completed_at"] = datetime.now().astimezone().isoformat(timespec="seconds")
            atomic_json(root / PLAN_FILE, plan)
            write_progress(root, plan)
            return 0
        if args.refresh_freeze:
            if any(item.get("attempts") for item in plan.get("runs", [])) or plan.get("status") != "FROZEN":
                raise RuntimeError("freeze may only refresh before the first attempt")
            plan["code_snapshot"] = code_snapshot(plan["maps"])
            plan["freeze_refreshed_at"] = datetime.now().astimezone().isoformat(timespec="seconds")
            atomic_json(root / PLAN_FILE, plan)
            print("Refreshed pre-run freeze; zero attempts exist.")
            return 0
        return run_batch(root, args)
    if args.resume or args.retry_sequence or args.repair_reason:
        ap.error("--resume and retry options require --batch-root")
    if not args.prepare_only:
        ap.error("freeze with --prepare-only first; launch requires the saved frozen batch")
    PARENT.mkdir(parents=True, exist_ok=True)
    root = PARENT / (datetime.now().strftime("%Y%m%d_%H%M%S") + "_" + str(os.getpid()))
    plan = prepare_plan(root, args.display, args.xauthority)
    validate_schedule(plan["runs"], plan["maps"])
    print(f"Frozen batch: {root} ; 144 items, 36 development maps, 72 per threshold; no scene started.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
