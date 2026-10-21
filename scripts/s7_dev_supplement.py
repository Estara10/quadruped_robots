"""Two-run development supplement on m02.xml (A then D).

This is deliberately separate from the frozen 240-item formal matrix.  It
reuses the formal runner's dispatch, ledger, recovery, safety classification,
and cleanup checks, but has its own plan, authorization schema, quota, output
directory, and data class.  It is not formal data and never touches holdout
maps.
"""
from __future__ import annotations

import json
import sys
from collections import Counter
from pathlib import Path
from typing import Any, Callable

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

import s7_formal_runner as runner  # noqa: E402
import s7_formal_analysis as analysis  # noqa: E402
from abs_scene import resolve_scene  # noqa: E402
from s1_run_diagnostic import resolve_map_manifest  # noqa: E402

PLAN_ID = "S7-DEV-SUPPLEMENT-M02-A-D-v1"
DEV_STATUS = "DEV_SUPPLEMENT_PLAN_FROZEN_NOT_FORMAL"
DATA_CLASS = "DEV_SUPPLEMENT_NOT_FORMAL"
CLASSIFICATION = "S7 dev supplement m02 A/D; NOT formal data; two-run preparation"
AUTHORIZATION_SCHEMA = "s7-dev-supplement-run-authorization-v1"
MAX_SCENE_STARTS = 2
MAX_SUBSTITUTIONS = 0
BATCH_BUDGET_GIB = 1.0
MIN_FREE_GIB = 15.0
SCHEDULE_SEED = 2026102101
OUTPUT_ROOT = ROOT / "logs/07_formal_preparation/dev_supplement_m02_AD"
PLAN_PATH = ROOT / "docs/thesis_project/designs/S7/dev_supplement/m02_AD_plan.json"
AUTHORIZATION_PATH = (ROOT / "docs/thesis_project/designs/S7/dev_supplement/"
                             "m02_AD_authorization.json")
M2_LAYOUTS = ROOT / "docs/thesis_project/designs/M2/selected_v2/layouts.json"


def _m02_layout() -> dict[str, Any]:
    library = json.loads(M2_LAYOUTS.read_text(encoding="utf-8"))
    matches = [item for item in library.get("scenes", [])
               if item.get("filename") == "m02.xml"]
    if len(matches) != 1:
        raise ValueError("m02.xml is missing or ambiguous in the M2 selected layouts")
    return matches[0]


def build_plan_records() -> list[dict[str, Any]]:
    spec = resolve_scene("m02.xml")
    layout = _m02_layout()
    if layout.get("scene_id") != spec.scene_id:
        raise ValueError("M2 layout scene_id differs from the production catalog")
    common = {
        "plan_item_id": None,
        "global_order": None,
        "block_order": 1,
        "display_name": "m02",
        "filename": spec.filename,
        "scene_id": spec.scene_id,
        "map_version": "m2-selected-development/2.1",
        "split": str(layout.get("split", "development_selection")),
        "density": str(layout.get("density", "challenge_candidate")),
        "route_structure": "mixed_offset_local_bypass",
        "repeat_block": 1,
        "williams_sequence_id": "DEV",
        "within_block_position": None,
        "group": None,
        "E": -0.05,
        "X": None,
        "H_s": None,
        "start_x_m": 0.0,
        "start_y_m": 0.0,
        "goal_x_m": 7.5,
        "goal_y_m": 0.0,
        "arrival_radius_m": 0.5,
        "sim_limit_s": 20.0,
        "wall_cap_s": 120.0,
        "standing_observation_sim_s": 2.0,
        "standing_wall_cap_s": 30.0,
        "mujoco_window": "hidden",
        "rviz_enabled": False,
        "schedule_seed": SCHEDULE_SEED,
    }
    records = []
    for order, group, plan_item_id, position in (
            (1, "A", "DEV-M02-A", 1), (2, "D", "DEV-M02-D", 2)):
        params = runner.GROUP_PARAMETERS[group]
        records.append(runner.normalize_record({
            **common,
            "plan_item_id": plan_item_id,
            "global_order": order,
            "within_block_position": position,
            "group": group,
            "E": params["E"], "X": params["X"], "H_s": params["H"],
        }))
    return records


def plan_metadata(records: list[dict[str, Any]]) -> dict[str, Any]:
    return {
        "plan_id": PLAN_ID,
        "status": DEV_STATUS,
        "authorized_to_run": False,
        "scope": "DEV_SUPPLEMENT",
        "data_class": DATA_CLASS,
        "not_formal_data": True,
        "holdout_prohibited": True,
        "max_scene_starts": MAX_SCENE_STARTS,
        "max_substitutions": MAX_SUBSTITUTIONS,
        "no_automatic_retry": True,
        "schedule_seed": SCHEDULE_SEED,
        "map_version": "m2-selected-development/2.1",
        "source": {
            "scene_catalog": "common/abs_scene_catalog.def",
            "m2_layouts": "docs/thesis_project/designs/M2/selected_v2/layouts.json",
            "selection": "m02.xml development map only; no holdout; A then D",
        },
        "plan_sha256": runner.plan_digest(records),
        "records": records,
    }


def write_plan_file(path: Path = PLAN_PATH) -> dict[str, Any]:
    records = build_plan_records()
    payload = plan_metadata(records)
    path.parent.mkdir(parents=True, exist_ok=True)
    runner.write_json_atomic(path, payload)
    return payload


def load_dev_plan(path: Path = PLAN_PATH) -> tuple[list[dict[str, Any]], dict[str, Any],
                                                   list[str]]:
    errors: list[str] = []
    if not path.is_file():
        return [], {}, [f"dev plan file missing: {path}"]
    meta = runner.read_json(path)
    records = [runner.normalize_record(row) for row in meta.get("records", [])]
    if meta.get("plan_sha256") != runner.plan_digest(records):
        errors.append("dev_plan_sha256_mismatch")
    return records, meta, errors


def validate_dev_plan(records: list[dict[str, Any]], meta: dict[str, Any]) -> dict[str, Any]:
    errors: list[str] = []
    if meta.get("plan_id") != PLAN_ID:
        errors.append(f"dev_plan_id_mismatch:{meta.get('plan_id')}")
    if meta.get("status") != DEV_STATUS:
        errors.append(f"dev_plan_status_mismatch:{meta.get('status')}")
    if meta.get("scope") != "DEV_SUPPLEMENT":
        errors.append("dev_plan_scope_mismatch")
    if meta.get("not_formal_data") is not True:
        errors.append("dev_plan_must_be_marked_not_formal")
    if meta.get("holdout_prohibited") is not True:
        errors.append("dev_plan_must_prohibit_holdout")
    if meta.get("authorized_to_run") is not False:
        errors.append("dev_plan_authorized_to_run_must_stay_false")
    if len(records) != MAX_SCENE_STARTS:
        errors.append(f"dev_plan_item_count:{len(records)}")
    if meta.get("plan_sha256") != runner.plan_digest(records):
        errors.append("dev_plan_sha256_mismatch")
    ordered = sorted(records, key=lambda item: int(item.get("global_order", 0)))
    expected = (("DEV-M02-A", "A", 1), ("DEV-M02-D", "D", 2))
    spec = None
    try:
        spec = resolve_scene("m02.xml")
        layout = _m02_layout()
        resolve_map_manifest(spec, 7.5, 0.0, 20.0, 120.0)
    except (OSError, ValueError, KeyError, TypeError) as error:
        errors.append(f"dev_scene_static_check_failed:{error}")
        layout = {}
    for row, (item_id, group, order) in zip(ordered, expected):
        if row.get("plan_item_id") != item_id or row.get("group") != group or \
                int(row.get("global_order", -1)) != order:
            errors.append(f"dev_plan_identity_mismatch:{row.get('plan_item_id')}:{row.get('group')}")
        if row.get("filename") != "m02.xml" or str(row.get("filename", "")).startswith("holdout"):
            errors.append(f"dev_plan_forbidden_scene:{row.get('filename')}")
        if spec is not None and (row.get("scene_id") != spec.scene_id or
                                 row.get("map_version") != "m2-selected-development/2.1"):
            errors.append(f"dev_plan_scene_binding_mismatch:{item_id}")
        params = runner.GROUP_PARAMETERS[group]
        for field, expected_value in (("E", params["E"]), ("X", params["X"]),
                                      ("H_s", params["H"])):
            if row.get(field) != expected_value:
                errors.append(f"dev_plan_parameter_mismatch:{item_id}:{field}")
        for field, expected_value in runner.REQUIRED_RUN_CONDITIONS.items():
            if row.get(field) != expected_value:
                errors.append(f"dev_plan_condition_mismatch:{item_id}:{field}")
        if row.get("start_x_m") != 0.0 or row.get("start_y_m") != 0.0:
            errors.append(f"dev_plan_start_mismatch:{item_id}")
        if row.get("split") == "held_out":
            errors.append(f"dev_plan_holdout_split:{item_id}")
    if layout and spec is not None:
        if (layout.get("goal_m") != [7.5, 0.0]
                or layout.get("field_bounds_m") != {"x": [-0.5, 8.0], "y": [-3.0, 3.0]}):
            errors.append("m02_goal_or_bounds_mismatch")
    return {
        "status": "PASS" if not errors else "REJECTED",
        "errors": sorted(set(errors)),
        "counts": {"plan_items": len(records), "groups": dict(Counter(
            row.get("group") for row in records)), "order": [
            row.get("plan_item_id") for row in ordered]},
        "scope": "two development runs on m02.xml; not formal data; no holdout",
    }


def dev_binding(records: list[dict[str, Any]], meta: dict[str, Any],
                plan_path: Path = PLAN_PATH) -> dict[str, Any]:
    return {
        "matrix_id": PLAN_ID,
        "matrix_status": DEV_STATUS,
        "plan_sha256": runner.plan_digest(records),
        "plan_file_sha256": runner.sha256_file(plan_path),
        "plan_items": len(records),
        "data_class": DATA_CLASS,
        "not_formal_data": True,
        "holdout_prohibited": True,
        "authorized_to_run": False,
    }


def dev_limits() -> dict[str, Any]:
    return {
        "base_scene_starts": MAX_SCENE_STARTS,
        "max_substitutions": MAX_SUBSTITUTIONS,
        "max_total_scene_starts": MAX_SCENE_STARTS,
        "batch_budget_gib": BATCH_BUDGET_GIB,
        "min_free_gib": MIN_FREE_GIB,
    }


def validate_dev_authorization(authorization: dict[str, Any], binding: dict[str, Any],
                               batch_id: str) -> dict[str, Any]:
    errors: list[str] = []
    if not isinstance(authorization, dict):
        return {"status": "REJECTED", "errors": ["missing_authorization"]}
    if authorization.get("fixture_only") is True:
        errors.append("fixture_authorization_rejected")
    if authorization.get("schema") != AUTHORIZATION_SCHEMA:
        errors.append("authorization_schema_mismatch")
    if authorization.get("authorized_to_run") is not True:
        errors.append("authorization_does_not_authorize_run")
    if authorization.get("scope") != "DEV_SUPPLEMENT":
        errors.append("authorization_scope_mismatch")
    if authorization.get("not_formal_data") is not True:
        errors.append("authorization_must_mark_not_formal_data")
    if authorization.get("holdout_prohibited") is not True:
        errors.append("authorization_must_prohibit_holdout")
    if authorization.get("no_automatic_retry") is not True:
        errors.append("authorization_must_forbid_automatic_retry")
    if authorization.get("intentional_motion_interruption") is not False:
        errors.append("authorization_must_forbid_intentional_interruption")
    if authorization.get("failure_injection") is not False:
        errors.append("authorization_must_forbid_failure_injection")
    if authorization.get("replacements"):
        errors.append("dev_authorization_must_not_include_replacements")
    for field in ("authorization_id", "authorized_by", "authorization_basis"):
        if not str(authorization.get(field, "")).strip():
            errors.append(f"authorization_{field}_missing")
    review = authorization.get("review")
    if not isinstance(review, dict) or not str(review.get("reference", "")).strip() \
            or not str(review.get("date", "")).strip():
        errors.append("authorization_review_missing")
    if authorization.get("batch_id") != batch_id:
        errors.append("authorization_batch_id_mismatch")
    if authorization.get("plan_id") != PLAN_ID:
        errors.append("authorization_plan_id_mismatch")
    if authorization.get("plan_sha256") != binding.get("plan_sha256"):
        errors.append("authorization_plan_sha256_mismatch")
    if authorization.get("max_scene_starts") != MAX_SCENE_STARTS:
        errors.append("authorization_scene_start_quota_mismatch")
    if authorization.get("max_substitutions") != MAX_SUBSTITUTIONS:
        errors.append("authorization_substitution_quota_mismatch")
    if authorization.get("batch_budget_gib") != BATCH_BUDGET_GIB:
        errors.append("authorization_batch_budget_mismatch")
    if authorization.get("min_free_gib") != MIN_FREE_GIB:
        errors.append("authorization_free_space_guard_mismatch")
    return {
        "status": "PASS" if not errors else "REJECTED",
        "errors": sorted(set(errors)),
        "authorization_id": authorization.get("authorization_id"),
        "data_class": DATA_CLASS,
    }


def batch_root_allowed(batch_root: Path) -> bool:
    try:
        batch_root.resolve().relative_to((ROOT / "logs/07_formal_preparation").resolve())
    except ValueError:
        return False
    return True


def run_dev_batch(*, batch_root: Path, batch_id: str, records: list[dict[str, Any]],
                  meta: dict[str, Any], authorization: dict[str, Any],
                  authorization_path: Path, executor: Callable[[list[str], Path], dict[str, Any]],
                  analyze_run: Callable[[Path], dict[str, Any]] | None,
                  resume: bool, stop_after_scene_starts: int = 1,
                  disk_probe: Callable[[], float] = runner.free_gib_default,
                  overlay_preparer: Callable[[Path], dict[str, Any]] | None = None,
                  display: str = ":0", xauthority: str | None = None,
                  plan_path: Path = PLAN_PATH) -> dict[str, Any]:
    if not batch_root_allowed(batch_root):
        return {"status": "REFUSED", "reason": "dev_batch_root_outside_logs_07",
                "batch_root": str(batch_root), "dispatched": 0}
    binding = dev_binding(records, meta, plan_path)
    return runner.run_batch(
        batch_root=batch_root, batch_id=batch_id, plan=records, meta=meta,
        binding=binding, authorization=authorization,
        authorization_path=authorization_path, executor=executor,
        analyze_run=analyze_run, disk_probe=disk_probe, allow_fixture=False,
        resume=resume, require_full_plan=False,
        overlay_preparer=overlay_preparer,
        plan_validator=validate_dev_plan,
        authorization_validator=validate_dev_authorization,
        max_substitutions=MAX_SUBSTITUTIONS, max_per_plan_item=1,
        scene_start_cap=MAX_SCENE_STARTS,
        stop_after_scene_starts=stop_after_scene_starts,
        allow_pre_scene_retry=False,
        limits=dev_limits(), data_class=DATA_CLASS, classification=CLASSIFICATION,
        display=display, xauthority=xauthority)


def analyze_dev_batch(batch_root: Path, *, output_dir: Path | None = None,
                      reverse_window_s: float = 0.5,
                      plan_path: Path = PLAN_PATH) -> dict[str, Any]:
    records, meta, load_errors = load_dev_plan(plan_path)
    if load_errors:
        return {"status": "REFUSED", "reason": "dev_plan_load_errors",
                "errors": load_errors}
    validation = validate_dev_plan(records, meta)
    if validation["status"] != "PASS":
        return {"status": "REFUSED", "reason": "dev_plan_validation_failed",
                "validation": validation}
    ledger_path = batch_root / "ledger" / "attempts.json"
    if not ledger_path.is_file():
        return {"status": "REFUSED", "reason": "ledger_missing",
                "path": str(ledger_path)}
    ledger = runner.read_json(ledger_path)
    audit = runner.audit_ledger(records, ledger.get("attempts", []),
                                max_substitutions=MAX_SUBSTITUTIONS,
                                max_per_plan_item=1)
    output_dir = (output_dir or batch_root / "analysis").resolve()
    summaries, provenance = analysis.recompute_scene_start_summaries(
        records, audit, output_dir=output_dir, reverse_window_s=reverse_window_s,
        decision_clock_override=True)
    reconstruction_sensitivity = analysis.reconstruction_sensitivity(
        records, audit, summaries, output_dir=output_dir,
        reverse_window_s=reverse_window_s)
    views = analysis.build_views(records, audit, summaries)
    plan_by_id = {item["plan_item_id"]: item for item in records}
    consistency = []
    issues: list[str] = []
    for row in audit.get("attempt_facts", []):
        if row.get("scene_created") is not True:
            continue
        plan_item = plan_by_id.get(row.get("plan_item_id"), {})
        run_dir = Path(row["run_directory"]) if row.get("run_directory") else None
        run_binding_ok = bool(
            run_dir is not None and row.get("run_id") == run_dir.name
            and row.get("output_directory")
            and str(run_dir).startswith(str(row["output_directory"])))
        consumption = row.get("runtime_config_consumption") or {}
        consumption_ok = (consumption.get("status") == "CONFIRMED"
                          and not consumption.get("errors"))
        process = row.get("process_facts") or {}
        cleanup_ok = process.get("shutdown_complete") is True
        raw_safety = row.get("safety_task_completion", "UNKNOWN")
        metrics = analysis.extract_metrics(summaries.get(row.get("attempt_id")))
        unified_safety = metrics.get("safe_complete") if metrics else None
        safety = unified_safety if unified_safety is not None else raw_safety
        record_val_ok = row.get("record_validity") == "VALID"
        terminal_ok = not row.get("terminal_conflict", False)
        entered_rl_ok = row.get("entered_rl") is True
        terminal_present = isinstance(row.get("raw_terminal"), str) and bool(row.get("raw_terminal"))
        item = {
            "plan_item_id": row.get("plan_item_id"), "group": row.get("group"),
            "run_id": row.get("run_id"), "run_directory": str(run_dir) if run_dir else None,
            "scene_created": True, "entered_rl": row.get("entered_rl"),
            "raw_terminal": row.get("raw_terminal"),
            "record_validity": row.get("record_validity"),
            "raw_safety_task_completion": raw_safety,
            "unified_safety_task_completion": unified_safety,
            "safety_task_completion": safety,
            "reconstruction_applied": bool(
                (summaries.get(row.get("attempt_id")) or {}).get(
                    "timing_clock_override_applied")),
            "runtime_config_consumption_status": consumption.get("status"),
            "runtime_config_errors": consumption.get("errors"),
            "scene_run_binding_ok": run_binding_ok,
            "config_consumption_ok": consumption_ok,
            "record_validity_ok": record_val_ok,
            "terminal_sources_consistent": terminal_ok,
            "entered_rl_ok": entered_rl_ok,
            "terminal_present": terminal_present,
            "cleanup_confirmed": cleanup_ok,
            "metrics": metrics,
        }
        consistency.append(item)
        for name, ok in (("scene_run_binding", run_binding_ok),
                         ("config_consumption", consumption_ok),
                         ("record_validity", record_val_ok),
                         ("terminal_sources", terminal_ok),
                         ("entered_rl", entered_rl_ok),
                         ("terminal_present", terminal_present),
                         ("cleanup", cleanup_ok)):
            if not ok:
                issues.append(f"{row.get('plan_item_id')}:{name}")
    a_starts = sum(1 for row in consistency if row["group"] == "A")
    d_starts = sum(1 for row in consistency if row["group"] == "D")
    if a_starts != 1:
        issues.append(f"A_scene_start_count:{a_starts}")
    if d_starts > 1:
        issues.append(f"D_scene_start_count:{d_starts}")
    if len(consistency) < MAX_SCENE_STARTS:
        issues.append(f"supplement_incomplete_scene_starts:{len(consistency)}")
    if len(consistency) > MAX_SCENE_STARTS:
        issues.append(f"total_scene_start_count:{len(consistency)}")
    a_indexes = [index for index, row in enumerate(consistency) if row["group"] == "A"]
    d_indexes = [index for index, row in enumerate(consistency) if row["group"] == "D"]
    a_before_d = (not d_indexes or (a_indexes and max(a_indexes) < min(d_indexes)))
    if not a_before_d:
        issues.append("D_scene_start_not_after_A")
    if audit.get("status") != "PASS":
        issues.append(f"ledger_audit:{audit.get('status')}")
    manifest = runner.read_json(batch_root / "batch_manifest.json") \
        if (batch_root / "batch_manifest.json").is_file() else {}
    if manifest.get("data_class") != DATA_CLASS:
        issues.append("manifest_data_class_mismatch")
    report = {
        "schema": "s7-dev-supplement-analysis-v1",
        "data_class": "NON_FORMAL_DEV_SUPPLEMENT_DATA",
        "not_formal_data": True,
        "holdout_used": False,
        "batch_root": str(batch_root),
        "plan_id": PLAN_ID,
        "plan_sha256": runner.plan_digest(records),
        "manifest": manifest,
        "audit_status": audit.get("status"),
        "audit_errors": audit.get("errors"),
        "audit_counts": audit.get("counts"),
        "views": views,
        "attempt_consistency": consistency,
        "consistency_issues": sorted(set(issues)),
        "consistency_status": "PASS" if not issues else "REVIEW",
        "A_before_D": a_before_d,
        "A_scene_starts": a_starts,
        "D_scene_starts": d_starts,
        "provenance": provenance,
        "reconstruction_sensitivity": reconstruction_sensitivity,
        "scope": "two development runs on m02.xml; A then D; no holdout; "
                 "no formal comparison, success-rate, or inference claim",
    }
    output_dir.mkdir(parents=True, exist_ok=True)
    runner.write_json_atomic(output_dir / "dev_supplement_analysis.json", report)
    lines = [
        "# S7 dev supplement m02 A/D analysis",
        "",
        "**NON_FORMAL_DEV_SUPPLEMENT_DATA**: two development runs, not formal data.",
        "",
        f"- audit status: {audit.get('status')}",
        f"- actual scene starts: {audit.get('counts', {}).get('actual_scene_starts')}",
        f"- A scene starts: {a_starts}; D scene starts: {d_starts}",
        f"- consistency: {report['consistency_status']}",
        f"- issues: {report['consistency_issues'] or 'none'}",
        f"- unified primary outcomes: {report['views']['primary_plan_item']['outcome_counts']}",
        f"- raw publication-state outcomes: "
        f"{report['views']['primary_plan_item'].get('raw_default_outcome_counts')}",
        f"- decision-clock reconstruction: "
        f"{reconstruction_sensitivity.get('reconstruction_runs')} runs; "
        f"safety changed {reconstruction_sensitivity.get('safety_changed_runs')}; "
        f"timing changed {reconstruction_sensitivity.get('timing_changed_runs')}",
        "",
        "A and D are mechanism checks. Legal collision/timeout is a result, not a failure "
        "of the supplement. No success-rate or group comparison is inferred from two runs.",
    ]
    (output_dir / "DEV_SUPPLEMENT_ANALYSIS.md").write_text("\n".join(lines) + "\n",
                                                           encoding="utf-8")
    return report
