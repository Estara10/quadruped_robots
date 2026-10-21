"""Frozen S7 formal 240-item summaries: three views, map-equal comparisons.

This module only consumes the production analyzer's output (or an explicitly
constructed fixture).  It does not detect safety events and does not replace
``scripts/s1_analyze_run.py`` as the frozen analyzer.
"""
from __future__ import annotations

import csv
import hashlib
import json
import random
import statistics
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any, Callable, Iterable

ROOT = Path(__file__).resolve().parents[1]
ANALYSIS_SCHEMA = "s7-formal-analysis-v1"
BOOTSTRAP_SEED = 2026102002
BOOTSTRAP_REPLICATES = 10000

LEGAL_TERMINALS = {
    "ARRIVED", "COLLISION_TERMINATION", "FALL_TERMINATION",
    "DIAGNOSTIC_SIM_TIME_LIMIT", "WALL_CLOCK_GUARD_INCOMPLETE",
    "TIMEOUT", "OPERATOR_ABORT",
}
IN_BOUNDS_STATUSES = {
    "BASE_LINK_IN_BOUNDS_PHYSICS_STEPS_OBSERVED",
    "NO_BASE_LINK_OUTSIDE_STEP_AFTER_TASK_START_OBSERVED",
}
OUT_OF_BOUNDS_STATUS = "BASE_LINK_OUT_OF_BOUNDS_PHYSICS_STEP_OBSERVED"
UNKNOWN_PLAN_CATEGORIES = (
    "SAFETY_UNKNOWN", "UNRESOLVED_ENGINEERING", "UNRESOLVED_SCENE_ATTEMPT",
    "UNRESOLVED", "NOT_STARTED",
)

METRIC_SPECS: tuple[dict[str, Any], ...] = (
    {"key": "safe_complete", "label": "已确认安全任务完成", "kind": "rate",
     "direction": "higher_better", "unit": "proportion"},
    {"key": "arrived", "label": "ARRIVED", "kind": "rate",
     "direction": "higher_better", "unit": "proportion"},
    {"key": "collision", "label": "非足端障碍碰撞", "kind": "rate",
     "direction": "lower_better", "unit": "proportion"},
    {"key": "fall", "label": "跌倒", "kind": "rate",
     "direction": "lower_better", "unit": "proportion"},
    {"key": "timeout", "label": "仿真限时", "kind": "rate",
     "direction": "lower_better", "unit": "proportion"},
    {"key": "out_of_bounds", "label": "base_link越界", "kind": "rate",
     "direction": "lower_better", "unit": "proportion"},
    {"key": "foot_impact_any", "label": "存在足端撞击", "kind": "rate",
     "direction": "none", "unit": "proportion"},
    {"key": "recovery_share", "label": "Recovery有效时间占比", "kind": "continuous",
     "direction": "tradeoff", "unit": "fraction"},
    {"key": "recovery_residence_median_s", "label": "Recovery单次驻留中位数",
     "kind": "continuous", "direction": "tradeoff", "unit": "s"},
    {"key": "recovery_reentry_within_0_5s_count",
     "label": "退出后0.5s内重入次数", "kind": "continuous",
     "direction": "tradeoff", "unit": "count_per_run"},
    {"key": "short_reverse_per_min", "label": "短反向切换频率",
     "kind": "continuous", "direction": "lower_better", "unit": "per_valid_minute"},
    {"key": "elapsed_task_duration_s", "label": "任务耗时", "kind": "continuous",
     "direction": "tradeoff", "unit": "s"},
    {"key": "target_progress_m", "label": "目标推进", "kind": "continuous",
     "direction": "higher_better", "unit": "m"},
    {"key": "foot_contact_count", "label": "足端接触次数", "kind": "continuous",
     "direction": "none", "unit": "count"},
    {"key": "foot_impact_count", "label": "足端撞击次数", "kind": "continuous",
     "direction": "none", "unit": "count"},
)
COMPARISONS = (
    ("B-A", "B", "A"), ("C-A", "C", "A"), ("D-B", "D", "B"), ("D-C", "D", "C"),
)
INTERACTION_KEY = "(D-C)-(B-A)"


# ---------------------------------------------------------------------------
# Production-analyzer output extraction
# ---------------------------------------------------------------------------

def _finite(value: Any) -> float | None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    number = float(value)
    return number if number == number and abs(number) != float("inf") else None


def _tri(value: Any) -> bool | None:
    return value if isinstance(value, bool) else None


def _navigation_fact(terminal: Any) -> bool | None:
    if terminal == "ARRIVED":
        return True
    if terminal in LEGAL_TERMINALS:
        return False
    return None


def _timeout_fact(terminal: Any) -> bool | None:
    if terminal == "DIAGNOSTIC_SIM_TIME_LIMIT":
        return True
    if terminal in LEGAL_TERMINALS:
        return False
    return None


def _recovery_reentry(transitions: list[dict[str, Any]], window_s: float = 0.5) -> int | None:
    edges: list[tuple[float, str]] = []
    for edge in transitions:
        if edge.get("sim_clock_valid") is not True:
            return None
        time_s = _finite(edge.get("sim_time_s"))
        if time_s is None:
            return None
        edges.append((time_s, f"{edge.get('mode_before')}->{edge.get('mode_after')}"))
    edges.sort()
    count = 0
    exit_time: float | None = None
    for time_s, name in edges:
        if name == "1->0":
            exit_time = time_s
        elif name == "0->1" and exit_time is not None:
            if 0.0 <= time_s - exit_time <= window_s:
                count += 1
            exit_time = None
    return count


def extract_metrics(summary: dict[str, Any] | None) -> dict[str, Any]:
    """Map one production summary to the frozen reporting fields."""
    summary = summary or {}
    terminal = summary.get("navigation_terminal_result", summary.get("terminal_result"))
    safety = summary.get("task_window_safety") or summary.get("motion_safety") or {}
    bounds = summary.get("declared_boundary_physics_observation") or {}
    task = summary.get("rl_task_metric_boundary") or {}
    composite = summary.get("safety_task_completion") or {}

    collision = _tri(safety.get("nonfoot_collision_failure"))
    fall = _tri(safety.get("fall"))
    boundary_status = bounds.get("status")
    if boundary_status in IN_BOUNDS_STATUSES:
        out_of_bounds: bool | None = False
    elif boundary_status == OUT_OF_BOUNDS_STATUS:
        out_of_bounds = True
    else:
        out_of_bounds = None
    foot_impact = _finite(summary.get("foot_impact_count"))
    if foot_impact is None and isinstance(summary.get("foot_impact_count"), int):
        foot_impact = float(summary["foot_impact_count"])
    if isinstance(summary.get("foot_impact_count"), bool):
        foot_impact = None
    foot_impact_int = (int(summary["foot_impact_count"])
                       if isinstance(summary.get("foot_impact_count"), int)
                       and not isinstance(summary.get("foot_impact_count"), bool) else None)

    intervals = [item for item in summary.get("recovery_intervals") or []
                 if isinstance(item, dict)]
    completed_durations = [_finite(item.get("duration_s")) for item in intervals
                           if item.get("censored") is False]
    completed_durations = [value for value in completed_durations if value is not None]
    residence_median = statistics.median(completed_durations) if completed_durations else None

    short_count = summary.get("short_reverse_switch_count")
    valid_s = _finite(task.get("valid_continuous_sim_s"))
    if valid_s is None:
        valid_s = _finite(summary.get("valid_motion_duration_s"))
    frequency = None
    if (isinstance(short_count, int) and not isinstance(short_count, bool)
            and summary.get("short_reverse_switch_coverage") == "complete"
            and valid_s is not None and valid_s > 0):
        frequency = short_count / (valid_s / 60.0)

    start_distance = _finite(summary.get("target_distance_start_m"))
    final_distance = _finite(summary.get("target_distance_final_m"))
    progress = (start_distance - final_distance
                if start_distance is not None and final_distance is not None else None)
    elapsed = _finite(task.get("elapsed_sim_s"))
    if elapsed is None:
        elapsed = _finite(summary.get("elapsed_task_duration_s"))

    closeout = (composite.get("standing_and_cleanup") or {}) if isinstance(composite, dict) else {}
    shutdown_complete = closeout.get("shutdown_complete")
    exit_code = closeout.get("process_exit_code")
    if shutdown_complete is True and exit_code == 0:
        cleanup_complete: bool | None = True
    elif shutdown_complete is False or (isinstance(exit_code, int) and exit_code != 0):
        cleanup_complete = False
    else:
        cleanup_complete = None
    standing = closeout.get("standing_observation")
    standing_complete: bool | None = None
    if isinstance(standing, dict):
        standing_complete = standing.get("complete") if isinstance(standing.get("complete"), bool) else None

    return {
        "safe_complete": composite.get("status") if isinstance(composite.get("status"), bool) else None,
        "arrived": _navigation_fact(terminal),
        "collision": collision,
        "fall": fall,
        "timeout": _timeout_fact(terminal),
        "out_of_bounds": out_of_bounds,
        "foot_impact_any": (foot_impact_int > 0 if foot_impact_int is not None else None),
        "foot_impact_count": float(foot_impact_int) if foot_impact_int is not None else None,
        "foot_contact_count": (float(summary["foot_contact_count"])
                               if isinstance(summary.get("foot_contact_count"), int)
                               and not isinstance(summary.get("foot_contact_count"), bool) else None),
        "recovery_share": _finite(summary.get("recovery_share_valid_motion_time")),
        "recovery_residence_median_s": residence_median,
        "recovery_completed_interval_count": len(completed_durations),
        "recovery_censored_interval_count": sum(1 for item in intervals if item.get("censored") is True),
        "recovery_reentry_within_0_5s_count": _recovery_reentry(summary.get("transitions") or []),
        "short_reverse_per_min": frequency,
        "elapsed_task_duration_s": elapsed,
        "target_progress_m": progress,
        "cleanup_complete": cleanup_complete,
        "standing_observation_complete": standing_complete,
        "navigation_terminal_result": terminal,
        "record_validity": summary.get("record_validity", "UNKNOWN"),
    }


# ---------------------------------------------------------------------------
# Three views
# ---------------------------------------------------------------------------

def _summary_safety_value(summary: dict[str, Any] | None) -> bool | str | None:
    if not isinstance(summary, dict):
        return None
    value = summary.get("safety_task_completion_status")
    return value if value in (True, False, "UNKNOWN") else None


def build_views(plan: list[dict[str, Any]], audit: dict[str, Any],
                summary_by_attempt: dict[str, dict[str, Any]]) -> dict[str, Any]:
    plan_by_id = {item["plan_item_id"]: item for item in plan}
    primary_rows = []
    for row in audit.get("plan_items", []):
        plan_item = plan_by_id.get(row["plan_item_id"], {})
        attempt_id = row.get("selected_attempt_id")
        selected_summary = summary_by_attempt.get(attempt_id) if attempt_id else None
        metrics = extract_metrics(selected_summary) if attempt_id else None
        plan_status = row.get("plan_status")
        raw_safety = row.get("selected_safety_task_completion")
        unified_safety = _summary_safety_value(selected_summary)
        outcome = plan_status
        if plan_status == "SELECTED_RESULT":
            value = unified_safety if unified_safety is not None else raw_safety
            outcome = ("CONFIRMED_SAFE_COMPLETE" if value is True else
                       "KNOWN_NOT_COMPLETE" if value is False else "SAFETY_UNKNOWN")
        elif plan_status == "KNOWN_SAFETY_FAILURE_RECORD_INCOMPLETE":
            outcome = "KNOWN_NOT_COMPLETE"
        if metrics is not None:
            if outcome == "CONFIRMED_SAFE_COMPLETE":
                metrics["safe_complete"] = True
            elif outcome == "KNOWN_NOT_COMPLETE":
                # A confirmed task-window safety failure stays failure even if
                # the record is invalid; it must not enter a potential-success
                # upper bound as an UNKNOWN.
                metrics["safe_complete"] = False
            elif outcome in UNKNOWN_PLAN_CATEGORIES:
                metrics["safe_complete"] = None
        primary_rows.append({
            "plan_item_id": row["plan_item_id"],
            "group": row.get("group"),
            "map": plan_item.get("display_name"),
            "repeat_block": plan_item.get("repeat_block"),
            "global_order": plan_item.get("global_order"),
            "plan_status": plan_status,
            "outcome": outcome,
            "selected_attempt_id": attempt_id,
            "selected_run_id": row.get("selected_run_id"),
            "selected_terminal": row.get("selected_terminal"),
            "raw_selected_safety_task_completion": raw_safety,
            "unified_safety_task_completion": unified_safety,
            "metrics": metrics,
        })

    per_group_outcomes: dict[str, Counter[str]] = defaultdict(Counter)
    for row in primary_rows:
        per_group_outcomes[row["group"]][row["outcome"]] += 1
    group_bounds = {}
    for group in sorted(per_group_outcomes):
        counts = per_group_outcomes[group]
        denominator = sum(counts.values())
        confirmed = counts["CONFIRMED_SAFE_COMPLETE"]
        unknown = sum(counts[key] for key in UNKNOWN_PLAN_CATEGORIES)
        group_bounds[group] = {
            "plan_items": denominator,
            "confirmed_safe_complete": confirmed,
            "known_not_complete": counts["KNOWN_NOT_COMPLETE"],
            "safety_unknown": counts["SAFETY_UNKNOWN"],
            "unresolved_engineering": counts["UNRESOLVED_ENGINEERING"],
            "unresolved_scene_attempt": counts["UNRESOLVED_SCENE_ATTEMPT"],
            "unresolved": counts["UNRESOLVED"],
            "not_started": counts["NOT_STARTED"],
            "safe_completion_lower_over_all_plan_items": confirmed / denominator if denominator else None,
            "safe_completion_upper_over_all_plan_items":
                (confirmed + unknown) / denominator if denominator else None,
            "all_outcome_counts": dict(sorted(counts.items())),
        }

    all_start_by_group: dict[str, Counter[str]] = defaultdict(Counter)
    for fact in audit.get("attempt_facts", []):
        if fact.get("scene_created") is True:
            all_start_by_group[fact.get("group")][fact["classification"]["status"]] += 1

    first_rows = []
    for row in audit.get("plan_items", []):
        plan_item = plan_by_id.get(row["plan_item_id"], {})
        first_id = row.get("first_scene_attempt_id")
        first_summary = summary_by_attempt.get(first_id) if first_id else None
        metrics = extract_metrics(first_summary) if first_id else None
        raw_first_safety = row.get("first_scene_safety_task_completion", "UNKNOWN")
        unified_first_safety = _summary_safety_value(first_summary)
        if first_id is None:
            outcome = "NOT_STARTED"
        elif row.get("first_scene_navigation_result") == "NOT_STARTED":
            outcome = "NOT_STARTED"
        elif row.get("confirmed_task_safety_failure_present") is True:
            outcome = "KNOWN_NOT_COMPLETE"
        else:
            value = (unified_first_safety if unified_first_safety is not None
                     else raw_first_safety)
            outcome = ("CONFIRMED_SAFE_COMPLETE" if value is True else
                       "KNOWN_NOT_COMPLETE" if value is False else "SAFETY_UNKNOWN")
        first_rows.append({
            "plan_item_id": row["plan_item_id"],
            "group": row.get("group"),
            "map": plan_item.get("display_name"),
            "repeat_block": plan_item.get("repeat_block"),
            "global_order": plan_item.get("global_order"),
            "first_scene_attempt_id": first_id,
            "first_scene_terminal": row.get("first_scene_terminal"),
            "first_scene_record_validity": row.get("first_scene_record_validity"),
            "first_scene_navigation_result": row.get("first_scene_navigation_result"),
            "raw_first_scene_safety_task_completion": raw_first_safety,
            "unified_first_scene_safety_task_completion": unified_first_safety,
            "outcome": outcome,
            "metrics": metrics,
        })
    first_counts: Counter[str] = Counter(item["outcome"] for item in first_rows)
    return {
        "primary_plan_item": {
            "rows": primary_rows,
            "outcome_counts": dict(sorted(Counter(
                item["outcome"] for item in primary_rows).items())),
            "raw_default_outcome_counts": audit.get("plan_item_outcomes", {}),
            "per_group": group_bounds,
            "selection_rule": "one pre-specified result per plan item, independent of group/outcome",
        },
        "all_actual_scene_starts": {
            "counts": audit.get("counts", {}),
            "class_counts": audit.get("all_actual_scene_start_class_counts", {}),
            "invocation_class_counts": audit.get("all_invocation_class_counts", {}),
            "per_group_class_counts": {key: dict(sorted(value.items()))
                                       for key, value in sorted(all_start_by_group.items())},
            "note": "all starts retained, including replaced first attempts and safety aborts",
        },
        "first_actual_scene_only_sensitivity": {
            **audit.get("first_actual_scene_only_sensitivity", {}),
            "rows": first_rows,
            "outcome_counts": dict(sorted(first_counts.items())),
            "note": "sensitivity only; first scene start is never replaced by a later substitute",
        },
    }


# ---------------------------------------------------------------------------
# Map-equal aggregation and bootstrap
# ---------------------------------------------------------------------------

def _map_group_values(primary_rows: list[dict[str, Any]], spec: dict[str, Any]
                      ) -> dict[str, dict[str, dict[str, Any]]]:
    """map -> group -> {lower/value, upper, n_plan, n_known, n_unknown}."""
    buckets: dict[tuple[str, str], list[dict[str, Any]]] = defaultdict(list)
    for row in primary_rows:
        buckets[(str(row["map"]), str(row["group"]))].append(row)
    result: dict[str, dict[str, dict[str, Any]]] = defaultdict(dict)
    for (map_name, group), rows in buckets.items():
        if spec["kind"] == "rate":
            confirmed = 0
            known_false = 0
            unknown = 0
            for row in rows:
                if spec["key"] == "safe_complete":
                    outcome = row.get("outcome")
                    if outcome == "CONFIRMED_SAFE_COMPLETE":
                        confirmed += 1
                    elif outcome == "KNOWN_NOT_COMPLETE":
                        known_false += 1
                    else:
                        unknown += 1
                    continue
                value = (row.get("metrics") or {}).get(spec["key"])
                if value is True:
                    confirmed += 1
                elif value is False:
                    known_false += 1
                else:
                    unknown += 1
            n = len(rows)
            result[map_name][group] = {
                "value": confirmed / n if n else None,
                "lower": confirmed / n if n else None,
                "upper": (confirmed + unknown) / n if n else None,
                "n_plan": n, "n_known_true": confirmed,
                "n_known_false": known_false, "n_unknown": unknown,
            }
        else:
            values = []
            unknown = 0
            for row in rows:
                value = (row.get("metrics") or {}).get(spec["key"])
                if isinstance(value, (int, float)) and not isinstance(value, bool):
                    values.append(float(value))
                else:
                    unknown += 1
            result[map_name][group] = {
                "value": statistics.fmean(values) if values else None,
                "lower": None, "upper": None,
                "n_plan": len(rows), "n_known": len(values), "n_unknown": unknown,
                "known_values": values,
            }
    return result


def _group_summary(map_values: dict[str, dict[str, Any]], spec: dict[str, Any]
                   ) -> dict[str, Any]:
    if spec["kind"] == "rate":
        maps = [value for value in map_values.values() if value.get("lower") is not None]
        lower = statistics.fmean(value["lower"] for value in maps) if maps else None
        upper = statistics.fmean(value["upper"] for value in maps) if maps else None
        return {
            "map_equal_value": lower, "map_equal_lower": lower, "map_equal_upper": upper,
            "map_equal_percentage_point": lower * 100.0 if lower is not None else None,
            "map_equal_lower_percentage_point": lower * 100.0 if lower is not None else None,
            "map_equal_upper_percentage_point": upper * 100.0 if upper is not None else None,
            "n_maps": len(maps),
            "n_plan_items": sum(value["n_plan"] for value in maps),
            "n_known_true": sum(value["n_known_true"] for value in maps),
            "n_known_false": sum(value["n_known_false"] for value in maps),
            "n_unknown": sum(value["n_unknown"] for value in maps),
        }
    maps = [value for value in map_values.values() if value.get("value") is not None]
    return {
        "map_equal_value": statistics.fmean(value["value"] for value in maps) if maps else None,
        "n_maps_known": len(maps),
        "n_maps_total": len(map_values),
        "n_values_known": sum(value.get("n_known", 0) for value in maps),
        "n_unknown": sum(value.get("n_unknown", 0) for value in map_values.values()),
        "n_plan_items": sum(value.get("n_plan", 0) for value in map_values.values()),
    }


def _difference(values: dict[str, dict[str, Any]], candidate: str, reference: str,
                maps: Iterable[str]) -> float | None:
    differences = []
    for map_name in maps:
        candidate_value = values.get(map_name, {}).get(candidate, {}).get("value")
        reference_value = values.get(map_name, {}).get(reference, {}).get("value")
        if candidate_value is not None and reference_value is not None:
            differences.append(candidate_value - reference_value)
    return statistics.fmean(differences) if differences else None


def _bootstrap_difference_from_samples(values: dict[str, dict[str, Any]], candidate: str,
                                       reference: str,
                                       samples: list[list[str]]) -> dict[str, Any]:
    draws: list[float] = []
    for sample in samples:
        value = _difference(values, candidate, reference, sample)
        if value is not None:
            draws.append(value)
    if not draws:
        return {"replicates_requested": len(samples), "replicates_used": 0,
                "lower": None, "upper": None}
    draws.sort()
    return {
        "replicates_requested": len(samples), "replicates_used": len(draws),
        "lower": draws[int(0.025 * (len(draws) - 1))],
        "upper": draws[int(0.975 * (len(draws) - 1))],
    }


def _bootstrap_interaction_from_samples(values: dict[str, dict[str, Any]],
                                        map_names: list[str],
                                        samples: list[list[str]]) -> dict[str, Any]:
    map_interaction: dict[str, float] = {}
    for map_name in map_names:
        row = values.get(map_name, {})
        d = row.get("D", {}).get("value")
        c = row.get("C", {}).get("value")
        b = row.get("B", {}).get("value")
        a = row.get("A", {}).get("value")
        if None not in (d, c, b, a):
            map_interaction[map_name] = (d - c) - (b - a)
    point = statistics.fmean(map_interaction.values()) if map_interaction else None
    draws: list[float] = []
    for sample in samples:
        usable = [name for name in sample if name in map_interaction]
        if usable:
            draws.append(statistics.fmean(map_interaction[name] for name in usable))
    draws.sort()
    return {
        "point": point, "n_maps": len(map_interaction),
        "replicates_requested": len(samples), "replicates_used": len(draws),
        "lower": draws[int(0.025 * (len(draws) - 1))] if draws else None,
        "upper": draws[int(0.975 * (len(draws) - 1))] if draws else None,
    }


def _endpoint_labels(spec: dict[str, Any], lower: float | None, upper: float | None
                     ) -> dict[str, Any]:
    direction = spec["direction"]
    if lower is None or upper is None or direction in {"tradeoff", "none"}:
        return {"worst_case": None, "best_case": None,
                "note": "no single direction; report both endpoints and context"}
    if direction == "higher_better":
        return {"worst_case": lower, "best_case": upper,
                "note": "higher is better: mathematical lower is least favorable"}
    return {"worst_case": upper, "best_case": lower,
            "note": "lower is better: mathematical upper is least favorable"}


def build_comparisons(primary_rows: list[dict[str, Any]], *,
                      replicates: int = BOOTSTRAP_REPLICATES,
                      seed: int = BOOTSTRAP_SEED) -> dict[str, Any]:
    map_names = sorted({str(row["map"]) for row in primary_rows})
    groups = sorted({str(row["group"]) for row in primary_rows})
    generator = random.Random(seed)
    samples = ([[map_names[generator.randrange(len(map_names))] for _ in map_names]
                for _ in range(replicates)] if map_names else [])
    report: dict[str, Any] = {}
    for spec in METRIC_SPECS:
        values = _map_group_values(primary_rows, spec)
        summaries = {}
        for group in groups:
            group_map_values = {map_name: values.get(map_name, {}).get(group, {})
                                for map_name in map_names}
            summaries[group] = _group_summary(group_map_values, spec)
        comparisons: dict[str, Any] = {}
        scale = 100.0 if spec["kind"] == "rate" else 1.0
        unit = "percentage_point" if spec["kind"] == "rate" else spec["unit"]
        for key, candidate, reference in COMPARISONS:
            if candidate not in groups or reference not in groups:
                continue
            point = _difference(values, candidate, reference, map_names)
            interval = _bootstrap_difference_from_samples(values, candidate, reference, samples)
            scenario = None
            if spec["kind"] == "rate":
                candidate_summary = summaries[candidate]
                reference_summary = summaries[reference]
                lower = (candidate_summary["map_equal_lower"] -
                         reference_summary["map_equal_upper"])
                upper = (candidate_summary["map_equal_upper"] -
                         reference_summary["map_equal_lower"])
                scenario = {"lower": lower * scale, "upper": upper * scale,
                            **_endpoint_labels(spec, lower, upper),
                            "lower_proportion": lower, "upper_proportion": upper,
                            "definition": "candidate_lower - reference_upper, "
                                          "candidate_upper - reference_lower",
                            "scope": "UNKNOWN sensitivity over the full plan denominator; "
                                     "NOT a confidence interval"}
                scenario["worst_case"] = (scenario["worst_case"] * scale
                                          if scenario["worst_case"] is not None else None)
                scenario["best_case"] = (scenario["best_case"] * scale
                                         if scenario["best_case"] is not None else None)
            comparisons[key] = {
                "candidate": candidate, "reference": reference,
                "point": point * scale if point is not None else None,
                "point_proportion": point,
                "unit": unit,
                "bootstrap_95": {
                    "replicates_requested": interval["replicates_requested"],
                    "replicates_used": interval["replicates_used"],
                    "lower": (interval["lower"] * scale
                              if interval["lower"] is not None else None),
                    "upper": (interval["upper"] * scale
                              if interval["upper"] is not None else None),
                },
                "scenario_unknown_bounds": scenario,
            }
        if {"A", "B", "C", "D"}.issubset(groups):
            interaction = _bootstrap_interaction_from_samples(values, map_names, samples)
            comparisons[INTERACTION_KEY] = {
                "definition": "(D-C)-(B-A)",
                "point": (interaction["point"] * scale
                          if interaction["point"] is not None else None),
                "point_proportion": interaction["point"],
                "unit": unit, "bootstrap_95": {
                    "replicates_requested": interaction["replicates_requested"],
                    "replicates_used": interaction["replicates_used"],
                    "lower": (interaction["lower"] * scale
                              if interaction["lower"] is not None else None),
                    "upper": (interaction["upper"] * scale
                              if interaction["upper"] is not None else None)},
                "n_maps": interaction["n_maps"],
                "scenario_unknown_bounds": None,
            }
        report[spec["key"]] = {
            "label": spec["label"], "kind": spec["kind"], "direction": spec["direction"],
            "unit": spec["unit"], "comparison_unit": (
                "percentage_point" if spec["kind"] == "rate" else spec["unit"]),
            "groups": summaries, "comparisons": comparisons,
        }
    return {
        "map_count": len(map_names), "groups": groups,
        "bootstrap": {"replicates": replicates, "seed": seed,
                      "unit": "map clusters; one shared resampling keeps both repeats and all "
                              "four groups across metrics and comparisons",
                      "interval": "percentile 2.5%-97.5%"},
        "metrics": report,
        "notes": [
            "Each map is aggregated over its two repeats before the 30 maps are equally weighted.",
            "Rate differences and their bounds are reported in percentage points; "
            "the underlying proportions remain in each group summary. Unknown events widen the "
            "separate scenario bounds and are never filled as zero.",
            "Scenario bounds express UNKNOWN sensitivity and are not confidence intervals.",
            "Continuous metrics report known-map coverage; scenario bounds are not defined for them.",
            "Completion-time subsets select on success and do not prove overall speed-up.",
            "The legacy composite score is not used for ranking.",
        ],
    }


# ---------------------------------------------------------------------------
# Input loading helpers (CLI)
# ---------------------------------------------------------------------------

def load_plan_csv(path: Path) -> list[dict[str, Any]]:
    from s7_formal_runner import normalize_record, PLAN_FIELDS
    with path.open(newline="", encoding="utf-8-sig") as handle:
        reader = csv.DictReader(handle)
        if tuple(reader.fieldnames or ()) != PLAN_FIELDS:
            raise ValueError("unexpected plan CSV header")
        return [normalize_record(row) for row in reader]


def analyzer_fingerprint() -> dict[str, str | None]:
    from s7_formal_runner import sha256_file
    return {
        "script": "scripts/s1_analyze_run.py",
        "sha256": sha256_file(ROOT / "scripts/s1_analyze_run.py"),
        "safety_layer_sha256": sha256_file(ROOT / "scripts/safety_task_completion.py"),
        "accounting_sha256": sha256_file(ROOT / "scripts/formal_attempt_accounting.py"),
    }


def recompute_scene_start_summaries(plan: list[dict[str, Any]], audit: dict[str, Any],
                                    *, output_dir: Path,
                                    reverse_window_s: float = 0.5,
                                    decision_clock_override: bool = True
                                    ) -> tuple[dict[str, dict[str, Any]], dict[str, Any]]:
    """Re-run the frozen production analyzer for every actual scene start."""
    import sys
    sys.path.insert(0, str(ROOT / "scripts"))
    from s1_analyze_run import calculate

    derived_dir = output_dir / "derived_scene_starts"
    derived_dir.mkdir(parents=True, exist_ok=True)
    fingerprint = analyzer_fingerprint()
    plan_by_id = {item["plan_item_id"]: item for item in plan}
    selected_ids = {row.get("selected_attempt_id") for row in audit.get("plan_items", [])}
    first_ids = {row.get("first_scene_attempt_id") for row in audit.get("plan_items", [])}
    summaries: dict[str, dict[str, Any]] = {}
    records: list[dict[str, Any]] = []
    seen: set[str] = set()
    for fact in audit.get("attempt_facts", []):
        attempt_id = fact.get("attempt_id")
        if not attempt_id or attempt_id in seen or fact.get("scene_created") is not True:
            continue
        seen.add(attempt_id)
        if not fact.get("run_directory"):
            records.append({"plan_item_id": fact.get("plan_item_id"),
                            "attempt_id": attempt_id,
                            "status": "SCENE_STARTED_WITHOUT_RUN_DIRECTORY"})
            continue
        run_dir = Path(fact["run_directory"])
        record_path = run_dir / "runtime_record.jsonl"
        context_path = run_dir / "run_context.json"
        plan_item = plan_by_id.get(fact.get("plan_item_id"), {})
        if not record_path.is_file() or not context_path.is_file():
            records.append({"plan_item_id": fact.get("plan_item_id"), "attempt_id": attempt_id,
                            "status": "RAW_RECORD_MISSING"})
            continue
        summary = calculate(record_path, context_path, reverse_window_s,
                            decision_clock_override=decision_clock_override)
        summary["scene"] = plan_item.get("display_name")
        summary["scene_id"] = plan_item.get("scene_id")
        derived_path = derived_dir / f"{fact.get('plan_item_id')}_{attempt_id}.json"
        with derived_path.open("w", encoding="utf-8") as handle:
            json.dump(summary, handle, indent=2, ensure_ascii=False, sort_keys=True)
            handle.write("\n")
        original = run_dir / "metrics.json"
        summaries[attempt_id] = summary
        records.append({
            "plan_item_id": fact.get("plan_item_id"), "attempt_id": attempt_id,
            "selected_for_plan_item": attempt_id in selected_ids,
            "first_actual_scene_start": attempt_id in first_ids,
            "run_directory": str(run_dir), "derived_path": str(derived_path),
            "original_metrics_sha256": hashlib.sha256(original.read_bytes()).hexdigest()
            if original.is_file() else None,
            "derived_sha256": hashlib.sha256(derived_path.read_bytes()).hexdigest(),
            "analyzer_sha256": fingerprint["sha256"],
            "status": "RECOMPUTED",
        })
    return summaries, {
        "analyzer": fingerprint,
        "analyzer_versions_observed": sorted({row.get("analyzer_sha256")
                                              for row in records
                                              if row.get("analyzer_sha256")}),
        "records": records,
        "decision_clock_reconstruction": {
            "enabled": decision_clock_override,
            "rule": "first active cycle publication-stale frame + matching controller S6-04 "
                    "decision trace with the same run/session/cycle and clock sample",
            "raw_frame_status_preserved": True,
            "raw_records_written": False,
            "scope": "formal analysis uses the same-cycle decision-time clock for timing and "
                     "safety start; no control behavior or raw record is changed",
        },
        "scope": "all actual scene starts recomputed from raw records; original metrics untouched",
    }


def reconstruction_sensitivity(plan: list[dict[str, Any]], audit: dict[str, Any],
                               unified_summaries: dict[str, dict[str, Any]], *,
                               output_dir: Path,
                               reverse_window_s: float = 0.5) -> dict[str, Any]:
    """Compare the unified decision-clock analysis with the raw publication-state view.

    Only runs whose unified summary actually applied the reconstruction are
    recomputed in the default mode; this is a small sensitivity pass, not a
    second analysis pipeline.
    """
    import sys
    sys.path.insert(0, str(ROOT / "scripts"))
    from s1_analyze_run import calculate

    plan_by_id = {item["plan_item_id"]: item for item in plan}
    details: list[dict[str, Any]] = []
    for fact in audit.get("attempt_facts", []):
        attempt_id = fact.get("attempt_id")
        if not attempt_id or fact.get("scene_created") is not True or not fact.get("run_directory"):
            continue
        unified = unified_summaries.get(attempt_id)
        if not unified:
            continue
        clock = unified.get("task_start_clock") or {}
        if clock.get("status") != "VALID_AT_DECISION" or \
                clock.get("timing_clock_override_applied") is not True:
            continue
        run_dir = Path(fact["run_directory"])
        record_path = run_dir / "runtime_record.jsonl"
        context_path = run_dir / "run_context.json"
        if not record_path.is_file() or not context_path.is_file():
            details.append({"plan_item_id": fact.get("plan_item_id"),
                            "attempt_id": attempt_id,
                            "status": "RAW_RECORD_MISSING"})
            continue
        default = calculate(record_path, context_path, reverse_window_s,
                            decision_clock_override=False)
        plan_item = plan_by_id.get(fact.get("plan_item_id"), {})
        safety_changed = (default.get("safety_task_completion_status") !=
                          unified.get("safety_task_completion_status"))
        timing_changed = (
            default.get("task_start_sim_time_s") != unified.get("task_start_sim_time_s") or
            default.get("elapsed_task_duration_s") != unified.get("elapsed_task_duration_s") or
            default.get("short_reverse_switch_coverage") !=
            unified.get("short_reverse_switch_coverage")
        )
        unified_dimensions = (unified.get("safety_task_completion", {}) or {}).get(
            "task_safety_dimensions", {}) or {}
        details.append({
            "plan_item_id": fact.get("plan_item_id"),
            "group": fact.get("group"),
            "attempt_id": attempt_id,
            "run_id": fact.get("run_id"),
            "map": plan_item.get("display_name"),
            "reconstruction_applied": True,
            "frame_sim_clock_valid": clock.get("frame_sim_clock_valid"),
            "frame_sim_clock_status": clock.get("frame_sim_clock_status"),
            "frame_sim_clock_age_ns": clock.get("frame_sim_clock_age_ns"),
            "trace_cycle": clock.get("rl_step"),
            "trace_sim_time_s": clock.get("sim_time_s"),
            "raw_invalid_clock_frames": unified.get("raw_invalid_clock_frames"),
            "timing_invalid_clock_frames": unified.get("invalid_clock_frames"),
            "default": {
                "safety_task_completion": default.get("safety_task_completion_status"),
                "task_start_sim_time_s": default.get("task_start_sim_time_s"),
                "elapsed_task_duration_s": default.get("elapsed_task_duration_s"),
                "short_reverse_switch_coverage": default.get(
                    "short_reverse_switch_coverage"),
                "nonfoot_collision": (default.get("safety_task_completion", {}) or {}).get(
                    "task_safety_dimensions", {}).get("nonfoot_obstacle_collision", {}).get("value"),
                "fall": (default.get("safety_task_completion", {}) or {}).get(
                    "task_safety_dimensions", {}).get("fall", {}).get("value"),
            },
            "unified": {
                "safety_task_completion": unified.get("safety_task_completion_status"),
                "task_start_sim_time_s": unified.get("task_start_sim_time_s"),
                "elapsed_task_duration_s": unified.get("elapsed_task_duration_s"),
                "short_reverse_switch_coverage": unified.get(
                    "short_reverse_switch_coverage"),
                "nonfoot_collision": unified_dimensions.get(
                    "nonfoot_obstacle_collision", {}).get("value"),
                "fall": unified_dimensions.get("fall", {}).get("value"),
                "foot_impact_count": unified_dimensions.get("foot_events", {}).get(
                    "impact_count"),
            },
            "safety_changed": safety_changed,
            "timing_changed": timing_changed,
            "status": "COMPARED",
        })
    summary = {
        "enabled": True,
        "rule": "same-cycle decision clock only; raw publication-stale frame preserved",
        "reconstruction_runs": sum(1 for item in details if item.get("status") == "COMPARED"),
        "safety_changed_runs": sum(1 for item in details if item.get("safety_changed") is True),
        "timing_changed_runs": sum(1 for item in details if item.get("timing_changed") is True),
        "unresolved_runs": sum(1 for item in details if item.get("status") != "COMPARED"),
        "runs": details,
        "scope": "sensitivity of the formal analysis to the decision-clock reconstruction; "
                 "not a separate result pipeline",
    }
    output_dir.mkdir(parents=True, exist_ok=True)
    write_path = output_dir / "reconstruction_sensitivity.json"
    with write_path.open("w", encoding="utf-8") as handle:
        json.dump(summary, handle, indent=2, ensure_ascii=False, sort_keys=True)
        handle.write("\n")
    return summary


def build_report(*, plan: list[dict[str, Any]], audit: dict[str, Any],
                 summary_by_attempt: dict[str, dict[str, Any]],
                 provenance: dict[str, Any] | None = None,
                 replicates: int = BOOTSTRAP_REPLICATES,
                 seed: int = BOOTSTRAP_SEED) -> dict[str, Any]:
    views = build_views(plan, audit, summary_by_attempt)
    comparisons = build_comparisons(views["primary_plan_item"]["rows"],
                                    replicates=replicates, seed=seed)
    return {
        "schema": ANALYSIS_SCHEMA,
        "provenance": provenance or {},
        "plan_binding": {
            "matrix_id": (provenance or {}).get("matrix_id"),
            "plan_sha256": (provenance or {}).get("plan_sha256"),
            "plan_items": len(plan),
        },
        "views": views,
        "comparisons": comparisons,
        "scope": "frozen formal analysis; no composite score; no confirmatory p-values",
    }
