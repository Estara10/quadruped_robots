"""Small, pure attempt-ledger audit/selection rules for formal S7 planning.

This is not a batch runner and never starts a process. Callers must preserve
every invocation row and provide explicit evidence for a replacement.
"""
from __future__ import annotations

from collections import Counter
from copy import deepcopy
from pathlib import PurePosixPath
from typing import Any


LEGAL_TASK_TERMINALS = {
    "ARRIVED", "COLLISION_TERMINATION", "FALL_TERMINATION",
    "DIAGNOSTIC_SIM_TIME_LIMIT", "WALL_CLOCK_GUARD_INCOMPLETE",
    "TIMEOUT", "OPERATOR_ABORT",
}


def _attempt_order(row: dict[str, Any]) -> tuple[int, str]:
    value = row.get("invocation_number", row.get("attempt_number", row.get("attempt", 0)))
    try:
        return int(value), str(row.get("attempt_id", ""))
    except (TypeError, ValueError):
        return 0, str(row.get("attempt_id", ""))


def _contained(run_directory: str | None, output_directory: str | None) -> bool:
    if not run_directory or not output_directory:
        return False
    run = PurePosixPath(run_directory)
    root = PurePosixPath(output_directory)
    return run == root or root in run.parents


def classify_attempt(row: dict[str, Any]) -> dict[str, Any]:
    """Classify one invocation without inferring missing scene/run facts."""
    scene_fact = row.get("scene_created")
    scene_created = scene_fact is True
    run_id = row.get("run_id")
    entered_rl = row.get("entered_rl") is True
    terminal = row.get("raw_terminal")
    validity = row.get("record_validity", "UNKNOWN")
    kind = row.get("kind", "scenario_invocation" if scene_created else "orchestration")
    if scene_fact not in (True, False):
        return {"status": "SCENE_CREATION_UNKNOWN", "scene_start_consumed": "UNKNOWN",
                "navigation_result": "UNKNOWN", "replacement_eligible": False}
    if not scene_created:
        if run_id is not None or row.get("run_directory") is not None or entered_rl:
            return {"status": "CONTRADICTORY_NO_SCENE_FACTS", "scene_start_consumed": True,
                    "navigation_result": "UNKNOWN", "replacement_eligible": False}
        return {"status": "ORCHESTRATION_FAILURE_NO_SCENE", "scene_start_consumed": False,
                "navigation_result": "NOT_APPLICABLE", "replacement_eligible": False}

    if not run_id:
        return {"status": "SCENE_STARTED_WITHOUT_RUN_ID", "scene_start_consumed": True,
                "navigation_result": "UNKNOWN", "replacement_eligible": False}

    if validity == "VALID" and terminal in LEGAL_TASK_TERMINALS:
        status = "VALID_TASK_RESULT" if entered_rl else "STARTED_PREPARATION_TERMINAL"
        return {"status": status, "scene_start_consumed": True,
                "navigation_result": terminal if entered_rl else "NOT_STARTED",
                "replacement_eligible": False}

    if terminal == "SYSTEM_SAFETY_ABORT":
        evidence = row.get("cause_evidence", {})
        independently_supported = (
            evidence.get("class") == "infrastructure_trigger_observed" and
            bool(evidence.get("references")) and
            evidence.get("independent_infrastructure_corroboration") is True
        )
        # A safety veto remains a safety event in the all-start ledger even if
        # an independently corroborated infrastructure fault permits a plan substitute.
        return {"status": "VALID_TASK_RESULT" if validity == "VALID" and entered_rl else "SYSTEM_SAFETY_ABORT",
                "scene_start_consumed": True,
                "navigation_result": "SYSTEM_SAFETY_ABORT" if entered_rl else "NOT_STARTED",
                "replacement_eligible": independently_supported,
                "replacement_reason": ("independent evidence corroborates an infrastructure trigger; safety event remains in all-start reporting"
                                       if independently_supported else
                                       "trigger may be infrastructure-related, but independent infrastructure corroboration is insufficient")}

    evidence = row.get("cause_evidence", {})
    cause_proven = evidence.get("class") in {
        "infrastructure_failure_confirmed", "recording_failure_confirmed",
        "identity_failure_confirmed",
    } and bool(evidence.get("references"))
    if not entered_rl and cause_proven:
        status = "PREPARATION_INFRASTRUCTURE_FAILURE"
    elif validity != "VALID" and cause_proven:
        status = "RECORD_OR_IDENTITY_FAILURE_CONFIRMED"
    elif not entered_rl:
        status = "PREPARATION_FAILURE_CAUSE_UNKNOWN"
    elif validity != "VALID":
        status = "STARTED_RECORD_INVALID_OR_UNKNOWN"
    else:
        status = "STARTED_TERMINAL_UNRESOLVED"
    return {"status": status, "scene_start_consumed": True,
            "navigation_result": "UNKNOWN" if entered_rl else "NOT_STARTED",
            "replacement_eligible": cause_proven,
            "replacement_reason": "documented cause evidence required" if not cause_proven else
                                 "cause evidence supports an engineering substitute"}


def audit_attempt_ledger(plan_items: list[dict[str, Any]], attempts: list[dict[str, Any]],
                         *, max_substitutions: int = 12,
                         max_per_plan_item: int = 1) -> dict[str, Any]:
    """Audit attempts, select at most one result per plan item, and count starts.

    Identical duplicate attempt rows are idempotently collapsed. Conflicting
    duplicates, cross-item run-id reuse, bad replacement links, and quota
    breaches are rejected. This function does not launch or recover runs.
    """
    errors: list[str] = []
    plan_by_id: dict[str, dict[str, Any]] = {}
    for item in plan_items:
        item_id = str(item.get("plan_item_id", ""))
        if not item_id or item_id in plan_by_id:
            errors.append(f"duplicate_or_missing_plan_item_id:{item_id}")
        else:
            plan_by_id[item_id] = item

    unique: dict[str, dict[str, Any]] = {}
    for row in attempts:
        attempt_id = str(row.get("attempt_id", ""))
        if not attempt_id:
            errors.append("missing_attempt_id")
            continue
        prior = unique.get(attempt_id)
        if prior is not None:
            if prior != row:
                errors.append(f"conflicting_duplicate_attempt_id:{attempt_id}")
            continue
        unique[attempt_id] = deepcopy(row)
        item_id = str(row.get("plan_item_id", ""))
        if item_id not in plan_by_id:
            errors.append(f"attempt_references_unknown_plan_item:{attempt_id}:{item_id}")
        group = row.get("group")
        expected_group = plan_by_id.get(item_id, {}).get("group")
        if group is not None and expected_group is not None and group != expected_group:
            errors.append(f"attempt_group_mismatch:{attempt_id}")

    run_owner: dict[str, tuple[str, str]] = {}
    path_owner: dict[str, tuple[str, str]] = {}
    for attempt_id, row in unique.items():
        item_id = str(row.get("plan_item_id", ""))
        run_id = row.get("run_id")
        if run_id:
            previous = run_owner.get(str(run_id))
            owner = (item_id, attempt_id)
            if previous is not None and previous != owner:
                errors.append(f"run_id_reused:{run_id}:{previous[0]}:{item_id}")
            run_owner[str(run_id)] = owner
        run_dir = row.get("run_directory")
        if run_dir:
            previous = path_owner.get(str(run_dir))
            owner = (item_id, attempt_id)
            if previous is not None and previous != owner:
                errors.append(f"run_directory_reused:{run_dir}")
            path_owner[str(run_dir)] = owner
        if row.get("scene_created") is False and (run_id or run_dir or row.get("entered_rl") is True):
            errors.append(f"scene_creation_claim_conflict:{attempt_id}")
        if row.get("entered_rl") is True and row.get("scene_created") is not True:
            errors.append(f"rl_without_scene_start:{attempt_id}")
        if row.get("confirmed_task_safety_failure") is True:
            failure_evidence = row.get("safety_failure_evidence", {})
            if not failure_evidence.get("references"):
                errors.append(f"confirmed_safety_failure_missing_evidence:{attempt_id}")

    by_plan: dict[str, list[dict[str, Any]]] = {key: [] for key in plan_by_id}
    for row in unique.values():
        item_id = str(row.get("plan_item_id", ""))
        if item_id in by_plan:
            by_plan[item_id].append(row)
    for rows in by_plan.values():
        rows.sort(key=_attempt_order)

    starts = sum(row.get("scene_created") is True for row in unique.values())
    unknown_starts = sum(row.get("scene_created") not in (True, False) for row in unique.values())
    task_starts = sum(row.get("scene_created") is True and row.get("entered_rl") is True
                      for row in unique.values())
    invocation_count = len(unique)
    # The formal replacement quota counts replacement scene launches, not
    # pre-scene orchestration invocations that happen to be tagged for one.
    replacements = [row for row in unique.values()
                    if row.get("is_replacement") is True and row.get("scene_created") is True]
    if len(replacements) > max_substitutions:
        errors.append(f"batch_substitution_quota_exceeded:{len(replacements)}>{max_substitutions}")

    selected: dict[str, dict[str, Any] | None] = {}
    item_rows: dict[str, dict[str, Any]] = {}
    for item_id, rows in by_plan.items():
        scene_rows = [row for row in rows if row.get("scene_created") is True]
        if len(scene_rows) > 1:
            order_values = [row.get("invocation_number", row.get("attempt_number", row.get("attempt")))
                            for row in scene_rows]
            try:
                normalized_order = [int(value) for value in order_values]
            except (TypeError, ValueError):
                normalized_order = []
            if len(normalized_order) != len(scene_rows) or len(set(normalized_order)) != len(scene_rows):
                errors.append(f"scene_attempt_order_missing_or_ambiguous:{item_id}")
        item_replacements = [row for row in scene_rows if row.get("is_replacement") is True]
        if len(item_replacements) > max_per_plan_item:
            errors.append(f"plan_item_substitution_quota_exceeded:{item_id}")
        for row in item_replacements:
            replaced_id = row.get("replaces_attempt_id")
            replaced = unique.get(str(replaced_id)) if replaced_id is not None else None
            if replaced is None or replaced.get("plan_item_id") != item_id:
                errors.append(f"replacement_wrong_plan_or_missing_prior_attempt:{row.get('attempt_id')}")
                continue
            prior_facts = classify_attempt(replaced)
            if not prior_facts.get("replacement_eligible"):
                errors.append(f"replacement_not_supported_by_prior_attempt:{row.get('attempt_id')}")
            if replaced is not None and replaced.get("confirmed_task_safety_failure") is True:
                errors.append(f"replacement_cannot_erase_confirmed_safety_failure:{row.get('attempt_id')}")
            if replaced_id == row.get("attempt_id"):
                errors.append(f"replacement_self_reference:{row.get('attempt_id')}")
            if replaced is not None and _attempt_order(replaced) >= _attempt_order(row):
                errors.append(f"replacement_does_not_follow_replaced_attempt:{row.get('attempt_id')}")
            if replaced.get("run_directory") and row.get("run_directory") == replaced.get("run_directory"):
                errors.append(f"replacement_reuses_failed_run_directory:{row.get('attempt_id')}")
            if not row.get("replacement_authorized"):
                errors.append(f"replacement_not_authorized:{row.get('attempt_id')}")
            root = row.get("output_directory")
            if not root or not row.get("run_directory"):
                errors.append(f"replacement_directory_evidence_missing:{row.get('attempt_id')}")
            elif not _contained(row["run_directory"], root):
                errors.append(f"replacement_run_outside_own_attempt_directory:{row.get('attempt_id')}")

        chosen: dict[str, Any] | None = None
        prior_scene: dict[str, Any] | None = None
        for row in scene_rows:
            facts = classify_attempt(row)
            if row.get("is_replacement") is True:
                replaced = unique.get(str(row.get("replaces_attempt_id")))
                prior_eligible = bool(replaced and classify_attempt(replaced).get("replacement_eligible")
                                      and replaced.get("confirmed_task_safety_failure") is not True)
                if (row.get("replacement_authorized") and prior_eligible and
                        facts.get("status") == "VALID_TASK_RESULT"):
                    chosen = row
                elif (row.get("replacement_authorized") and prior_eligible and
                      row.get("record_validity") == "VALID" and row.get("raw_terminal") in LEGAL_TASK_TERMINALS):
                    chosen = row
                else:
                    # A non-eligible replacement attempt cannot erase a prior
                    # valid plan result (notably a system safety abort).
                    break
            elif prior_scene is None:
                if facts.get("status") == "VALID_TASK_RESULT":
                    chosen = row
                elif facts.get("replacement_eligible"):
                    prior_scene = row
                else:
                    chosen = None
                    break
            else:
                errors.append(f"additional_scene_start_without_single_replacement:{item_id}")
                chosen = None
                break
        selected[item_id] = chosen
        first_scene = scene_rows[0] if scene_rows else None
        first_facts = classify_attempt(first_scene) if first_scene else None
        confirmed_failure = any(
            row.get("entered_rl") is True and row.get("confirmed_task_safety_failure") is True
            for row in scene_rows
        )
        unresolved_status = (
            "KNOWN_SAFETY_FAILURE_RECORD_INCOMPLETE" if confirmed_failure else
            "UNRESOLVED_ENGINEERING" if scene_rows and any(classify_attempt(r).get("replacement_eligible") for r in scene_rows) else
            "UNRESOLVED_SCENE_ATTEMPT" if scene_rows else
            "NOT_STARTED" if not any(r.get("scene_created") is True for r in rows) else "UNRESOLVED"
        )
        item_rows[item_id] = {
            "plan_item_id": item_id,
            "group": plan_by_id[item_id].get("group"),
            "invocation_attempts": [row.get("attempt_id") for row in rows],
            "scene_start_attempts": [row.get("attempt_id") for row in scene_rows],
            "selected_attempt_id": chosen.get("attempt_id") if chosen else None,
            "selected_run_id": chosen.get("run_id") if chosen else None,
            "selected_terminal": chosen.get("raw_terminal") if chosen else None,
            "selected_safety_task_completion": chosen.get("safety_task_completion", "UNKNOWN") if chosen else "UNKNOWN",
            "first_scene_attempt_id": first_scene.get("attempt_id") if first_scene else None,
            "first_scene_terminal": first_scene.get("raw_terminal") if first_scene else None,
            "first_scene_record_validity": first_scene.get("record_validity") if first_scene else None,
            "first_scene_navigation_result": first_facts.get("navigation_result") if first_facts else "NOT_STARTED",
            "first_scene_safety_task_completion": first_scene.get("safety_task_completion", "UNKNOWN") if first_scene else "UNKNOWN",
            "confirmed_task_safety_failure_present": confirmed_failure,
            "plan_status": "SELECTED_RESULT" if chosen else
                          unresolved_status,
        }

    outcome_counts: Counter[str] = Counter()
    for row in item_rows.values():
        status = row["plan_status"]
        if status == "SELECTED_RESULT":
            result = row["selected_safety_task_completion"]
            outcome_counts["CONFIRMED_SAFE_COMPLETE" if result is True else
                           "KNOWN_NOT_COMPLETE" if result is False else "SAFETY_UNKNOWN"] += 1
        else:
            outcome_counts[status] += 1
    n_items = len(plan_by_id)
    confirmed = outcome_counts["CONFIRMED_SAFE_COMPLETE"]
    potential_unknown = sum(outcome_counts[k] for k in (
        "SAFETY_UNKNOWN", "UNRESOLVED_ENGINEERING", "UNRESOLVED_SCENE_ATTEMPT", "UNRESOLVED", "NOT_STARTED"))
    denominator = n_items
    all_start_classes: Counter[str] = Counter()
    all_invocation_classes: Counter[str] = Counter()
    for row in unique.values():
        all_invocation_classes[classify_attempt(row)["status"]] += 1
        if row.get("scene_created") is True:
            all_start_classes[classify_attempt(row)["status"]] += 1
    first_only_counts: Counter[str] = Counter()
    for row in item_rows.values():
        if row["first_scene_attempt_id"] is None:
            first_only_counts["NOT_STARTED"] += 1
        elif row["first_scene_navigation_result"] == "NOT_STARTED":
            first_only_counts["NOT_STARTED"] += 1
        elif any(r.get("attempt_id") == row["first_scene_attempt_id"] and
                 r.get("confirmed_task_safety_failure") is True for r in unique.values()):
            first_only_counts["KNOWN_NOT_COMPLETE"] += 1
        else:
            value = row["first_scene_safety_task_completion"]
            first_only_counts["CONFIRMED_SAFE_COMPLETE" if value is True else
                              "KNOWN_NOT_COMPLETE" if value is False else "SAFETY_UNKNOWN"] += 1
    return {
        "status": "PASS" if not errors else "REJECTED",
        "errors": sorted(set(errors)),
        "counts": {
            "plan_items": n_items,
            "invocation_attempts": invocation_count,
            "actual_scene_starts": starts,
            "scene_creation_unknown_attempts": unknown_starts,
            "started_motion_tasks": task_starts,
            "unique_run_ids": len(run_owner),
            "replacement_scene_starts": len(replacements),
        },
        "plan_item_outcomes": dict(sorted(outcome_counts.items())),
        "all_actual_scene_start_class_counts": dict(sorted(all_start_classes.items())),
        "all_invocation_class_counts": dict(sorted(all_invocation_classes.items())),
        "first_actual_scene_only_sensitivity": {
            "plan_item_outcomes": dict(sorted(first_only_counts.items())),
            "denominator_plan_items": denominator,
            "note": "sensitivity only; retains first actual scene result/status and does not replace the primary plan-item selection",
        },
        "safe_completion_bounds_over_all_plan_items": {
            "confirmed_lower": confirmed / denominator if denominator else None,
            "potential_upper": (confirmed + potential_unknown) / denominator if denominator else None,
            "numerator_unknown_categories": ["SAFETY_UNKNOWN", "UNRESOLVED_ENGINEERING", "UNRESOLVED_SCENE_ATTEMPT", "UNRESOLVED", "NOT_STARTED"],
            "denominator_plan_items": denominator,
            "known_task_failure_categories_excluded_from_upper": ["KNOWN_NOT_COMPLETE", "KNOWN_SAFETY_FAILURE_RECORD_INCOMPLETE"],
        },
        "plan_items": [item_rows[key] for key in sorted(item_rows)],
        "attempt_facts": [
            {**row, "classification": classify_attempt(row)}
            for row in sorted(unique.values(), key=lambda x: (_attempt_order(x), str(x.get("plan_item_id"))))
        ],
        "all_scene_starts_retained": True,
        "scope": "pure ledger audit/selection only; no process launch or filesystem recovery",
    }
