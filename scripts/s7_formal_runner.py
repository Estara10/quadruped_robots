"""Thin S7 A/B/C/D formal batch runner core.

This module reads the frozen 240-item plan and dispatches the existing
production single-run diagnostic.  It is deliberately not a general
experiment platform: it owns one batch at a time, keeps an atomic attempt
ledger, refuses to infer scene creation, and never launches a scene without a
separate, explicit batch authorization record.

Offline callers inject a fake executor and disk probe.  The production entry
point passes ``dispatch_production`` and never enables fixture authorization.
"""
from __future__ import annotations

import csv
import hashlib
import json
import os
import shutil
import signal
import subprocess
import sys
import time
from collections import Counter
from contextlib import contextmanager
from datetime import datetime, timezone
from pathlib import Path, PurePosixPath
from typing import Any, Callable, Iterable

ROOT = Path(__file__).resolve().parents[1]
MATRIX_CSV = ROOT / "docs/thesis_project/designs/S7/formal_matrix_240.csv"
MATRIX_JSON = ROOT / "docs/thesis_project/designs/S7/formal_matrix_240.json"
DEFAULT_BATCH_ROOT = ROOT / "logs/08_formal_experiment/S7_ABCD_240"
DEFAULT_OFFLINE_ROOT = ROOT / "logs/07_formal_preparation/runner_readiness"
OVERLAY_SOURCE = ROOT / "quadruped_ros2_control_humble/install/go2_description"
DIAGNOSTIC = ROOT / "scripts/s1_run_diagnostic.py"
ANALYZER = ROOT / "scripts/s1_analyze_run.py"

GROUP_PARAMETERS = {
    "A": {"E": -0.05, "X": None, "H": None},
    "B": {"E": -0.05, "X": -0.10, "H": None},
    "C": {"E": -0.05, "X": None, "H": 0.20},
    "D": {"E": -0.05, "X": -0.10, "H": 0.20},
}
GROUP_ORDER = ("A", "B", "C", "D")
# Entry-point compatibility values only.  A/C never write X and A/B never
# write H; these values merely satisfy the existing CLI's shared validation.
CLI_FALLBACK_EXIT_X = -0.10
CLI_FALLBACK_HOLD_S = 0.30
BASE_SCENE_STARTS = 240
MAX_SUBSTITUTIONS = 12
MAX_TOTAL_SCENE_STARTS = BASE_SCENE_STARTS + MAX_SUBSTITUTIONS
BATCH_BUDGET_GIB = 6.0
MIN_FREE_GIB = 15.0
MAX_PRE_SCENE_INVOCATIONS = 3
FORMAL_CLASSIFICATION = "S7 formal A/B/C/D frozen 240-item batch; authorized formal run"

MATRIX_ID = "S7-formal-240-candidate-v1"
M3_MATRIX_ID = "S7-M3-challenge-240-v1.4"
FROZEN_STATUS = "METHOD_FROZEN_AUTHORIZED_TO_RUN_FALSE"
AUTHORIZATION_SCHEMA = "s7-formal-run-authorization-v1"
MANIFEST_SCHEMA = "s7-formal-batch-manifest-v1"
LEDGER_SCHEMA = "s7-formal-attempt-ledger-v1"
STATE_SCHEMA = "s7-formal-batch-state-v1"

PLAN_FIELDS = (
    "plan_item_id", "global_order", "block_order", "display_name", "filename",
    "scene_id", "map_version", "split", "density", "route_structure",
    "repeat_block", "williams_sequence_id", "within_block_position", "group",
    "E", "X", "H_s", "start_x_m", "start_y_m", "goal_x_m", "goal_y_m",
    "arrival_radius_m", "sim_limit_s", "wall_cap_s",
    "standing_observation_sim_s", "standing_wall_cap_s", "mujoco_window",
    "rviz_enabled", "schedule_seed",
)
INT_FIELDS = {
    "global_order", "block_order", "repeat_block", "within_block_position",
    "schedule_seed",
}
FLOAT_FIELDS = {
    "E", "X", "H_s", "start_x_m", "start_y_m", "goal_x_m", "goal_y_m",
    "arrival_radius_m", "sim_limit_s", "wall_cap_s",
    "standing_observation_sim_s", "standing_wall_cap_s",
}
BOOL_FIELDS = {"rviz_enabled"}

REQUIRED_RUN_CONDITIONS = {
    "goal_x_m": 7.5, "goal_y_m": 0.0, "arrival_radius_m": 0.5,
    "sim_limit_s": 20.0, "wall_cap_s": 120.0,
    "standing_observation_sim_s": 2.0, "standing_wall_cap_s": 30.0,
    "mujoco_window": "hidden", "rviz_enabled": False,
}

WILLIAMS = {
    "W0": ["A", "B", "D", "C"],
    "W1": ["B", "C", "A", "D"],
    "W2": ["C", "D", "B", "A"],
    "W3": ["D", "A", "C", "B"],
}
LEGAL_TASK_TERMINALS = {
    "ARRIVED", "COLLISION_TERMINATION", "FALL_TERMINATION",
    "DIAGNOSTIC_SIM_TIME_LIMIT", "WALL_CLOCK_GUARD_INCOMPLETE",
    "TIMEOUT", "OPERATOR_ABORT",
}
PRE_SCENE_REJECTIONS = {
    "SCENE_PREFLIGHT_REJECTED", "M1_SCENE_SETTINGS_REJECTED",
    "M2_SELECTED_SCENE_SETTINGS_REJECTED",
}


# ---------------------------------------------------------------------------
# Small file helpers
# ---------------------------------------------------------------------------

def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


def sha256_file(path: Path) -> str | None:
    if not path.is_file():
        return None
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def sha256_json(value: Any) -> str:
    payload = json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=False)
    return hashlib.sha256(payload.encode("utf-8")).hexdigest()


def read_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def write_json_atomic(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".tmp")
    with temporary.open("w", encoding="utf-8") as handle:
        json.dump(value, handle, indent=2, ensure_ascii=False, sort_keys=True)
        handle.write("\n")
        handle.flush()
        os.fsync(handle.fileno())
    os.replace(temporary, path)


def free_gib_default() -> float:
    usage = shutil.disk_usage(ROOT)
    return usage.free / (1024 ** 3)


def directory_bytes(path: Path) -> int:
    total = 0
    if not path.exists():
        return 0
    for item in path.rglob("*"):
        try:
            if item.is_file() and not item.is_symlink():
                total += item.stat().st_size
        except OSError:
            continue
    return total


def scan_batch_processes(batch_root: Path) -> list[dict[str, Any]]:
    """Read-only scan for processes whose command line names this batch."""
    markers = [str(batch_root.resolve()), str((batch_root / "overlays").resolve())]
    hits: list[dict[str, Any]] = []
    proc = Path("/proc")
    if not proc.is_dir():
        return hits
    for entry in proc.iterdir():
        if not entry.name.isdigit() or int(entry.name) == os.getpid():
            continue
        try:
            raw = (entry / "cmdline").read_bytes()
        except OSError:
            continue
        command = raw.replace(b"\x00", b" ").decode("utf-8", errors="replace").strip()
        if command and any(marker in command for marker in markers):
            hits.append({"pid": int(entry.name), "command": command[:500]})
    return hits


def version_binding_changes(manifest: dict[str, Any]) -> dict[str, Any]:
    previous = manifest.get("version_record", {}) or {}
    current = version_record()
    keys = ("git_commit", "source_config_sha256", "models_sha256", "scripts_sha256")
    return {key: {"batch": previous.get(key), "current": current.get(key)}
            for key in keys if previous.get(key) != current.get(key)}


# ---------------------------------------------------------------------------
# Frozen plan reading and validation
# ---------------------------------------------------------------------------

def _normalize_value(field: str, value: Any) -> Any:
    if isinstance(value, str):
        value = value.strip()
    if value in ("", None):
        return None
    if field in INT_FIELDS:
        return int(value)
    if field in FLOAT_FIELDS:
        return float(value)
    if field in BOOL_FIELDS:
        if isinstance(value, bool):
            return value
        lowered = str(value).strip().lower()
        if lowered in {"true", "1"}:
            return True
        if lowered in {"false", "0"}:
            return False
        raise ValueError(f"invalid boolean value for {field}: {value!r}")
    return str(value)


def normalize_record(row: dict[str, Any]) -> dict[str, Any]:
    return {field: _normalize_value(field, row.get(field)) for field in PLAN_FIELDS}


def plan_digest(records: Iterable[dict[str, Any]]) -> str:
    canonical = [normalize_record(item) for item in records]
    canonical.sort(key=lambda item: int(item["global_order"]))
    return sha256_json(canonical)


def load_frozen_plan(csv_path: Path = MATRIX_CSV,
                     json_path: Path = MATRIX_JSON) -> tuple[list[dict[str, Any]], dict[str, Any], list[str]]:
    errors: list[str] = []
    with csv_path.open(newline="", encoding="utf-8-sig") as handle:
        reader = csv.DictReader(handle)
        if tuple(reader.fieldnames or ()) != PLAN_FIELDS:
            errors.append(f"csv_header_mismatch:{reader.fieldnames}")
        csv_records = [normalize_record(row) for row in reader]
    meta = read_json(json_path)
    json_records = [normalize_record(row) for row in meta.get("records", [])]
    csv_sorted = sorted(csv_records, key=lambda item: int(item["global_order"]))
    json_sorted = sorted(json_records, key=lambda item: int(item["global_order"]))
    if csv_sorted != json_sorted:
        errors.append("csv_json_record_mismatch")
    is_m3 = meta.get("matrix_id") == M3_MATRIX_ID
    expected_meta = {
        "matrix_id": M3_MATRIX_ID if is_m3 else MATRIX_ID,
        "schedule_seed": 2026102001,
        "bootstrap_seed": 2026102002,
        "map_version": "s7-challenge-test/1.4" if is_m3 else "m1-mapset/1.0",
    }
    for key, expected in expected_meta.items():
        if meta.get(key) != expected:
            errors.append(f"matrix_meta_mismatch:{key}:{meta.get(key)!r}")
    groups = meta.get("groups", {})
    for group, params in GROUP_PARAMETERS.items():
        observed = groups.get(group, {})
        if (observed.get("E") != params["E"] or observed.get("X") != params["X"]
                or observed.get("H_s") != params["H"]):
            errors.append(f"matrix_group_parameters_mismatch:{group}:{observed!r}")
    if meta.get("williams_sequences") != WILLIAMS:
        errors.append("matrix_williams_sequences_mismatch")
    if meta.get("field_bounds_m") != {"x": [-0.5, 8.0], "y": [-3.0, 3.0]}:
        errors.append("matrix_field_bounds_mismatch")
    if meta.get("field_is_physical_wall") is not False:
        errors.append("matrix_field_wall_flag_mismatch")
    if meta.get("authorized_to_run") is not False:
        errors.append("matrix_must_keep_authorized_to_run_false")
    if is_m3:
        for row in csv_sorted:
            name = str(row["filename"]).removesuffix(".xml")
            if (name not in {f"challenge{i:02d}" for i in range(1, 31)}
                    or row["display_name"] != name
                    or row["scene_id"] != "s7ch" + name.removeprefix("challenge")
                    or row["split"] != "challenge_test"):
                errors.append(f"m3_scene_binding_mismatch:{row['plan_item_id']}")
    return csv_sorted, meta, errors


def validate_plan(records: list[dict[str, Any]], meta: dict[str, Any], *,
                  require_full: bool = True) -> dict[str, Any]:
    """Validate plan identity/order/parameters without running anything."""
    errors: list[str] = []
    warnings: list[str] = []
    if not records:
        return {"status": "REJECTED", "errors": ["empty_plan"], "warnings": warnings}

    ids = [str(item.get("plan_item_id")) for item in records]
    if len(ids) != len(set(ids)):
        errors.append("duplicate_plan_item_id")
    orders = [item.get("global_order") for item in records]
    if len(set(orders)) != len(orders):
        errors.append("duplicate_global_order")
    ordered = sorted(records, key=lambda item: int(item["global_order"]))
    expected_orders = list(range(1, len(ordered) + 1))
    if [int(item["global_order"]) for item in ordered] != expected_orders:
        errors.append("global_order_not_contiguous")

    by_block: dict[tuple[str, int], list[dict[str, Any]]] = {}
    by_map: Counter[str] = Counter()
    by_group: Counter[str] = Counter()
    by_sequence: Counter[str] = Counter()
    transitions: Counter[tuple[str, str]] = Counter()
    block_order_seen: dict[int, tuple[str, int]] = {}
    for item in ordered:
        group = item.get("group")
        if group not in GROUP_PARAMETERS:
            errors.append(f"unknown_group:{item.get('plan_item_id')}:{group}")
            continue
        params = GROUP_PARAMETERS[group]
        if item.get("E") != params["E"]:
            errors.append(f"entry_threshold_mismatch:{item.get('plan_item_id')}")
        if item.get("X") != params["X"]:
            errors.append(f"exit_threshold_field_mismatch:{item.get('plan_item_id')}")
        if item.get("H_s") != params["H"]:
            errors.append(f"hold_field_mismatch:{item.get('plan_item_id')}")
        for field, expected in REQUIRED_RUN_CONDITIONS.items():
            if item.get(field) != expected:
                errors.append(f"run_condition_mismatch:{item.get('plan_item_id')}:{field}:{item.get(field)!r}")
        if item.get("schedule_seed") != meta.get("schedule_seed"):
            errors.append(f"schedule_seed_mismatch:{item.get('plan_item_id')}")
        if item.get("map_version") != meta.get("map_version"):
            errors.append(f"map_version_mismatch:{item.get('plan_item_id')}")
        if item.get("start_x_m") != 0.0 or item.get("start_y_m") != 0.0:
            errors.append(f"start_position_mismatch:{item.get('plan_item_id')}")
        key = (str(item.get("display_name")), int(item.get("repeat_block")))
        by_block.setdefault(key, []).append(item)
        by_map[str(item.get("display_name"))] += 1
        by_group[group] += 1
        block_order = int(item.get("block_order"))
        prior_owner = block_order_seen.setdefault(block_order, key)
        if prior_owner != key:
            errors.append(f"block_order_reused:{block_order}")

    if require_full:
        if len(ordered) != 240:
            errors.append(f"plan_item_count:{len(ordered)}")
        if len(by_map) != 30:
            errors.append(f"map_count:{len(by_map)}")
        if any(count != 8 for count in by_map.values()):
            errors.append("map_item_count_mismatch")
        if any(count != 60 for count in by_group.values()):
            errors.append("group_item_count_mismatch")
        if len(by_block) != 60:
            errors.append(f"block_count:{len(by_block)}")
        if sorted(block_order_seen) != list(range(1, 61)):
            errors.append("block_order_not_contiguous")
    else:
        if any(len(count) != 4 for count in by_block.values()):
            errors.append("fixture_block_incomplete")

    sequences_per_map: dict[str, list[str]] = {}
    for (display_name, repeat_block), items in by_block.items():
        positions = sorted(items, key=lambda item: int(item["within_block_position"]))
        if [int(item["within_block_position"]) for item in positions] != [1, 2, 3, 4]:
            errors.append(f"block_positions_incomplete:{display_name}:{repeat_block}")
            continue
        if len({item["group"] for item in positions}) != 4:
            errors.append(f"block_groups_not_unique:{display_name}:{repeat_block}")
        seq_id = positions[0].get("williams_sequence_id")
        expected_sequence = WILLIAMS.get(str(seq_id))
        if expected_sequence is None or [item["group"] for item in positions] != expected_sequence:
            errors.append(f"williams_order_mismatch:{display_name}:{repeat_block}:{seq_id}")
        if len({item.get("williams_sequence_id") for item in positions}) != 1:
            errors.append(f"williams_sequence_inconsistent:{display_name}:{repeat_block}")
        by_sequence[str(seq_id)] += 1
        sequences_per_map.setdefault(display_name, []).append(str(seq_id))
        for left, right in zip(positions, positions[1:]):
            transitions[(left["group"], right["group"])] += 1
        block_orders = [int(item["block_order"]) for item in positions]
        if len(set(block_orders)) != 1:
            errors.append(f"block_order_inconsistent:{display_name}:{repeat_block}")
        if [int(item["global_order"]) for item in positions] != list(
                range(int(positions[0]["global_order"]), int(positions[0]["global_order"]) + 4)):
            errors.append(f"block_not_contiguous_in_global_order:{display_name}:{repeat_block}")
    if require_full:
        expected_transitions = {(a, b): 15 for a in GROUP_ORDER for b in GROUP_ORDER if a != b}
        if dict(transitions) != expected_transitions:
            errors.append(f"group_adjacency_imbalance:{dict(sorted(transitions.items()))}")
        if dict(by_sequence) != {name: 15 for name in WILLIAMS}:
            errors.append(f"williams_sequence_count_mismatch:{dict(sorted(by_sequence.items()))}")
        if any(len(set(values)) != 2 for values in sequences_per_map.values()):
            errors.append("map_repeats_do_not_use_two_distinct_sequences")

    if meta.get("status") not in {FROZEN_STATUS, "CANDIDATE_FROZEN_PENDING_DIRECTOR_REVIEW"}:
        warnings.append(f"unexpected_matrix_status:{meta.get('status')}")
    return {
        "status": "PASS" if not errors else "REJECTED",
        "errors": sorted(set(errors)),
        "warnings": sorted(set(warnings)),
        "counts": {
            "plan_items": len(ordered), "maps": len(by_map), "blocks": len(by_block),
            "groups": dict(sorted(by_group.items())),
            "williams_sequences": dict(sorted(by_sequence.items())),
        },
        "transitions": {f"{a}->{b}": count for (a, b), count in sorted(transitions.items())},
    }


def matrix_binding(records: list[dict[str, Any]], meta: dict[str, Any],
                   csv_path: Path = MATRIX_CSV, json_path: Path = MATRIX_JSON) -> dict[str, Any]:
    return {
        "matrix_id": meta.get("matrix_id", MATRIX_ID),
        "matrix_status": meta.get("status"),
        "plan_sha256": plan_digest(records),
        "csv_sha256": sha256_file(csv_path),
        "json_sha256": sha256_file(json_path),
        "map_version": meta.get("map_version"),
        "schedule_seed": meta.get("schedule_seed"),
        "bootstrap_seed": meta.get("bootstrap_seed"),
        "plan_items": len(records),
        "authorized_to_run": False,
    }


# ---------------------------------------------------------------------------
# Authorization
# ---------------------------------------------------------------------------

def validate_authorization(authorization: dict[str, Any] | None, binding: dict[str, Any], *,
                           batch_id: str, allow_fixture: bool = False) -> dict[str, Any]:
    """Validate an explicit batch-run authorization record.

    No CLI flag can bypass this.  Fixture records are accepted only by offline
    checks that also inject a non-production executor.
    """
    errors: list[str] = []
    if not isinstance(authorization, dict):
        return {"status": "REJECTED", "errors": ["missing_authorization"]}
    fixture = authorization.get("fixture_only") is True
    if fixture and not allow_fixture:
        errors.append("fixture_authorization_rejected_by_production_entry")
    if authorization.get("schema") != AUTHORIZATION_SCHEMA:
        errors.append("authorization_schema_mismatch")
    if authorization.get("authorized_to_run") is not True:
        errors.append("authorization_does_not_authorize_run")
    if not str(authorization.get("authorization_id", "")).strip():
        errors.append("authorization_id_missing")
    if authorization.get("batch_id") != batch_id:
        errors.append("authorization_batch_id_mismatch")
    if authorization.get("matrix_id") != binding.get("matrix_id"):
        errors.append("authorization_matrix_id_mismatch")
    if authorization.get("plan_sha256") != binding.get("plan_sha256"):
        errors.append("authorization_plan_sha256_mismatch")
    if authorization.get("base_scene_starts") != BASE_SCENE_STARTS:
        errors.append("authorization_base_scene_starts_mismatch")
    if authorization.get("max_substitutions") != MAX_SUBSTITUTIONS:
        errors.append("authorization_substitution_quota_mismatch")
    if authorization.get("max_total_scene_starts") != MAX_TOTAL_SCENE_STARTS:
        errors.append("authorization_total_scene_cap_mismatch")
    if authorization.get("batch_budget_gib") != BATCH_BUDGET_GIB:
        errors.append("authorization_batch_budget_mismatch")
    if authorization.get("min_free_gib") != MIN_FREE_GIB:
        errors.append("authorization_free_space_guard_mismatch")
    review = authorization.get("director_review")
    if not isinstance(review, dict) or not str(review.get("reference", "")).strip() \
            or not str(review.get("date", "")).strip():
        errors.append("authorization_director_review_missing")
    replacements = authorization.get("replacements", [])
    if not isinstance(replacements, list):
        errors.append("authorization_replacements_not_a_list")
    return {
        "status": "PASS" if not errors else "REJECTED",
        "errors": sorted(set(errors)),
        "fixture_only": fixture,
        "authorization_id": authorization.get("authorization_id"),
    }


def validate_replacement_entry(entry: dict[str, Any], *, plan_item_id: str,
                               attempts: list[dict[str, Any]]) -> dict[str, Any]:
    """Validate one explicit replacement authorization against its prior attempt."""
    from formal_attempt_accounting import classify_attempt

    errors: list[str] = []
    required_text = ("authorization_id", "reason", "authorized_by", "authorized_date")
    for field in required_text:
        if not str(entry.get(field, "")).strip():
            errors.append(f"replacement_{field}_missing")
    if entry.get("plan_item_id") != plan_item_id:
        errors.append("replacement_plan_item_binding_mismatch")
    if entry.get("authorized_by") not in {"Director", "director"}:
        errors.append("replacement_authorized_by_is_not_director")
    references = entry.get("evidence_references")
    if not isinstance(references, list) or not references:
        errors.append("replacement_independent_evidence_missing")
    replaced_id = entry.get("replaces_attempt_id")
    replaced = next((row for row in attempts if row.get("attempt_id") == replaced_id), None)
    if replaced is None:
        errors.append("replacement_prior_attempt_missing")
    else:
        facts = classify_attempt(replaced)
        if not facts.get("replacement_eligible"):
            errors.append("replacement_prior_attempt_not_eligible")
        if facts.get("status") == "SYSTEM_SAFETY_ABORT" and not entry.get("independent_infrastructure_corroboration"):
            errors.append("replacement_safety_abort_without_independent_corroboration")
        if replaced.get("confirmed_task_safety_failure") is True:
            errors.append("replacement_cannot_erase_confirmed_task_safety_failure")
        if any(row.get("is_replacement") is True and row.get("replaces_attempt_id") == replaced_id
               for row in attempts):
            errors.append("replacement_already_used_for_prior_attempt")
    if any(row.get("is_replacement") is True for row in attempts
           if row.get("plan_item_id") == plan_item_id):
        errors.append("plan_item_already_has_replacement")
    return {"status": "PASS" if not errors else "REJECTED", "errors": sorted(set(errors))}


# ---------------------------------------------------------------------------
# Batch lock and ledger
# ---------------------------------------------------------------------------

class BatchLockError(RuntimeError):
    pass


@contextmanager
def batch_lock(batch_root: Path):
    lock_path = batch_root / "ledger" / "batch.lock"
    lock_path.parent.mkdir(parents=True, exist_ok=True)
    if lock_path.exists():
        try:
            existing = read_json(lock_path)
            pid = int(existing.get("pid"))
            os.kill(pid, 0)
        except (OSError, ValueError, TypeError, json.JSONDecodeError):
            stale_dir = batch_root / "ledger" / "stale_locks"
            stale_dir.mkdir(parents=True, exist_ok=True)
            target = stale_dir / f"batch_{int(time.time() * 1000)}.lock"
            os.replace(lock_path, target)
        else:
            raise BatchLockError(f"another batch entry is active (pid={pid}): {lock_path}")
    descriptor = os.open(lock_path, os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o644)
    with os.fdopen(descriptor, "w", encoding="utf-8") as handle:
        json.dump({"pid": os.getpid(), "started_utc": utc_now()}, handle)
        handle.flush()
        os.fsync(handle.fileno())
    try:
        yield lock_path
    finally:
        try:
            if lock_path.exists():
                current = read_json(lock_path)
                if int(current.get("pid", -1)) == os.getpid():
                    lock_path.unlink()
        except (OSError, ValueError, TypeError, json.JSONDecodeError):
            pass


class Ledger:
    def __init__(self, path: Path, batch_id: str):
        self.path = path
        self.batch_id = batch_id
        self.value: dict[str, Any] = {
            "schema": LEDGER_SCHEMA, "batch_id": batch_id,
            "attempts": [], "events": [], "checkpoints": [],
        }
        if path.is_file():
            loaded = read_json(path)
            if loaded.get("schema") != LEDGER_SCHEMA:
                raise ValueError(f"unexpected ledger schema: {loaded.get('schema')}")
            if loaded.get("batch_id") != batch_id:
                raise ValueError("ledger batch_id mismatch")
            self.value = loaded

    @property
    def attempts(self) -> list[dict[str, Any]]:
        return self.value["attempts"]

    def save(self) -> None:
        write_json_atomic(self.path, self.value)

    def upsert_attempt(self, attempt: dict[str, Any]) -> None:
        attempt_id = attempt.get("attempt_id")
        for index, row in enumerate(self.value["attempts"]):
            if row.get("attempt_id") == attempt_id:
                self.value["attempts"][index] = attempt
                break
        else:
            self.value["attempts"].append(attempt)
        self.value["events"].append({
            "utc": utc_now(), "event": "attempt_upsert",
            "attempt_id": attempt_id, "status": attempt.get("runner_status"),
            "scene_created": attempt.get("scene_created"),
        })
        self.save()

    def event(self, name: str, **fields: Any) -> None:
        self.value["events"].append({"utc": utc_now(), "event": name, **fields})
        self.save()

    def checkpoint(self, audit: dict[str, Any], *, free_gib: float,
                   batch_gib: float, reason: str) -> None:
        self.value["checkpoints"].append({
            "utc": utc_now(), "reason": reason,
            "actual_scene_starts": audit.get("counts", {}).get("actual_scene_starts"),
            "replacement_scene_starts": audit.get("counts", {}).get("replacement_scene_starts"),
            "free_gib": round(free_gib, 3), "batch_gib": round(batch_gib, 3),
            "plan_item_outcomes": audit.get("plan_item_outcomes"),
        })
        self.save()


def validate_attempt_paths(attempts: list[dict[str, Any]]) -> list[str]:
    """Enforce attempt-directory/run-directory/run-id binding."""
    errors: list[str] = []
    for row in attempts:
        output_directory = row.get("output_directory")
        run_directory = row.get("run_directory")
        run_id = row.get("run_id")
        if output_directory and run_directory:
            root = PurePosixPath(str(output_directory))
            run = PurePosixPath(str(run_directory))
            if not (run == root or root in run.parents):
                errors.append(f"run_directory_outside_attempt_directory:{row.get('attempt_id')}")
        if run_id and run_directory:
            if PurePosixPath(str(run_directory)).name != str(run_id):
                errors.append(f"run_id_does_not_match_directory:{row.get('attempt_id')}")
        if row.get("scene_created") is not True and run_id:
            errors.append(f"run_id_without_confirmed_scene:{row.get('attempt_id')}")
    return errors


def validate_replacement_authorizations(attempts: list[dict[str, Any]],
                                         authorization: dict[str, Any]) -> list[str]:
    """Every replacement in the ledger must match an explicit authorization entry."""
    errors: list[str] = []
    entries = authorization.get("replacements", []) if isinstance(authorization, dict) else []
    for row in attempts:
        if row.get("is_replacement") is not True or row.get("scene_created") is not True:
            continue
        matches = [entry for entry in entries
                   if entry.get("plan_item_id") == row.get("plan_item_id")
                   and entry.get("replaces_attempt_id") == row.get("replaces_attempt_id")]
        if len(matches) != 1:
            errors.append(f"replacement_without_single_authorization:{row.get('attempt_id')}")
            continue
        if row.get("replacement_authorization_id") != matches[0].get("authorization_id"):
            errors.append(f"replacement_authorization_id_mismatch:{row.get('attempt_id')}")
    return errors


def audit_ledger(plan: list[dict[str, Any]], attempts: list[dict[str, Any]], *,
                 max_substitutions: int = MAX_SUBSTITUTIONS,
                 max_per_plan_item: int = 1) -> dict[str, Any]:
    from formal_attempt_accounting import audit_attempt_ledger
    audit = audit_attempt_ledger(plan, attempts, max_substitutions=max_substitutions,
                                 max_per_plan_item=max_per_plan_item)
    path_errors = validate_attempt_paths(attempts)
    if path_errors:
        audit = {**audit, "status": "REJECTED",
                 "errors": sorted(set(list(audit.get("errors", [])) + path_errors))}
    return audit


# ---------------------------------------------------------------------------
# Overlay preparation
# ---------------------------------------------------------------------------

def _config_hashes(overlay: Path) -> dict[str, str | None]:
    return {
        relative: sha256_file(overlay / relative)
        for relative in (
            "share/go2_description/config/abs/config.yaml",
            "share/go2_description/config/abs/policy.pt",
            "share/go2_description/config/abs/ra_value.pt",
            "share/go2_description/config/rec/policy.pt",
        )
    }


def prepare_group_overlays(batch_root: Path, *, source: Path = OVERLAY_SOURCE,
                           groups: Iterable[str] = GROUP_ORDER) -> dict[str, Any]:
    """Create one detached overlay per requested group; never write back to source."""
    from manual_ab_compare import (
        prepare_overlay, set_overlay_goal, verify_overlay_detached,
    )
    from s1_run_diagnostic import overlay_configuration_matches

    overlays_root = batch_root / "overlays"
    overlays_root.mkdir(parents=True, exist_ok=True)
    requested = tuple(groups)
    unknown = [group for group in requested if group not in GROUP_PARAMETERS]
    if unknown:
        raise ValueError(f"unknown overlay groups: {unknown}")
    report: dict[str, Any] = {"source": str(source), "groups": {}}
    for group in requested:
        params = GROUP_PARAMETERS[group]
        destination = overlays_root / group
        reused = destination.exists()
        if not reused:
            prepare_overlay(source, destination, group, params["E"],
                            CLI_FALLBACK_EXIT_X if params["X"] is None else params["X"],
                            CLI_FALLBACK_HOLD_S if params["H"] is None else params["H"])
            set_overlay_goal(destination / "share/go2_description/config/abs/config.yaml",
                             float(REQUIRED_RUN_CONDITIONS["goal_x_m"]),
                             float(REQUIRED_RUN_CONDITIONS["goal_y_m"]))
        detached = verify_overlay_detached(source, destination)
        config_path = destination / "share/go2_description/config/abs/config.yaml"
        config_text = config_path.read_text(encoding="utf-8")
        if not overlay_configuration_matches(
                config_text, group, params["E"],
                CLI_FALLBACK_EXIT_X if params["X"] is None else params["X"],
                CLI_FALLBACK_HOLD_S if params["H"] is None else params["H"],
                float(REQUIRED_RUN_CONDITIONS["goal_x_m"]),
                float(REQUIRED_RUN_CONDITIONS["goal_y_m"])):
            raise RuntimeError(f"group {group} overlay failed production configuration validation")
        if any((destination / relative).is_symlink() for relative in detached):
            raise RuntimeError(f"group {group} overlay contains a symlink")
        report["groups"][group] = {
            "status": "DETACHED_CONFIG_VALIDATED",
            "reused": reused,
            "config_path": str(config_path),
            "group_fields": {
                "E": params["E"],
                "X_configured": params["X"],
                "H_configured_sim_s": params["H"],
            },
            "cli_compatibility_only": {
                "exit_threshold": CLI_FALLBACK_EXIT_X if params["X"] is None else None,
                "hold_s": CLI_FALLBACK_HOLD_S if params["H"] is None else None,
            },
            "independent_regular_files": detached,
            "sha256": _config_hashes(destination),
            "scope": "static detached-file and YAML field check; NOT proof of controller consumption",
        }
    return report


# ---------------------------------------------------------------------------
# Version record and preflight
# ---------------------------------------------------------------------------

def _git(*args: str) -> str | None:
    try:
        result = subprocess.run(["git", *args], cwd=ROOT, capture_output=True, text=True,
                                timeout=20, check=False)
    except (OSError, subprocess.SubprocessError):
        return None
    return result.stdout.strip() if result.returncode == 0 else None


def version_record() -> dict[str, Any]:
    status_text = _git("status", "--porcelain=v1") or ""
    script_names = (
        "s7_formal_runner.py", "run_s7_formal_240.py", "s7_formal_analysis.py",
        "analyze_s7_formal_240.py", "s1_run_diagnostic.py", "s1_analyze_run.py",
        "safety_task_completion.py", "formal_attempt_accounting.py",
        "manual_ab_compare.py",
    )
    return {
        "captured_utc": utc_now(),
        "git_commit": _git("rev-parse", "HEAD"),
        "git_branch": _git("rev-parse", "--abbrev-ref", "HEAD"),
        "git_dirty": bool(status_text.strip()),
        "git_status_sha256": hashlib.sha256(status_text.encode()).hexdigest(),
        "python": sys.version.split()[0],
        "scripts_sha256": {name: sha256_file(ROOT / "scripts" / name) for name in script_names},
        "source_config_sha256": sha256_file(
            OVERLAY_SOURCE / "share/go2_description/config/abs/config.yaml"),
        "models_sha256": {
            "policy.pt": sha256_file(OVERLAY_SOURCE / "share/go2_description/config/abs/policy.pt"),
            "ra_value.pt": sha256_file(OVERLAY_SOURCE / "share/go2_description/config/abs/ra_value.pt"),
            "rec_policy.pt": sha256_file(OVERLAY_SOURCE / "share/go2_description/config/rec/policy.pt"),
        },
    }


def preflight(records: list[dict[str, Any]], meta: dict[str, Any], *,
              require_full: bool = True, source: Path = OVERLAY_SOURCE,
              batch_root: Path | None = None,
              free_gib: float | None = None,
              prepare: bool = False,
              required_status: str | None = FROZEN_STATUS,
              plan_validator: Callable[[list[dict[str, Any]], dict[str, Any]], dict[str, Any]] | None = None,
              binding: dict[str, Any] | None = None) -> dict[str, Any]:
    """Static, no-scene preflight.  Never claims config consumption."""
    errors: list[str] = []
    validation = (plan_validator(records, meta) if plan_validator
                  else validate_plan(records, meta, require_full=require_full))
    if validation["status"] != "PASS":
        errors.extend(validation["errors"])
    if required_status is not None and meta.get("status") != required_status:
        errors.append(f"matrix_not_in_required_status:{meta.get('status')}")
    if meta.get("authorized_to_run") is not False:
        errors.append("matrix_authorized_to_run_field_must_stay_false")

    from abs_scene import resolve_scene
    from s1_run_diagnostic import resolve_map_manifest
    scene_checks: dict[str, Any] = {}
    for filename in sorted({item["filename"] for item in records}):
        try:
            spec = resolve_scene(filename)
            manifest, version = resolve_map_manifest(
                spec, float(REQUIRED_RUN_CONDITIONS["goal_x_m"]),
                float(REQUIRED_RUN_CONDITIONS["goal_y_m"]),
                float(REQUIRED_RUN_CONDITIONS["sim_limit_s"]),
                float(REQUIRED_RUN_CONDITIONS["wall_cap_s"]))
            expected_scene_id = next(item["scene_id"] for item in records
                                     if item["filename"] == filename)
            if spec.scene_id != expected_scene_id:
                errors.append(f"scene_id_mismatch:{filename}:{spec.scene_id}:{expected_scene_id}")
            if version != meta.get("map_version"):
                errors.append(f"scene_map_version_mismatch:{filename}:{version}:{meta.get('map_version')}")
            scene_checks[filename] = {
                "status": "STATICALLY_BOUND", "scene_id": spec.scene_id,
                "map_library_version": version,
                "goal_m": manifest.get("goal_m") if manifest else None,
                "field_bounds_m": manifest.get("field_bounds_m") if manifest else None,
            }
        except (OSError, ValueError, KeyError, TypeError) as error:
            errors.append(f"scene_static_check_failed:{filename}:{error}")

    overlay_check: dict[str, Any] = {"status": "NOT_PREPARED"}
    if prepare:
        if batch_root is None:
            errors.append("overlay_prepare_requires_batch_root")
        else:
            try:
                overlay_check = prepare_group_overlays(
                    batch_root, source=source,
                    groups=sorted({item["group"] for item in records}))
            except Exception as error:  # noqa: BLE001 - report, never hide
                errors.append(f"overlay_prepare_failed:{error}")
                overlay_check = {"status": "FAILED", "reason": str(error)}
    else:
        overlay_check = {
            "status": "STATIC_SOURCE_CHECK",
            "source": str(source),
            "source_config_sha256": sha256_file(
                source / "share/go2_description/config/abs/config.yaml"),
            "scope": "detached per-group overlays are prepared before a batch starts",
        }

    free_value = free_gib if free_gib is not None else free_gib_default()
    if free_value < MIN_FREE_GIB:
        errors.append(f"free_space_below_guard:{free_value:.3f}GiB<{MIN_FREE_GIB}GiB")
    budget = {
        "free_gib": round(free_value, 3), "min_free_gib": MIN_FREE_GIB,
        "batch_budget_gib": BATCH_BUDGET_GIB, "status": "PASS" if free_value >= MIN_FREE_GIB else "REJECTED",
    }
    return {
        "status": "PASS" if not errors else "REJECTED",
        "errors": sorted(set(errors)),
        "plan_validation": validation,
        "scene_static_checks": scene_checks,
        "overlay_check": overlay_check,
        "disk": budget,
        "matrix": binding or matrix_binding(records, meta),
        "scope": "no scene, ROS, model inference, or controller process is started; "
                 "static checks do NOT prove runtime config consumption",
    }


# ---------------------------------------------------------------------------
# Production dispatch and run-fact inspection
# ---------------------------------------------------------------------------

def build_diagnostic_command(record: dict[str, Any], *, overlay: Path,
                             output_root: Path, display: str = ":0",
                             xauthority: str | None = None,
                             classification: str = FORMAL_CLASSIFICATION) -> list[str]:
    group = record["group"]
    params = GROUP_PARAMETERS[group]
    command = [
        "/usr/bin/python3", "-u", str(DIAGNOSTIC),
        "--group", group,
        "--scene", str(record["filename"]),
        "--goal-x", f"{float(record['goal_x_m']):.17g}",
        "--goal-y", f"{float(record['goal_y_m']):.17g}",
        "--entry-threshold", f"{params['E']:.17g}",
        "--exit-threshold", f"{(params['X'] if params['X'] is not None else CLI_FALLBACK_EXIT_X):.17g}",
        "--hold-s", f"{(params['H'] if params['H'] is not None else CLI_FALLBACK_HOLD_S):.17g}",
        "--sim-limit-s", f"{float(record['sim_limit_s']):.17g}",
        "--wall-cap-s", f"{float(record['wall_cap_s']):.17g}",
        "--standing-observation-sim-s", f"{float(record['standing_observation_sim_s']):.17g}",
        "--standing-observation-wall-cap-s", f"{float(record['standing_wall_cap_s']):.17g}",
        "--startup-gain-trace",
        "--overlay", str(overlay),
        "--output-root", str(output_root),
        "--classification", classification,
    ]
    if display:
        command += ["--display", display]
    if xauthority:
        command += ["--xauthority", xauthority]
    if record.get("mujoco_window") == "hidden" or record.get("rviz_enabled") is False:
        command.append("--no-window")
    command.append("--disable-framebuffer-capture")
    return command


def dispatch_production(command: list[str], log_path: Path, *, cwd: Path = ROOT) -> dict[str, Any]:
    """Run the existing diagnostic in the foreground and parse its JSON result."""
    last_result: dict[str, Any] | None = None
    interrupted = False
    with log_path.open("w", encoding="utf-8") as log:
        child = subprocess.Popen(command, cwd=cwd, stdout=subprocess.PIPE,
                                 stderr=subprocess.STDOUT, text=True, bufsize=1)
        try:
            assert child.stdout is not None
            for line in child.stdout:
                print(line, end="", flush=True)
                log.write(line)
                log.flush()
                try:
                    value = json.loads(line)
                except json.JSONDecodeError:
                    continue
                if isinstance(value, dict) and ("run_id" in value or "result" in value):
                    last_result = value
            returncode = child.wait()
        except KeyboardInterrupt:
            interrupted = True
            print("\n已中断当前run，等待诊断器自行清理；不会启动下一项。", flush=True)
            if child.poll() is None:
                child.send_signal(signal.SIGINT)
            try:
                returncode = child.wait(timeout=60)
            except subprocess.TimeoutExpired:
                print("尚未确认诊断器退出；保留中断事实，需人工核对进程。", flush=True)
                returncode = None
    return {"stdout_result": last_result, "returncode": returncode, "interrupted": interrupted}


def analyze_production(run_dir: Path, *, timeout_s: float = 300.0) -> dict[str, Any]:
    env = os.environ.copy()
    env["PYTHONNOUSERSITE"] = "1"
    env.setdefault("MPLCONFIGDIR", "/tmp/s7_formal_240_mpl")
    command = [sys.executable, str(ANALYZER), "--runs-root", str(run_dir.parent),
               "--output-dir", str(run_dir.parent / "analysis"),
               "--task-label", f"S7-{run_dir.name[:8]}"]
    try:
        result = subprocess.run(command, cwd=ROOT, env=env, text=True,
                                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                timeout=timeout_s, check=False)
        return {"status": "PASS" if result.returncode == 0 else "FAILED",
                "returncode": result.returncode, "output": result.stdout[-4000:]}
    except (OSError, subprocess.SubprocessError) as error:
        return {"status": "FAILED", "returncode": None, "output": str(error)}


MUJOCO_START_MARKERS = ("[MUJOCO-WINDOW]", "MuJoCo version")


def _find_run_directories(result_root: Path) -> list[Path]:
    if not result_root.is_dir():
        return []
    markers = ("runtime_record.jsonl", "run_context.json", "terminal.json",
               "process_facts.json", "mujoco.log", "console.log")
    return sorted(item for item in result_root.iterdir()
                  if item.is_dir() and any((item / marker).is_file() for marker in markers))


def _scene_marker(run_dir: Path) -> dict[str, Any]:
    context_path = run_dir / "run_context.json"
    if context_path.is_file():
        try:
            context = read_json(context_path)
        except (OSError, json.JSONDecodeError):
            context = {}
        started = (context.get("lifecycle", {}) or {}).get("mujoco_process_started_monotonic_ns")
        if started is not None:
            return {"created": True, "reason": "run_context.lifecycle.mujoco_process_started_monotonic_ns",
                    "reference": str(context_path)}
    mujoco_log = run_dir / "mujoco.log"
    if mujoco_log.is_file():
        try:
            head = mujoco_log.read_text(encoding="utf-8", errors="replace")[:20000]
        except OSError:
            head = ""
        if any(marker in head for marker in MUJOCO_START_MARKERS):
            return {"created": True, "reason": "mujoco.log startup marker",
                    "reference": str(mujoco_log)}
    return {"created": None, "reason": "no independent scene-start marker",
            "reference": str(run_dir)}


def inspect_attempt(attempt_dir: Path, stdout_result: dict[str, Any] | None,
                    returncode: int | None, *, interrupted: bool = False) -> dict[str, Any]:
    """Derive scene/run facts from artifacts; never infer from path alone."""
    result_root = attempt_dir / "result"
    run_directories = _find_run_directories(result_root)
    if not run_directories:
        console_log = attempt_dir / "console.log"
        console_head = ""
        if console_log.is_file():
            try:
                console_head = console_log.read_text(encoding="utf-8", errors="replace")[:20000]
            except OSError:
                console_head = ""
        if any(marker in console_head for marker in MUJOCO_START_MARKERS):
            return {
                "scene_created": True, "run_directory": None, "run_id": None,
                "entered_rl": None, "raw_terminal": None, "record_validity": "UNKNOWN",
                "scene_evidence": {"created": True,
                                   "reason": "console log shows a MuJoCo start marker but no run directory/run_id",
                                   "reference": str(console_log)},
                "run_context": None, "terminal": None, "metrics": None, "process_facts": None,
                "run_id_matches_directory": None, "facts_complete": False,
            }
        preflight_result = (stdout_result or {}).get("result")
        if preflight_result in PRE_SCENE_REJECTIONS:
            return {
                "scene_created": False, "run_directory": None, "run_id": None,
                "entered_rl": False, "raw_terminal": None, "record_validity": "NOT_APPLICABLE",
                "scene_evidence": {"created": False, "reason": f"explicit pre-scene rejection:{preflight_result}",
                                   "reference": "diagnostic stdout"},
                "run_context": None, "terminal": None, "metrics": None, "process_facts": None,
                "run_id_matches_directory": None, "facts_complete": True,
            }
        # A missing result directory is not proof that no scene was created.
        return {
            "scene_created": None, "run_directory": None, "run_id": None,
            "entered_rl": None, "raw_terminal": None, "record_validity": "UNKNOWN",
            "scene_evidence": {"created": None,
                               "reason": "no result directory and no explicit pre-scene rejection"
                                         + ("; interrupted" if interrupted else ""),
                               "reference": str(attempt_dir)},
            "run_context": None, "terminal": None, "metrics": None, "process_facts": None,
            "run_id_matches_directory": None, "facts_complete": False,
        }
    if len(run_directories) > 1:
        return {
            "scene_created": None, "run_directory": None, "run_id": None,
            "entered_rl": None, "raw_terminal": None, "record_validity": "UNKNOWN",
            "scene_evidence": {"created": None, "reason": "multiple run directories under one attempt",
                               "reference": str(result_root)},
            "run_context": None, "terminal": None, "metrics": None, "process_facts": None,
            "run_id_matches_directory": None, "facts_complete": False,
        }

    run_dir = run_directories[0]
    marker = _scene_marker(run_dir)
    if marker["created"] is not True:
        return {
            "scene_created": None, "run_directory": str(run_dir), "run_id": run_dir.name,
            "entered_rl": None, "raw_terminal": None, "record_validity": "UNKNOWN",
            "scene_evidence": marker, "run_context": None, "terminal": None,
            "metrics": None, "process_facts": None,
            "run_id_matches_directory": None, "facts_complete": False,
        }

    context = read_json(run_dir / "run_context.json") if (run_dir / "run_context.json").is_file() else None
    terminal = read_json(run_dir / "terminal.json") if (run_dir / "terminal.json").is_file() else None
    metrics = read_json(run_dir / "metrics.json") if (run_dir / "metrics.json").is_file() else None
    process_facts = read_json(run_dir / "process_facts.json") if (run_dir / "process_facts.json").is_file() else None

    stdout_run_id = (stdout_result or {}).get("run_id")
    run_id_matches = None if stdout_run_id is None else str(stdout_run_id) == run_dir.name
    lifecycle = (context or {}).get("lifecycle", {})
    task_start = lifecycle.get("task_start") or lifecycle.get("policy_active_start")
    entered_rl = True if isinstance(task_start, dict) and isinstance(task_start.get("rl_step"), int) else None
    if entered_rl is None and isinstance(metrics, dict):
        start_cycle = (metrics.get("rl_task_metric_boundary") or {}).get("start_cycle")
        if isinstance(start_cycle, int):
            entered_rl = True
    if entered_rl is None and isinstance(terminal, dict) and \
            terminal.get("run_terminal_result") in {"PREPARATION_FAILURE", "STARTUP_FAILURE"}:
        entered_rl = False
    if entered_rl is None and isinstance(context, dict) and \
            (context.get("result_contract", {}) or {}).get("classification") in {
                "PREPARATION_FAILURE", "STARTUP_FAILURE", "SCENE_PREFLIGHT_REJECTED"}:
        entered_rl = False

    context_terminal = ((context or {}).get("result_contract", {}) or {}).get("classification")
    record_terminal = (terminal or {}).get("run_terminal_result")
    metric_terminal = (metrics or {}).get("navigation_terminal_result") or \
                      (metrics or {}).get("terminal_result")
    terminal_sources = {"context": context_terminal, "record": record_terminal,
                        "derived": metric_terminal}
    known_terminals = {value for value in terminal_sources.values()
                       if isinstance(value, str) and value != "UNKNOWN"}
    raw_terminal = record_terminal or context_terminal or metric_terminal
    terminal_conflict = len(known_terminals) > 1
    validity = (metrics or {}).get("record_validity", "UNKNOWN")
    safety = ((metrics or {}).get("safety_task_completion", {}) or {}).get("status", "UNKNOWN")

    evidence = {
        "scene_created": True, "marker": marker,
        "lifecycle_task_start_cycle": (task_start or {}).get("rl_step") if isinstance(task_start, dict) else None,
        "terminal_sources": terminal_sources, "terminal_conflict": terminal_conflict,
        "process_facts": {
            "exit_code": (process_facts or {}).get("exit_code"),
            "shutdown_complete": (process_facts or {}).get("shutdown_complete"),
            "closeout_complete": (process_facts or {}).get("closeout_complete"),
            "forced_termination": (process_facts or {}).get("forced_termination"),
        },
        "terminal_cleanup": {
            "normal_shutdown": (terminal or {}).get("normal_shutdown"),
            "shutdown_complete": (terminal or {}).get("shutdown_complete"),
            "process_exit_code": (terminal or {}).get("process_exit_code"),
        },
        "standing_and_cleanup": ((metrics or {}).get("safety_task_completion", {}) or {}).get(
            "standing_and_cleanup", "UNKNOWN"),
        "run_directory_inside_attempt": True,
    }
    return {
        "scene_created": True, "run_directory": str(run_dir), "run_id": run_dir.name,
        "entered_rl": entered_rl, "raw_terminal": raw_terminal,
        "record_validity": validity, "safety_task_completion": safety,
        "scene_evidence": evidence, "run_context": context, "terminal": terminal,
        "metrics": metrics, "process_facts": process_facts,
        "run_id_matches_directory": run_id_matches, "facts_complete": not terminal_conflict,
        "terminal_sources": terminal_sources, "terminal_conflict": terminal_conflict,
    }


def cause_evidence_from_facts(facts: dict[str, Any], *, stdout_result: dict[str, Any] | None) -> dict[str, Any]:
    if facts.get("scene_created") is False:
        return {"class": "pre_scene_preflight_rejected",
                "references": [str(facts.get("scene_evidence", {}).get("reference"))],
                "independent_infrastructure_corroboration": False}
    terminal = facts.get("raw_terminal")
    context = facts.get("run_context") or {}
    events = (facts.get("terminal") or {}).get("external_safety_events") or []
    if terminal == "SYSTEM_SAFETY_ABORT":
        references = ["terminal:SYSTEM_SAFETY_ABORT"]
        infrastructure_observed = False
        for event in events:
            diagnostics = event.get("ray_read_diagnostics") if isinstance(event, dict) else None
            if diagnostics:
                infrastructure_observed = True
                references.append("terminal.external_safety_events.ray_read_diagnostics")
        return {
            "class": "infrastructure_trigger_observed" if infrastructure_observed else "safety_veto_trigger_unclassified",
            "references": references,
            # A read-side trigger does not prove the physical publisher was healthy.
            "independent_infrastructure_corroboration": False,
        }
    if facts.get("entered_rl") is False:
        return {"class": "preparation_failure_observed",
                "references": [str(facts.get("run_directory"))],
                "independent_infrastructure_corroboration": False}
    if facts.get("record_validity") not in {"VALID", None}:
        return {"class": "record_failure_observed", "references": [str(facts.get("run_directory"))],
                "independent_infrastructure_corroboration": False}
    return {"class": "none", "references": [], "independent_infrastructure_corroboration": False}


# ---------------------------------------------------------------------------
# Dispatch planning, run, status
# ---------------------------------------------------------------------------

def _attempt_dir(batch_root: Path, plan_item_id: str, invocation: int) -> Path:
    return batch_root / "runs" / plan_item_id / f"attempt_{invocation:02d}"


def _attempt_id(batch_id: str, plan_item_id: str, invocation: int) -> str:
    return f"{batch_id}:{plan_item_id}:inv{invocation:02d}"


def _ordered_attempts(attempts: list[dict[str, Any]], plan_item_id: str) -> list[dict[str, Any]]:
    rows = [row for row in attempts if row.get("plan_item_id") == plan_item_id]
    return sorted(rows, key=lambda row: (int(row.get("invocation_number", 0)),
                                         str(row.get("attempt_id"))))


def _replacement_entry(authorization: dict[str, Any], plan_item_id: str) -> dict[str, Any] | None:
    entries = [entry for entry in authorization.get("replacements", [])
               if entry.get("plan_item_id") == plan_item_id]
    return entries[0] if len(entries) == 1 else None


def plan_dispatch(plan: list[dict[str, Any]], ledger: Ledger, authorization: dict[str, Any], *,
                  reconcile: Callable[[dict[str, Any]], dict[str, Any]] | None = None,
                  allow_pre_scene_retry: bool = True) -> dict[str, Any]:
    """Return the next action for each plan item in frozen global order."""
    attempts = ledger.attempts
    actions: list[dict[str, Any]] = []
    for record in sorted(plan, key=lambda item: int(item["global_order"])):
        plan_item_id = record["plan_item_id"]
        rows = _ordered_attempts(attempts, plan_item_id)
        if not rows:
            actions.append({"plan_item_id": plan_item_id, "action": "DISPATCH_NEW"})
            continue

        # Re-inspect attempts whose scene-creation fact is not yet known.  A
        # still-unknown fact blocks the batch; it is never auto-retried.
        unknown = [row for row in rows if row.get("scene_created") not in (True, False)]
        if unknown:
            if reconcile is None:
                actions.append({"plan_item_id": plan_item_id, "action": "PAUSE",
                                "reason": "scene_creation_unknown"})
                continue
            resolved = True
            for row in unknown:
                updated = reconcile(row)
                ledger.upsert_attempt(updated)
                if updated.get("scene_created") not in (True, False):
                    resolved = False
            if not resolved:
                actions.append({"plan_item_id": plan_item_id, "action": "PAUSE",
                                "reason": "scene_creation_unknown"})
                continue
            rows = _ordered_attempts(ledger.attempts, plan_item_id)

        from formal_attempt_accounting import classify_attempt

        scene_rows = [row for row in rows if row.get("scene_created") is True]
        selected = None
        for row in scene_rows:
            facts = classify_attempt(row)
            if row.get("is_replacement") is True:
                if facts.get("status") == "VALID_TASK_RESULT" and row.get("replacement_authorized"):
                    selected = row
            elif facts.get("status") == "VALID_TASK_RESULT":
                selected = row
                break
        if selected is not None:
            actions.append({"plan_item_id": plan_item_id, "action": "SKIP_COMPLETED",
                            "attempt_id": selected.get("attempt_id"),
                            "terminal": selected.get("raw_terminal")})
            continue

        if scene_rows:
            last = scene_rows[-1]
            facts = classify_attempt(last)
            if any(row.get("is_replacement") is True for row in scene_rows):
                actions.append({"plan_item_id": plan_item_id, "action": "PAUSE",
                                "reason": "replacement_attempt_unresolved"})
                continue
            if not facts.get("replacement_eligible"):
                actions.append({"plan_item_id": plan_item_id, "action": "PAUSE",
                                "reason": f"scene_attempt_unresolved:{facts.get('status')}"})
                continue
            entry = _replacement_entry(authorization, plan_item_id)
            if entry is None:
                actions.append({"plan_item_id": plan_item_id, "action": "PAUSE",
                                "reason": "replacement_not_authorized"})
                continue
            check = validate_replacement_entry(entry, plan_item_id=plan_item_id, attempts=ledger.attempts)
            if check["status"] != "PASS":
                actions.append({"plan_item_id": plan_item_id, "action": "PAUSE",
                                "reason": "replacement_authorization_rejected",
                                "errors": check["errors"]})
                continue
            actions.append({"plan_item_id": plan_item_id, "action": "DISPATCH_REPLACEMENT",
                            "replaces_attempt_id": entry.get("replaces_attempt_id"),
                            "authorization_id": entry.get("authorization_id"),
                            "authorization_entry": entry})
            continue

        # Only confirmed pre-scene failures may be re-invoked, and only a
        # bounded number of times; they do not consume scene-start quota.
        invocations = len(rows)
        if not allow_pre_scene_retry:
            actions.append({"plan_item_id": plan_item_id, "action": "PAUSE",
                            "reason": "pre_scene_failure_no_auto_retry"})
        elif invocations >= MAX_PRE_SCENE_INVOCATIONS:
            actions.append({"plan_item_id": plan_item_id, "action": "PAUSE",
                            "reason": "pre_scene_invocation_limit"})
        else:
            actions.append({"plan_item_id": plan_item_id, "action": "DISPATCH_NEW"})
    return {"actions": actions}


def runtime_consumption_evidence(record: dict[str, Any], context: dict[str, Any]) -> dict[str, Any]:
    errors = (runtime_consumption_errors(record, context)
              if isinstance(context, dict) and context else ["run_context_missing"])
    observed = (context.get("runtime_config_observed") or {}
                if isinstance(context, dict) else {})
    validation = (context.get("runtime_source_validation") or {}
                  if isinstance(context, dict) else {})
    preflight_context = (context.get("preflight") or {}
                         if isinstance(context, dict) else {})
    return {
        "status": "CONFIRMED" if not errors else "MISMATCH_OR_UNKNOWN",
        "errors": errors,
        "observed": {
            key: observed.get(key) for key in (
                "group", "entry_threshold", "exit_threshold", "hysteresis",
                "hold", "recovery_min_hold_s", "switching_mode", "session_id")
        },
        "source_binding": observed.get("source_binding"),
        "runtime_source_validation": {
            "status": validation.get("status"),
            "reason": validation.get("reason"),
            "evidence": validation.get("evidence"),
        },
        "preflight": {
            key: preflight_context.get(key) for key in (
                "candidate_config_group_verified",
                "candidate_config_entry_E_verified",
                "candidate_config_goal_x_m_verified",
                "candidate_config_legacy_fields_absent",
                "candidate_config_path",
                "collision_reader_scene_binding")
        },
        "scope": "controller/source events recorded in run_context; not a static parse",
    }


def reconcile_attempt_from_disk(batch_root: Path, row: dict[str, Any],
                                record: dict[str, Any] | None = None) -> dict[str, Any]:
    """Re-inspect one durable attempt; never infer a scene from a bare path."""
    output_directory = row.get("output_directory")
    if not output_directory:
        return row
    attempt_dir = Path(str(output_directory)).parent
    try:
        attempt_dir.resolve().relative_to(batch_root.resolve())
    except ValueError:
        return {**row, "runner_status": "RECONCILE_REJECTED_OUTSIDE_BATCH"}
    facts = inspect_attempt(attempt_dir, None, None)
    context = facts.get("run_context") or {}
    settings = context.get("settings", {}) if isinstance(context, dict) else {}
    lifecycle = context.get("lifecycle", {}) if isinstance(context, dict) else {}
    task_start = lifecycle.get("task_start") or lifecycle.get("policy_active_start") or {}
    setting_errors = (runtime_setting_errors(record, settings)
                      if record is not None and facts.get("scene_created") is True else [])
    updated = {
        **row,
        "scene_created": facts.get("scene_created"),
        "run_id": facts.get("run_id"),
        "session_id": (context.get("session_id") if isinstance(context, dict) else None)
                      or (task_start.get("session_id") if isinstance(task_start, dict) else None),
        "entered_rl": facts.get("entered_rl"),
        "raw_terminal": facts.get("raw_terminal"),
        "record_validity": facts.get("record_validity", "UNKNOWN"),
        "run_directory": facts.get("run_directory"),
        "safety_task_completion": facts.get("safety_task_completion", "UNKNOWN"),
        "standing_and_cleanup": (facts.get("scene_evidence", {}) or {}).get(
            "standing_and_cleanup", "UNKNOWN"),
        "actual_run_context_settings": {
            key: settings.get(key) for key in (
                "group", "entry_threshold", "exit_threshold", "hysteresis",
                "hold", "recovery_min_hold_s", "goal_world_m",
                "arrival_threshold_m", "diagnostic_sim_limit_after_motion_s",
                "standing_observation_sim_s", "scene_id",
                "controller_config", "overlay")
        } if settings else None,
        "runtime_setting_mismatch": setting_errors,
        "runtime_config_consumption": (runtime_consumption_evidence(record, context)
                                       if record is not None and
                                       facts.get("scene_created") is True else None),
        "scene_evidence": facts.get("scene_evidence"),
        "terminal_sources": facts.get("terminal_sources"),
        "terminal_conflict": facts.get("terminal_conflict"),
        "run_id_matches_directory": (
            True if facts.get("run_id") and facts.get("run_directory")
            and Path(facts["run_directory"]).name == facts["run_id"] else
            facts.get("run_id_matches_directory")),
        "process_facts": facts.get("process_facts") or {},
        "cause_evidence": cause_evidence_from_facts(facts, stdout_result=None),
        "runner_status": ("RECONCILED_PRE_SCENE_FAILURE" if facts.get("scene_created") is False else
                          "RECONCILED_SCENE_STARTED" if facts.get("scene_created") is True else
                          "RECONCILE_SCENE_CREATION_UNKNOWN"),
    }
    return updated


def runtime_setting_errors(record: dict[str, Any], settings: dict[str, Any]) -> list[str]:
    """Check the run context's recorded group/scene/threshold semantics."""
    if not settings:
        return ["run_context_settings_missing"]
    errors: list[str] = []
    group = record["group"]
    expected = GROUP_PARAMETERS[group]
    if settings.get("group") != group:
        errors.append(f"group:{settings.get('group')}!={group}")
    if settings.get("scene_id") != record.get("scene_id"):
        errors.append(f"scene_id:{settings.get('scene_id')}!={record.get('scene_id')}")
    goal = settings.get("goal_world_m")
    if not (isinstance(goal, list) and len(goal) == 2
            and abs(float(goal[0]) - float(record["goal_x_m"])) <= 1e-9
            and abs(float(goal[1]) - float(record["goal_y_m"])) <= 1e-9):
        errors.append(f"goal_world_m:{goal}")
    if not isinstance(settings.get("entry_threshold"), (int, float)) or \
            abs(float(settings["entry_threshold"]) - expected["E"]) > 1e-9:
        errors.append(f"entry_threshold:{settings.get('entry_threshold')}")
    expected_exit = expected["X"] if group in {"B", "D"} else expected["E"]
    if not isinstance(settings.get("exit_threshold"), (int, float)) or \
            abs(float(settings["exit_threshold"]) - float(expected_exit)) > 1e-9:
        errors.append(f"exit_threshold:{settings.get('exit_threshold')}")
    expected_hold = expected["H"] if group in {"C", "D"} else 0.0
    if not isinstance(settings.get("recovery_min_hold_s"), (int, float)) or \
            abs(float(settings["recovery_min_hold_s"]) - float(expected_hold)) > 1e-9:
        errors.append(f"recovery_min_hold_s:{settings.get('recovery_min_hold_s')}")
    if settings.get("hysteresis") != ("true" if group in {"B", "D"} else "false"):
        errors.append(f"hysteresis:{settings.get('hysteresis')}")
    if settings.get("hold") != ("true" if group in {"C", "D"} else "false"):
        errors.append(f"hold:{settings.get('hold')}")
    return errors


def runtime_consumption_errors(record: dict[str, Any], context: dict[str, Any]) -> list[str]:
    """Check controller/source-consumption evidence recorded during the run."""
    if not isinstance(context, dict) or not context:
        return ["run_context_missing"]
    observed = context.get("runtime_config_observed") or {}
    validation = context.get("runtime_source_validation") or {}
    preflight = context.get("preflight") or {}
    group = record["group"]
    expected = GROUP_PARAMETERS[group]
    errors: list[str] = []
    if validation.get("status") != "confirmed":
        errors.append(f"runtime_source_validation:{validation.get('status')}")
    if validation.get("reason") != "all_source_events_match":
        errors.append(f"runtime_source_reason:{validation.get('reason')}")
    if observed.get("group") != group:
        errors.append(f"observed_group:{observed.get('group')}!={group}")
    expected_exit = expected["X"] if group in {"B", "D"} else expected["E"]
    expected_hold = expected["H"] if group in {"C", "D"} else 0.0
    checks = {
        "entry_threshold": (expected["E"], 1e-6),
        "exit_threshold": (float(expected_exit), 1e-6),
        "recovery_min_hold_s": (float(expected_hold), 1e-6),
    }
    for key, (value, tolerance) in checks.items():
        actual = observed.get(key)
        if not isinstance(actual, (int, float)) or abs(float(actual) - value) > tolerance:
            errors.append(f"observed_{key}:{actual}!={value}")
    if observed.get("hysteresis") is not (group in {"B", "D"}):
        errors.append(f"observed_hysteresis:{observed.get('hysteresis')}")
    if observed.get("hold") is not (group in {"C", "D"}):
        errors.append(f"observed_hold:{observed.get('hold')}")
    if preflight.get("candidate_config_group_verified") != group:
        errors.append(f"preflight_group:{preflight.get('candidate_config_group_verified')}")
    if preflight.get("candidate_config_entry_E_verified") is not True:
        errors.append("preflight_entry_not_verified")
    if preflight.get("candidate_config_goal_x_m_verified") is not True:
        errors.append("preflight_goal_not_verified")
    if preflight.get("candidate_config_legacy_fields_absent") is not True:
        errors.append("preflight_legacy_fields_present_or_unknown")
    binding = observed.get("source_binding") or {}
    if not binding.get("controller_pid") or not binding.get("writer_pid"):
        errors.append("source_binding_pid_missing")
    if binding.get("group") != group:
        errors.append(f"source_binding_group:{binding.get('group')}")
    return errors


def _count_scene_starts(attempts: list[dict[str, Any]]) -> int:
    return sum(row.get("scene_created") is True for row in attempts)


def block_complete(audit: dict[str, Any], record: dict[str, Any],
                   plan: list[dict[str, Any]]) -> bool:
    item_ids = {item["plan_item_id"] for item in plan
                if item.get("display_name") == record.get("display_name")
                and item.get("repeat_block") == record.get("repeat_block")}
    rows = [row for row in audit.get("plan_items", [])
            if row.get("plan_item_id") in item_ids]
    return (len(item_ids) == 4 and len(rows) == 4
            and all(row.get("plan_status") == "SELECTED_RESULT" for row in rows))


def _count_replacements(attempts: list[dict[str, Any]]) -> int:
    return sum(row.get("scene_created") is True and row.get("is_replacement") is True
               for row in attempts)


def _write_batch_manifest(batch_root: Path, *, batch_id: str, binding: dict[str, Any],
                          authorization: dict[str, Any], authorization_path: Path,
                          overlay_report: dict[str, Any],
                          limits: dict[str, Any] | None = None,
                          data_class: str = "FORMAL_240",
                          classification: str = FORMAL_CLASSIFICATION) -> dict[str, Any]:
    manifest = {
        "schema": MANIFEST_SCHEMA,
        "batch_id": batch_id,
        "data_class": data_class,
        "classification": classification,
        "created_utc": utc_now(),
        "matrix": binding,
        "authorization": {
            "authorization_id": authorization.get("authorization_id"),
            "sha256": sha256_file(authorization_path),
            "director_review": authorization.get("director_review"),
            "authorization_basis": authorization.get("authorization_basis"),
            "fixture_only": authorization.get("fixture_only") is True,
        },
        "groups": GROUP_PARAMETERS,
        "group_overlays": overlay_report,
        "cli_compatibility_only": {
            "exit_threshold": CLI_FALLBACK_EXIT_X,
            "hold_s": CLI_FALLBACK_HOLD_S,
            "scope": "shared CLI validation only; A/C do not write X and A/B do not write H",
        },
        "limits": limits or {
            "base_scene_starts": BASE_SCENE_STARTS,
            "max_substitutions": MAX_SUBSTITUTIONS,
            "max_total_scene_starts": MAX_TOTAL_SCENE_STARTS,
            "batch_budget_gib": BATCH_BUDGET_GIB,
            "min_free_gib": MIN_FREE_GIB,
        },
        "version_record": version_record(),
        "scope": "thin batch orchestration around existing production single-run diagnostic",
    }
    write_json_atomic(batch_root / "batch_manifest.json", manifest)
    return manifest


def _batch_state(batch_root: Path) -> dict[str, Any]:
    path = batch_root / "batch_state.json"
    if path.is_file():
        return read_json(path)
    return {"schema": STATE_SCHEMA, "status": "NOT_STARTED", "attempts_completed": 0}


def _save_batch_state(batch_root: Path, state: dict[str, Any]) -> None:
    state = {**state, "schema": STATE_SCHEMA, "updated_utc": utc_now()}
    write_json_atomic(batch_root / "batch_state.json", state)


def _scene_fact_unknown(attempt: dict[str, Any]) -> bool:
    return attempt.get("scene_created") not in (True, False)


def run_batch(*, batch_root: Path, batch_id: str, plan: list[dict[str, Any]],
              meta: dict[str, Any], binding: dict[str, Any],
              authorization: dict[str, Any], authorization_path: Path,
              executor: Callable[[list[str], Path], dict[str, Any]],
              analyze_run: Callable[[Path], dict[str, Any]] | None = None,
              disk_probe: Callable[[], float] = free_gib_default,
              allow_fixture: bool = False, resume: bool = False,
              require_full_plan: bool = True, prepare_overlays: bool = True,
              overlay_source: Path = OVERLAY_SOURCE,
              overlay_preparer: Callable[[Path], dict[str, Any]] | None = None,
              max_new_starts: int | None = None,
              plan_validator: Callable[[list[dict[str, Any]], dict[str, Any]], dict[str, Any]] | None = None,
              authorization_validator: Callable[[dict[str, Any], dict[str, Any], str], dict[str, Any]] | None = None,
              max_substitutions: int = MAX_SUBSTITUTIONS,
              max_per_plan_item: int = 1,
              scene_start_cap: int = MAX_TOTAL_SCENE_STARTS,
              stop_after_scene_starts: int | None = None,
              allow_pre_scene_retry: bool = True,
              limits: dict[str, Any] | None = None,
              data_class: str = "FORMAL_240",
              classification: str = FORMAL_CLASSIFICATION,
              display: str = ":0", xauthority: str | None = None) -> dict[str, Any]:
    """Run or resume one batch; every scene start requires explicit authorization."""
    auth_check = (authorization_validator(authorization, binding, batch_id)
                  if authorization_validator else
                  validate_authorization(authorization, binding, batch_id=batch_id,
                                         allow_fixture=allow_fixture))
    if auth_check["status"] != "PASS":
        return {"status": "REFUSED", "reason": "authorization_rejected",
                "authorization": auth_check, "dispatched": 0}
    if allow_fixture and meta.get("matrix_id") != "S7-OFFLINE-FIXTURE":
        return {"status": "REFUSED", "reason": "fixture_authorization_scope_mismatch",
                "dispatched": 0}
    validation = (plan_validator(plan, meta) if plan_validator
                  else validate_plan(plan, meta, require_full=require_full_plan))
    if validation["status"] != "PASS":
        return {"status": "REFUSED", "reason": "plan_validation_failed",
                "plan_validation": validation, "dispatched": 0}
    limit_free_gib = float((limits or {}).get("min_free_gib", MIN_FREE_GIB))
    limit_batch_gib = float((limits or {}).get("batch_budget_gib", BATCH_BUDGET_GIB))

    batch_root = batch_root.resolve()
    existing = ((batch_root / "batch_manifest.json").is_file()
                or (batch_root / "ledger" / "attempts.json").is_file())
    if existing and not resume:
        return {"status": "REFUSED", "reason": "existing_batch_requires_explicit_resume",
                "batch_root": str(batch_root), "dispatched": 0}
    batch_root.mkdir(parents=True, exist_ok=True)
    try:
        with batch_lock(batch_root):
            if not (batch_root / "batch_manifest.json").is_file():
                overlay_report = (overlay_preparer(batch_root) if overlay_preparer
                                  else prepare_group_overlays(
                                      batch_root, source=overlay_source,
                                      groups=sorted({item["group"] for item in plan})))
                manifest = _write_batch_manifest(
                    batch_root, batch_id=batch_id, binding=binding,
                    authorization=authorization, authorization_path=authorization_path,
                    overlay_report=overlay_report, limits=limits,
                    data_class=data_class, classification=classification)
            else:
                manifest = read_json(batch_root / "batch_manifest.json")
                if manifest.get("matrix", {}).get("plan_sha256") != binding.get("plan_sha256"):
                    return {"status": "REFUSED", "reason": "batch_plan_binding_mismatch",
                            "dispatched": 0}
                if manifest.get("matrix", {}).get("matrix_id") != binding.get("matrix_id"):
                    return {"status": "REFUSED", "reason": "batch_matrix_id_mismatch",
                            "dispatched": 0}
                differences = version_binding_changes(manifest)
                if differences:
                    _save_batch_state(batch_root, {**_batch_state(batch_root),
                                                   "status": "PAUSED",
                                                   "pause_reason": "version_binding_changed",
                                                   "version_differences": differences})
                    return {"status": "PAUSED", "reason": "version_binding_changed",
                            "pause_reason": "version_binding_changed",
                            "version_differences": differences, "dispatched": 0}

            running = scan_batch_processes(batch_root)
            if running:
                ledger = Ledger(batch_root / "ledger" / "attempts.json", batch_id)
                ledger.event("pause_batch_processes_still_running", processes=running)
                _save_batch_state(batch_root, {**_batch_state(batch_root),
                                               "status": "PAUSED",
                                               "pause_reason": "batch_processes_still_running"})
                return {"status": "PAUSED", "reason": "batch_processes_still_running",
                        "pause_reason": "batch_processes_still_running",
                        "processes": running, "dispatched": 0}

            ledger = Ledger(batch_root / "ledger" / "attempts.json", batch_id)
            state = _batch_state(batch_root)
            initial_audit = audit_ledger(plan, ledger.attempts, max_substitutions=max_substitutions, max_per_plan_item=max_per_plan_item)
            replacement_errors = validate_replacement_authorizations(
                ledger.attempts, authorization)
            if replacement_errors:
                initial_audit = {**initial_audit, "status": "REJECTED",
                                 "errors": sorted(set(list(initial_audit.get("errors", []))
                                                      + replacement_errors))}
            if initial_audit.get("status") != "PASS":
                ledger.event("ledger_audit_rejected_on_entry", errors=initial_audit.get("errors"))
                _save_batch_state(batch_root, {**state, "status": "PAUSED",
                                               "pause_reason": "ledger_audit_rejected_on_entry",
                                               "ledger_errors": initial_audit.get("errors")})
                return {"status": "PAUSED", "reason": "ledger_audit_rejected_on_entry",
                        "pause_reason": "ledger_audit_rejected_on_entry",
                        "dispatched": 0, "audit": initial_audit}
            free_gib = float(disk_probe())
            budget_gib = directory_bytes(batch_root) / (1024 ** 3)
            if free_gib < limit_free_gib:
                ledger.event("pause_disk_guard", free_gib=free_gib,
                             min_free_gib=limit_free_gib)
                _save_batch_state(batch_root, {**state, "status": "PAUSED",
                                               "pause_reason": "free_space_below_guard",
                                               "free_gib": free_gib})
                return {"status": "PAUSED", "reason": "free_space_below_guard",
                        "pause_reason": "free_space_below_guard",
                        "free_gib": free_gib, "dispatched": 0,
                        "audit": audit_ledger(plan, ledger.attempts,
                                              max_substitutions=max_substitutions,
                                              max_per_plan_item=max_per_plan_item)}
            if budget_gib > limit_batch_gib:
                ledger.event("pause_batch_budget", batch_gib=budget_gib,
                             batch_budget_gib=limit_batch_gib)
                _save_batch_state(batch_root, {**state, "status": "PAUSED",
                                               "pause_reason": "batch_budget_exceeded",
                                               "batch_gib": budget_gib})
                return {"status": "PAUSED", "reason": "batch_budget_exceeded",
                        "pause_reason": "batch_budget_exceeded",
                        "batch_gib": budget_gib, "dispatched": 0,
                        "audit": audit_ledger(plan, ledger.attempts,
                                              max_substitutions=max_substitutions,
                                              max_per_plan_item=max_per_plan_item)}

            counts = Counter()
            dispatched = 0
            scene_starts_this_entry = 0
            pause_reason = None
            staged_reason = None
            plan_by_id = {item["plan_item_id"]: item for item in plan}
            reconcile = lambda row: reconcile_attempt_from_disk(
                batch_root, row, plan_by_id.get(row.get("plan_item_id")))  # noqa: E731
            while pause_reason is None:
                plan_state = plan_dispatch(plan, ledger, authorization, reconcile=reconcile,
                                           allow_pre_scene_retry=allow_pre_scene_retry)
                pending = [item for item in plan_state["actions"]
                           if item["action"].startswith("DISPATCH_")]
                pause_items = [item for item in plan_state["actions"] if item["action"] == "PAUSE"]
                if pause_items:
                    pause_reason = pause_items[0].get("reason", "paused")
                    break
                if not pending:
                    break
                if stop_after_scene_starts is not None and \
                        scene_starts_this_entry >= stop_after_scene_starts:
                    staged_reason = f"staged_after_{scene_starts_this_entry}_scene_start"
                    break

                # Preserve frozen order: the first unresolved item decides.
                action = pending[0]
                plan_item_id = action["plan_item_id"]
                record = plan_by_id[plan_item_id]
                rows = _ordered_attempts(ledger.attempts, plan_item_id)
                scene_starts = _count_scene_starts(ledger.attempts)
                replacements = _count_replacements(ledger.attempts)
                if (max_new_starts is not None and dispatched >= max_new_starts) or \
                        scene_starts >= scene_start_cap:
                    pause_reason = "scene_start_cap_reached"
                    break
                if action["action"] == "DISPATCH_REPLACEMENT" and \
                        replacements >= max_substitutions:
                    pause_reason = "substitution_quota_exhausted"
                    break
                free_gib = float(disk_probe())
                budget_gib = directory_bytes(batch_root) / (1024 ** 3)
                if free_gib < limit_free_gib:
                    pause_reason = "free_space_below_guard"
                    break
                if budget_gib > limit_batch_gib:
                    pause_reason = "batch_budget_exceeded"
                    break

                invocation = (max((int(row.get("invocation_number", 0)) for row in rows), default=0) + 1)
                attempt_dir = _attempt_dir(batch_root, plan_item_id, invocation)
                if attempt_dir.exists():
                    pause_reason = "attempt_directory_already_exists"
                    break
                attempt_dir.mkdir(parents=True, exist_ok=False)
                is_replacement = action["action"] == "DISPATCH_REPLACEMENT"
                overlay_dir = batch_root / "overlays" / record["group"]
                overlay_config = overlay_dir / "share/go2_description/config/abs/config.yaml"
                attempt = {
                    "attempt_id": _attempt_id(batch_id, plan_item_id, invocation),
                    "plan_item_id": plan_item_id,
                    "group": record["group"],
                    "map": record["display_name"],
                    "scene": record["filename"],
                    "repeat_block": record["repeat_block"],
                    "global_order": record["global_order"],
                    "block_order": record["block_order"],
                    "within_block_position": record["within_block_position"],
                    "williams_sequence_id": record["williams_sequence_id"],
                    "invocation_number": invocation,
                    "kind": "scenario_invocation",
                    "data_class": data_class,
                    "scene_created": None,
                    "run_id": None,
                    "session_id": None,
                    "entered_rl": None,
                    "raw_terminal": None,
                    "record_validity": "UNKNOWN",
                    "output_directory": str(attempt_dir / "result"),
                    "run_directory": None,
                    "overlay": str(overlay_dir),
                    "overlay_config_sha256": sha256_file(overlay_config),
                    "actual_run_context_settings": None,
                    "is_replacement": is_replacement,
                    "replaces_attempt_id": action.get("replaces_attempt_id"),
                    "replacement_authorized": is_replacement,
                    "replacement_authorization_id": action.get("authorization_id"),
                    "replacement_authorization_entry": (dict(action.get("authorization_entry"))
                                                        if action.get("authorization_entry") else None),
                    "cause_evidence": {"class": "not_yet_known", "references": [],
                                       "independent_infrastructure_corroboration": False},
                    "safety_task_completion": "UNKNOWN",
                    "standing_and_cleanup": "UNKNOWN",
                    "runner_status": "INTENT",
                    "launch_intent": {
                        "recorded_utc": utc_now(),
                        "command": None,
                        "authorization_file_sha256": sha256_file(authorization_path),
                        "overlay": str(overlay_dir),
                        "group_parameters": GROUP_PARAMETERS[record["group"]],
                        "cli_compatibility_only": {
                            "exit_threshold": CLI_FALLBACK_EXIT_X,
                            "hold_s": CLI_FALLBACK_HOLD_S,
                        },
                    },
                }
                ledger.upsert_attempt(attempt)
                command = build_diagnostic_command(
                    record, overlay=overlay_dir,
                    output_root=attempt_dir / "result", display=display,
                    xauthority=xauthority, classification=classification)
                attempt["launch_intent"]["command"] = command
                ledger.upsert_attempt(attempt)
                console_log = attempt_dir / "console.log"
                try:
                    outcome = executor(command, console_log)
                except KeyboardInterrupt:
                    raise
                except Exception as error:  # noqa: BLE001 - record and pause, never guess
                    outcome = {"stdout_result": None, "returncode": None,
                               "interrupted": False,
                               "launch_error": f"{type(error).__name__}: {error}"}
                dispatched += 1

                facts = inspect_attempt(attempt_dir, outcome.get("stdout_result"),
                                        outcome.get("returncode"),
                                        interrupted=bool(outcome.get("interrupted")))
                run_dir = Path(facts["run_directory"]) if facts.get("run_directory") else None
                analysis_result = None
                if run_dir is not None and analyze_run is not None and \
                        not (run_dir / "metrics.json").is_file():
                    analysis_result = analyze_run(run_dir)
                    if (run_dir / "metrics.json").is_file():
                        facts = inspect_attempt(attempt_dir, outcome.get("stdout_result"),
                                                outcome.get("returncode"),
                                                interrupted=bool(outcome.get("interrupted")))
                run_context = facts.get("run_context") or {}
                run_settings = run_context.get("settings", {}) if isinstance(run_context, dict) else {}
                lifecycle = run_context.get("lifecycle", {}) if isinstance(run_context, dict) else {}
                task_start = lifecycle.get("task_start") or lifecycle.get("policy_active_start") or {}
                attempt.update({
                    "scene_created": facts.get("scene_created"),
                    "run_id": facts.get("run_id"),
                    "session_id": (run_context.get("session_id")
                                   if isinstance(run_context, dict) else None)
                                  or (task_start.get("session_id") if isinstance(task_start, dict) else None),
                    "entered_rl": facts.get("entered_rl"),
                    "raw_terminal": facts.get("raw_terminal"),
                    "record_validity": facts.get("record_validity", "UNKNOWN"),
                    "run_directory": facts.get("run_directory"),
                    "safety_task_completion": facts.get("safety_task_completion", "UNKNOWN"),
                    "standing_and_cleanup": (facts.get("scene_evidence", {}) or {}).get(
                        "standing_and_cleanup", "UNKNOWN"),
                    "actual_run_context_settings": {
                        key: run_settings.get(key) for key in (
                            "group", "entry_threshold", "exit_threshold", "hysteresis",
                            "hold", "recovery_min_hold_s", "goal_world_m",
                            "arrival_threshold_m", "diagnostic_sim_limit_after_motion_s",
                            "standing_observation_sim_s", "scene_id",
                            "controller_config", "overlay")
                    } if run_settings else None,
                    "scene_evidence": facts.get("scene_evidence"),
                    "terminal_sources": facts.get("terminal_sources"),
                    "terminal_conflict": facts.get("terminal_conflict"),
                    "run_id_matches_directory": facts.get("run_id_matches_directory"),
                    "process_facts": (facts.get("process_facts") or {}),
                    "analysis": analysis_result,
                    "launch_error": outcome.get("launch_error"),
                    "cause_evidence": cause_evidence_from_facts(
                        facts, stdout_result=outcome.get("stdout_result")),
                })
                setting_errors = (runtime_setting_errors(record, run_settings)
                                  if facts.get("scene_created") is True else [])
                consumption_errors = (runtime_consumption_errors(record, run_context)
                                      if facts.get("scene_created") is True else [])
                attempt["runtime_setting_mismatch"] = setting_errors
                attempt["runtime_config_consumption"] = runtime_consumption_evidence(
                    record, run_context)
                if facts.get("scene_created") is False:
                    attempt["runner_status"] = "PRE_SCENE_FAILURE"
                elif facts.get("scene_created") is True:
                    attempt["runner_status"] = "SCENE_STARTED"
                else:
                    attempt["runner_status"] = "SCENE_CREATION_UNKNOWN"
                if outcome.get("interrupted"):
                    attempt["runner_status"] += "_INTERRUPTED"
                ledger.upsert_attempt(attempt)
                counts["dispatched"] += 1
                counts[f"scene_created_{facts.get('scene_created')}"] += 1
                if facts.get("scene_created") is True:
                    scene_starts_this_entry += 1

                residual = scan_batch_processes(batch_root)
                if residual:
                    ledger.event("residual_processes_after_run", processes=residual)
                    pause_reason = "residual_processes_after_run"
                    break

                audit = audit_ledger(plan, ledger.attempts, max_substitutions=max_substitutions, max_per_plan_item=max_per_plan_item)
                replacement_errors = validate_replacement_authorizations(
                    ledger.attempts, authorization)
                if replacement_errors:
                    audit = {**audit, "status": "REJECTED",
                             "errors": sorted(set(list(audit.get("errors", []))
                                                  + replacement_errors))}
                if audit.get("status") != "PASS":
                    pause_reason = "ledger_audit_rejected"
                    break
                completed = audit.get("counts", {}).get("actual_scene_starts", 0)
                if completed and completed % 12 == 0:
                    ledger.checkpoint(audit, free_gib=free_gib, batch_gib=budget_gib,
                                      reason=f"every_12_scene_starts:{completed}")
                block_reason = f"block_complete:{record['display_name']}:{record['repeat_block']}"
                if block_complete(audit, record, plan) and not any(
                        item.get("reason") == block_reason
                        for item in ledger.value.get("checkpoints", [])):
                    ledger.checkpoint(audit, free_gib=free_gib, batch_gib=budget_gib,
                                      reason=block_reason)
                if facts.get("scene_created") is None:
                    pause_reason = "scene_creation_unknown_after_launch"
                    break
                if outcome.get("interrupted"):
                    pause_reason = "operator_interrupted"
                    break
                if facts.get("terminal_conflict"):
                    pause_reason = "terminal_source_conflict"
                    break
                if setting_errors:
                    pause_reason = "run_context_setting_mismatch"
                    break
                if consumption_errors:
                    pause_reason = "runtime_config_consumption_not_confirmed"
                    break
                if facts.get("scene_created") is True and facts.get("entered_rl") is True and \
                        attempt.get("session_id") is None:
                    pause_reason = "session_identity_missing"
                    break
                if facts.get("run_id_matches_directory") is False:
                    pause_reason = "run_id_directory_mismatch"
                    break
                if facts.get("scene_created") is True and facts.get("record_validity") == "UNKNOWN":
                    pause_reason = "record_validity_unknown"
                    break
                if facts.get("scene_created") is True and facts.get("entered_rl") is None:
                    pause_reason = "rl_entry_fact_unknown"
                    break
                if facts.get("scene_created") is True and \
                        (facts.get("process_facts") or {}).get("shutdown_complete") is not True:
                    pause_reason = "run_cleanup_unconfirmed"
                    break
                if facts.get("scene_created") is True and analysis_result and \
                        analysis_result.get("status") != "PASS":
                    pause_reason = "run_analysis_failed"
                    break
                if facts.get("scene_created") is False and outcome.get("returncode") not in (0, 2):
                    pause_reason = "pre_scene_failure_unexplained"
                    break

            audit = audit_ledger(plan, ledger.attempts, max_substitutions=max_substitutions, max_per_plan_item=max_per_plan_item)
            if staged_reason:
                ledger.checkpoint(audit, free_gib=free_gib, batch_gib=budget_gib,
                                  reason=staged_reason)
            status = "PAUSED" if pause_reason else (
                "STAGED" if staged_reason else
                "COMPLETE" if audit.get("counts", {}).get("actual_scene_starts") and
                all(item["action"] in {"SKIP_COMPLETED"} for item in plan_dispatch(
                    plan, ledger, authorization, reconcile=reconcile,
                    allow_pre_scene_retry=allow_pre_scene_retry)["actions"]) else "INCOMPLETE")
            _save_batch_state(batch_root, {
                "status": status, "pause_reason": pause_reason,
                "staged_reason": staged_reason,
                "stage": ("STAGED_AFTER_SCENE_START" if staged_reason else
                          "COMPLETE" if status == "COMPLETE" else "PAUSED_OR_INCOMPLETE"),
                "dispatched_this_entry": dispatched,
                "actual_scene_starts": audit.get("counts", {}).get("actual_scene_starts"),
                "replacement_scene_starts": audit.get("counts", {}).get("replacement_scene_starts"),
                "plan_item_outcomes": audit.get("plan_item_outcomes"),
                "audit_status": audit.get("status"),
                "ledger_errors": audit.get("errors"),
                "next_pending": [item["plan_item_id"]
                                 for item in plan_dispatch(
                                     plan, ledger, authorization, reconcile=reconcile,
                                     allow_pre_scene_retry=allow_pre_scene_retry)["actions"]
                                 if item["action"].startswith("DISPATCH_")],
            })
            return {
                "status": status, "pause_reason": pause_reason,
                "staged_reason": staged_reason,
                "dispatched": dispatched, "batch_root": str(batch_root),
                "manifest": manifest, "audit": audit,
                "plan_state": {item["plan_item_id"]: item["action"]
                               for item in plan_dispatch(plan, ledger, authorization,
                                                         reconcile=reconcile,
                                                         allow_pre_scene_retry=allow_pre_scene_retry)["actions"]},
            }
    except BatchLockError as error:
        return {"status": "REFUSED", "reason": "concurrent_batch_entry", "detail": str(error),
                "dispatched": 0}


def status(batch_root: Path, plan: list[dict[str, Any]], *,
           max_substitutions: int = MAX_SUBSTITUTIONS,
           max_per_plan_item: int = 1) -> dict[str, Any]:
    batch_root = batch_root.resolve()
    if not batch_root.is_dir():
        return {"status": "NOT_FOUND", "batch_root": str(batch_root)}
    manifest = read_json(batch_root / "batch_manifest.json") if \
        (batch_root / "batch_manifest.json").is_file() else None
    ledger_path = batch_root / "ledger" / "attempts.json"
    if not ledger_path.is_file():
        return {"status": "NO_LEDGER", "batch_manifest": manifest}
    ledger_batch_id = (manifest or {}).get("batch_id") or read_json(ledger_path).get("batch_id", "")
    ledger = Ledger(ledger_path, ledger_batch_id)
    audit = audit_ledger(plan, ledger.attempts, max_substitutions=max_substitutions, max_per_plan_item=max_per_plan_item)
    state = _batch_state(batch_root)
    return {
        "status": "OK",
        "batch_root": str(batch_root),
        "batch_manifest": manifest,
        "batch_state": state,
        "audit": audit,
        "attempts": [
            {key: row.get(key) for key in (
                "attempt_id", "plan_item_id", "group", "invocation_number",
                "scene_created", "run_id", "entered_rl", "raw_terminal",
                "record_validity", "safety_task_completion", "runner_status",
                "is_replacement", "replaces_attempt_id")}
            for row in sorted(ledger.attempts, key=lambda row: (
                str(row.get("plan_item_id")), int(row.get("invocation_number", 0))))
        ],
    }
