#!/usr/bin/env python3
"""Versioned formal-experiment artifacts and offline validation for ABS-Go2.

This module deliberately does not launch ROS2 or MuJoCo.  It is the formal
contract boundary used by a future runtime adapter: incomplete runtime inputs
are written as UNKNOWN and are classified INVALID by the offline validator.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence
from xml.etree import ElementTree


SCHEMA_PATH = Path(__file__).resolve().parents[1] / "schemas" / "formal_experiment_run_v1.json"


def load_contract_schema() -> Dict[str, Any]:
    """Load the single canonical contract specification used by this validator."""
    payload = json.loads(SCHEMA_PATH.read_text(encoding="utf-8"))
    if not isinstance(payload, dict) or "$id" not in payload:
        raise ValueError(f"invalid formal contract schema: {SCHEMA_PATH}")
    return payload


CONTRACT_SCHEMA = load_contract_schema()
SCHEMA_VERSION = str(CONTRACT_SCHEMA["$id"])
VALID_VARIANTS = frozenset(CONTRACT_SCHEMA["properties"]["variant"]["enum"])
TERMINAL_OUTCOMES = frozenset({"SUCCESS", "COLLISION", "FALL", "TIMEOUT"})
REQUIRED_PLOTS = (
    "ra_switching.svg",
    "trajectory_obstacles.svg",
    "command_tracking.svg",
    "stability.svg",
    "recovery_markers.svg",
)
_TELEMETRY_SPEC = CONTRACT_SCHEMA["x-abs-telemetry"]
_VECTOR_COLUMNS = tuple(
    f"{group['prefix']}_{index:02d}"
    for group in _TELEMETRY_SPEC["vector_groups"]
    for index in range(int(group["length"]))
)
REQUIRED_TELEMETRY_FIELDS = tuple(_TELEMETRY_SPEC["required_scalar_columns"]) + _VECTOR_COLUMNS


@dataclass(frozen=True)
class ValidationResult:
    validator_completed: bool
    episode_state: str
    reasons: List[str]
    run_id: Optional[str]

    def to_dict(self) -> Dict[str, Any]:
        return {
            "schema_version": SCHEMA_VERSION,
            "validator_completed": self.validator_completed,
            "episode_state": self.episode_state,
            "reasons": self.reasons,
            "run_id": self.run_id,
        }


def canonical_json(value: Any) -> str:
    return json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=False)


def sha256_json(value: Any) -> str:
    return hashlib.sha256(canonical_json(value).encode("utf-8")).hexdigest()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _resolve_ref(schema: Mapping[str, Any], root: Mapping[str, Any]) -> Mapping[str, Any]:
    ref = schema.get("$ref")
    if not ref:
        return schema
    if not isinstance(ref, str) or not ref.startswith("#/"):
        raise ValueError(f"unsupported schema reference: {ref}")
    target: Any = root
    for key in ref[2:].split("/"):
        target = target[key]
    if not isinstance(target, Mapping):
        raise ValueError(f"schema reference is not an object: {ref}")
    return target


def schema_errors(value: Any, schema: Mapping[str, Any] = CONTRACT_SCHEMA, root: Mapping[str, Any] = CONTRACT_SCHEMA, path: str = "$") -> List[str]:
    """Small dependency-free evaluator for the exact JSON-Schema subset we ship."""
    schema = _resolve_ref(schema, root)
    errors: List[str] = []
    if "const" in schema and value != schema["const"]:
        errors.append(f"{path}:const")
    if "enum" in schema and value not in schema["enum"]:
        errors.append(f"{path}:enum")
    expected_type = schema.get("type")
    type_ok = True
    if expected_type == "object": type_ok = isinstance(value, Mapping)
    elif expected_type == "array": type_ok = isinstance(value, list)
    elif expected_type == "string": type_ok = isinstance(value, str)
    elif expected_type == "integer": type_ok = isinstance(value, int) and not isinstance(value, bool)
    elif expected_type == "number": type_ok = isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(float(value))
    elif expected_type == "boolean": type_ok = isinstance(value, bool)
    if expected_type and not type_ok:
        return errors + [f"{path}:type={expected_type}"]
    if isinstance(value, str):
        if len(value) < int(schema.get("minLength", 0)): errors.append(f"{path}:minLength")
        if "pattern" in schema and re.fullmatch(str(schema["pattern"]), value) is None: errors.append(f"{path}:pattern")
    if isinstance(value, (int, float)) and not isinstance(value, bool):
        if "minimum" in schema and value < schema["minimum"]: errors.append(f"{path}:minimum")
        if "exclusiveMinimum" in schema and value <= schema["exclusiveMinimum"]: errors.append(f"{path}:exclusiveMinimum")
    if isinstance(value, Mapping):
        properties = schema.get("properties", {})
        for required in schema.get("required", []):
            if required not in value: errors.append(f"{path}.{required}:required")
        if schema.get("additionalProperties") is False:
            for key in value:
                if key not in properties: errors.append(f"{path}.{key}:additionalProperty")
        for key, child in properties.items():
            if key in value: errors.extend(schema_errors(value[key], child, root, f"{path}.{key}"))
    if isinstance(value, list):
        if len(value) < int(schema.get("minItems", 0)): errors.append(f"{path}:minItems")
        if "items" in schema:
            for index, item in enumerate(value): errors.extend(schema_errors(item, schema["items"], root, f"{path}[{index}]"))
    return errors


def _finite_float(value: Any) -> Optional[float]:
    if isinstance(value, bool):
        return None
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return None
    return parsed if math.isfinite(parsed) else None


def _nonnegative_int(value: Any) -> Optional[int]:
    if isinstance(value, bool):
        return None
    try:
        parsed = int(value)
    except (TypeError, ValueError, OverflowError):
        return None
    if str(value).strip() not in {str(parsed), f"+{parsed}"} or parsed < 0:
        return None
    return parsed


def _bool_flag(value: Any) -> Optional[bool]:
    normalized = str(value).lower()
    if normalized in {"1", "true"}: return True
    if normalized in {"0", "false"}: return False
    return None


def derive_seed(root_seed: int, source_name: str) -> int:
    """Stable, documented seed derivation independent of Python hash randomization."""
    digest = hashlib.sha256(f"{root_seed}:{source_name}".encode("utf-8")).digest()
    return int.from_bytes(digest[:8], byteorder="big", signed=False) & 0x7FFFFFFF


def pairing_key(manifest: Mapping[str, Any]) -> Optional[str]:
    try:
        payload = {
            "scenario_id": manifest["scenario"]["id"],
            "scenario_sha256": manifest["scenario"]["sha256"],
            "root_seed": manifest["seeds"]["root_seed"],
            "effective_config_sha256": manifest["effective_config"]["sha256"],
            "models": {
                name: manifest["models"][name]["sha256"]
                for name in ("agile_policy", "ra_value", "recovery_policy")
            },
        }
    except (KeyError, TypeError):
        return None
    return sha256_json(payload)


def validate_variant_group(manifests: Sequence[Mapping[str, Any]]) -> List[str]:
    """Return pairing errors; an empty list means a correctly paired variant group."""
    errors: List[str] = []
    labels = [manifest.get("variant") for manifest in manifests]
    if len(manifests) != len(VALID_VARIANTS):
        errors.append("comparison_requires_exactly_three_variants")
    invalid_labels = [str(label) for label in labels if label not in VALID_VARIANTS]
    if invalid_labels:
        errors.append("invalid_variant:" + ",".join(sorted(invalid_labels)))
    keys = [pairing_key(manifest) for manifest in manifests]
    if not keys or any(key is None for key in keys):
        errors.append("missing_pairing_key_input")
    elif len(set(keys)) != 1:
        errors.append("paired_variant_key_mismatch")
    if len(set(labels)) != len(labels):
        errors.append("duplicate_variant_label")
    if set(labels) != set(VALID_VARIANTS):
        errors.append("missing_required_variant")
    return errors


def validate_comparison_manifests(manifest_paths: Sequence[Path]) -> Dict[str, Any]:
    """Formal comparison gate: validate complete, unique, paired variant manifests."""
    manifests: List[Mapping[str, Any]] = []
    errors: List[str] = []
    for path in manifest_paths:
        payload = _load_json(Path(path), errors, "comparison_manifest")
        if payload is None:
            continue
        errors.extend(f"{Path(path).name}:{error}" for error in schema_errors(payload))
        if payload.get("pairing_key") != pairing_key(payload):
            errors.append(f"{Path(path).name}:pairing_key_mismatch")
        manifests.append(payload)
    errors.extend(validate_variant_group(manifests))
    return {"schema_version": SCHEMA_VERSION, "comparison_valid": not errors, "manifest_count": len(manifests), "errors": sorted(set(errors))}


class FormalRunWriter:
    """Writes a run-local artifact set; it never manufactures missing runtime fields."""

    def __init__(self, run_dir: Path):
        self.run_dir = Path(run_dir)
        self.run_dir.mkdir(parents=True, exist_ok=True)
        (self.run_dir / "plots").mkdir(exist_ok=True)
        self._run_id: Optional[str] = None

    @property
    def manifest_path(self) -> Path:
        return self.run_dir / "manifest.json"

    @property
    def telemetry_path(self) -> Path:
        return self.run_dir / "telemetry.csv"

    @property
    def events_path(self) -> Path:
        return self.run_dir / "events.jsonl"

    @property
    def summary_path(self) -> Path:
        return self.run_dir / "summary.json"

    def write_manifest(self, manifest: Mapping[str, Any]) -> None:
        payload = dict(manifest)
        payload.setdefault("schema_version", SCHEMA_VERSION)
        payload.setdefault("pairing_key", pairing_key(payload))
        errors = schema_errors(payload)
        if errors:
            raise ValueError("manifest violates formal schema: " + "; ".join(errors))
        self._run_id = str(payload["run_id"])
        self.manifest_path.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

    def write_telemetry(self, rows: Iterable[Mapping[str, Any]]) -> None:
        if self._run_id is None:
            raise RuntimeError("write manifest before telemetry")
        materialized = list(rows)
        for row in materialized:
            if row.get("run_id") != self._run_id:
                raise ValueError("telemetry run_id must match manifest")
        with self.telemetry_path.open("w", encoding="utf-8", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=REQUIRED_TELEMETRY_FIELDS, extrasaction="ignore")
            writer.writeheader()
            writer.writerows(materialized)

    def emit_event(self, event: Mapping[str, Any]) -> None:
        if self._run_id is None:
            raise RuntimeError("write manifest before events")
        required = set(CONTRACT_SCHEMA["x-abs-events"]["required_fields"])
        missing = sorted(required.difference(event))
        if missing:
            raise ValueError(f"event missing fields: {', '.join(missing)}")
        if event.get("run_id") != self._run_id:
            raise ValueError("event run_id must match manifest")
        with self.events_path.open("a", encoding="utf-8") as handle:
            handle.write(canonical_json(dict(event)) + "\n")

    def write_summary(self, summary: Mapping[str, Any]) -> None:
        if self._run_id is None:
            raise RuntimeError("write manifest before summary")
        payload = dict(summary)
        payload.setdefault("schema_version", SCHEMA_VERSION)
        payload.setdefault("run_id", self._run_id)
        payload.setdefault("artifacts", {
            "manifest": "manifest.json",
            "telemetry": "telemetry.csv",
            "events": "events.jsonl",
            "plots": [f"plots/{name}" for name in REQUIRED_PLOTS],
        })
        payload.setdefault("artifact_hashes", {
            "manifest": sha256_file(self.manifest_path),
            "telemetry": sha256_file(self.telemetry_path),
            "events": sha256_file(self.events_path),
            "plots": {name: sha256_file(self.run_dir / "plots" / name) for name in REQUIRED_PLOTS},
        })
        self.summary_path.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

    def write_data_plot(self, name: str, data_points: int) -> None:
        """Write a deterministic plot carrying hashes of the exact source artifacts."""
        if self._run_id is None:
            raise RuntimeError("write manifest before plots")
        if name not in REQUIRED_PLOTS:
            raise ValueError(f"not a registered fixed plot: {name}")
        if data_points <= 0:
            raise ValueError("data-driven plot requires at least one data point")
        if not self.telemetry_path.exists() or not self.events_path.exists():
            raise RuntimeError("write telemetry and events before plots")
        telemetry_hash = sha256_file(self.telemetry_path)
        events_hash = sha256_file(self.events_path)
        input_hash = sha256_json({"plot": name, "run_id": self._run_id, "telemetry": telemetry_hash, "events": events_hash, "data_points": data_points})
        (self.run_dir / "plots" / name).write_text(
            f"<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"100\" height=\"20\" data-run-id=\"{self._run_id}\" data-telemetry-sha256=\"{telemetry_hash}\" data-events-sha256=\"{events_hash}\" data-input-sha256=\"{input_hash}\" data-point-count=\"{data_points}\"><polyline points=\"0,19 100,1\"/></svg>\n",
            encoding="utf-8",
        )


def _load_json(path: Path, reasons: List[str], label: str) -> Optional[Dict[str, Any]]:
    if not path.exists():
        reasons.append(f"missing_{label}")
        return None
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        reasons.append(f"malformed_{label}")
        return None
    if not isinstance(payload, dict):
        reasons.append(f"malformed_{label}")
        return None
    return payload


def _nested_value(payload: Mapping[str, Any], path: Sequence[str]) -> Any:
    value: Any = payload
    for key in path:
        if not isinstance(value, Mapping) or key not in value:
            return None
        value = value[key]
    return value


def _has_known_value(value: Any) -> bool:
    return value not in (None, "", "UNKNOWN")


def _validate_manifest(manifest: Optional[Mapping[str, Any]], reasons: List[str]) -> None:
    if manifest is None:
        return
    reasons.extend("schema:" + error for error in schema_errors(manifest))
    if manifest.get("pairing_key") != pairing_key(manifest):
        reasons.append("pairing_key_mismatch")


def _read_events(path: Path, run_id: Optional[str], reasons: List[str]) -> List[Dict[str, Any]]:
    if not path.exists():
        reasons.append("missing_events")
        return []
    events: List[Dict[str, Any]] = []
    for line_number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), start=1):
        try:
            event = json.loads(line)
        except json.JSONDecodeError:
            reasons.append(f"malformed_event:{line_number}")
            continue
        if not isinstance(event, dict):
            reasons.append(f"malformed_event:{line_number}")
            continue
        if not set(CONTRACT_SCHEMA["x-abs-events"]["required_fields"]).issubset(event):
            reasons.append(f"missing_event_field:{line_number}")
            continue
        if run_id is not None and event.get("run_id") != run_id:
            reasons.append(f"wrong_run_event:{line_number}")
        events.append(event)
    return events


def _validate_events(events: Sequence[Mapping[str, Any]], reasons: List[str]) -> None:
    expected = set(CONTRACT_SCHEMA["x-abs-events"]["required_types"])
    actual = {str(event.get("type")) for event in events}
    for name in sorted(expected.difference(actual)):
        reasons.append("missing_event:" + name)
    previous_sequence = -1
    previous_monotonic = -1
    previous_simulation = -math.inf
    for index, event in enumerate(events, start=1):
        sequence = _nonnegative_int(event.get("sequence"))
        monotonic_ns = _nonnegative_int(event.get("monotonic_time_ns"))
        simulation_s = _finite_float(event.get("simulation_time_s"))
        if sequence is None or monotonic_ns is None or simulation_s is None or simulation_s < 0:
            reasons.append(f"invalid_event_clock:{index}")
            continue
        if sequence <= previous_sequence or monotonic_ns <= previous_monotonic or simulation_s < previous_simulation:
            reasons.append("non_monotonic_events")
        previous_sequence, previous_monotonic, previous_simulation = sequence, monotonic_ns, simulation_s


def _validate_telemetry(path: Path, run_id: Optional[str], reasons: List[str]) -> List[Dict[str, Any]]:
    safety_rows: List[Dict[str, Any]] = []
    if not path.exists():
        reasons.append("missing_telemetry")
        return safety_rows
    try:
        with path.open(encoding="utf-8", newline="") as handle:
            reader = csv.DictReader(handle)
            fields = set(reader.fieldnames or [])
            missing = sorted(set(REQUIRED_TELEMETRY_FIELDS).difference(fields))
            if missing:
                reasons.append("missing_telemetry_fields:" + ",".join(missing))
            previous_sequence = -1
            previous_monotonic = -1
            previous_simulation = -math.inf
            row_count = 0
            for row in reader:
                row_count += 1
                if run_id is not None and row.get("run_id") != run_id:
                    reasons.append(f"wrong_run_telemetry:{row_count}")
                sequence = _nonnegative_int(row.get("sequence"))
                monotonic_ns = _nonnegative_int(row.get("monotonic_time_ns"))
                simulation_s = _finite_float(row.get("simulation_time_s"))
                if sequence is None or monotonic_ns is None or simulation_s is None or simulation_s < 0:
                    reasons.append("non_numeric_telemetry_clock")
                    continue
                if sequence <= previous_sequence or monotonic_ns <= previous_monotonic or simulation_s < previous_simulation:
                    reasons.append("non_monotonic_telemetry")
                previous_sequence, previous_monotonic, previous_simulation = sequence, monotonic_ns, simulation_s
                for name in _TELEMETRY_SPEC["finite_scalar_columns"] + list(_VECTOR_COLUMNS):
                    try:
                        if not math.isfinite(float(row[name])):
                            reasons.append("non_finite_telemetry:" + name)
                    except (KeyError, TypeError, ValueError):
                        reasons.append("non_numeric_telemetry:" + name)
                flags: Dict[str, Optional[bool]] = {}
                for name, invalid_reason in (
                    ("telemetry_fresh", "telemetry_stale"), ("controller_active", "controller_inactive"),
                    ("rl_active", "rl_not_entered"), ("collision_available", "collision_unavailable"), ("ray_valid", "perception_invalid"),
                ):
                    flags[name] = _bool_flag(row.get(name))
                    if flags[name] is not True:
                        reasons.append(invalid_reason)
                collision = _bool_flag(row.get("collision"))
                fall = _bool_flag(row.get("fall"))
                if collision is None:
                    reasons.append("invalid_collision_flag")
                if fall is None:
                    reasons.append("invalid_fall_flag")
                if collision is not None and fall is not None:
                    safety_rows.append({"sequence": sequence, "collision": collision, "fall": fall})
            if row_count == 0:
                reasons.append("empty_telemetry")
    except OSError:
        reasons.append("malformed_telemetry")
    return safety_rows


def _terminal_from_events(events: Sequence[Mapping[str, Any]]) -> Optional[str]:
    terminal = [event for event in events if event.get("type") == "terminal"]
    if len(terminal) != 1:
        return None
    outcome = terminal[0].get("outcome")
    return str(outcome) if outcome is not None else None


def _event_sequences(events: Sequence[Mapping[str, Any]], event_types: set) -> List[int]:
    values: List[int] = []
    for event in events:
        if event.get("type") not in event_types:
            continue
        sequence = _nonnegative_int(event.get("sequence"))
        if sequence is not None:
            values.append(sequence)
    return values


def _validate_safety_consistency(summary: Optional[Mapping[str, Any]], events: Sequence[Mapping[str, Any]], telemetry_safety: Sequence[Mapping[str, Any]], reasons: List[str]) -> None:
    collision_events = _event_sequences(events, {"collision_start"})
    fall_events = _event_sequences(events, {"fall"})
    collision_telemetry = [int(row["sequence"]) for row in telemetry_safety if row["collision"]]
    fall_telemetry = [int(row["sequence"]) for row in telemetry_safety if row["fall"]]
    if collision_telemetry and not collision_events:
        reasons.append("telemetry_collision_without_collision_event")
    if fall_telemetry and not fall_events:
        reasons.append("telemetry_fall_without_fall_event")
    if collision_events and not collision_telemetry:
        reasons.append("collision_event_without_telemetry")
    if fall_events and not fall_telemetry:
        reasons.append("fall_event_without_telemetry")
    def transition_windows(flag: str) -> List[tuple]:
        previous_sequence = -1
        previous_value = False
        windows: List[tuple] = []
        for row in telemetry_safety:
            current_sequence = int(row["sequence"])
            current_value = bool(row[flag])
            if current_value and not previous_value:
                windows.append((previous_sequence, current_sequence))
            previous_sequence, previous_value = current_sequence, current_value
        return windows
    for previous, current in transition_windows("collision"):
        if not any(previous < sequence <= current for sequence in collision_events):
            reasons.append("collision_event_not_aligned_to_telemetry_transition")
    for previous, current in transition_windows("fall"):
        if not any(previous < sequence <= current for sequence in fall_events):
            reasons.append("fall_event_not_aligned_to_telemetry_transition")
    terminal = summary.get("terminal_outcome") if summary is not None else None
    if (collision_telemetry or fall_telemetry or collision_events or fall_events) and terminal == "SUCCESS":
        reasons.append("safety_evidence_vetoes_success")
    if terminal == "COLLISION" and not (collision_telemetry and collision_events):
        reasons.append("collision_terminal_without_consistent_safety_evidence")
    if terminal == "FALL" and not (fall_telemetry and fall_events):
        reasons.append("fall_terminal_without_consistent_safety_evidence")


def _validate_outcome(summary: Optional[Mapping[str, Any]], events: Sequence[Mapping[str, Any]], telemetry_safety: Sequence[Mapping[str, Any]], reasons: List[str]) -> None:
    if summary is None:
        return
    terminal = _terminal_from_events(events)
    if terminal not in TERMINAL_OUTCOMES:
        reasons.append("invalid_terminal_event")
    if summary.get("terminal_outcome") != terminal:
        reasons.append("summary_terminal_mismatch")
    safety_sequences = _event_sequences(events, {"collision_start", "fall"})
    safety_sequences.extend(int(row["sequence"]) for row in telemetry_safety if row["collision"] or row["fall"])
    arrival_sequences = _event_sequences(events, {"arrival_accepted"})
    if terminal == "SUCCESS":
        if not arrival_sequences:
            reasons.append("success_without_arrival_accept")
        if safety_sequences and min(safety_sequences) <= max(arrival_sequences or [-1]):
            reasons.append("safety_event_vetoes_arrival")
    _validate_safety_consistency(summary, events, telemetry_safety, reasons)


def _plot_input_hash(name: str, run_id: str, telemetry_hash: str, events_hash: str, data_points: int) -> str:
    return sha256_json({"plot": name, "run_id": run_id, "telemetry": telemetry_hash, "events": events_hash, "data_points": data_points})


def _validate_summary_and_artifacts(run_dir: Path, manifest: Optional[Mapping[str, Any]], summary: Optional[Mapping[str, Any]], reasons: List[str]) -> None:
    if summary is None:
        return
    summary_spec = CONTRACT_SCHEMA["x-abs-summary"]
    for field in summary_spec["required_fields"]:
        if field not in summary:
            reasons.append("missing_summary:" + field)
    if summary.get("schema_version") != SCHEMA_VERSION: reasons.append("summary_schema_version")
    if manifest is not None and summary.get("run_id") != manifest.get("run_id"): reasons.append("summary_run_id_mismatch")
    if not isinstance(summary.get("metrics"), Mapping): reasons.append("missing_summary_metrics")
    artifacts = summary.get("artifacts")
    if not isinstance(artifacts, Mapping):
        reasons.append("missing_summary_artifacts")
        return
    expected = summary_spec["required_artifact_paths"]
    for key, expected_path in expected.items():
        if artifacts.get(key) != expected_path:
            reasons.append("artifact_link_mismatch:" + key)
    plots = artifacts.get("plots")
    expected_plots = [f"plots/{name}" for name in REQUIRED_PLOTS]
    if plots != expected_plots:
        reasons.append("artifact_link_mismatch:plots")
    for plot in REQUIRED_PLOTS:
        plot_path = run_dir / "plots" / plot
        if not plot_path.exists():
            reasons.append("missing_plot:" + plot)
            continue
        try:
            if plot_path.stat().st_size < 128:
                reasons.append("placeholder_plot:" + plot)
                continue
            root = ElementTree.fromstring(plot_path.read_text(encoding="utf-8"))
            hashes = summary.get("artifact_hashes", {})
            telemetry_hash = hashes.get("telemetry") if isinstance(hashes, Mapping) else None
            events_hash = hashes.get("events") if isinstance(hashes, Mapping) else None
            point_count = int(root.attrib.get("data-point-count", "0"))
            expected_input = _plot_input_hash(plot, str(summary.get("run_id")), str(telemetry_hash), str(events_hash), point_count)
            if point_count <= 0 or not list(root): reasons.append("non_data_plot:" + plot)
            if root.attrib.get("data-run-id") != summary.get("run_id"): reasons.append("plot_run_id_mismatch:" + plot)
            if root.attrib.get("data-telemetry-sha256") != telemetry_hash or root.attrib.get("data-events-sha256") != events_hash: reasons.append("plot_source_hash_mismatch:" + plot)
            if root.attrib.get("data-input-sha256") != expected_input: reasons.append("plot_provenance_mismatch:" + plot)
        except (OSError, ValueError, ElementTree.ParseError):
            reasons.append("malformed_plot:" + plot)
    hashes = summary.get("artifact_hashes")
    if not isinstance(hashes, Mapping):
        reasons.append("missing_artifact_hashes")
        return
    for name, relative in (("manifest", "manifest.json"), ("telemetry", "telemetry.csv"), ("events", "events.jsonl")):
        artifact_path = run_dir / relative
        if not artifact_path.exists():
            reasons.append("missing_artifact_for_hash:" + name)
        elif hashes.get(name) != sha256_file(artifact_path):
            reasons.append("artifact_hash_mismatch:" + name)
    plot_hashes = hashes.get("plots")
    if not isinstance(plot_hashes, Mapping):
        reasons.append("missing_artifact_hashes:plots")
    else:
        for plot in REQUIRED_PLOTS:
            plot_path = run_dir / "plots" / plot
            if plot_path.exists() and plot_hashes.get(plot) != sha256_file(plot_path): reasons.append("artifact_hash_mismatch:plot:" + plot)


def validate_run(run_dir: Path) -> ValidationResult:
    """Classify one artifact directory without treating INVALID as a passed run."""
    run_dir = Path(run_dir)
    reasons: List[str] = []
    manifest = _load_json(run_dir / "manifest.json", reasons, "manifest")
    summary = _load_json(run_dir / "summary.json", reasons, "summary")
    if manifest is None and summary is not None and summary.get("classification") == "LEGACY / NON-ACCEPTANCE":
        return ValidationResult(validator_completed=True, episode_state="LEGACY / NON-ACCEPTANCE", reasons=["legacy_artifact"], run_id=None)
    _validate_manifest(manifest, reasons)
    run_id = manifest.get("run_id") if manifest else None
    events = _read_events(run_dir / "events.jsonl", run_id, reasons)
    _validate_events(events, reasons)
    telemetry_safety = _validate_telemetry(run_dir / "telemetry.csv", run_id, reasons)
    _validate_outcome(summary, events, telemetry_safety, reasons)
    _validate_summary_and_artifacts(run_dir, manifest, summary, reasons)
    deduplicated = sorted(set(reasons))
    if deduplicated:
        return ValidationResult(validator_completed=True, episode_state="INVALID", reasons=deduplicated, run_id=run_id)
    if summary is None or summary.get("validity") != "VALID":
        return ValidationResult(validator_completed=True, episode_state="INVALID", reasons=["summary_not_valid"], run_id=run_id)
    return ValidationResult(validator_completed=True, episode_state="VALID", reasons=[], run_id=run_id)


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_dir", type=Path, nargs="?", help="formal run artifact directory")
    parser.add_argument("--validate-comparison", type=Path, nargs="+", metavar="MANIFEST", help="validate exactly one paper-faithful/stabilized/agile-only comparison group")
    args = parser.parse_args(argv)
    if args.validate_comparison:
        if args.run_dir is not None:
            parser.error("run_dir and --validate-comparison are mutually exclusive")
        result = validate_comparison_manifests(args.validate_comparison)
        print(json.dumps(result, indent=2, ensure_ascii=False))
        return 0 if result["comparison_valid"] else 1
    if args.run_dir is None:
        parser.error("run_dir or --validate-comparison is required")
    result = validate_run(args.run_dir)
    print(json.dumps(result.to_dict(), indent=2, ensure_ascii=False))
    return 0 if result.episode_state == "VALID" else 1


if __name__ == "__main__":
    raise SystemExit(main())
