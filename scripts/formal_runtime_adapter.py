#!/usr/bin/env python3
"""Minimal additive formal-runtime adapter boundary for ABS-Go2 (P1-09B).

OFFLINE ONLY. This module never launches ROS2, MuJoCo, a benchmark, a pilot, a
formal episode, or the real robot. It is the observational boundary that connects
explicit runtime context/snapshot inputs to the accepted P1-02
``FormalRunWriter``.

Contract guarantees:

1. The adapter allocates and owns exactly one ``FormalRunWriter`` and its
   allocated ``run_id``.
2. It accepts explicit manifest context and telemetry snapshot inputs only.
3. It writes run-bound manifest and telemetry exclusively through
   ``FormalRunWriter``; it never writes those artifacts directly.
4. It fails closed *before* any write on missing, malformed, wrong-run,
   non-finite, or inconsistent (non-monotonic) fields.
5. It never invents a value for a field that has no authoritative source.
   ``UNKNOWN`` is passed through only where the schema permits it (e.g.
   ``models.*.source_provenance``); where the schema requires a concrete number,
   an unresolved field is rejected rather than fabricated.
"""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any, List, Mapping, Sequence

from formal_experiment_contract import (
    CONTRACT_SCHEMA,
    REQUIRED_TELEMETRY_FIELDS,
    FormalRunWriter,
)


class AdapterValidationError(ValueError):
    """Raised when an adapter input fails closed before any write."""


# ------------------------------------------------------------------ input origin
# Every adapter input (manifest context / telemetry snapshot) must carry an
# explicit origin classification. Only SYNTHETIC_TEST is writable today: legacy
# evaluator output must never pass through the adapter as formal data, and
# AUTHORITATIVE_RUNTIME is reserved until a real runtime producer is wired.
AUTHORITATIVE_RUNTIME = "AUTHORITATIVE_RUNTIME"
SYNTHETIC_TEST = "SYNTHETIC_TEST"
LEGACY_ONLY = "LEGACY_ONLY"

_RECOGNIZED_ORIGINS = frozenset({AUTHORITATIVE_RUNTIME, SYNTHETIC_TEST, LEGACY_ONLY})
_WRITABLE_ORIGINS = frozenset({SYNTHETIC_TEST})

# Sidecar recording the origin of the written run. It sits outside the P1-02
# formal artifact set (manifest/telemetry/events/summary/plots), so it never
# touches the accepted schema.
ORIGIN_SIDECAR = "adapter_origin.json"

_UNSET = object()


def _validate_origin(origin: Any) -> str:
    """Return the writable origin or fail closed before any write."""
    if origin is _UNSET:
        raise AdapterValidationError("missing adapter input origin")
    if not isinstance(origin, str) or origin not in _RECOGNIZED_ORIGINS:
        raise AdapterValidationError(f"unrecognized adapter input origin: {origin!r}")
    if origin == LEGACY_ONLY:
        raise AdapterValidationError(
            "LEGACY_ONLY input rejected: legacy evaluator output cannot pass through the adapter as formal data"
        )
    if origin == AUTHORITATIVE_RUNTIME:
        raise AdapterValidationError(
            "AUTHORITATIVE_RUNTIME input not available: no authoritative runtime producer is wired yet"
        )
    return origin  # SYNTHETIC_TEST only


_TELEMETRY_SPEC = CONTRACT_SCHEMA["x-abs-telemetry"]

# Numeric telemetry columns that must be finite.
_FINITE_SCALAR_COLUMNS = tuple(_TELEMETRY_SPEC["finite_scalar_columns"])
_VECTOR_COLUMNS = tuple(
    f"{group['prefix']}_{index:02d}"
    for group in _TELEMETRY_SPEC["vector_groups"]
    for index in range(int(group["length"]))
)
_NUMERIC_COLUMNS = _FINITE_SCALAR_COLUMNS + _VECTOR_COLUMNS

# Boolean-flag telemetry columns that must be parseable (not malformed).
_TRUE_COLUMNS = tuple(_TELEMETRY_SPEC["true_columns"])
_BOOL_COLUMNS = _TRUE_COLUMNS + ("collision", "fall", "arrival_candidate")

# The 11-ray column and the 5x12 command-chain column groups, addressed explicitly
# so the fail-closed tests can target them by name.
_RAY_PREFIX = "ray_log2"
_COMMAND_CHAIN_PREFIXES = ("action_raw", "action_clipped", "joint_target_rad", "torque_nm", "torque_saturated")

# Manifest sections the caller must supply explicitly (run_id, schema_version and
# pairing_key are writer-owned and must not be supplied by the caller).
_REQUIRED_CONTEXT_SECTIONS = (
    "created_at",
    "variant",
    "git",
    "models",
    "effective_config",
    "environment",
    "scenario",
    "seeds",
    "perception",
    "rates_hz",
    "thresholds",
)

# Manifest numeric fields the schema requires to be concrete numbers. The adapter
# refuses to let ``UNKNOWN``/``None`` masquerade as any of these (never invent).
_MANIFEST_NUMERIC_PATHS: tuple = (
    ("seeds", "root_seed"),
    ("seeds", "sources", "scene_generator"),
    ("seeds", "sources", "controller_goal"),
    ("seeds", "sources", "perception"),
    ("seeds", "sources", "evaluator"),
    ("environment", "timestep_s"),
    ("scenario", "metadata", "obstacle_count"),
    ("rates_hz", "controller"),
    ("rates_hz", "pd"),
    ("rates_hz", "policy"),
    ("rates_hz", "ra"),
    ("rates_hz", "perception"),
    ("thresholds", "arrival_region_m"),
    ("thresholds", "arrival_hold_s"),
    ("thresholds", "fall_height_m"),
    ("thresholds", "fall_angle_rad"),
    ("thresholds", "ra_entry_threshold"),
    ("thresholds", "ra_exit_threshold"),
)


def _finite(value: Any):
    """Return the finite float of ``value``, or None if not finite/numeric."""
    if isinstance(value, bool):
        return None
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return None
    return parsed if math.isfinite(parsed) else None


def _nonneg_int(value: Any):
    """Return the non-negative int of ``value``, or None if not a clean int."""
    if isinstance(value, bool):
        return None
    try:
        parsed = int(value)
    except (TypeError, ValueError, OverflowError):
        return None
    if str(value).strip() not in {str(parsed), f"+{parsed}"} or parsed < 0:
        return None
    return parsed


def _bool_flag(value: Any):
    normalized = str(value).lower()
    if normalized in {"1", "true"}:
        return True
    if normalized in {"0", "false"}:
        return False
    return None


def _nested_get(mapping: Mapping[str, Any], path: Sequence[str]) -> Any:
    value: Any = mapping
    for key in path:
        if not isinstance(value, Mapping) or key not in value:
            return None
        value = value[key]
    return value


class FormalRuntimeAdapter:
    """Minimal fail-closed adapter boundary over the P1-02 ``FormalRunWriter``."""

    def __init__(self, run_dir: Path):
        self._writer = FormalRunWriter(run_dir)
        self._pending: List[Mapping[str, Any]] = []
        self._last_sequence = -1
        self._last_monotonic_ns = -1
        self._last_simulation_s = -math.inf

    @property
    def run_id(self) -> str:
        return self._writer.run_id

    @property
    def writer(self) -> FormalRunWriter:
        return self._writer

    # ------------------------------------------------------------------ manifest
    def bind_manifest(self, context: Mapping[str, Any], origin: Any = _UNSET) -> None:
        """Validate an explicit manifest context and write it through the writer.

        ``context`` must provide every schema-required manifest section except the
        writer-owned fields (``run_id``, ``schema_version``, ``pairing_key``), and
        must carry an explicit ``origin`` classification. A missing, unrecognized,
        legacy, or not-yet-wired origin — or a missing section, mismatched
        ``run_id``, or unresolved numeric field — fails closed before any write.
        """
        _validate_origin(origin)
        if not isinstance(context, Mapping):
            raise AdapterValidationError("manifest context must be a mapping")
        missing = [key for key in _REQUIRED_CONTEXT_SECTIONS if key not in context]
        if missing:
            raise AdapterValidationError("missing manifest section(s): " + ", ".join(missing))
        requested_run_id = context.get("run_id")
        if requested_run_id is not None and requested_run_id != self.run_id:
            raise AdapterValidationError("manifest run_id must match writer-allocated run_id")
        for path in _MANIFEST_NUMERIC_PATHS:
            value = _nested_get(context, path)
            if value is None or (isinstance(value, str) and value == "UNKNOWN"):
                raise AdapterValidationError(
                    f"manifest field '{'.'.join(path)}' has no authoritative source and must not be invented"
                )
            if not isinstance(value, (int, float)) or isinstance(value, bool):
                raise AdapterValidationError(
                    f"manifest field '{'.'.join(path)}' must be a concrete number"
                )
        self._writer.write_manifest(context)
        self._record_origin(origin)

    # ----------------------------------------------------------------- telemetry
    def _validate_snapshot(self, snapshot: Mapping[str, Any]) -> None:
        if not isinstance(snapshot, Mapping):
            raise AdapterValidationError("telemetry snapshot must be a mapping")
        if snapshot.get("run_id") != self.run_id:
            raise AdapterValidationError("telemetry run_id must match writer-allocated run_id")
        missing = sorted(set(REQUIRED_TELEMETRY_FIELDS).difference(snapshot))
        if missing:
            raise AdapterValidationError("missing telemetry field(s): " + ", ".join(missing))
        for name in _NUMERIC_COLUMNS:
            if _finite(snapshot.get(name)) is None:
                raise AdapterValidationError(f"non-finite telemetry value: {name}")
        for name in _BOOL_COLUMNS:
            if _bool_flag(snapshot.get(name)) is None:
                raise AdapterValidationError(f"malformed telemetry flag: {name}")
        sequence = _nonneg_int(snapshot.get("sequence"))
        monotonic_ns = _nonneg_int(snapshot.get("monotonic_time_ns"))
        simulation_s = _finite(snapshot.get("simulation_time_s"))
        if sequence is None or monotonic_ns is None or simulation_s is None or simulation_s < 0:
            raise AdapterValidationError("non-numeric or negative telemetry clock")
        if sequence <= self._last_sequence:
            raise AdapterValidationError("non-monotonic telemetry sequence")
        if monotonic_ns <= self._last_monotonic_ns:
            raise AdapterValidationError("non-monotonic telemetry monotonic_time_ns")
        if simulation_s < self._last_simulation_s:
            raise AdapterValidationError("non-monotonic telemetry simulation_time_s")

    def append_telemetry(self, snapshot: Mapping[str, Any], origin: Any = _UNSET) -> None:
        """Validate one explicit snapshot and buffer it; nothing is written yet."""
        _validate_origin(origin)
        self._validate_snapshot(snapshot)
        sequence = _nonneg_int(snapshot.get("sequence"))
        monotonic_ns = _nonneg_int(snapshot.get("monotonic_time_ns"))
        simulation_s = _finite(snapshot.get("simulation_time_s"))
        self._last_sequence = int(sequence)
        self._last_monotonic_ns = int(monotonic_ns)
        self._last_simulation_s = float(simulation_s)
        self._pending.append(dict(snapshot))

    def write_telemetry(self) -> None:
        """Flush buffered snapshots through the writer (fail-closed, run-bound).

        A no-op when no snapshot was accepted, so a fully rejected episode writes
        no telemetry artifact at all.
        """
        if not self._pending:
            return
        self._writer.write_telemetry(self._pending)
        self._pending.clear()

    def _record_origin(self, origin: str) -> None:
        """Record the run's input origin in an out-of-schema sidecar."""
        (self._writer.run_dir / ORIGIN_SIDECAR).write_text(
            json.dumps({"origin": origin, "evidence_class": "synthetic-test-only"}, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
