#!/usr/bin/env python3
"""Synthetic adapter-contract tests for the P1-09B formal-runtime adapter.

These tests are OFFLINE and SYNTHETIC ONLY. Every fixture input is declared with
origin ``SYNTHETIC_TEST`` and is test-only; nothing here is authoritative runtime
evidence, and no ROS2, MuJoCo, benchmark, pilot, formal episode, or real-robot
process is launched. The suite also proves the adapter's input-origin boundary:
legacy evaluator data and any unrecognized/missing origin are rejected before any
artifact is written.
"""

from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

from formal_experiment_contract import (
    REQUIRED_TELEMETRY_FIELDS,
    FormalRunWriter,
    validate_run,
)
from formal_runtime_adapter import (
    AUTHORITATIVE_RUNTIME,
    LEGACY_ONLY,
    ORIGIN_SIDECAR,
    SYNTHETIC_TEST,
    AdapterValidationError,
    FormalRuntimeAdapter,
)


HASH = "a" * 64


def _manifest_context(variant: str = "paper-faithful"):
    """Explicit, complete manifest context (writer-owned fields omitted)."""
    return {
        "created_at": "2026-08-28T00:00:00+08:00",
        "variant": variant,
        "git": {"commit": HASH, "branch": "fixture", "dirty_state": "clean"},
        "models": {
            "agile_policy": {"path": "models/agile.pt", "sha256": HASH, "source_provenance": "UNKNOWN"},
            "ra_value": {"path": "models/ra.pt", "sha256": HASH, "source_provenance": "UNKNOWN"},
            "recovery_policy": {"path": "models/recovery.pt", "sha256": HASH, "source_provenance": "UNKNOWN"},
        },
        "effective_config": {"path": "config/effective.yaml", "sha256": HASH},
        "environment": {
            "mujoco_binary_path": "bin/unitree_mujoco",
            "mujoco_binary_sha256": HASH,
            "mujoco_version": "fixture",
            "timestep_s": 0.002,
            "solver": "fixture",
            "go2_mjcf_path": "go2.xml",
            "go2_mjcf_sha256": HASH,
            "go2_assets_sha256": HASH,
            "hardware_mode": "simulation",
        },
        "scenario": {"id": "fixture-scene", "path": "scene.xml", "sha256": HASH, "metadata": {"schema": "fixture/v1", "obstacle_count": 0}},
        "seeds": {"root_seed": 17, "sources": {"scene_generator": 1, "controller_goal": 2, "perception": 3, "evaluator": 4}},
        "perception": {"source": "fixture", "version": "v1", "sha256": HASH, "frame_contract_version": "fixture/v1"},
        "rates_hz": {"controller": 200, "pd": 200, "policy": 50, "ra": 50, "perception": 50},
        "thresholds": {"arrival_region_m": 0.5, "arrival_hold_s": 0.5, "fall_height_m": 0.2, "fall_angle_rad": 0.8, "collision_definition_id": "fixture-v1", "ra_entry_threshold": -0.05, "ra_exit_threshold": -0.08},
    }


def _snapshot(sequence: int, monotonic_ns: int, simulation_s: float, run_id: str):
    """Explicit, complete telemetry snapshot with finite values (synthetic)."""
    row = {field: 0 for field in REQUIRED_TELEMETRY_FIELDS}
    row.update({
        "run_id": run_id,
        "sequence": sequence,
        "monotonic_time_ns": monotonic_ns,
        "simulation_time_s": simulation_s,
        "policy_state": "AGILE",
        "telemetry_fresh": 1,
        "ray_valid": 1,
        "controller_active": 1,
        "rl_active": 1,
        "collision_available": 1,
        "collision": 0,
        "fall": 0,
        "arrival_candidate": 0,
    })
    return row


def _snapshot_with_nan(field: str, sequence: int = 1):
    def build(run_id: str):
        row = _snapshot(sequence, sequence * 1_000_000, sequence * 0.01, run_id)
        row[field] = float("nan")
        return row
    return build


class FormalRuntimeAdapterContractTests(unittest.TestCase):
    """All cases are synthetic adapter-contract tests, not runtime evidence."""

    def test_writer_owned_run_id_binds_manifest_and_telemetry(self):
        with tempfile.TemporaryDirectory() as tmp:
            adapter = FormalRuntimeAdapter(Path(tmp))
            adapter.bind_manifest(_manifest_context(), origin=SYNTHETIC_TEST)
            adapter.append_telemetry(_snapshot(1, 1_000_000, 0.01, adapter.run_id), origin=SYNTHETIC_TEST)
            adapter.append_telemetry(_snapshot(2, 2_000_000, 0.02, adapter.run_id), origin=SYNTHETIC_TEST)
            adapter.write_telemetry()
            manifest = json.loads((Path(tmp) / "manifest.json").read_text(encoding="utf-8"))
            telemetry_lines = (Path(tmp) / "telemetry.csv").read_text(encoding="utf-8").splitlines()
            self.assertEqual(manifest["run_id"], adapter.run_id)
            self.assertEqual(telemetry_lines[0].split(",")[0], "run_id")
            self.assertTrue(all(line.startswith(adapter.run_id) for line in telemetry_lines[1:]))

    def test_mismatched_run_id_is_rejected_before_write(self):
        with tempfile.TemporaryDirectory() as tmp:
            adapter = FormalRuntimeAdapter(Path(tmp))
            adapter.bind_manifest(_manifest_context(), origin=SYNTHETIC_TEST)
            with self.assertRaises(AdapterValidationError):
                adapter.append_telemetry(_snapshot(1, 1_000_000, 0.01, "other-run"), origin=SYNTHETIC_TEST)
            self.assertFalse((Path(tmp) / "telemetry.csv").exists())

    def test_missing_required_manifest_section_fails_closed(self):
        with tempfile.TemporaryDirectory() as tmp:
            adapter = FormalRuntimeAdapter(Path(tmp))
            context = _manifest_context()
            del context["rates_hz"]
            with self.assertRaisesRegex(AdapterValidationError, "missing manifest section"):
                adapter.bind_manifest(context, origin=SYNTHETIC_TEST)
            self.assertFalse((Path(tmp) / "manifest.json").exists())

    def test_missing_required_telemetry_field_fails_closed(self):
        with tempfile.TemporaryDirectory() as tmp:
            adapter = FormalRuntimeAdapter(Path(tmp))
            adapter.bind_manifest(_manifest_context(), origin=SYNTHETIC_TEST)
            row = _snapshot(1, 1_000_000, 0.01, adapter.run_id)
            del row["ray_log2_05"]
            with self.assertRaisesRegex(AdapterValidationError, "missing telemetry field"):
                adapter.append_telemetry(row, origin=SYNTHETIC_TEST)
            self.assertFalse((Path(tmp) / "telemetry.csv").exists())

    def test_nonfinite_11_ray_values_fail_closed(self):
        for field in (f"ray_log2_{i:02d}" for i in range(11)):
            with self.subTest(field=field), tempfile.TemporaryDirectory() as tmp:
                adapter = FormalRuntimeAdapter(Path(tmp))
                adapter.bind_manifest(_manifest_context(), origin=SYNTHETIC_TEST)
                with self.assertRaisesRegex(AdapterValidationError, "non-finite"):
                    adapter.append_telemetry(_snapshot_with_nan(field)(adapter.run_id), origin=SYNTHETIC_TEST)
                self.assertFalse((Path(tmp) / "telemetry.csv").exists())

    def test_nonfinite_command_chain_values_fail_closed(self):
        fields = [f"{prefix}_{index:02d}" for prefix in ("action_raw", "action_clipped", "joint_target_rad", "torque_nm", "torque_saturated") for index in range(12)]
        for field in fields:
            with self.subTest(field=field), tempfile.TemporaryDirectory() as tmp:
                adapter = FormalRuntimeAdapter(Path(tmp))
                adapter.bind_manifest(_manifest_context(), origin=SYNTHETIC_TEST)
                with self.assertRaisesRegex(AdapterValidationError, "non-finite"):
                    adapter.append_telemetry(_snapshot_with_nan(field)(adapter.run_id), origin=SYNTHETIC_TEST)
                self.assertFalse((Path(tmp) / "telemetry.csv").exists())

    def test_non_monotonic_clock_is_rejected_as_inconsistent(self):
        with tempfile.TemporaryDirectory() as tmp:
            adapter = FormalRuntimeAdapter(Path(tmp))
            adapter.bind_manifest(_manifest_context(), origin=SYNTHETIC_TEST)
            adapter.append_telemetry(_snapshot(1, 1_000_000, 0.01, adapter.run_id), origin=SYNTHETIC_TEST)
            with self.assertRaisesRegex(AdapterValidationError, "non-monotonic"):
                adapter.append_telemetry(_snapshot(1, 1_000_000, 0.01, adapter.run_id), origin=SYNTHETIC_TEST)

    def test_unresolved_numeric_manifest_field_is_rejected_not_invented(self):
        with tempfile.TemporaryDirectory() as tmp:
            adapter = FormalRuntimeAdapter(Path(tmp))
            context = _manifest_context()
            context["rates_hz"]["policy"] = "UNKNOWN"
            with self.assertRaisesRegex(AdapterValidationError, "has no authoritative source"):
                adapter.bind_manifest(context, origin=SYNTHETIC_TEST)
            self.assertFalse((Path(tmp) / "manifest.json").exists())

    def test_unknown_source_provenance_is_preserved_not_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            adapter = FormalRuntimeAdapter(Path(tmp))
            adapter.bind_manifest(_manifest_context(), origin=SYNTHETIC_TEST)
            manifest = json.loads((Path(tmp) / "manifest.json").read_text(encoding="utf-8"))
            self.assertEqual(manifest["models"]["agile_policy"]["source_provenance"], "UNKNOWN")

    def test_legacy_evaluator_artifact_cannot_be_upgraded_to_valid(self):
        with tempfile.TemporaryDirectory() as tmp:
            (Path(tmp) / "summary.json").write_text(
                json.dumps({"classification": "LEGACY / NON-ACCEPTANCE"}), encoding="utf-8"
            )
            adapter = FormalRuntimeAdapter(Path(tmp))  # owns a fresh run_id, writes nothing
            result = validate_run(Path(tmp))
            self.assertEqual(result.episode_state, "LEGACY / NON-ACCEPTANCE")
            self.assertNotEqual(result.episode_state, "VALID")
            self.assertNotEqual(result.run_id, adapter.run_id)

    def test_adapter_never_upgrades_partial_artifacts_to_valid(self):
        with tempfile.TemporaryDirectory() as tmp:
            adapter = FormalRuntimeAdapter(Path(tmp))
            adapter.bind_manifest(_manifest_context(), origin=SYNTHETIC_TEST)
            result = validate_run(Path(tmp))
            self.assertEqual(result.episode_state, "INVALID")
            self.assertNotEqual(result.episode_state, "VALID")

    # ------------------------------------------------- input-origin boundary

    def test_legacy_origin_rejects_manifest_and_snapshot_before_write(self):
        with tempfile.TemporaryDirectory() as tmp:
            adapter = FormalRuntimeAdapter(Path(tmp))
            legacy_context = _manifest_context()
            legacy_context["classification"] = "LEGACY / NON-ACCEPTANCE"
            with self.assertRaisesRegex(AdapterValidationError, "LEGACY_ONLY"):
                adapter.bind_manifest(legacy_context, origin=LEGACY_ONLY)
            self.assertFalse((Path(tmp) / "manifest.json").exists())
            with self.assertRaisesRegex(AdapterValidationError, "LEGACY_ONLY"):
                adapter.append_telemetry(_snapshot(1, 1_000_000, 0.01, adapter.run_id), origin=LEGACY_ONLY)
            self.assertFalse((Path(tmp) / "telemetry.csv").exists())

    def test_missing_origin_rejects_before_write(self):
        with tempfile.TemporaryDirectory() as tmp:
            adapter = FormalRuntimeAdapter(Path(tmp))
            with self.assertRaisesRegex(AdapterValidationError, "missing adapter input origin"):
                adapter.bind_manifest(_manifest_context())
            self.assertFalse((Path(tmp) / "manifest.json").exists())
            with self.assertRaisesRegex(AdapterValidationError, "missing adapter input origin"):
                adapter.append_telemetry(_snapshot(1, 1_000_000, 0.01, adapter.run_id))
            self.assertFalse((Path(tmp) / "telemetry.csv").exists())

    def test_unrecognized_origin_rejects_before_write(self):
        with tempfile.TemporaryDirectory() as tmp:
            adapter = FormalRuntimeAdapter(Path(tmp))
            with self.assertRaisesRegex(AdapterValidationError, "unrecognized adapter input origin"):
                adapter.bind_manifest(_manifest_context(), origin="RANDOM_ORIGIN")
            self.assertFalse((Path(tmp) / "manifest.json").exists())
            with self.assertRaisesRegex(AdapterValidationError, "unrecognized adapter input origin"):
                adapter.append_telemetry(_snapshot(1, 1_000_000, 0.01, adapter.run_id), origin=42)
            self.assertFalse((Path(tmp) / "telemetry.csv").exists())

    def test_authoritative_runtime_origin_not_available_until_wired(self):
        with tempfile.TemporaryDirectory() as tmp:
            adapter = FormalRuntimeAdapter(Path(tmp))
            with self.assertRaisesRegex(AdapterValidationError, "no authoritative runtime producer is wired"):
                adapter.bind_manifest(_manifest_context(), origin=AUTHORITATIVE_RUNTIME)
            self.assertFalse((Path(tmp) / "manifest.json").exists())

    def test_synthetic_origin_is_recorded_as_test_only(self):
        with tempfile.TemporaryDirectory() as tmp:
            adapter = FormalRuntimeAdapter(Path(tmp))
            adapter.bind_manifest(_manifest_context(), origin=SYNTHETIC_TEST)
            sidecar = json.loads((Path(tmp) / ORIGIN_SIDECAR).read_text(encoding="utf-8"))
            self.assertEqual(sidecar["origin"], SYNTHETIC_TEST)
            self.assertEqual(sidecar["evidence_class"], "synthetic-test-only")


if __name__ == "__main__":
    unittest.main(verbosity=2)
