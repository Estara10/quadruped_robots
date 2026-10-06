#!/usr/bin/env python3
"""Deterministic mechanical tests for the P1-02 formal experiment contract."""

from __future__ import annotations

import copy
import csv
import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

from formal_experiment_contract import (
    REQUIRED_PLOTS,
    REQUIRED_TELEMETRY_FIELDS,
    FormalRunWriter,
    derive_seed,
    pairing_key,
    schema_errors,
    validate_run,
    validate_comparison_manifests,
    validate_variant_group,
)


HASH = "a" * 64


def manifest(run_id: str = "fixture-run", variant: str = "paper-faithful"):
    data = {
        "schema_version": "abs-go2-formal-run/v1",
        "run_id": run_id,
        "created_at": "2026-08-25T00:00:00+08:00",
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
        "seeds": {"root_seed": 17, "sources": {name: derive_seed(17, name) for name in ("scene_generator", "controller_goal", "perception", "evaluator")}},
        "perception": {"source": "fixture", "version": "v1", "sha256": HASH, "frame_contract_version": "fixture/v1"},
        "rates_hz": {"controller": 200, "pd": 200, "policy": 50, "ra": 50, "perception": 50},
        "thresholds": {"arrival_region_m": 0.5, "arrival_hold_s": 0.5, "fall_height_m": 0.2, "fall_angle_rad": 0.8, "collision_definition_id": "fixture-v1", "ra_entry_threshold": -0.05, "ra_exit_threshold": -0.08},
    }
    data["pairing_key"] = pairing_key(data)
    return data


def telemetry_row(sequence: int, monotonic_ns: int, simulation_time_s: float, run_id: str = "fixture-run"):
    row = {field: 0 for field in REQUIRED_TELEMETRY_FIELDS}
    row.update({
        "sequence": sequence,
        "run_id": run_id,
        "monotonic_time_ns": monotonic_ns,
        "simulation_time_s": simulation_time_s,
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


def event(sequence: int, event_type: str, outcome=None, run_id: str = "fixture-run"):
    payload = {
        "run_id": run_id,
        "sequence": sequence,
        "monotonic_time_ns": sequence * 1_000_000,
        "simulation_time_s": sequence * 0.01,
        "type": event_type,
    }
    if outcome is not None:
        payload["outcome"] = outcome
    return payload


def write_valid_run(path: Path, variant: str = "paper-faithful") -> FormalRunWriter:
    writer = FormalRunWriter(path)
    writer.write_manifest(manifest(run_id=writer.run_id, variant=variant))
    writer.write_telemetry([telemetry_row(1, 1_000_000, 0.01, writer.run_id), telemetry_row(2, 2_000_000, 0.02, writer.run_id)])
    for item in [
        event(1, "episode_start", run_id=writer.run_id), event(2, "controller_active", run_id=writer.run_id), event(3, "rl_entered", run_id=writer.run_id),
        event(4, "valid_ready", run_id=writer.run_id), event(5, "recovery_enter", run_id=writer.run_id), event(6, "recovery_exit", run_id=writer.run_id),
        event(7, "arrival_start", run_id=writer.run_id), event(8, "arrival_accepted", run_id=writer.run_id), event(9, "terminal", "SUCCESS", run_id=writer.run_id),
        event(10, "shutdown", run_id=writer.run_id),
    ]:
        writer.emit_event(item)
    for plot in REQUIRED_PLOTS:
        writer.write_data_plot(plot, data_points=2)
    writer.write_summary({
        "run_id": writer.run_id,
        "validity": "VALID",
        "terminal_outcome": "SUCCESS",
        "invalid_reasons": [],
        "metrics": {"switch_count": 2, "path_length_m": 1.0},
    })
    return writer


class FormalExperimentContractTests(unittest.TestCase):
    def test_valid_fixture_is_valid(self):
        with tempfile.TemporaryDirectory() as tmp:
            write_valid_run(Path(tmp))
            result = validate_run(Path(tmp))
            self.assertTrue(result.validator_completed)
            self.assertEqual(result.episode_state, "VALID")
            self.assertEqual(result.reasons, [])

    def test_manifest_completeness_and_schema_file(self):
        schema_path = Path(__file__).resolve().parents[1] / "schemas" / "formal_experiment_run_v1.json"
        schema = json.loads(schema_path.read_text(encoding="utf-8"))
        self.assertEqual(schema["$id"], "abs-go2-formal-run/v1")
        with tempfile.TemporaryDirectory() as tmp:
            writer = write_valid_run(Path(tmp))
            data = json.loads(writer.manifest_path.read_text(encoding="utf-8"))
            del data["environment"]["solver"]
            writer.manifest_path.write_text(json.dumps(data), encoding="utf-8")
            result = validate_run(Path(tmp))
            self.assertEqual(result.episode_state, "INVALID")
            self.assertIn("schema:$.environment.solver:required", result.reasons)

    def test_schema_rejects_nested_variant_hash_and_pairing_errors(self):
        base = manifest()
        self.assertEqual(schema_errors({**base, "schema_version": "wrong/v1"}), ["$.schema_version:const"])
        bad_variant = copy.deepcopy(base)
        bad_variant["variant"] = "legacy"
        self.assertIn("$.variant:enum", schema_errors(bad_variant))
        bad_hash = copy.deepcopy(base)
        bad_hash["models"]["agile_policy"]["sha256"] = "bad"
        self.assertIn("$.models.agile_policy.sha256:pattern", schema_errors(bad_hash))
        with tempfile.TemporaryDirectory() as tmp:
            writer = write_valid_run(Path(tmp))
            data = json.loads(writer.manifest_path.read_text(encoding="utf-8"))
            data["pairing_key"] = "b" * 64
            writer.manifest_path.write_text(json.dumps(data), encoding="utf-8")
            result = validate_run(Path(tmp))
            self.assertIn("pairing_key_mismatch", result.reasons)

    def test_ordering_and_required_artifact_failures_are_invalid(self):
        with tempfile.TemporaryDirectory() as tmp:
            writer = write_valid_run(Path(tmp))
            events = writer.events_path.read_text(encoding="utf-8").splitlines()
            mutated = json.loads(events[5])
            mutated["sequence"] = 3
            events[5] = json.dumps(mutated)
            writer.events_path.write_text("\n".join(events) + "\n", encoding="utf-8")
            (Path(tmp) / "plots" / REQUIRED_PLOTS[0]).unlink()
            result = validate_run(Path(tmp))
            self.assertEqual(result.episode_state, "INVALID")
            self.assertIn("non_monotonic_events", result.reasons)
            self.assertIn("missing_plot:" + REQUIRED_PLOTS[0], result.reasons)

    def test_validity_matrix_rejects_missing_runtime_contract_artifacts(self):
        cases = (("manifest", "missing_manifest"), ("events", "missing_events"), ("telemetry", "missing_telemetry"))
        for artifact, reason in cases:
            with self.subTest(artifact=artifact), tempfile.TemporaryDirectory() as tmp:
                writer = write_valid_run(Path(tmp))
                {"manifest": writer.manifest_path, "events": writer.events_path, "telemetry": writer.telemetry_path}[artifact].unlink()
                result = validate_run(Path(tmp))
                self.assertEqual(result.episode_state, "INVALID")
                self.assertIn(reason, result.reasons)

    def test_stale_or_invalid_runtime_telemetry_is_invalid(self):
        with tempfile.TemporaryDirectory() as tmp:
            writer = write_valid_run(Path(tmp))
            with writer.telemetry_path.open(encoding="utf-8", newline="") as handle:
                rows = list(csv.DictReader(handle))
            rows[0]["telemetry_fresh"] = "0"
            rows[0]["ray_valid"] = "0"
            writer.write_telemetry(rows)
            result = validate_run(Path(tmp))
            self.assertEqual(result.episode_state, "INVALID")
            self.assertIn("telemetry_stale", result.reasons)
            self.assertIn("perception_invalid", result.reasons)

    def test_wrong_run_and_incomplete_or_nonfinite_vectors_are_invalid(self):
        with tempfile.TemporaryDirectory() as tmp:
            writer = write_valid_run(Path(tmp))
            with writer.telemetry_path.open(encoding="utf-8", newline="") as handle:
                rows = list(csv.DictReader(handle))
            rows[0]["run_id"] = "other-run"
            rows[1]["action_raw_00"] = "nan"
            fields = [field for field in REQUIRED_TELEMETRY_FIELDS if field != "ray_log2_10"]
            with writer.telemetry_path.open("w", encoding="utf-8", newline="") as handle:
                out = csv.DictWriter(handle, fieldnames=fields, extrasaction="ignore")
                out.writeheader(); out.writerows(rows)
            events = [json.loads(line) for line in writer.events_path.read_text(encoding="utf-8").splitlines()]
            events[0]["run_id"] = "other-run"
            writer.events_path.write_text("\n".join(json.dumps(item) for item in events) + "\n", encoding="utf-8")
            result = validate_run(Path(tmp))
            self.assertEqual(result.episode_state, "INVALID")
            self.assertIn("wrong_run_telemetry:1", result.reasons)
            self.assertIn("wrong_run_event:1", result.reasons)
            self.assertTrue(any(reason.startswith("missing_telemetry_fields:ray_log2_10") for reason in result.reasons))
            self.assertIn("non_finite_telemetry:action_raw_00", result.reasons)

    def test_summary_hash_and_placeholder_plot_cannot_make_valid(self):
        with tempfile.TemporaryDirectory() as tmp:
            writer = write_valid_run(Path(tmp))
            summary = json.loads(writer.summary_path.read_text(encoding="utf-8"))
            summary["run_id"] = "other-run"
            summary["artifact_hashes"]["telemetry"] = "b" * 64
            writer.summary_path.write_text(json.dumps(summary), encoding="utf-8")
            (Path(tmp) / "plots" / REQUIRED_PLOTS[0]).write_text("<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"1\" height=\"1\"/>", encoding="utf-8")
            result = validate_run(Path(tmp))
            self.assertEqual(result.episode_state, "INVALID")
            self.assertIn("summary_run_id_mismatch", result.reasons)
            self.assertIn("artifact_hash_mismatch:telemetry", result.reasons)
            self.assertIn("placeholder_plot:" + REQUIRED_PLOTS[0], result.reasons)

    def test_safety_event_vetoes_arrival_success(self):
        with tempfile.TemporaryDirectory() as tmp:
            writer = write_valid_run(Path(tmp))
            events = [json.loads(line) for line in writer.events_path.read_text(encoding="utf-8").splitlines()]
            events.insert(7, event(8, "collision_start"))
            for index, item in enumerate(events, start=1):
                item["sequence"] = index
                item["monotonic_time_ns"] = index * 1_000_000
                item["simulation_time_s"] = index * 0.01
            writer.events_path.write_text("\n".join(json.dumps(item) for item in events) + "\n", encoding="utf-8")
            result = validate_run(Path(tmp))
            self.assertEqual(result.episode_state, "INVALID")
            self.assertIn("safety_event_vetoes_arrival", result.reasons)

    def test_nonfinite_or_wrong_type_event_clock_is_invalid_not_exception(self):
        for bad_value in (float("nan"), float("inf"), "not-an-integer"):
            with self.subTest(bad_value=str(bad_value)), tempfile.TemporaryDirectory() as tmp:
                writer = write_valid_run(Path(tmp))
                events = [json.loads(line) for line in writer.events_path.read_text(encoding="utf-8").splitlines()]
                events[1]["monotonic_time_ns"] = bad_value
                writer.events_path.write_text("\n".join(json.dumps(item) for item in events) + "\n", encoding="utf-8")
                result = validate_run(Path(tmp))
                self.assertEqual(result.episode_state, "INVALID")
                self.assertIn("invalid_event_clock:2", result.reasons)

    def test_missing_event_clock_field_is_invalid_not_exception(self):
        with tempfile.TemporaryDirectory() as tmp:
            writer = write_valid_run(Path(tmp))
            events = [json.loads(line) for line in writer.events_path.read_text(encoding="utf-8").splitlines()]
            del events[1]["simulation_time_s"]
            writer.events_path.write_text("\n".join(json.dumps(item) for item in events) + "\n", encoding="utf-8")
            result = validate_run(Path(tmp))
            self.assertEqual(result.episode_state, "INVALID")
            self.assertIn("missing_event_field:2", result.reasons)

    def test_telemetry_safety_evidence_requires_event_and_vetoes_success(self):
        with tempfile.TemporaryDirectory() as tmp:
            writer = write_valid_run(Path(tmp))
            with writer.telemetry_path.open(encoding="utf-8", newline="") as handle:
                rows = list(csv.DictReader(handle))
            rows[0]["collision"] = "1"
            writer.write_telemetry(rows)
            result = validate_run(Path(tmp))
            self.assertEqual(result.episode_state, "INVALID")
            self.assertIn("telemetry_collision_without_collision_event", result.reasons)
            self.assertIn("safety_evidence_vetoes_success", result.reasons)

    def test_safety_event_and_telemetry_cannot_be_overridden_by_success(self):
        with tempfile.TemporaryDirectory() as tmp:
            writer = write_valid_run(Path(tmp))
            with writer.telemetry_path.open(encoding="utf-8", newline="") as handle:
                rows = list(csv.DictReader(handle))
            rows[0]["fall"] = "1"
            writer.write_telemetry(rows)
            events = [json.loads(line) for line in writer.events_path.read_text(encoding="utf-8").splitlines()]
            events.insert(1, event(2, "fall"))
            for index, item in enumerate(events, start=1):
                item["sequence"] = index; item["monotonic_time_ns"] = index * 1_000_000; item["simulation_time_s"] = index * 0.01
            writer.events_path.write_text("\n".join(json.dumps(item) for item in events) + "\n", encoding="utf-8")
            result = validate_run(Path(tmp))
            self.assertEqual(result.episode_state, "INVALID")
            self.assertIn("safety_evidence_vetoes_success", result.reasons)

    def test_aligned_collision_telemetry_event_and_terminal_are_valid(self):
        with tempfile.TemporaryDirectory() as tmp:
            writer = FormalRunWriter(Path(tmp))
            writer.write_manifest(manifest(run_id=writer.run_id))
            first = telemetry_row(1, 1_000_000, 0.01, writer.run_id)
            collision = telemetry_row(5, 5_000_000, 0.05, writer.run_id); collision["collision"] = 1
            writer.write_telemetry([first, collision])
            for item in [event(1, "episode_start", run_id=writer.run_id), event(2, "controller_active", run_id=writer.run_id), event(3, "rl_entered", run_id=writer.run_id), event(4, "valid_ready", run_id=writer.run_id), event(5, "collision_start", run_id=writer.run_id), event(6, "terminal", "COLLISION", run_id=writer.run_id), event(7, "shutdown", run_id=writer.run_id)]:
                writer.emit_event(item)
            for plot in REQUIRED_PLOTS:
                writer.write_data_plot(plot, data_points=2)
            writer.write_summary({"validity": "VALID", "terminal_outcome": "COLLISION", "invalid_reasons": [], "metrics": {"switch_count": 0}})
            result = validate_run(Path(tmp))
            self.assertEqual(result.episode_state, "VALID")

    def test_comparison_validation_requires_complete_unique_paired_variants(self):
        group = [manifest(run_id=f"fixture-{variant}", variant=variant) for variant in ("paper-faithful", "stabilized", "agile-only")]
        with tempfile.TemporaryDirectory() as tmp:
            paths = []
            for index, item in enumerate(group):
                path = Path(tmp) / f"{index}.json"; path.write_text(json.dumps(item), encoding="utf-8"); paths.append(path)
            result = validate_comparison_manifests(paths)
            self.assertTrue(result["comparison_valid"])
            cli = subprocess.run([sys.executable, str(Path(__file__).with_name("formal_experiment_contract.py")), "--validate-comparison", *map(str, paths)], text=True, capture_output=True, check=False)
            self.assertEqual(cli.returncode, 0, cli.stdout + cli.stderr)
            self.assertIn('"comparison_valid": true', cli.stdout)
            self.assertIn("comparison_requires_exactly_three_variants", validate_comparison_manifests(paths[:2])["errors"])
            duplicate = [paths[0], paths[0], paths[2]]
            self.assertIn("duplicate_variant_label", validate_comparison_manifests(duplicate)["errors"])
            mismatched = copy.deepcopy(group)
            mismatched[1]["scenario"]["sha256"] = "b" * 64
            mismatched[1]["pairing_key"] = pairing_key(mismatched[1])
            mismatch_path = Path(tmp) / "mismatch.json"; mismatch_path.write_text(json.dumps(mismatched[1]), encoding="utf-8")
            self.assertIn("paired_variant_key_mismatch", validate_comparison_manifests([paths[0], mismatch_path, paths[2]])["errors"])

    def test_comparison_validation_rejects_duplicate_run_ids(self):
        group = [manifest(run_id="shared-run", variant=variant) for variant in ("paper-faithful", "stabilized", "agile-only")]
        self.assertIn("duplicate_run_id", validate_variant_group(group))
        with tempfile.TemporaryDirectory() as tmp:
            paths = []
            for index, item in enumerate(group):
                path = Path(tmp) / f"duplicate-{index}.json"
                path.write_text(json.dumps(item), encoding="utf-8")
                paths.append(path)
            result = validate_comparison_manifests(paths)
            self.assertFalse(result["comparison_valid"])
            self.assertIn("duplicate_run_id", result["errors"])
            cli = subprocess.run([sys.executable, str(Path(__file__).with_name("formal_experiment_contract.py")), "--validate-comparison", *map(str, paths)], text=True, capture_output=True, check=False)
            self.assertNotEqual(cli.returncode, 0)
            self.assertIn("duplicate_run_id", cli.stdout)

    def test_comparison_validation_accepts_distinct_run_ids(self):
        group = [manifest(run_id=f"run-{variant}", variant=variant) for variant in ("paper-faithful", "stabilized", "agile-only")]
        self.assertEqual(validate_variant_group(group), [])
        with tempfile.TemporaryDirectory() as tmp:
            paths = []
            for index, item in enumerate(group):
                path = Path(tmp) / f"distinct-{index}.json"
                path.write_text(json.dumps(item), encoding="utf-8")
                paths.append(path)
            result = validate_comparison_manifests(paths)
            self.assertTrue(result["comparison_valid"])
            cli = subprocess.run([sys.executable, str(Path(__file__).with_name("formal_experiment_contract.py")), "--validate-comparison", *map(str, paths)], text=True, capture_output=True, check=False)
            self.assertEqual(cli.returncode, 0, cli.stdout + cli.stderr)

    def test_writer_allocates_distinct_run_ids(self):
        with tempfile.TemporaryDirectory() as tmp:
            first = FormalRunWriter(Path(tmp) / "first")
            second = FormalRunWriter(Path(tmp) / "second")
            self.assertNotEqual(first.run_id, second.run_id)
            self.assertRegex(first.run_id, r"^[A-Za-z0-9][A-Za-z0-9._-]{0,127}$")
            self.assertRegex(second.run_id, r"^[A-Za-z0-9][A-Za-z0-9._-]{0,127}$")

    def test_writer_rejects_caller_supplied_run_id(self):
        with tempfile.TemporaryDirectory() as tmp:
            writer = FormalRunWriter(Path(tmp) / "run")
            with self.assertRaises(ValueError):
                writer.write_manifest(manifest(run_id="caller-chosen-id"))

    def test_writer_summary_rejects_mismatched_run_id_without_overwrite(self):
        with tempfile.TemporaryDirectory() as tmp:
            writer = write_valid_run(Path(tmp) / "run")
            summary_path = writer.summary_path
            original = summary_path.read_text(encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "summary run_id must match writer-allocated run_id"):
                writer.write_summary({"run_id": "other-run", "validity": "VALID"})
            self.assertEqual(summary_path.read_text(encoding="utf-8"), original)

            missing_summary_dir = Path(tmp) / "missing-summary"
            missing_writer = FormalRunWriter(missing_summary_dir)
            self.assertFalse(missing_writer.summary_path.exists())
            with self.assertRaisesRegex(ValueError, "summary run_id must match writer-allocated run_id"):
                missing_writer.write_summary({"run_id": "other-run", "validity": "VALID"})
            self.assertFalse(missing_writer.summary_path.exists())

            writer.write_summary({"validity": "VALID", "terminal_outcome": "SUCCESS", "invalid_reasons": [], "metrics": {}})
            self.assertEqual(json.loads(summary_path.read_text(encoding="utf-8"))["run_id"], writer.run_id)
            self.assertEqual(validate_run(Path(tmp) / "run").episode_state, "VALID")

    def test_seed_derivation_and_paired_variants(self):
        self.assertNotEqual(derive_seed(23, "scene"), derive_seed(24, "scene"))
        group = [manifest(run_id=f"fixture-{variant}", variant=variant) for variant in ("paper-faithful", "stabilized", "agile-only")]
        self.assertEqual(validate_variant_group(group), [])
        mismatched = copy.deepcopy(group)
        mismatched[1]["scenario"]["sha256"] = "b" * 64
        self.assertIn("paired_variant_key_mismatch", validate_variant_group(mismatched))
        self.assertIsNotNone(pairing_key(group[0]))

    def test_legacy_is_never_reclassified_valid(self):
        with tempfile.TemporaryDirectory() as tmp:
            Path(tmp, "summary.json").write_text(json.dumps({"classification": "LEGACY / NON-ACCEPTANCE"}), encoding="utf-8")
            result = validate_run(Path(tmp))
            self.assertEqual(result.episode_state, "LEGACY / NON-ACCEPTANCE")
            self.assertNotEqual(result.episode_state, "VALID")


if __name__ == "__main__":
    unittest.main(verbosity=2)
