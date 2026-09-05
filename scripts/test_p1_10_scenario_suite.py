#!/usr/bin/env python3
"""Offline P1-10 scenario/seed/preflight tests; no runtime is started."""

from __future__ import annotations

import argparse
import copy
import json
import sys
import tempfile
import unittest
from pathlib import Path

SCRIPTS = Path(__file__).resolve().parent
REPO = SCRIPTS.parent
sys.path.insert(0, str(SCRIPTS))

import p1_10_scenario_suite as suite  # noqa: E402
from formal_experiment_contract import derive_seed  # noqa: E402


class P110ScenarioSuiteTests(unittest.TestCase):
    def test_suite_is_valid_and_contains_required_minimum(self):
        result = suite.validate_suite_manifest()
        self.assertTrue(result["valid"], result["errors"])
        entries = result["suite"]["scenarios"]
        self.assertEqual({e["scenario_id"] for e in entries}, {
            "flat_goal_forward", "flat_goal_lateral", "static_obstacle_authority_unavailable"})
        self.assertEqual(sum(e["status"] == "SUPPORTED" for e in entries), 2)
        self.assertEqual(sum(e["status"] == "UNSUPPORTED" for e in entries), 1)

    def test_same_scenario_seed_is_byte_stable(self):
        first = suite.resolve_scenario("flat_goal_forward", 12345, "stabilized")
        second = suite.resolve_scenario("flat_goal_forward", 12345, "stabilized")
        self.assertEqual(suite.canonical_json(first), suite.canonical_json(second))
        self.assertEqual(first["seeds"], second["seeds"])
        self.assertEqual(first["pairing"], second["pairing"])

    def test_different_root_seed_changes_derived_seed_and_pairing(self):
        first = suite.resolve_scenario("flat_goal_forward", 1)
        second = suite.resolve_scenario("flat_goal_forward", 2)
        self.assertEqual(first["seeds"]["derived_seeds"], {})
        self.assertEqual(second["seeds"]["derived_seeds"], {})
        self.assertEqual(first["seeds"]["sources"]["python_orchestrator"]["status"], "DECLARED_NOT_CONSUMED")
        self.assertNotEqual(first["pairing"]["scenario_root_seed_key"], second["pairing"]["scenario_root_seed_key"])
        self.assertNotEqual(first["pairing"]["p1_02_pairing_key"], second["pairing"]["p1_02_pairing_key"])

    def test_hash_dependencies_are_explicit(self):
        source = json.loads((suite.SCENARIO_DIR / "flat_goal_forward.json").read_text())
        original_hash = suite.sha256_file(suite.SCENARIO_DIR / "flat_goal_forward.json")
        for path in ("goal", "scene", "config_overrides", "random_sources"):
            mutated = copy.deepcopy(source)
            if path == "goal":
                mutated["goal"]["world_xy_m"][0] = 7.25
            elif path == "scene":
                mutated["scene"]["model_closure_sha256"] = "0" * 64
            elif path == "config_overrides":
                mutated["config_overrides"]["abs.unregistered"] = 1
            else:
                mutated["random_sources"]["python_orchestrator"]["derived_label"] = "changed"
            mutated_hash = suite.sha256_bytes(suite.canonical_json(mutated).encode())
            self.assertNotEqual(original_hash, mutated_hash, path)
        initial = copy.deepcopy(source)
        initial["initial_state"]["qpos"][2] = 0.5
        self.assertIn("initial_state.qpos_sha256:does_not_match_qpos", suite.validate_scenario_document(initial))
        source_mutated = copy.deepcopy(source)
        source_mutated["initial_state_source"]["startup_path"] = "wrong"
        self.assertIn("initial_state_source.startup_path:mismatch", suite.validate_scenario_document(source_mutated))
        hash_mutated = copy.deepcopy(source)
        hash_mutated["initial_state"]["qpos_sha256"] = "0" * 64
        self.assertIn("initial_state.qpos_sha256:does_not_match_qpos", suite.validate_scenario_document(hash_mutated))
        suite_result = suite.validate_suite_manifest()
        self.assertNotEqual(suite_result["suite_sha256"], suite.sha256_bytes(
            suite.canonical_json({"suite": suite_result["suite"], "mutation": True}).encode()))

    def test_fail_closed_missing_escape_override_and_unsupported_source(self):
        source = json.loads((suite.SCENARIO_DIR / "flat_goal_forward.json").read_text())
        missing = copy.deepcopy(source)
        del missing["goal"]
        self.assertTrue(any("goal:missing" in e for e in suite.validate_scenario_document(missing)))
        escaped = copy.deepcopy(source)
        escaped["scene"]["root_xml"] = "../outside.xml"
        self.assertTrue(any("path_escape" in e for e in suite.validate_scenario_document(escaped)))
        override = copy.deepcopy(source)
        override["config_overrides"] = {"abs.unknown": 1}
        self.assertIn("config_overrides:nonempty_without_registered_override", suite.validate_scenario_document(override))
        random_unknown = copy.deepcopy(source)
        random_unknown["random_sources"]["cpp_rand"]["status"] = "UNSUPPORTED"
        self.assertIn("random_sources:unsupported_source_in_supported_scenario", suite.validate_scenario_document(random_unknown))

    def test_goal_resample_true_is_rejected_for_formal_scenario(self):
        source = json.loads((suite.SCENARIO_DIR / "flat_goal_forward.json").read_text())
        source["goal"]["resample_goal_on_arrival"] = True
        self.assertIn("goal.resample_goal_on_arrival:formal_scenario_must_be_false",
                      suite.validate_scenario_document(source))

    def test_obstacle_slot_is_explicitly_unsupported_and_cannot_resolve(self):
        result = suite.validate_suite_manifest()
        self.assertTrue(result["valid"], result["errors"])
        with self.assertRaisesRegex(ValueError, "scenario_status_unsupported"):
            suite.resolve_scenario("static_obstacle_authority_unavailable", 7)

    def _args(self, **changes):
        values = dict(scenario="flat_goal_forward", root_seed=7, variant="stabilized",
                      scene="scene_flat.xml", manifest=str(suite.BASELINE_MANIFEST), window_s=25.0,
                      initial_state_source="scene_default")
        values.update(changes)
        return argparse.Namespace(**values)

    def test_capture_arguments_match_resolved_manifest(self):
        resolved = suite.prepare_capture_context(self._args())
        self.assertEqual(resolved["launch_contract"]["scene"], "scene_flat.xml")
        self.assertEqual(resolved["launch_contract"]["root_seed"], 7)
        self.assertEqual(resolved["launch_contract"]["initial_state_source"], "scene_default")
        self.assertEqual(resolved["baseline"]["identity_sha256"], "59dd13fed5ebd026ec519f2659643237502be8e4d8df5174a65b7d35ceb4f7e0")
        self.assertEqual(resolved["initial_state"]["qpos_sha256"], "a604dd11dc57ea655bf6d746dcf068a91e80a0a1eddc73d20c1a3800468f59d8")
        self.assertEqual(resolved["formal_context"]["scenario_sha256"], resolved["scenario_sha256"])
        self.assertIn("python_orchestrator", resolved["formal_context"]["derived_seed_registry"])
        self.assertIn("ABS_P1_10_DERIVED_SEED_REGISTRY", resolved["process_context"]["environment"])
        for bad in ({"scene": "scene_test3.xml"}, {"window_s": 24.0},
                    {"manifest": str(REPO / "docs/evidence/P1-08/P1-08_baseline_identity.json")},
                    {"initial_state_source": "keyframe:home"}):
            with self.assertRaises(ValueError):
                suite.prepare_capture_context(self._args(**bad))
        tampered = copy.deepcopy(resolved)
        tampered["scene"]["launch_arg"] = "other.xml"
        with self.assertRaises(ValueError):
            suite.validate_launch_arguments(self._args(), tampered)
        tampered_variant = copy.deepcopy(resolved)
        tampered_variant["process_context"]["environment"]["ABS_P1_10_VARIANT"] = "paper-faithful"
        with self.assertRaises(ValueError):
            suite.validate_resolved_variant_context(tampered_variant)

    def test_actual_initial_state_and_supported_variant_binding(self):
        resolved = suite.resolve_scenario("flat_goal_forward", 7, "stabilized")
        actual = suite.extract_runtime_initial_state(suite.REPO / "unitree_mujoco/unitree_robots/go2/scene_flat.xml")
        self.assertEqual(resolved["initial_state"]["qpos"], actual["qpos"])
        self.assertEqual(resolved["initial_state"]["base_pose_world_m"], [0.0, 0.0, 0.445])
        self.assertEqual(resolved["initial_state"]["yaw_rad"], 0.0)
        self.assertEqual(resolved["variant_binding"]["status"], "SUPPORTED")
        self.assertEqual(resolved["variant_binding"]["runtime_configuration"]["switching_mode"], "stabilized_switch")
        self.assertEqual(resolved["variant_binding"]["binding_sha256"], "2f0dfc4e8bf5237a578d99030facc38459fd5f899af49b508e48e29b7e8a4e1c")

    def test_unbound_variants_fail_closed(self):
        for label in ("paper-faithful", "agile-only"):
            with self.assertRaisesRegex(ValueError, "variant_not_supported"):
                suite.resolve_scenario("flat_goal_forward", 7, label)

    def test_paired_variant_labels_share_scenario_seed_tuple(self):
        stabilized = suite.resolve_scenario("flat_goal_forward", 17, "stabilized")
        unsupported_label = copy.deepcopy(stabilized)
        unsupported_label["pairing"]["variant"] = "paper-faithful"
        unsupported_label["variant_binding"] = {"label": "paper-faithful", "status": "UNSUPPORTED"}
        agile_label = copy.deepcopy(stabilized)
        agile_label["pairing"]["variant"] = "agile-only"
        agile_label["variant_binding"] = {"label": "agile-only", "status": "UNSUPPORTED"}
        contexts = [stabilized, unsupported_label, agile_label]
        self.assertIn("paired_variant_not_supported_by_runtime_binding", suite.validate_paired_contexts(contexts))
        cross_pair = [stabilized, copy.deepcopy(stabilized), copy.deepcopy(stabilized)]
        cross_pair[1]["seeds"]["root_seed"] = 18
        cross_pair[1]["pairing"]["scenario_root_seed_key"] = "different"
        self.assertIn("paired_scenario_root_seed_key_mismatch", suite.validate_paired_contexts(cross_pair))

    def test_resolved_manifest_write_is_deterministic(self):
        resolved = suite.resolve_scenario("flat_goal_lateral", 99, "stabilized")
        with tempfile.TemporaryDirectory() as td:
            path = Path(td) / "resolved.json"
            suite.write_resolved_manifest(path, resolved)
            first = path.read_bytes()
            suite.write_resolved_manifest(path, resolved)
            self.assertEqual(first, path.read_bytes())
            self.assertEqual(json.loads(path.read_text())["goal"]["world_xy_m"], [7.0, 1.5])


if __name__ == "__main__":
    unittest.main(verbosity=2)
