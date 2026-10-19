#!/usr/bin/env python3
"""Regression tests for the historical ungated Stage-A identity boundary."""

from __future__ import annotations

import json
import subprocess
import sys
import unittest
from pathlib import Path

SCRIPTS = Path(__file__).resolve().parent
REPO = SCRIPTS.parent
sys.path.insert(0, str(SCRIPTS))

import p1_10_stage_a_common_start as common_manifest  # noqa: E402
import p1_10_stage_a_execution as stage  # noqa: E402


HISTORICAL_MANIFEST = REPO / "docs/evidence/P1-10/stage_a_execution_manifest_20260905.json"
COMMON_MANIFEST = REPO / "docs/evidence/P1-10/stage_a_common_start_execution_manifest_20260907.json"


class HistoricalStageAIdentityTests(unittest.TestCase):
    def test_old_identity_is_not_rebound_to_current_artifacts(self):
        # The old manifest is retained as historical evidence.  It must not be
        # silently made current after the common-start rebuild changed the
        # instrumented artifacts.
        original = json.loads(HISTORICAL_MANIFEST.read_text(encoding="utf-8"))
        self.assertEqual(original["status"], "FROZEN_OFFLINE_PENDING_INDEPENDENT_REVIEW")
        with self.assertRaises(stage.StageAIdentityError):
            stage.validate_manifest(
                HISTORICAL_MANIFEST, expected_executable=str(stage.EXECUTABLE))

    def test_common_start_identity_is_the_current_stage_a_path(self):
        validated = common_manifest.validate_manifest(
            COMMON_MANIFEST, expected_executable=str(common_manifest.EXECUTABLE))
        identity = validated["identity_input"]
        self.assertEqual(identity["execution_kind"], common_manifest.EXECUTION_KIND)
        self.assertEqual(identity["fixed_binding"]["common_start_enabled"], True)
        self.assertNotEqual(
            identity["artifacts"][0]["sha256"],
            identity["baseline_parent"]["accepted_executable_sha256"],
        )

    def test_p1_08_manifest_cannot_be_stage_a_identity(self):
        with self.assertRaises(stage.StageAIdentityError):
            stage.validate_manifest(REPO / "docs/evidence/P1-08/P1-08_baseline_manifest.json")

    def test_manual_capture_id_is_not_a_cli_input(self):
        result = subprocess.run(
            [sys.executable, str(REPO / "scripts/p1_08_baseline_capture.py"),
             "--out-dir", "/tmp/p1-10-manual-capture-id-test",
             "--manifest", str(REPO / "docs/evidence/P1-08/P1-08_baseline_manifest.json"),
             "--scenario", "flat_goal_forward", "--root-seed", "20260902",
             "--variant", "stabilized", "--initial-state-source", "scene_default",
             "--capture-id", "p1-10-capture-" + "0" * 32],
            cwd=str(REPO), capture_output=True, text=True,
        )
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("unrecognized arguments", result.stderr)


if __name__ == "__main__":
    unittest.main(verbosity=2)
