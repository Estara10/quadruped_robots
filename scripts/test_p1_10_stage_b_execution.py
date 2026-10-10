#!/usr/bin/env python3
"""Regression tests for the historical Stage-B identity boundary."""

from __future__ import annotations

import json
import unittest
from pathlib import Path

import sys

SCRIPTS = Path(__file__).resolve().parent
REPO = SCRIPTS.parent
sys.path.insert(0, str(SCRIPTS))

import p1_10_stage_b_execution as stage  # noqa: E402


HISTORICAL_MANIFEST = REPO / "docs/evidence/P1-10/stage_b_execution_manifest_20260905.json"


class HistoricalStageBIdentityTests(unittest.TestCase):
    def test_old_stage_b_identity_is_not_rebound_to_rebuilt_binary(self):
        original = json.loads(HISTORICAL_MANIFEST.read_text(encoding="utf-8"))
        self.assertEqual(original["status"], "FROZEN_OFFLINE_PENDING_INDEPENDENT_REVIEW")
        with self.assertRaises(stage.StageBManifestError):
            stage.validate_manifest(
                HISTORICAL_MANIFEST, expected_executable=str(stage.STAGE_B_EXECUTABLE))

    def test_p1_08_manifest_cannot_be_stage_b_identity(self):
        with self.assertRaises(stage.StageBManifestError):
            stage.validate_manifest(REPO / "docs/evidence/P1-08/P1-08_baseline_manifest.json")


if __name__ == "__main__":
    unittest.main(verbosity=2)
