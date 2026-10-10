#!/usr/bin/env python3
"""Non-mutating regression checks for the historical Stage-A comparator path.

The 2026-09-07 pair is completed evidence and is never used as a test fixture.
The full comparator filesystem fixtures live in test_p1_10_saved_record_compare.py
and allocate temporary pair directories instead.
"""

from __future__ import annotations

import unittest
from pathlib import Path

import sys

SCRIPTS = Path(__file__).resolve().parent
REPO = SCRIPTS.parent
sys.path.insert(0, str(SCRIPTS))

import p1_10_stage_a_saved_record_compare as comparator  # noqa: E402


PAIR_DIR = REPO / comparator.PAIR_DIR_REL


class HistoricalStageAComparatorTests(unittest.TestCase):
    def test_historical_pair_is_not_mutated_or_retried_by_regression(self):
        # A historical pair may be inspected, but this test intentionally does
        # not create, delete, or rewrite run artifacts or comparison outputs.
        self.assertTrue((PAIR_DIR / "pair_manifest.json").is_file())
        with self.assertRaises(Exception):
            comparator.compare_pair_dir(PAIR_DIR)

    def test_external_pair_directory_is_rejected(self):
        with self.assertRaises(Exception):
            comparator.compare_pair_dir(REPO / "docs/evidence/P1-10")


if __name__ == "__main__":
    unittest.main(verbosity=2)
