#!/usr/bin/env python3
"""Offline fixtures for the P1-03 trace validator and paper Eq.22 arithmetic."""

from __future__ import annotations

import argparse
import json
import tempfile
import unittest
from copy import deepcopy
from pathlib import Path

import yaml

from validate_p1_03_trace import validate


def paper_eq22(vx: float, vy: float, wz: float, delta_t: float) -> tuple[float, float]:
    """Independent scalar transcription of ABS paper Eq.22 for test evidence only."""
    return (
        vx * delta_t - 0.5 * vy * wz * delta_t * delta_t,
        vy * delta_t + 0.5 * vx * wz * delta_t * delta_t,
    )


def first_order(vx: float, vy: float, delta_t: float) -> tuple[float, float]:
    return vx * delta_t, vy * delta_t


class TraceValidatorFixtures(unittest.TestCase):
    fixture_count = 6

    def setUp(self) -> None:
        self.temp_dir = tempfile.TemporaryDirectory()
        self.root = Path(self.temp_dir.name)
        (self.root / "paper_notes.md").write_text("paper line one\npaper line two\n", encoding="utf-8")
        (self.root / "impl.txt").write_text("alpha = 1\nbeta = 2\n", encoding="utf-8")

    def tearDown(self) -> None:
        self.temp_dir.cleanup()

    @staticmethod
    def base_record() -> dict:
        return {
            "id": "FIXTURE-001",
            "kind": "fixture",
            "paper": {
                "source": "paper_notes.md:1-2",
                "expression": "fixture expression",
                "variables": ["x"],
            },
            "training": {
                "path": "impl.txt",
                "symbol": "alpha",
                "line_range": "1-1",
                "expression_or_value": "alpha = 1",
            },
            "reference": {"status": "ABSENT", "path": "NONE", "symbol": "NONE"},
            "runtime": {
                "path": "impl.txt",
                "symbol": "beta",
                "line_range": "2-2",
                "expression_or_value": "beta = 2",
            },
            "classification": "MISMATCH",
            "confidence": "DIRECT_SOURCE",
            "claims_not_made": ["fixture only"],
            "validation": {"fixture": "NONE", "method": "offline only"},
        }

    def validate_fixture(self, record: dict) -> list[str]:
        trace = self.root / "trace.yaml"
        trace.write_text(
            yaml.safe_dump(
                {"schema_version": "abs-go2-formula-parameter-trace/v1", "records": [record]},
                sort_keys=False,
            ),
            encoding="utf-8",
        )
        return validate(self.root, trace)

    def test_unknown_reference_preserves_explicit_nonclaim(self) -> None:
        record = self.base_record()
        record["classification"] = "UNKNOWN"
        record["reference"] = {"status": "UNKNOWN", "path": "NONE", "symbol": "NONE"}
        self.assertEqual(self.validate_fixture(record), [])

    def test_present_missing_source_is_rejected(self) -> None:
        record = self.base_record()
        record["reference"] = {
            "status": "PRESENT", "path": "missing_reference.py", "symbol": "anchor", "line_range": "1-1",
        }
        self.assertTrue(any("reference path missing" in error for error in self.validate_fixture(record)))

    def test_present_reference_without_range_is_rejected(self) -> None:
        record = self.base_record()
        record["reference"] = {"status": "PRESENT", "path": "impl.txt", "symbol": "alpha"}
        self.assertTrue(any("reference line_range invalid" in error for error in self.validate_fixture(record)))

    def test_deliberate_mismatch_is_not_reclassified(self) -> None:
        record = self.base_record()
        self.assertEqual(record["classification"], "MISMATCH")
        self.assertEqual(self.validate_fixture(record), [])

    def test_symbol_outside_declared_range_is_rejected(self) -> None:
        record = self.base_record()
        record["training"]["line_range"] = "2-2"
        self.assertTrue(any("training cited symbol not within line_range" in error for error in self.validate_fixture(record)))

    def test_bad_paper_notes_range_is_rejected(self) -> None:
        record = self.base_record()
        record["paper"]["source"] = "paper_notes.md:99-100"
        self.assertTrue(any("paper-notes line_range invalid" in error for error in self.validate_fixture(record)))


class Eq22ArithmeticFixtures(unittest.TestCase):
    def test_nonzero_yaw_retains_second_order_terms(self) -> None:
        vx, vy, wz, delta_t = 1.2, -0.4, 2.0, 0.05
        dx, dy = paper_eq22(vx, vy, wz, delta_t)
        first_dx, first_dy = first_order(vx, vy, delta_t)
        self.assertAlmostEqual(dx, 0.061)
        self.assertAlmostEqual(dy, -0.017)
        self.assertNotEqual((dx, dy), (first_dx, first_dy))

    def test_zero_yaw_degenerates_to_first_order(self) -> None:
        vx, vy, wz, delta_t = 1.2, -0.4, 0.0, 0.05
        self.assertEqual(paper_eq22(vx, vy, wz, delta_t), first_order(vx, vy, delta_t))


class ActualTrace(unittest.TestCase):
    def test_all_current_records_pass_strengthened_validator(self) -> None:
        root = Path(__file__).resolve().parents[1]
        self.assertEqual(validate(root, root / "docs/evidence/P1-03/formula_parameter_trace.yaml"), [])


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--json-out", default="")
    args = parser.parse_args()
    suite = unittest.defaultTestLoader.loadTestsFromModule(__import__(__name__))
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    output = {
        "task": "P1-03",
        "scope": "offline trace mechanical validation and paper Eq.22 arithmetic fixture",
        "fixtures": TraceValidatorFixtures.fixture_count,
        "eq22_cases": 2,
        "tests_run": result.testsRun,
        "failures": len(result.failures),
        "errors": len(result.errors),
        "result": "PASS" if result.wasSuccessful() else "FAIL",
        "nonzero_yaw_expected": {"dx": 0.061, "dy": -0.017},
        "zero_yaw_expected": {"dx": 0.06, "dy": -0.02},
    }
    print(json.dumps(output, indent=2))
    if args.json_out:
        output_path = Path(__file__).resolve().parents[1] / args.json_out
        output_path.write_text(json.dumps(output, indent=2) + "\n", encoding="utf-8")
    return 0 if result.wasSuccessful() else 1


if __name__ == "__main__":
    raise SystemExit(main())
