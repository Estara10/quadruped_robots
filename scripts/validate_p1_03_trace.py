#!/usr/bin/env python3
"""Offline mechanical validator for P1-03 formula/parameter trace evidence.

This validates source anchors, not paper fidelity. A cited path, line range, and
symbol must form a mechanically verifiable reference. ``UNKNOWN``/``ABSENT``/
``NOT_APPLICABLE`` remain explicit non-claims and are never forced to invent a
repository citation.
"""

from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path
from typing import Any

import yaml


REQUIRED = {
    "id", "kind", "paper", "training", "reference", "runtime", "classification",
    "confidence", "claims_not_made", "validation",
}
CLASSIFICATIONS = {"MATCH", "PAPER_FAITHFUL", "STABILIZED_VARIANT", "MISMATCH", "UNKNOWN", "CONFLICT"}
CONFIDENCE = {"DIRECT_SOURCE", "DERIVED_MECHANICALLY", "CONDITIONAL", "UNKNOWN"}
SKIP_SOURCE_STATUS = {"ABSENT", "NOT_APPLICABLE", "UNKNOWN"}
PRESENT_STATUS = "PRESENT"


def source_path(root: Path, value: str) -> Path:
    """Return a repository-relative path before the paper-note line suffix."""
    return root / value.split(":", 1)[0]


def parse_line_ranges(value: Any) -> list[tuple[int, int]] | None:
    """Parse comma-separated inclusive ``start-end`` ranges."""
    if not isinstance(value, str) or not value.strip():
        return None
    ranges: list[tuple[int, int]] = []
    for fragment in value.split(","):
        match = re.fullmatch(r"\s*(\d+)\s*-\s*(\d+)\s*", fragment)
        if not match:
            return None
        start, end = int(match.group(1)), int(match.group(2))
        if start < 1 or end < start:
            return None
        ranges.append((start, end))
    return ranges or None


def selected_lines(text: str, ranges: list[tuple[int, int]]) -> str:
    lines = text.splitlines()
    return "\n".join(
        lines[index - 1]
        for start, end in ranges
        for index in range(start, end + 1)
    )


def line_ranges_are_in_file(ranges: list[tuple[int, int]] | None, line_count: int) -> bool:
    return bool(ranges) and all(end <= line_count for _, end in ranges)


def symbol_anchor(symbol: str) -> str:
    """Use the concrete final identifier from a qualified source symbol."""
    token = symbol.strip().split()[0]
    return re.split(r"::|\.", token)[-1]


def validate_citation(
    errors: list[str], root: Path, label: str, field: str, source: dict[str, Any],
) -> None:
    path_value = source.get("path")
    symbol = source.get("symbol")
    line_range_value = source.get("line_range")
    if not isinstance(path_value, str) or path_value in {"", "NONE"}:
        errors.append(f"{label}: {field} has no repository path")
        return
    if not isinstance(symbol, str) or not symbol.strip() or symbol == "NONE":
        errors.append(f"{label}: {field} symbol missing")
        return
    path = source_path(root, path_value)
    if not path.is_file():
        errors.append(f"{label}: {field} path missing: {path_value}")
        return
    text = path.read_text(encoding="utf-8")
    ranges = parse_line_ranges(line_range_value)
    if not line_ranges_are_in_file(ranges, len(text.splitlines())):
        errors.append(f"{label}: {field} line_range invalid: {line_range_value}")
        return
    anchor = symbol_anchor(symbol)
    if anchor not in selected_lines(text, ranges):
        errors.append(
            f"{label}: {field} cited symbol not within line_range: {symbol} @ {line_range_value}"
        )


def validate_reference(errors: list[str], root: Path, label: str, source: dict[str, Any]) -> None:
    status = source.get("status")
    if status == PRESENT_STATUS:
        validate_citation(errors, root, label, "reference", source)
        return
    if status in SKIP_SOURCE_STATUS:
        # Explicit non-claims cannot smuggle in unverified source metadata.
        if source.get("path") != "NONE" or source.get("symbol") != "NONE":
            errors.append(f"{label}: reference status {status} must use path/symbol NONE")
        if "line_range" in source:
            errors.append(f"{label}: reference status {status} must not declare line_range")
        return
    errors.append(f"{label}: reference status invalid or missing: {status}")


def validate_paper(errors: list[str], root: Path, label: str, paper: dict[str, Any]) -> None:
    source = paper.get("source")
    if not isinstance(source, str) or ":" not in source:
        errors.append(f"{label}: paper source/range missing")
        return
    path_value, line_range_value = source.split(":", 1)
    path = root / path_value
    if not path.is_file():
        errors.append(f"{label}: paper-notes path missing: {path_value}")
        return
    ranges = parse_line_ranges(line_range_value)
    text = path.read_text(encoding="utf-8")
    if not line_ranges_are_in_file(ranges, len(text.splitlines())):
        errors.append(f"{label}: paper-notes line_range invalid: {line_range_value}")
    if not isinstance(paper.get("expression"), str) or not paper["expression"].strip():
        errors.append(f"{label}: paper expression missing")
    if not isinstance(paper.get("variables"), list) or not paper["variables"]:
        errors.append(f"{label}: paper variables missing")


def validate(root: Path, trace_path: Path) -> list[str]:
    data = yaml.safe_load(trace_path.read_text(encoding="utf-8"))
    errors: list[str] = []
    if not isinstance(data, dict) or data.get("schema_version") != "abs-go2-formula-parameter-trace/v1":
        return ["invalid schema_version"]
    records = data.get("records")
    if not isinstance(records, list) or not records:
        return ["records missing or empty"]
    for index, record in enumerate(records):
        label = record.get("id", f"record[{index}]") if isinstance(record, dict) else f"record[{index}]"
        if not isinstance(record, dict):
            errors.append(f"{label}: record is not a mapping")
            continue
        if set(record) != REQUIRED:
            errors.append(f"{label}: record keys differ from required contract")
        if record.get("classification") not in CLASSIFICATIONS:
            errors.append(f"{label}: invalid classification")
        if record.get("confidence") not in CONFIDENCE:
            errors.append(f"{label}: invalid confidence")
        validate_paper(errors, root, label, record.get("paper", {}))
        for field in ("training", "runtime"):
            source = record.get(field, {})
            if not isinstance(source, dict):
                errors.append(f"{label}: {field} is not a mapping")
            else:
                validate_citation(errors, root, label, field, source)
        reference = record.get("reference", {})
        if not isinstance(reference, dict):
            errors.append(f"{label}: reference is not a mapping")
        else:
            validate_reference(errors, root, label, reference)
    return errors


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--trace", default="docs/evidence/P1-03/formula_parameter_trace.yaml")
    parser.add_argument("--json-out", default="")
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    trace_path = root / args.trace
    errors = validate(root, trace_path)
    records = len(yaml.safe_load(trace_path.read_text(encoding="utf-8"))["records"])
    result = {
        "task": "P1-03",
        "trace": args.trace,
        "records": records,
        "validator": "mechanical source-anchor validation",
        "result": "PASS" if not errors else "FAIL",
        "errors": errors,
    }
    print(json.dumps(result, indent=2, ensure_ascii=False))
    if args.json_out:
        (root / args.json_out).write_text(json.dumps(result, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    return 0 if not errors else 1


if __name__ == "__main__":
    sys.exit(main())
