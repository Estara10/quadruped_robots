#!/usr/bin/env python3
"""Frozen S7 formal 240-item analysis: three views and map-equal comparisons."""
from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

import s7_formal_analysis as analysis  # noqa: E402
import s7_formal_runner as runner  # noqa: E402


def _read_ledger(batch_root: Path) -> dict:
    ledger_path = batch_root / "ledger" / "attempts.json"
    if not ledger_path.is_file():
        raise SystemExit(f"ledger not found: {ledger_path}")
    return runner.read_json(ledger_path)


def _stored_summaries(plan: list[dict], audit: dict, batch_root: Path) -> dict[str, dict]:
    summaries: dict[str, dict] = {}
    for fact in audit.get("attempt_facts", []):
        if fact.get("scene_created") is not True or not fact.get("run_directory"):
            continue
        metrics_path = Path(fact["run_directory"]) / "metrics.json"
        if metrics_path.is_file():
            summaries[str(fact["attempt_id"])] = runner.read_json(metrics_path)
    return summaries


def _write_csv(path: Path, rows: list[dict], columns: list[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=columns, extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            writer.writerow({key: (json.dumps(value, ensure_ascii=False)
                                   if isinstance(value, (dict, list, bool)) and value is not None
                                   else value) for key, value in row.items()})


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--batch-root", type=Path, required=True)
    parser.add_argument("--matrix-csv", type=Path, default=runner.MATRIX_CSV)
    parser.add_argument("--matrix-json", type=Path, default=runner.MATRIX_JSON)
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--replicates", type=int, default=analysis.BOOTSTRAP_REPLICATES)
    parser.add_argument("--seed", type=int, default=analysis.BOOTSTRAP_SEED)
    parser.add_argument("--reverse-window-s", type=float, default=0.5)
    parser.add_argument("--use-stored-metrics", action="store_true",
                        help="explicit non-formal mode; skips frozen re-analysis")
    parser.add_argument("--allow-constructed-plan", action="store_true",
                        help="analysis-only offline fixture plan; rejected unless matrix_id is "
                             "S7-OFFLINE-FIXTURE")
    args = parser.parse_args()

    batch_root = args.batch_root.resolve()
    output_dir = (args.output_dir or batch_root / "analysis" / "formal_summary").resolve()
    if args.allow_constructed_plan:
        records = analysis.load_plan_csv(args.matrix_csv)
        meta = runner.read_json(args.matrix_json)
        json_records = [runner.normalize_record(row) for row in meta.get("records", [])]
        if sorted(json_records, key=lambda item: int(item["global_order"])) != sorted(
                records, key=lambda item: int(item["global_order"])):
            raise SystemExit("constructed plan CSV/JSON records differ")
        load_errors: list[str] = []
    else:
        records, meta, load_errors = runner.load_frozen_plan(args.matrix_csv, args.matrix_json)
    if load_errors:
        raise SystemExit("frozen plan load errors: " + "; ".join(load_errors))
    if args.allow_constructed_plan and meta.get("matrix_id") != "S7-OFFLINE-FIXTURE":
        raise SystemExit("--allow-constructed-plan is restricted to S7-OFFLINE-FIXTURE")
    if args.use_stored_metrics and not args.allow_constructed_plan:
        raise SystemExit("formal analysis must recompute with the frozen analyzer; "
                         "--use-stored-metrics is limited to offline fixtures")
    if not args.allow_constructed_plan and meta.get("status") != runner.FROZEN_STATUS:
        raise SystemExit(f"formal matrix is not method-frozen: {meta.get('status')}")
    validation = runner.validate_plan(records, meta,
                                      require_full=not args.allow_constructed_plan)
    if validation["status"] != "PASS":
        raise SystemExit("frozen plan validation failed: " + "; ".join(validation["errors"]))
    binding = runner.matrix_binding(records, meta, args.matrix_csv, args.matrix_json)
    manifest_path = batch_root / "batch_manifest.json"
    if manifest_path.is_file():
        manifest = runner.read_json(manifest_path)
        manifest_binding = manifest.get("matrix", {})
        if manifest_binding.get("plan_sha256") != binding.get("plan_sha256") or \
                manifest_binding.get("matrix_id") != binding.get("matrix_id"):
            raise SystemExit("batch manifest is bound to a different plan; refusing mixed analysis")
    ledger = _read_ledger(batch_root)
    if ledger.get("schema") != runner.LEDGER_SCHEMA:
        raise SystemExit("unexpected ledger schema")
    audit = runner.audit_ledger(records, ledger.get("attempts", []))
    if audit.get("status") != "PASS":
        raise SystemExit("ledger audit rejected: " + "; ".join(audit.get("errors", [])))

    if args.use_stored_metrics:
        summaries = _stored_summaries(records, audit, batch_root)
        provenance = {
            "analysis_mode": "STORED_METRICS_NOT_FORMAL",
            "note": "explicit non-formal mode; stored metrics were not recomputed by the frozen analyzer",
            "matrix": binding,
            "analyzer": analysis.analyzer_fingerprint(),
            "ledger_sha256": runner.sha256_file(batch_root / "ledger" / "attempts.json"),
        }
    else:
        summaries, provenance = analysis.recompute_scene_start_summaries(
            records, audit, output_dir=output_dir,
            reverse_window_s=args.reverse_window_s,
            decision_clock_override=True)
        provenance["analysis_mode"] = "FROZEN_ANALYZER_RECOMPUTED_FROM_RAW_RECORDS"
        provenance["matrix"] = binding
        provenance["ledger_sha256"] = runner.sha256_file(batch_root / "ledger" / "attempts.json")
        provenance["original_metrics_preserved"] = True
        provenance["decision_clock_reconstruction"] = {
            "enabled": True,
            "rule": "publication-stale first active cycle + matching controller S6-04 "
                    "decision trace with the same run/session/cycle and clock sample",
            "applies_to_groups": ["A", "B", "C", "D"],
            "raw_publication_stale_preserved": True,
            "raw_records_written": False,
        }

    if args.use_stored_metrics:
        sensitivity = {"enabled": False, "reason": "stored-metrics non-formal mode"}
    else:
        sensitivity = analysis.reconstruction_sensitivity(
            records, audit, summaries, output_dir=output_dir,
            reverse_window_s=args.reverse_window_s)
    report = analysis.build_report(
        plan=records, audit=audit, summary_by_attempt=summaries,
        provenance=provenance, replicates=args.replicates, seed=args.seed)
    report["reconstruction_sensitivity"] = sensitivity
    output_dir.mkdir(parents=True, exist_ok=True)
    runner.write_json_atomic(output_dir / "formal_summary.json", report)

    primary_columns = [
        "plan_item_id", "group", "map", "repeat_block", "global_order",
        "plan_status", "outcome", "selected_attempt_id", "selected_run_id",
        "selected_terminal", "raw_selected_safety_task_completion",
        "unified_safety_task_completion",
    ] + [spec["key"] for spec in analysis.METRIC_SPECS] + [
        "recovery_completed_interval_count", "recovery_censored_interval_count",
        "cleanup_complete", "standing_observation_complete", "record_validity",
    ]
    primary_rows = []
    for row in report["views"]["primary_plan_item"]["rows"]:
        flat = {key: row.get(key) for key in primary_columns if key in row}
        for key, value in (row.get("metrics") or {}).items():
            if key not in flat:
                flat[key] = value
        primary_rows.append(flat)
    _write_csv(output_dir / "primary_plan_items.csv", primary_rows, primary_columns)

    first_columns = [
        "plan_item_id", "group", "map", "repeat_block", "global_order",
        "first_scene_attempt_id", "first_scene_terminal",
        "first_scene_record_validity", "first_scene_navigation_result", "outcome",
    ] + [spec["key"] for spec in analysis.METRIC_SPECS] + [
        "cleanup_complete", "standing_observation_complete", "record_validity",
    ]
    first_rows = []
    for row in report["views"]["first_actual_scene_only_sensitivity"]["rows"]:
        flat = {key: row.get(key) for key in first_columns if key in row}
        for key, value in (row.get("metrics") or {}).items():
            if key not in flat:
                flat[key] = value
        first_rows.append(flat)
    _write_csv(output_dir / "first_scene_sensitivity.csv", first_rows, first_columns)

    comparison_rows = []
    for metric_key, metric in report["comparisons"]["metrics"].items():
        for comparison_key, value in metric["comparisons"].items():
            scenario = value.get("scenario_unknown_bounds") or {}
            interval = value.get("bootstrap_95") or {}
            comparison_rows.append({
                "metric": metric_key, "label": metric["label"],
                "unit": value.get("unit", metric["unit"]),
                "direction": metric["direction"], "comparison": comparison_key,
                "point": value.get("point"),
                "point_proportion": value.get("point_proportion"),
                "bootstrap_95_lower": interval.get("lower"),
                "bootstrap_95_upper": interval.get("upper"),
                "bootstrap_replicates_used": interval.get("replicates_used"),
                "scenario_lower": scenario.get("lower"),
                "scenario_upper": scenario.get("upper"),
                "worst_case_endpoint": scenario.get("worst_case"),
                "best_case_endpoint": scenario.get("best_case"),
            })
    _write_csv(output_dir / "comparisons.csv", comparison_rows,
               ["metric", "label", "unit", "direction", "comparison", "point",
                "point_proportion",
                "bootstrap_95_lower", "bootstrap_95_upper", "bootstrap_replicates_used",
                "scenario_lower", "scenario_upper", "worst_case_endpoint",
                "best_case_endpoint"])

    starts = report["views"]["all_actual_scene_starts"]
    _write_csv(output_dir / "all_actual_scene_start_counts.csv",
               [{"category": key, "count": value}
                for key, value in starts.get("class_counts", {}).items()],
               ["category", "count"])

    lines = [
        "# S7 formal 240-item analysis",
        "",
        f"- plan items: {len(records)}",
        f"- analysis mode: {provenance.get('analysis_mode')}",
        f"- actual scene starts: {starts.get('counts', {}).get('actual_scene_starts')}",
        f"- plan-item outcomes: {report['views']['primary_plan_item']['outcome_counts']}",
        f"- bootstrap: {args.replicates} map-cluster replicates, seed {args.seed}",
        f"- decision-clock reconstruction: {sensitivity.get('reconstruction_runs', 'N/A')} runs; "
        f"safety changed {sensitivity.get('safety_changed_runs', 'N/A')}; "
        f"timing changed {sensitivity.get('timing_changed_runs', 'N/A')}",
        "",
        "Scenario bounds are UNKNOWN sensitivity ranges, not confidence intervals. "
        "Completion-time subsets do not prove overall speed-up.",
    ]
    (output_dir / "SUMMARY.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(json.dumps({
        "status": "OK", "output_dir": str(output_dir),
        "analysis_mode": provenance.get("analysis_mode"),
        "plan_items": len(records),
        "actual_scene_starts": starts.get("counts", {}).get("actual_scene_starts"),
        "outcomes": report["views"]["primary_plan_item"]["outcome_counts"],
    }, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
