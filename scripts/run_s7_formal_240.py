#!/usr/bin/env python3
"""S7 formal 240-item entry point: read-only plan, preflight, status, guarded run.

The ``run`` command requires an explicit batch authorization record matching
the selected plan.  There is no switch that bypasses this check.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

import s7_formal_runner as runner  # noqa: E402


def _load(matrix_csv: Path, matrix_json: Path):
    records, meta, errors = runner.load_frozen_plan(matrix_csv, matrix_json)
    return records, meta, errors


def _print(value) -> None:
    print(json.dumps(value, indent=2, ensure_ascii=False, sort_keys=True))


def command_plan(args) -> int:
    records, meta, errors = _load(args.matrix_csv, args.matrix_json)
    validation = runner.validate_plan(records, meta, require_full=True)
    binding = runner.matrix_binding(records, meta, args.matrix_csv, args.matrix_json)
    output = {
        "matrix_id": meta.get("matrix_id"),
        "matrix_status": meta.get("status"),
        "authorized_to_run": meta.get("authorized_to_run"),
        "method_freeze": meta.get("method_freeze"),
        "groups": meta.get("groups"),
        "schedule_seed": meta.get("schedule_seed"),
        "bootstrap_seed": meta.get("bootstrap_seed"),
        "plan_items": len(records),
        "binding": binding,
        "validation": validation,
        "load_errors": errors,
        "first_items": records[:4],
        "last_item": records[-1] if records else None,
        "read_only": True,
    }
    if args.output:
        runner.write_json_atomic(Path(args.output), output)
    _print(output)
    return 0 if validation["status"] == "PASS" and not errors else 2


def command_preflight(args) -> int:
    records, meta, errors = _load(args.matrix_csv, args.matrix_json)
    report = runner.preflight(
        records, meta, require_full=True,
        batch_root=Path(args.batch_root) if args.batch_root else None,
        free_gib=args.free_gib, prepare=args.prepare_overlays,
        binding=runner.matrix_binding(records, meta, args.matrix_csv, args.matrix_json))
    if errors:
        report["errors"] = sorted(set(report.get("errors", []) + errors))
        report["status"] = "REJECTED"
    report["authorization_present"] = False
    report["run_authorized"] = False
    if args.output:
        runner.write_json_atomic(Path(args.output), report)
    _print(report)
    return 0 if report["status"] == "PASS" else 2


def command_status(args) -> int:
    records, meta, errors = _load(args.matrix_csv, args.matrix_json)
    report = runner.status(Path(args.batch_root), records)
    report["load_errors"] = errors
    if args.output:
        runner.write_json_atomic(Path(args.output), report)
    _print(report)
    return 0 if report.get("status") in {"OK", "NO_LEDGER"} else 2


def command_run(args) -> int:
    records, meta, errors = _load(args.matrix_csv, args.matrix_json)
    if errors:
        _print({"status": "REFUSED", "reason": "frozen_plan_load_errors", "errors": errors})
        return 2
    validation = runner.validate_plan(records, meta, require_full=True)
    if validation["status"] != "PASS":
        _print({"status": "REFUSED", "reason": "frozen_plan_validation_failed",
                "validation": validation})
        return 2
    if meta.get("status") != runner.FROZEN_STATUS:
        _print({"status": "REFUSED", "reason": "matrix_not_method_frozen",
                "matrix_status": meta.get("status")})
        return 2
    if not args.authorization:
        _print({
            "status": "REFUSED",
            "reason": "missing_explicit_batch_authorization",
            "detail": "A separate authorization record is required. No valid S7 run "
                      "authorization exists in this repository; this command does not "
                      "create one and has no bypass switch.",
        })
        return 2
    authorization_path = Path(args.authorization)
    if not authorization_path.is_file():
        _print({"status": "REFUSED", "reason": "authorization_file_not_found",
                "path": str(authorization_path)})
        return 2
    authorization = runner.read_json(authorization_path)
    binding = runner.matrix_binding(records, meta, args.matrix_csv, args.matrix_json)
    check = runner.validate_authorization(authorization, binding, batch_id=args.batch_id,
                                          allow_fixture=False)
    if check["status"] != "PASS":
        _print({"status": "REFUSED", "reason": "authorization_rejected",
                "authorization": check})
        return 2
    result = runner.run_batch(
        batch_root=Path(args.batch_root), batch_id=args.batch_id,
        plan=records, meta=meta, binding=binding, authorization=authorization,
        authorization_path=authorization_path,
        executor=runner.dispatch_production, analyze_run=runner.analyze_production,
        allow_fixture=False, resume=args.resume, require_full_plan=True,
        data_class=("M3_CHALLENGE_240" if meta.get("matrix_id") == runner.M3_MATRIX_ID
                    else "FORMAL_240"),
        classification=("S7 M3 v1.4 challenge A/B/C/D 240-item comparison; "
                        "historically selected challenge maps, not independent holdout"
                        if meta.get("matrix_id") == runner.M3_MATRIX_ID
                        else runner.FORMAL_CLASSIFICATION),
        display=args.display, xauthority=args.xauthority)
    if args.output:
        runner.write_json_atomic(Path(args.output), result)
    _print(result)
    return 0 if result.get("status") in {"COMPLETE"} else (
        3 if result.get("status") == "PAUSED" else 2)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--matrix-csv", type=Path, default=runner.MATRIX_CSV)
    parser.add_argument("--matrix-json", type=Path, default=runner.MATRIX_JSON)
    subparsers = parser.add_subparsers(dest="command", required=True)

    plan = subparsers.add_parser("plan", help="read-only display of the frozen plan")
    plan.add_argument("--output", type=Path)
    plan.set_defaults(func=command_plan)

    preflight = subparsers.add_parser(
        "preflight", help="static parameter/directory preflight; starts no scene")
    preflight.add_argument("--batch-root", type=Path)
    preflight.add_argument("--prepare-overlays", action="store_true",
                           help="prepare detached per-group overlays under --batch-root")
    preflight.add_argument("--free-gib", type=float,
                           help="override the observed free-space reading (reporting only)")
    preflight.add_argument("--output", type=Path)
    preflight.set_defaults(func=command_preflight)

    status = subparsers.add_parser("status", help="read-only batch status query")
    status.add_argument("--batch-root", type=Path, required=True)
    status.add_argument("--output", type=Path)
    status.set_defaults(func=command_status)

    run = subparsers.add_parser("run", help="guarded real dispatch/resume")
    run.add_argument("--batch-root", type=Path, required=True)
    run.add_argument("--batch-id", required=True)
    run.add_argument("--authorization", type=Path,
                     help="explicit batch authorization JSON; required")
    run.add_argument("--resume", action="store_true",
                     help="resume an existing batch after reconciling the ledger")
    run.add_argument("--display", default=":0")
    run.add_argument("--xauthority")
    run.add_argument("--output", type=Path)
    run.set_defaults(func=command_run)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    return args.func(args)


if __name__ == "__main__":
    raise SystemExit(main())
