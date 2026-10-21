#!/usr/bin/env python3
"""S7 dev supplement entry: m02.xml A then D, not formal data.

The two-run authorization is separate from the formal 240-item authorization.
``run`` dispatches only A and stages; ``resume`` dispatches D and never reruns A.
No command starts a scene unless a valid dev authorization is supplied.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

import s7_dev_supplement as dev  # noqa: E402
import s7_formal_runner as runner  # noqa: E402


def _print(value) -> None:
    print(json.dumps(value, indent=2, ensure_ascii=False, sort_keys=True))


def _load_with(plan_path: Path):
    records, meta, errors = dev.load_dev_plan(plan_path)
    return records, meta, errors


def _authorization(path: Path | None) -> tuple[dict | None, Path | None]:
    if path is None:
        return None, None
    if not path.is_file():
        return None, path
    return runner.read_json(path), path


def command_plan(args) -> int:
    records, meta, errors = _load_with(args.plan)
    validation = dev.validate_dev_plan(records, meta)
    binding = dev.dev_binding(records, meta, args.plan)
    output = {
        "plan_id": meta.get("plan_id"),
        "status": meta.get("status"),
        "data_class": dev.DATA_CLASS,
        "not_formal_data": True,
        "holdout_prohibited": True,
        "authorized_to_run": meta.get("authorized_to_run"),
        "records": records,
        "validation": validation,
        "binding": binding,
        "load_errors": errors,
        "read_only": True,
    }
    if args.output:
        runner.write_json_atomic(Path(args.output), output)
    _print(output)
    return 0 if validation["status"] == "PASS" and not errors else 2


def command_preflight(args) -> int:
    records, meta, errors = _load_with(args.plan)
    binding = dev.dev_binding(records, meta, args.plan)
    report = runner.preflight(
        records, meta, require_full=False,
        batch_root=Path(args.batch_root) if args.batch_root else None,
        free_gib=args.free_gib, prepare=args.prepare_overlays,
        required_status=dev.DEV_STATUS, plan_validator=dev.validate_dev_plan,
        binding=binding)
    if errors:
        report["errors"] = sorted(set(report.get("errors", []) + errors))
        report["status"] = "REJECTED"
    report["data_class"] = dev.DATA_CLASS
    report["not_formal_data"] = True
    report["run_authorized"] = False
    if args.output:
        runner.write_json_atomic(Path(args.output), report)
    _print(report)
    return 0 if report["status"] == "PASS" else 2


def command_status(args) -> int:
    records, meta, errors = _load_with(args.plan)
    report = runner.status(Path(args.batch_root), records,
                           max_substitutions=dev.MAX_SUBSTITUTIONS,
                           max_per_plan_item=1)
    report["plan_id"] = meta.get("plan_id")
    report["data_class"] = dev.DATA_CLASS
    report["not_formal_data"] = True
    report["load_errors"] = errors
    if args.output:
        runner.write_json_atomic(Path(args.output), report)
    _print(report)
    return 0 if report.get("status") in {"OK", "NO_LEDGER"} else 2


def _run_common(args, *, resume: bool) -> int:
    records, meta, errors = _load_with(args.plan)
    if errors:
        _print({"status": "REFUSED", "reason": "dev_plan_load_errors", "errors": errors})
        return 2
    validation = dev.validate_dev_plan(records, meta)
    if validation["status"] != "PASS":
        _print({"status": "REFUSED", "reason": "dev_plan_validation_failed",
                "validation": validation})
        return 2
    authorization, authorization_path = _authorization(args.authorization)
    if authorization is None:
        _print({
            "status": "REFUSED",
            "reason": "missing_or_unreadable_dev_authorization",
            "path": str(args.authorization) if args.authorization else None,
            "detail": "The two-run dev supplement requires its separate authorization record; "
                      "the formal 240 authorization cannot be used and there is no bypass switch.",
        })
        return 2
    binding = dev.dev_binding(records, meta, args.plan)
    check = dev.validate_dev_authorization(authorization, binding, args.batch_id)
    if check["status"] != "PASS":
        _print({"status": "REFUSED", "reason": "dev_authorization_rejected",
                "authorization": check})
        return 2
    batch_root = Path(args.batch_root)
    if not dev.batch_root_allowed(batch_root):
        _print({"status": "REFUSED", "reason": "dev_batch_root_outside_logs_07",
                "batch_root": str(batch_root)})
        return 2
    exists = ((batch_root / "batch_manifest.json").is_file()
              or (batch_root / "ledger" / "attempts.json").is_file())
    if resume and not exists:
        _print({"status": "REFUSED", "reason": "resume_requires_existing_dev_batch",
                "batch_root": str(batch_root)})
        return 2
    if not resume and exists:
        _print({"status": "REFUSED", "reason": "existing_batch_requires_resume",
                "batch_root": str(batch_root)})
        return 2
    result = dev.run_dev_batch(
        batch_root=batch_root, batch_id=args.batch_id, records=records, meta=meta,
        authorization=authorization, authorization_path=authorization_path,
        executor=runner.dispatch_production, analyze_run=runner.analyze_production,
        resume=resume, stop_after_scene_starts=1,
        display=args.display, xauthority=args.xauthority, plan_path=args.plan)
    result["data_class"] = dev.DATA_CLASS
    result["not_formal_data"] = True
    if args.output:
        runner.write_json_atomic(Path(args.output), result)
    _print(result)
    if result.get("status") == "STAGED":
        print("\nA 已结束并保存检查点。下一步运行 D：")
        print(f"/usr/bin/python3 scripts/run_s7_dev_supplement.py resume "
              f"--batch-root {batch_root} --batch-id {args.batch_id} "
              f"--authorization {authorization_path}")
    return 0 if result.get("status") in {"COMPLETE", "STAGED"} else (
        3 if result.get("status") == "PAUSED" else 2)


def command_run(args) -> int:
    return _run_common(args, resume=False)


def command_resume(args) -> int:
    return _run_common(args, resume=True)


def command_analyze(args) -> int:
    report = dev.analyze_dev_batch(
        Path(args.batch_root), output_dir=Path(args.output_dir) if args.output_dir else None,
        reverse_window_s=args.reverse_window_s, plan_path=args.plan)
    if args.output:
        runner.write_json_atomic(Path(args.output), report)
    _print(report)
    return 0 if report.get("consistency_status") == "PASS" else (
        3 if report.get("status") != "REFUSED" else 2)


def _add_common(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--plan", type=Path, default=argparse.SUPPRESS)
    parser.add_argument("--authorization", type=Path, default=argparse.SUPPRESS)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plan", type=Path, default=dev.PLAN_PATH)
    parser.add_argument("--authorization", type=Path, default=None,
                        help="independent two-run dev authorization; required for run/resume")
    sub = parser.add_subparsers(dest="command", required=True)

    plan = sub.add_parser("plan", help="read-only dev plan")
    _add_common(plan)
    plan.add_argument("--output", type=Path)
    plan.set_defaults(func=command_plan)

    preflight = sub.add_parser("preflight", help="static no-scene preflight")
    _add_common(preflight)
    preflight.add_argument("--batch-root", type=Path)
    preflight.add_argument("--prepare-overlays", action="store_true")
    preflight.add_argument("--free-gib", type=float)
    preflight.add_argument("--output", type=Path)
    preflight.set_defaults(func=command_preflight)

    status = sub.add_parser("status", help="read-only status")
    _add_common(status)
    status.add_argument("--batch-root", type=Path, required=True)
    status.add_argument("--output", type=Path)
    status.set_defaults(func=command_status)

    run = sub.add_parser("run", help="run A only; then stage")
    _add_common(run)
    run.add_argument("--batch-root", type=Path, required=True)
    run.add_argument("--batch-id", required=True)
    run.add_argument("--display", default=":0")
    run.add_argument("--xauthority")
    run.add_argument("--output", type=Path)
    run.set_defaults(func=command_run)

    resume = sub.add_parser("resume", help="resume D; never rerun A")
    _add_common(resume)
    resume.add_argument("--batch-root", type=Path, required=True)
    resume.add_argument("--batch-id", required=True)
    resume.add_argument("--display", default=":0")
    resume.add_argument("--xauthority")
    resume.add_argument("--output", type=Path)
    resume.set_defaults(func=command_resume)

    analyze = sub.add_parser("analyze", help="dev consistency analysis; no formal inference")
    _add_common(analyze)
    analyze.add_argument("--batch-root", type=Path, required=True)
    analyze.add_argument("--output-dir", type=Path)
    analyze.add_argument("--reverse-window-s", type=float, default=0.5)
    analyze.add_argument("--output", type=Path)
    analyze.set_defaults(func=command_analyze)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    return args.func(args)


if __name__ == "__main__":
    raise SystemExit(main())
