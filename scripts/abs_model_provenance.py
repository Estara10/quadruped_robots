#!/usr/bin/env python3
"""Print model provenance for the current ABS Go2 deployment config."""

from __future__ import annotations

import argparse
import hashlib
import json
from datetime import datetime
from pathlib import Path
from typing import Dict, Optional

ROOT = Path.home() / "quadruped_robots"
GO2_SHARE = ROOT / "quadruped_ros2_control_humble" / "install" / "go2_description" / "share" / "go2_description"
DEFAULT_OUTPUT = ROOT / "logs" / "abs_eval" / "model_provenance.json"

MODELS = {
    "agile_policy": GO2_SHARE / "config" / "abs" / "policy.pt",
    "ra_value": GO2_SHARE / "config" / "abs" / "ra_value.pt",
    "recovery_policy": GO2_SHARE / "config" / "rec" / "policy.pt",
}
CONFIGS = {
    "abs_config": GO2_SHARE / "config" / "abs" / "config.yaml",
    "rec_config": GO2_SHARE / "config" / "rec" / "config.yaml",
    "robot_control": GO2_SHARE / "config" / "robot_control.yaml",
}


def sha256_file(path: Path) -> Optional[str]:
    if not path.exists():
        return None
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def describe(path: Path) -> Dict[str, object]:
    exists = path.exists()
    resolved = path.resolve() if exists else None
    return {
        "path": str(path),
        "exists": exists,
        "is_symlink": path.is_symlink(),
        "symlink_target": path.readlink().as_posix() if path.is_symlink() else None,
        "resolved_path": str(resolved) if resolved else None,
        "size_bytes": resolved.stat().st_size if resolved and resolved.exists() else None,
        "sha256": sha256_file(resolved) if resolved else None,
    }


def build_report() -> Dict[str, object]:
    return {
        "generated_at": datetime.now().isoformat(timespec="seconds"),
        "models": {name: describe(path) for name, path in MODELS.items()},
        "configs": {name: describe(path) for name, path in CONFIGS.items()},
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT, help="JSON output path")
    parser.add_argument("--no-write", action="store_true", help="Print only; do not write JSON")
    args = parser.parse_args()

    report = build_report()
    print(json.dumps(report, indent=2, ensure_ascii=False))
    if not args.no_write:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        print(f"\nWrote: {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
