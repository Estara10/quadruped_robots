#!/usr/bin/env python3
"""Targeted M1 rename compatibility check for saved runtime records."""
from __future__ import annotations

import json
from pathlib import Path

from run_record import _scene_binding_matches_current_or_m1_rename


def run() -> None:
    source = Path(
        "logs/manual_m1_a/library_v1/m1dod02/rep_02_attempt_01/A/rep_01/"
        "d541c96a822d4d489635bac07ba26b78/runtime_record.jsonl")
    meta = json.loads(source.open(encoding="utf-8").readline())
    binding = meta["scene_binding"]
    assert binding["filename"] == "scene_m1_dense_staggered_channel_dev_02.xml"
    assert _scene_binding_matches_current_or_m1_rename(binding)

    wrong = dict(binding)
    wrong["root_sha256"] = "0" * 64
    assert not _scene_binding_matches_current_or_m1_rename(wrong)
    wrong = dict(binding)
    wrong["model_fingerprint"] = "0" * 64
    assert not _scene_binding_matches_current_or_m1_rename(wrong)
    wrong = dict(binding)
    wrong["obstacle_count"] += 1
    assert not _scene_binding_matches_current_or_m1_rename(wrong)

    print("B1 historical M1 scene-alias checks: PASS")


if __name__ == "__main__":
    run()
