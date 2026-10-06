#!/usr/bin/env python3
"""P1-08 — corrected v2 reader statistics (stride-2 sequence-gap fix).

Recomputes sim-clock sequence-gap statistics from the UNMODIFIED raw
sim_clock_timing.jsonl using the v2 even stride=2 (missing =
(next-prev)/2 - 1 for strictly-increasing even sequences), which the original
reader_stats.json computed with a wrong stride-1 formula. rt_frame rl_step uses
stride=1 and is unchanged (correct). The original reader_stats.json is never
overwritten — it is preserved as reader_stats_pre_stride2_correction.json and
this corrected artifact records its provenance + supersedes reason.

Usage:
    python3 scripts/build_p1_08_corrected_reader_stats.py --capture-dir <dir>
"""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
import time
from pathlib import Path

SCRIPTS = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPTS))

from p1_08_baseline_capture import compute_stride2_gaps  # noqa: E402

GENERATOR_VERSION = "1.0"
SCHEMA = "abs-go2-p1-08-reader-stats/v2"
PRE_STRIDE2_NAME = "reader_stats_pre_stride2_correction.json"


def sha256_file(path: Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def distinct_values(path: Path, key: str):
    vals = []
    with open(path) as f:
        for line in f:
            vals.append(json.loads(line)[key])
    uniq = []
    for s in vals:
        if not uniq or s != uniq[-1]:
            uniq.append(s)
    return uniq


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--capture-dir", required=True)
    args = ap.parse_args()

    d = Path(args.capture_dir)
    sim_raw = d / "sim_clock_timing.jsonl"
    rt_raw = d / "rt_frame_timing.jsonl"
    old_stats = d / PRE_STRIDE2_NAME
    if not (sim_raw.exists() and rt_raw.exists() and old_stats.exists()):
        print(f"FAIL: require {sim_raw.name}, {rt_raw.name}, {PRE_STRIDE2_NAME}")
        return 1

    old = json.loads(old_stats.read_text())
    sim_old = old.get("sim_clock", {})
    rt_old = old.get("rt_frame", {})

    # sim-clock stride-2 gaps over distinct even sequences (raw, unmodified)
    gap = compute_stride2_gaps(distinct_values(sim_raw, "sequence"))
    # rt_frame rl_step stride-1 (unchanged): recompute distinct rl_step count
    rt_seqs = distinct_values(rt_raw, "rl_step")

    corrected = {
        "schema": SCHEMA,
        "generator_version": GENERATOR_VERSION,
        "generator_sha256": sha256_file(Path(__file__).resolve()),
        "generated_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "sequence_stride_note": "v2 sim-clock even sequence advances +2 per publish; "
                                "missing = (next-prev)/2 - 1. rt_frame rl_step is stride-1.",
        "sim_clock": {
            "attempts_reads": sim_old.get("attempts"),
            "accepted_reads": sim_old.get("accepted"),
            "rejected": sim_old.get("rejected", 0),
            "reasons": sim_old.get("reasons", {}),
            "distinct_accepted": gap["distinct"],
            "sequence_stride": gap["sequence_stride"],
            "seq_gaps_total_missing": gap["total_missing"],
            "seq_gap_max_single": gap["max_single_gap"],
            "gap_errors": gap["errors"],
            "raw_timing_source_sha256": sha256_file(sim_raw),
            "note": "seq_gaps are MISSING PUBLISHES over distinct accepted sequences; "
                    "distinct from reader rejected reads (rejected=0).",
        },
        "rt_frame": {
            "attempts_reads": rt_old.get("attempts"),
            "accepted_reads": rt_old.get("accepted"),
            "rejected": rt_old.get("rejected", 0),
            "reasons": rt_old.get("reasons", {}),
            "distinct_rl_steps": len(rt_seqs),
            "rl_step_stride": 1,
            "rl_step_gaps": rt_old.get("rl_step_gaps", 0),
            "rl_step_gap_max": rt_old.get("rl_step_gap_max", 0),
            "raw_timing_source_sha256": sha256_file(rt_raw),
        },
        "record_statuses": old.get("record_statuses", {}),
        "supersedes": {
            "artifact": PRE_STRIDE2_NAME,
            "sha256": sha256_file(old_stats),
            "bytes": old_stats.stat().st_size,
            "reason": "original reader_stats.json used a stride-1 sequence-gap formula "
                      "(seq_gaps=12512) but the v2 sim-clock even sequence advances +2; "
                      "the raw timing data was valid, only the derived gap statistic was wrong.",
        },
    }
    (d / "reader_stats.json").write_text(json.dumps(corrected, indent=2, sort_keys=True) + "\n")
    print(f"wrote corrected {d / 'reader_stats.json'}")
    print(f"sim distinct={gap['distinct']} total_missing={gap['total_missing']} "
          f"max_single={gap['max_single_gap']} rt distinct_rl_steps={len(rt_seqs)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
