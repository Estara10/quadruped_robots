#!/usr/bin/env python3
"""P1-08 — observed timing statistics from one authoritative capture.

Reads the two JSONL timing streams produced by p1_08_baseline_capture.py and
computes observed periods (min/mean/median/P95/P99/max, sample count, missed
facts) for:

  - physics timestep / physics period   from /mujoco_sim_clock
  - policy (RL-step) tick period        from /mujoco_rt_frame
  - RA tick period                      = policy tick (runRAModel per RL step)
  - Recovery tick period                from policy_state==RECOVERY frames
  - controller callback period          declared 200 Hz (static) and derived
                                        from the observed policy period under
                                        the decimation-4 periodic assumption
                                        (direct per-callback timestamps are
                                        not exposed by any authoritative source)

Usage: python3 scripts/p1_08_timing_stats.py <capture-dir> [--decimation 4]
"""

from __future__ import annotations

import argparse
import json
import statistics
from pathlib import Path
from typing import Dict, List


def percentile(sorted_vals: List[float], p: float) -> float:
    if not sorted_vals:
        return float("nan")
    if len(sorted_vals) == 1:
        return sorted_vals[0]
    idx = (len(sorted_vals) - 1) * p / 100.0
    lo = int(idx)
    hi = min(lo + 1, len(sorted_vals) - 1)
    frac = idx - lo
    return sorted_vals[lo] * (1 - frac) + sorted_vals[hi] * frac


def period_stats(deltas: List[float], name: str) -> Dict:
    if len(deltas) < 2:
        return {"name": name, "sample_count": len(deltas), "period_ns": None,
                "note": "insufficient samples"}
    s = sorted(deltas)
    return {
        "name": name,
        "sample_count": len(deltas),
        "period_ns": {
            "min": s[0],
            "mean": statistics.fmean(deltas),
            "median": statistics.median(deltas),
            "p95": percentile(s, 95),
            "p99": percentile(s, 99),
            "max": s[-1],
        },
        "period_ms": {
            "min": s[0] / 1e6,
            "mean": statistics.fmean(deltas) / 1e6,
            "median": statistics.median(deltas) / 1e6,
            "p95": percentile(s, 95) / 1e6,
            "p99": percentile(s, 99) / 1e6,
            "max": s[-1] / 1e6,
        },
        "period_hz": {
            "mean": 1e9 / statistics.fmean(deltas),
            "median": 1e9 / statistics.median(deltas),
        },
    }


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("capture_dir")
    ap.add_argument("--decimation", type=int, default=4)
    args = ap.parse_args()

    d = Path(args.capture_dir)
    sim_rows = [json.loads(l) for l in (d / "sim_clock_timing.jsonl").read_text().splitlines() if l.strip()]
    rt_rows = [json.loads(l) for l in (d / "rt_frame_timing.jsonl").read_text().splitlines() if l.strip()]

    out: Dict = {
        "capture_dir": str(d),
        "declared_decimation": args.decimation,
    }

    # ---- physics (sim_clock) ----
    sim_ok = False
    if len(sim_rows) >= 2:
        sim_ok = True
        dsteps: List[float] = []   # observed sim-time advance per physics step (s)
        dmono: List[float] = []    # observed wall-clock per physics step (ns)
        for a, b in zip(sim_rows[:-1], sim_rows[1:]):
            ds = b["sim_time"] - a["sim_time"]
            steps = round(ds / 0.002)  # nominal opt.timestep from probe
            if steps >= 1:
                dsteps.append(ds / steps)
                dmono.append((b["monotonic_ns"] - a["monotonic_ns"]) / steps)

        def simple_stats(vals: List[float]) -> Dict:
            s = sorted(vals)
            return {
                "sample_count": len(vals),
                "min": s[0],
                "mean": statistics.fmean(vals),
                "median": statistics.median(vals),
                "p95": percentile(s, 95),
                "p99": percentile(s, 99),
                "max": s[-1],
            }

        sim_adv = simple_stats(dsteps) if dsteps else {}
        wall = simple_stats(dmono) if dmono else {}
        out["physics"] = {
            "observed_sim_advance_per_step_s": sim_adv,
            "observed_wallclock_per_physics_step": {
                "ns": wall,
                "mean_ms": (statistics.fmean(dmono) / 1e6) if dmono else None,
                "mean_hz": (1e9 / statistics.fmean(dmono)) if dmono else None,
            },
            "nominal_opt_timestep_s": 0.002,
            "samples": len(sim_rows),
            "conclusion": ("observed physics timestep == nominal opt.timestep 0.002 s"
                           if sim_adv and abs(sim_adv["mean"] - 0.002) < 1e-9 else "mismatch"),
            "note": "dstep counts assume opt.timestep 0.002 s (probe-confirmed); sim_advance in seconds",
        }
    else:
        out["physics"] = {"samples": len(sim_rows), "note": "insufficient samples"}

    # ---- controller / policy / RA / Recovery (rt_frame) ----
    if len(rt_rows) >= 2:
        policy_deltas = [b["monotonic_ns"] - a["monotonic_ns"]
                         for a, b in zip(rt_rows[:-1], rt_rows[1:])
                         if b["monotonic_ns"] > a["monotonic_ns"]]
        out["policy_tick"] = period_stats(policy_deltas, "policy_rl_step")

        rec_rows = [r for r in rt_rows if r["policy_state"] == 1]
        rec_deltas = [b["monotonic_ns"] - a["monotonic_ns"]
                      for a, b in zip(rec_rows[:-1], rec_rows[1:])
                      if b["monotonic_ns"] > a["monotonic_ns"]]
        out["recovery_tick"] = period_stats(rec_deltas, "recovery_active")
        out["recovery_active_samples"] = len(rec_rows)
        out["recovery_transitions"] = sum(1 for a, b in zip(rt_rows[:-1], rt_rows[1:])
                                          if a["policy_state"] != b["policy_state"])

        # RA runs every RL step (runRAModel per runModel) -> same cadence.
        out["ra_tick"] = {
            "source": "runRAModel() invoked every RL step (StateRL.cpp)",
            "observed_period": out["policy_tick"],
        }

        # Controller callback: declared static; direct timestamps not exposed.
        mean_policy = statistics.fmean(policy_deltas) if len(policy_deltas) else None
        out["controller_callback"] = {
            "declared_update_rate_hz": 200,
            "declared_source": "robot_control.yaml rl_quadruped_controller.update_rate",
            "derived_period_ns": (mean_policy / args.decimation) if mean_policy else None,
            "derived_period_ms": (mean_policy / args.decimation / 1e6) if mean_policy else None,
            "direct_per_callback_timestamps": "UNKNOWN - no authoritative runtime source exposes 200 Hz callbacks; derived under periodic-callback assumption",
        }
    else:
        out["policy_tick"] = {"sample_count": len(rt_rows), "note": "insufficient samples"}

    out_path = d / "timing_stats.json"
    out_path.write_text(json.dumps(out, indent=2, sort_keys=True) + "\n")
    print(json.dumps(out, indent=2, sort_keys=True))
    return 0 if sim_ok and len(rt_rows) >= 2 else 1


if __name__ == "__main__":
    raise SystemExit(main())
