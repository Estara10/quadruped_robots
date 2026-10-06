#!/usr/bin/env python3
"""P1-09E read-only sampler over /dev/shm/mujoco_rt_frame.

WHY THIS SCRIPT EXISTS (Director-required justification): the existing
``abs_rt_frame.py`` provides the read primitives (``read_shm_frame`` /
``classify_frame`` / ``RuntimeFrame``) but no function that samples a sequence of
frames over a bounded window and dumps the raw per-frame fields (session_id,
sequence, monotonic_ns, source, policy_state, rl_step, HUD status) as a
plain-text evidence trail. ``abs_live_hud.py`` renders a human terminal display
but omits parseable raw fields (sequence, source, monotonic_ns) and clears the
screen. This read-only sampler therefore calls ONLY the existing primitives and
writes a JSONL evidence log. It never writes the frame, never substitutes,
zero-fills, replays, or fabricates data, and creates no formal artifact (plain
JSONL only).

It also checks the P1-09E truth invariants over the captured LIVE frames and
prints a PASS/FAIL summary: source == AUTHORITATIVE_RUNTIME, sequence and
monotonic_ns strictly increasing across distinct frames, session_id consistent,
HUD status LIVE, and records the LIVE -> non-LIVE transition (e.g. controller
exit invalidation).

Usage:
  python3 scripts/abs_live_hud_sampler.py --duration 15 --out /tmp/p109e.jsonl
"""

from __future__ import annotations

import argparse
import json
import sys
import time

from abs_rt_frame import (
    POLICY_AGILE,
    POLICY_RECOVERY,
    POLICY_FAULTED,
    SOURCE_AUTHORITATIVE_RUNTIME,
    FrameStatus,
    classify_frame,
    read_shm_frame,
)


def monotonic_ns() -> int:
    return time.monotonic_ns()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=15.0, help="capture window in seconds")
    parser.add_argument("--interval", type=float, default=0.02, help="read interval in seconds")
    parser.add_argument("--out", default="", help="JSONL evidence log path (stdout if empty)")
    args = parser.parse_args()

    out = open(args.out, "w") if args.out else sys.stdout
    start = monotonic_ns()
    deadline = start + int(args.duration * 1e9)

    # Invariant tracking over distinct LIVE frames.
    distinct_live: list[dict] = []      # one record per distinct frame sequence
    seen_sequences: set[int] = set()
    prev_seq: int | None = None
    prev_mono: int | None = None
    seq_strictly_increasing = True
    mono_strictly_increasing = True
    session_ids: set[int] = set()
    all_source_authoritative = True
    all_policy_states_valid = True
    total_reads = 0
    live_reads = 0
    consecutive_live_reads = 0
    best_live_run = 0
    last_status: str | None = None
    transition: dict | None = None  # first LIVE -> non-LIVE
    first_live_read_t: int | None = None

    try:
        while monotonic_ns() < deadline:
            now = monotonic_ns()
            data = read_shm_frame()
            status, frame = classify_frame(data, now)
            total_reads += 1

            rec = {
                "t_monotonic_ns": now,
                "status": status.value,
                "source": frame.source if frame else None,
                "session_id": frame.session_id if frame else None,
                "sequence": frame.sequence if frame else None,
                "monotonic_ns": frame.monotonic_ns if frame else None,
                "policy_state": frame.policy_state if frame else None,
                "rl_step": frame.rl_step if frame else None,
            }
            out.write(json.dumps(rec) + "\n")
            out.flush()

            # First LIVE -> non-LIVE transition (controller exit / invalidation).
            if last_status == FrameStatus.LIVE.value and status is not FrameStatus.LIVE:
                if transition is None:
                    transition = {
                        "from": last_status,
                        "to": status.value,
                        "t_monotonic_ns": now,
                    }
            last_status = status.value

            if status is FrameStatus.LIVE and frame is not None:
                live_reads += 1
                consecutive_live_reads += 1
                best_live_run = max(best_live_run, consecutive_live_reads)
                if first_live_read_t is None:
                    first_live_read_t = now
                session_ids.add(frame.session_id)
                if frame.source != SOURCE_AUTHORITATIVE_RUNTIME:
                    all_source_authoritative = False
                if frame.policy_state not in (POLICY_AGILE, POLICY_RECOVERY, POLICY_FAULTED):
                    all_policy_states_valid = False
                if frame.sequence not in seen_sequences:
                    seen_sequences.add(frame.sequence)
                    if prev_seq is not None and frame.sequence <= prev_seq:
                        seq_strictly_increasing = False
                    if prev_mono is not None and frame.monotonic_ns <= prev_mono:
                        mono_strictly_increasing = False
                    prev_seq = frame.sequence
                    prev_mono = frame.monotonic_ns
                    distinct_live.append(rec)
            else:
                consecutive_live_reads = 0

            time.sleep(args.interval)
    except KeyboardInterrupt:
        # Early stop (e.g. the orchestration SIGINTs the sampler after the
        # controller exit evidence is captured): still emit the summary.
        pass

    session_consistent = len(session_ids) == 1
    summary = {
        "total_reads": total_reads,
        "live_reads": live_reads,
        "best_consecutive_live_run": best_live_run,
        "distinct_live_frames": len(distinct_live),
        "session_ids": sorted(session_ids),
        "session_consistent": session_consistent,
        "sequence_strictly_increasing": seq_strictly_increasing,
        "monotonic_strictly_increasing": mono_strictly_increasing,
        "all_source_authoritative": all_source_authoritative,
        "all_policy_states_valid": all_policy_states_valid,
        "first_live_read_t": first_live_read_t,
        "live_to_nonlive_transition": transition,
    }
    print("\n=== SAMPLER SUMMARY ===")
    print(json.dumps(summary, indent=2))
    if args.out:
        out.close()

    failed = False
    if len(distinct_live) < 10:
        print("RESULT=FAIL (fewer than 10 distinct LIVE frames captured)")
        failed = True
    elif not (seq_strictly_increasing and mono_strictly_increasing
              and session_consistent and all_source_authoritative
              and all_policy_states_valid):
        print("RESULT=FAIL (truth invariant violated)")
        failed = True
    else:
        print("RESULT=OK")
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
