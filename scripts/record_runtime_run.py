#!/usr/bin/env python3
"""Continuous per-run recorder over the real-time runtime frame (P1-09).

Thin loop that saves the full payload of every snapshot at the fixed source
``/dev/shm/mujoco_rt_frame`` into one per-run record (JSONL), then appends the
terminal block from the run's process facts. It reuses the existing reader and
classifier (``abs_rt_frame``) and the record/summary primitives in
``run_record``; it does not reimplement any frame source.

Usage (single run, e.g. under an orchestrator):

    python3 record_runtime_run.py --output run_<id>.jsonl \
        --facts process_facts.json

The ``--facts`` JSON file is written by the run orchestrator after the MuJoCo /
controller child is observed to exit; its fields are recorded verbatim and any
missing field is recorded UNKNOWN. Example:

    {"exit_code": 0, "forced_termination": false,
     "shutdown_request_source": "SIGINT", "shutdown_complete": true}

End conditions (whichever comes first): an interrupt (SIGINT/SIGTERM), a bounded
``--iters`` count, or ``--idle-exit-s`` seconds without a new frame. The loop
itself proves nothing about thread lifecycle or clean shutdown; it only archives
what the frame contained and records the orchestrator-supplied process facts.
"""

from __future__ import annotations

import argparse
import json
import signal
import sys
import time
from pathlib import Path
from typing import Any, Dict, Optional

from abs_rt_frame import DEFAULT_STALE_TIMEOUT_NS, read_shm_frame
from run_record import RunRecordRecorder


def _load_facts(path: Optional[str]) -> Dict[str, Any]:
    """Read the orchestrator's process-facts file, if present and parseable."""
    if not path:
        return {}
    facts_path = Path(path)
    if not facts_path.exists():
        print(f"[record] process-facts file not found at finalize: {facts_path}", file=sys.stderr)
        return {}
    try:
        data = json.loads(facts_path.read_text(encoding="utf-8"))
        if not isinstance(data, dict):
            print(f"[record] process-facts file is not a JSON object: {path}", file=sys.stderr)
            return {}
        return data
    except ValueError as exc:
        print(f"[record] process-facts file is not valid JSON: {path} ({exc})", file=sys.stderr)
        return {}


def run(
    output: str,
    *,
    iters: Optional[int] = None,
    interval_s: float = 0.05,
    idle_exit_s: Optional[float] = None,
    facts: Optional[str] = None,
    stale_timeout_ns: int = DEFAULT_STALE_TIMEOUT_NS,
) -> Dict[str, Any]:
    recorder = RunRecordRecorder(output, stale_timeout_ns=stale_timeout_ns)
    recorder.start()
    print(f"[record] run_id={recorder.run_id} -> {output}", file=sys.stderr)

    stop = False

    def _on_signal(signum: int, _frame: object) -> None:
        nonlocal stop
        print(f"[record] received signal {signum}; finalizing", file=sys.stderr)
        stop = True

    old_int = signal.signal(signal.SIGINT, _on_signal)
    old_term = signal.signal(signal.SIGTERM, _on_signal)
    try:
        iteration = 0
        last_seen_ns: Optional[int] = None
        frames_written = 0
        while not stop:
            if iters is not None and iteration >= iters:
                break
            raw = read_shm_frame()
            line = recorder.record_snapshot(raw)
            if line.get("payload") is not None:
                last_seen_ns = time.monotonic_ns()
                frames_written += 1
            elif idle_exit_s is not None and last_seen_ns is not None:
                idle_ns = time.monotonic_ns() - last_seen_ns
                if idle_ns >= idle_exit_s * 1e9:
                    print(f"[record] idle for {idle_exit_s}s without a frame; finalizing", file=sys.stderr)
                    break
            iteration += 1
            if iters is None or iteration < iters:
                if not stop:
                    time.sleep(interval_s)
        recorder.stop_sampling()
        terminal = recorder.finalize(_load_facts(facts))
        print(f"[record] wrote {frames_written} frame lines; finalized", file=sys.stderr)
        return terminal
    finally:
        signal.signal(signal.SIGINT, old_int)
        signal.signal(signal.SIGTERM, old_term)
        recorder.close()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, help="per-run record path (.jsonl)")
    parser.add_argument("--iters", type=int, default=None, help="bounded snapshot count")
    parser.add_argument("--interval-s", type=float, default=0.05, help="poll interval (s)")
    parser.add_argument("--idle-exit-s", type=float, default=None, help="finalize after this idle time without a frame")
    parser.add_argument("--facts", default=None, help="orchestrator process-facts JSON path")
    parser.add_argument("--stale-timeout-ns", type=int, default=DEFAULT_STALE_TIMEOUT_NS)
    args = parser.parse_args()
    run(
        args.output,
        iters=args.iters,
        interval_s=args.interval_s,
        idle_exit_s=args.idle_exit_s,
        facts=args.facts,
        stale_timeout_ns=args.stale_timeout_ns,
    )


if __name__ == "__main__":
    main()
