#!/usr/bin/env python3
"""Read-only capture of Go2 runtime policy frames and MuJoCo sim-clock samples.

Start before the simulator. Frames and physics-clock samples are saved separately
with their source monotonic timestamps; consumers must bracket-join them and must
not interpolate sim_time. No controller or simulator behavior is changed.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import signal
import struct
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[5]
sys.path.insert(0, str(ROOT / "scripts"))

from abs_rt_frame import (  # noqa: E402
    FRAME_MAGIC,
    FRAME_SIZE,
    FRAME_VERSION,
    SOURCE_AUTHORITATIVE_RUNTIME,
    FrameStatus,
    RuntimeFrame,
    classify_frame,
    read_shm_frame,
)
from run_record import RunRecordRecorder  # noqa: E402

CLOCK_PATH = "/dev/shm/mujoco_sim_clock"
CLOCK_MAGIC = 0x414253434C4F434B
CLOCK_VERSION = 2
CLOCK = struct.Struct("<4Qd")
STOP = False


def stop_handler(_signum, _frame):
    global STOP
    STOP = True


def read_clock():
    try:
        fd = os.open(CLOCK_PATH, os.O_RDONLY)
    except OSError:
        return None
    try:
        if os.fstat(fd).st_size < CLOCK.size:
            return None
        with os.fdopen(os.dup(fd), "rb") as stream:
            a = stream.read(CLOCK.size)
            b = os.pread(fd, CLOCK.size, 0)
        if len(a) != CLOCK.size or len(b) != CLOCK.size:
            return None
        ma, va, sa, mna, sima = CLOCK.unpack(a)
        mb, vb, sb, mnb, simb = CLOCK.unpack(b)
        if (ma, va, sa, mna) != (mb, vb, sb, mnb):
            return None
        if ma != CLOCK_MAGIC or va != CLOCK_VERSION or sa == 0 or sa & 1:
            return None
        if mna == 0 or not math.isfinite(sima) or sima != simb:
            return None
        return {"sequence": sa, "monotonic_ns": mna, "sim_time": sima}
    finally:
        os.close(fd)


def frame_dict(frame, prior):
    mode = {0: "AGILE", 1: "RECOVERY", 2: "FAULTED"}.get(frame.policy_state, "UNKNOWN")
    return {
        "sequence": frame.sequence,
        "session_id": frame.session_id,
        "rl_step": frame.rl_step,
        "policy_cycle_id": frame.rl_step,
        "monotonic_ns": frame.monotonic_ns,
        "ra_value": frame.ra_value,
        "mode_before": frame.mode_before,
        "mode_after": mode,
        "switching_mode": frame.switching_mode,
        "action_source": frame.action_source,
        "transition_reason": frame.transition_reason,
        "risk_condition_met": bool(frame.risk_condition_met),
        "risk_condition_entered": frame.risk_condition_entered,
        "risk_evaluation_ns": frame.risk_evaluation_ns,
        "risk_condition_entered_ns": frame.risk_condition_entered_ns or None,
        "policy_mode_changed": bool(frame.policy_mode_changed),
        "mode_change_ns": frame.mode_change_ns or None,
        "entry_threshold": frame.entry_threshold,
        "exit_threshold": frame.exit_threshold,
        "sim_time_s": frame.sim_time_s if frame.sim_clock_valid else None,
        "sim_clock_sequence": frame.sim_clock_sequence,
        "sim_clock_monotonic_ns": frame.sim_clock_monotonic_ns,
        "sim_clock_segment_id": frame.sim_clock_segment_id,
        "sim_clock_age_ns": frame.sim_clock_age_ns,
        "sim_clock_status": frame.sim_clock_status,
        "sim_clock_valid": bool(frame.sim_clock_valid),
        "mode_before_basis": "runtime_frame_v2::StateRL::runModel",
        "action_raw_policy_order": frame.action_raw,
        "action_clipped_controller_order": frame.action_clipped,
        "joint_target_rad_controller_order": frame.joint_target_rad,
        "ray_valid": bool(frame.ray_valid),
        "ray_age_ns": frame.ray_age_ns,
        "world_pose": frame.world_pose,
        "lin_vel": frame.lin_vel,
        "source": frame.source,
        "safety_faulted": bool(frame.safety_faulted),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out-dir", required=True, type=Path)
    ap.add_argument("--check", action="store_true", help="validate reader imports/layout and prepare outputs")
    ap.add_argument("--poll-s", type=float, default=0.001)
    args = ap.parse_args()
    if args.poll_s <= 0 or args.poll_s > 0.01:
        raise SystemExit("poll-s must be in (0, 0.01]")
    args.out_dir.mkdir(parents=True, exist_ok=True)
    if RuntimeFrame.from_bytes(bytes(FRAME_SIZE)).sequence != 0 or FRAME_MAGIC == 0 or FRAME_VERSION == 0:
        raise SystemExit("runtime frame layout preflight failed")
    (args.out_dir / "collector_ready.json").write_text(json.dumps({
        "ready": True,
        "reader": str(Path(__file__).resolve()),
        "runtime_frame_size": FRAME_SIZE,
        "sim_clock_size": CLOCK.size,
        "sim_clock_path": CLOCK_PATH,
        "preflight_monotonic_ns": time.monotonic_ns(),
        "note": "clock mapping is awaited during capture; this preflight does not claim a simulator is running",
    }, indent=2) + "\n")
    if args.check:
        return 0

    signal.signal(signal.SIGINT, stop_handler)
    signal.signal(signal.SIGTERM, stop_handler)
    meta = {
        "capture_start_monotonic_ns": time.monotonic_ns(),
        "poll_s_requested": args.poll_s,
        "clock_join_rule": "post-process bracketing on shared steady-clock monotonic_ns; no interpolation",
    }
    clocks_seen = set()
    frames_seen = set()
    prior = None
    run_record_path = args.out_dir / "runtime_record.jsonl"
    run_record = RunRecordRecorder(str(run_record_path))
    run_record.start()
    with (args.out_dir / "sim_clock.jsonl").open("w", buffering=1) as clock_out, \
            (args.out_dir / "policy_frames.jsonl").open("w", buffering=1) as frame_out:
        (args.out_dir / "capture_start.json").write_text(json.dumps(meta, indent=2) + "\n")
        while not STOP:
            clk = read_clock()
            if clk is not None and clk["sequence"] not in clocks_seen:
                clocks_seen.add(clk["sequence"])
                clock_out.write(json.dumps(clk, separators=(",", ":")) + "\n")
            raw = read_shm_frame()
            if raw:
                frame_now_ns = time.monotonic_ns()
                status, frame = classify_frame(raw, frame_now_ns)
                if (status is FrameStatus.LIVE and frame is not None
                        and frame.source == SOURCE_AUTHORITATIVE_RUNTIME
                        and frame.sequence not in frames_seen):
                    run_record.record_snapshot(raw, now_ns=frame_now_ns)
                    row = frame_dict(frame, prior)
                    frame_out.write(json.dumps(row, separators=(",", ":")) + "\n")
                    frames_seen.add(frame.sequence)
                    prior = row
            time.sleep(args.poll_s)
        clock_out.flush()
        frame_out.flush()
        os.fsync(clock_out.fileno())
        os.fsync(frame_out.fileno())
    run_record.stop_sampling()
    run_record.finalize({})
    meta.update({
        "capture_end_monotonic_ns": time.monotonic_ns(),
        "distinct_clock_samples": len(clocks_seen),
        "distinct_runtime_frames": len(frames_seen),
        "runtime_record_path": str(run_record_path),
        "stopped_by_signal": True,
    })
    (args.out_dir / "capture_end.json").write_text(json.dumps(meta, indent=2) + "\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
