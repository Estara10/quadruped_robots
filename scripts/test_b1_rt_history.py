#!/usr/bin/env python3
"""Check the production C++ ring publisher against the Python drain reader."""
from __future__ import annotations

import subprocess
import struct
import json
import sys
from pathlib import Path

from abs_rt_frame import FrameStatus, RuntimeFrame, read_shm_history, _HISTORY_HEADER
from s1_run_diagnostic import history_drain_limit


def run(writer: Path, output_dir: Path) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    overflow_path = output_dir / "overflow.bin"
    reset_path = output_dir / "new_session.bin"
    counter_reset_path = output_dir / "counter_reset.bin"
    uncommitted_path = output_dir / "uncommitted.bin"
    subprocess.run([str(writer), str(overflow_path), str(reset_path),
                    str(counter_reset_path), str(uncommitted_path)], check=True)

    first = read_shm_history(str(overflow_path), cursor=0, expected_session=101)
    assert first["status"] == "OVERFLOW" and first["overwritten_count"] == 2
    assert first["consumer_unread_loss_count"] == 2
    assert first["consumer_unread_loss_range"] == [0, 2]
    drained = read_shm_history(str(overflow_path), cursor=2048, expected_session=101)
    assert drained["status"] == "OK" and drained["cursor"] == 2050
    assert drained["overwritten_count"] == 2  # slots reused, but no unread loss at this cursor
    assert "consumer_unread_loss_count" not in drained
    assert [RuntimeFrame.from_bytes(raw).rl_step for _, raw in drained["frames"]] == [2048, 2049]
    assert all(RuntimeFrame.from_bytes(raw).session_id == 101 for _, raw in drained["frames"])

    bounded = read_shm_history(str(overflow_path), cursor=2048, expected_session=101,
                               max_source_sequence=history_drain_limit(
                                   FrameStatus.LIVE,
                                   RuntimeFrame.from_bytes(drained["frames"][0][1])))
    assert bounded["status"] == "OK" and bounded["cursor"] == 2049
    assert [RuntimeFrame.from_bytes(raw).rl_step for _, raw in bounded["frames"]] == [2048]
    assert history_drain_limit(FrameStatus.STALE,
                               RuntimeFrame.from_bytes(drained["frames"][1][1])) is None
    next_bounded = read_shm_history(str(overflow_path), cursor=bounded["cursor"],
                                    expected_session=101, max_source_sequence=4100)
    assert next_bounded["cursor"] == 2050
    assert [RuntimeFrame.from_bytes(raw).rl_step for _, raw in next_bounded["frames"]] == [2049]

    reused_current = read_shm_history(str(overflow_path), cursor=2049, expected_session=101)
    assert reused_current["status"] == "OK" and reused_current["cursor"] == 2050
    assert reused_current["overwritten_count"] == 2
    assert "consumer_unread_loss_count" not in reused_current

    session_changed = read_shm_history(str(reset_path), cursor=2050, expected_session=101)
    assert session_changed["status"] == "SESSION_CHANGED" and session_changed["session_id"] == 202
    new = read_shm_history(str(reset_path), cursor=0, expected_session=202)
    assert new["status"] == "OK" and [RuntimeFrame.from_bytes(raw).rl_step
                                       for _, raw in new["frames"]] == [0, 1]
    counter_reset = read_shm_history(str(counter_reset_path), cursor=5, expected_session=101)
    assert counter_reset["status"] == "RESET" and counter_reset["produced_count"] == 2
    incoherent = read_shm_history(str(uncommitted_path), cursor=0, expected_session=101)
    assert incoherent["status"] == "INCOHERENT" and incoherent["reason"] == "slot_uncommitted"

    bad_counter = output_dir / "bad_counter.bin"
    raw_bytes = bytearray(overflow_path.read_bytes())
    header = _HISTORY_HEADER.unpack_from(raw_bytes, 0)
    _HISTORY_HEADER.pack_into(raw_bytes, 0, *header[:5], header[5] - 1)
    bad_counter.write_bytes(raw_bytes)
    rejected = read_shm_history(str(bad_counter), cursor=2048, expected_session=101)
    assert rejected["status"] == "INVALID" and rejected["reason"] == "slot_reuse_counter_mismatch"
    (output_dir / "summary.json").write_text(json.dumps({
        "publisher": "common/abs_rt_frame_contract.h publishHistory via test_b1_rt_history.cpp",
        "reader": "scripts/abs_rt_frame.py read_shm_history",
        "cases": {
            "consumer_ahead_of_oldest_readable": {"status": first["status"],
                "unread_loss_count": first["consumer_unread_loss_count"],
                "unread_loss_range": first["consumer_unread_loss_range"]},
            "reused_slots_consumer_caught_up": {"status": drained["status"],
                "producer_slot_reuse_count": drained["overwritten_count"],
                "consumer_unread_loss": 0},
            "session_change": {"status": session_changed["status"]},
            "same_session_counter_reset": {"status": counter_reset["status"]},
            "uncommitted_slot": {"status": incoherent["status"], "reason": incoherent["reason"]},
            "reuse_counter_mismatch": {"status": rejected["status"], "reason": rejected["reason"]},
        },
    }, indent=2, sort_keys=True) + "\n", encoding="utf-8")


if __name__ == "__main__":
    if len(sys.argv) != 3:
        raise SystemExit("usage: test_b1_rt_history.py <cpp-writer> <output-dir>")
    run(Path(sys.argv[1]), Path(sys.argv[2]))
    print("B1 C++->Python history check: PASS")
