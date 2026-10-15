#!/usr/bin/env python3
"""Focused production-reader, freshness-race, and pre-RL gate checks."""
from __future__ import annotations

import os
import struct
import sys
import tempfile
from pathlib import Path
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parent))
import abs_base_bounds
from abs_base_bounds import MAGIC, SNAPSHOT, VERSION, read_snapshot
from s1_run_diagnostic import base_bounds_gate_failure


def packed(sequence: int = 2, *, version: int = VERSION, step: int = 1,
           monotonic_ns: int = 1_000_000_000, coverage: int = 1, valid: int = 1,
           scene: str = "m1dod01", capture: str = "p1-10-capture-" + "a" * 32,
           fingerprint: str = "b" * 64) -> bytes:
    q = (MAGIC, version, sequence, monotonic_ns, step, 55, coverage, valid, 0, 0, 0, 0)
    d = (1.0, 0.0, 0.0, -0.5, 8.0, -3.0, 3.0,
         0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    enc = lambda text, n: text.encode("ascii").ljust(n, b"\0")
    return SNAPSHOT.pack(*q, *d, enc(scene, 32), enc(capture, 64), enc(fingerprint, 64))


def run() -> None:
    assert SNAPSHOT.size == 360
    with tempfile.TemporaryDirectory(prefix="s6-12-bounds-reader-") as directory:
        path = Path(directory) / "snapshot"
        path.write_bytes(packed())

        # Explicit now_ns means an exact validation instant, retained for
        # deterministic offline checks. Production calls omit it.
        good = read_snapshot(path, now_ns=1_000_000_010,
            expected_scene_id="m1dod01", expected_capture_id="p1-10-capture-" + "a" * 32,
            expected_fingerprint="b" * 64)
        assert good["status"] == "LIVE" and good["snapshot"]["physics_step"] == 1
        assert good["snapshot"]["validation_monotonic_ns"] == 1_000_000_010
        assert good["snapshot"]["age_ns"] == 10
        assert base_bounds_gate_failure(good) is None

        # The exact freshness boundary remains LIVE; only age > 500 ms is stale.
        assert read_snapshot(path, now_ns=1_500_000_000)["status"] == "LIVE"
        expired = read_snapshot(path, now_ns=1_500_000_001)
        assert expired["status"] == "STALE"
        assert base_bounds_gate_failure(expired) is not None
        future = read_snapshot(path, now_ns=999_999_999)
        assert future["status"] == "INVALID" and future["reason"] == "future_timestamp"
        assert base_bounds_gate_failure(future) is not None

        wrong_scene = read_snapshot(path, now_ns=1_000_000_010, expected_scene_id="wrong")
        assert wrong_scene["status"] == "INVALID" and wrong_scene["reason"] == "identity_mismatch"
        assert base_bounds_gate_failure(wrong_scene) is not None
        path.write_bytes(packed(version=VERSION + 1))
        assert read_snapshot(path, now_ns=1_000_000_010)["reason"] == "magic_or_version_mismatch"
        path.write_bytes(packed(sequence=3))
        assert read_snapshot(path, now_ns=1_000_000_010)["reason"] == "seqlock_unstable"
        path.write_bytes(b"short")
        assert read_snapshot(path)["reason"] == "size_mismatch"

        # Model the concurrency window from the failed pre-RL read. The
        # producer publishes a complete newer snapshot after the old
        # read-start instant, before the reader fetches its payload. A reader
        # that samples time before I/O sees a false future timestamp; the
        # production reader now samples only after its consistent read.
        race_path = Path(directory) / "race_snapshot"
        race_path.write_bytes(packed(monotonic_ns=1_000_000_000))
        original_pread = os.pread
        original_pwrite = os.pwrite
        race = {"updated": False}

        def publish_before_payload_read(fd: int, count: int, offset: int) -> bytes:
            if not race["updated"]:
                writer_fd = os.open(race_path, os.O_WRONLY)
                try:
                    original_pwrite(writer_fd, packed(monotonic_ns=1_000_000_100), 0)
                finally:
                    os.close(writer_fd)
                race["updated"] = True
            return original_pread(fd, count, offset)

        def clock_around_publication() -> int:
            # Before publication this is 99 ns behind the new stamp; after
            # publication it is 1 ns ahead.
            return 1_000_000_101 if race["updated"] else 1_000_000_001

        with mock.patch.object(abs_base_bounds.os, "pread", side_effect=publish_before_payload_read), \
             mock.patch.object(abs_base_bounds.time, "monotonic_ns", side_effect=clock_around_publication):
            raced = read_snapshot(race_path)
        assert raced["status"] == "LIVE", raced
        assert raced["snapshot"]["monotonic_ns"] == 1_000_000_100, raced
        assert raced["snapshot"]["validation_monotonic_ns"] == 1_000_000_101, raced
        assert raced["snapshot"]["age_ns"] == 1, raced
        assert base_bounds_gate_failure(raced) is None

        # The production pre-RL predicate rejects non-LIVE/invalid/incomplete
        # results; command 3 remains later in the caller's success-only path.
        assert base_bounds_gate_failure({"status": "LIVE", "snapshot": {
            "bounds_valid": 0, "physics_coverage_complete": 1}}) is not None
        assert base_bounds_gate_failure({"status": "LIVE", "snapshot": {
            "bounds_valid": 1, "physics_coverage_complete": 0}}) is not None
    print("S6-12 base-bounds production reader and pre-RL gate checks: PASS")


if __name__ == "__main__":
    run()
