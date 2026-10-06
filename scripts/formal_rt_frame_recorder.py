#!/usr/bin/env python3
"""Fail-closed recorder-side boundary for ``/dev/shm/mujoco_rt_frame``.

This module is intentionally not a formal-run writer.  It accepts production
input only through the fixed shared-memory reader and refuses to create a
formal artifact until the fields that are absent from the current frame have
authoritative runtime producers.  Synthetic probes are private test-only
helpers and can never be promoted to runtime-valid input.
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import List, Optional, Tuple

from abs_rt_frame import (
    DEFAULT_STALE_TIMEOUT_NS,
    FrameStatus,
    RuntimeFrame,
    SOURCE_AUTHORITATIVE_RUNTIME,
    read_shm_frame,
    classify_frame,
)


FIXED_RUNTIME_SOURCE = "/dev/shm/mujoco_rt_frame"
FORMAL_MISSING_FIELDS = (
    "run_id/session_binding",
    "simulation_time_s",
    "seed_provenance",
    "effective_config_provenance",
    "collision_and_fall_authority",
    "torque_saturation_authority",
    "active_ray_source_provenance",
    "measured_cadence",
    "complete_shutdown_evidence",
)


@dataclass(frozen=True)
class CaptureDecision:
    status: str
    reasons: Tuple[str, ...]
    source_sequence: Optional[int] = None
    formal_sequence: Optional[int] = None
    session_id: Optional[int] = None
    rejection_index: Optional[int] = None

    @property
    def runtime_valid(self) -> bool:
        return self.status == "RUNTIME_VALID"


class FormalRtFrameRecorder:
    """Observe the fixed runtime frame without producing formal artifacts.

    The production entry point has no path or dictionary injection parameter.
    ``capture_once`` always reads ``FIXED_RUNTIME_SOURCE``.  It deliberately
    has no ``FormalRunWriter`` member: current authority is incomplete, so a
    writer cannot be reached without violating the formal contract.
    """

    def __init__(self, stale_timeout_ns: int = DEFAULT_STALE_TIMEOUT_NS):
        self.stale_timeout_ns = stale_timeout_ns
        self._session_id: Optional[int] = None
        self._last_source_sequence: Optional[int] = None
        self._last_rl_step: Optional[int] = None
        self._last_monotonic_ns: Optional[int] = None
        self._next_formal_sequence = 0
        self._rejection_index = 0
        self._shutdown_complete = False

    def _reject(self, reasons: Tuple[str, ...], frame: Optional[RuntimeFrame] = None) -> CaptureDecision:
        self._rejection_index += 1
        return CaptureDecision(
            "INVALID", reasons,
            frame.sequence if frame is not None else None,
            None,
            frame.session_id if frame is not None else None,
            self._rejection_index,
        )

    def capture_once(self, now_ns: Optional[int] = None) -> CaptureDecision:
        """Read and inspect the fixed production source once."""
        if now_ns is None:
            now_ns = time.monotonic_ns()
        raw = read_shm_frame(FIXED_RUNTIME_SOURCE)
        if not raw:
            return self._reject(("missing_or_incoherent_fixed_runtime_frame",))
        return self._accept_runtime_bytes(raw, now_ns)

    def _accept_runtime_bytes(self, raw: bytes, now_ns: int) -> CaptureDecision:
        status, frame = classify_frame(raw, now_ns, self.stale_timeout_ns)
        if status is not FrameStatus.LIVE or frame is None:
            return self._reject((f"frame_status:{status.value}",), frame)
        return self._accept_live_frame(frame)

    def _accept_live_frame(self, frame: RuntimeFrame, *, authority_complete: bool = False) -> CaptureDecision:
        """Validate a live candidate.

        ``authority_complete`` is private and defaults false.  It exists only
        for future implementation/tests once the missing runtime producers are
        actually connected; production capture cannot opt into it yet.
        """
        reasons: List[str] = []
        if frame.source != SOURCE_AUTHORITATIVE_RUNTIME:
            reasons.append("source_not_authoritative_runtime")
        if self._session_id is not None and frame.session_id != self._session_id:
            reasons.append("session_id_changed")
        if self._last_source_sequence is not None and frame.sequence <= self._last_source_sequence:
            reasons.append("source_sequence_reversed_or_repeated")
        if self._last_rl_step is not None and frame.rl_step <= self._last_rl_step:
            reasons.append("rl_step_reversed_or_repeated")
        if self._last_monotonic_ns is not None and frame.monotonic_ns <= self._last_monotonic_ns:
            reasons.append("monotonic_time_reversed_or_repeated")
        if reasons:
            return self._reject(tuple(reasons), frame)

        if not authority_complete:
            return self._reject(
                tuple(f"missing_authoritative:{field}" for field in FORMAL_MISSING_FIELDS),
                frame,
            )

        # Commit session and ordering only when the sample is truly eligible.
        # Rejected samples never consume formal telemetry sequence.
        if self._session_id is None:
            self._session_id = frame.session_id
        self._last_source_sequence = frame.sequence
        self._last_rl_step = frame.rl_step
        self._last_monotonic_ns = frame.monotonic_ns
        formal_sequence = self._next_formal_sequence
        self._next_formal_sequence += 1

        return CaptureDecision(
            "RUNTIME_VALID",
            (),
            frame.sequence,
            formal_sequence,
            frame.session_id,
        )

    def mark_shutdown_complete(self) -> None:
        """Record an external shutdown proof; no proof is inferred here."""
        self._shutdown_complete = True

    @property
    def shutdown_complete(self) -> bool:
        return self._shutdown_complete

    def finalize(
        self,
        terminal_outcome: str,
        *,
        collision: bool = False,
        fall: bool = False,
        shutdown_rc: Optional[int] = None,
        forced_termination: bool = False,
    ) -> CaptureDecision:
        """Return a fail-closed final decision; never writes a summary.

        This reducer is intentionally conservative.  Safety evidence is
        checked before arrival/SUCCESS, and clean shutdown is required in
        addition to all currently missing formal sources.
        """
        reasons: List[str] = []
        if terminal_outcome not in {"SUCCESS", "COLLISION", "FALL", "TIMEOUT"}:
            reasons.append("unknown_terminal_outcome")
        if (collision or fall) and terminal_outcome == "SUCCESS":
            reasons.append("safety_evidence_overrides_success")
        if shutdown_rc != 0:
            reasons.append("shutdown_not_rc0")
        if forced_termination:
            reasons.append("forced_termination_invalid")
        if not self._shutdown_complete:
            reasons.append("complete_shutdown_evidence_missing")
        reasons.extend(f"missing_authoritative:{field}" for field in FORMAL_MISSING_FIELDS)
        return CaptureDecision("INVALID", tuple(reasons))


def _synthetic_probe_only(frame: RuntimeFrame, now_ns: int) -> CaptureDecision:
    """Test-only probe; explicit synthetic frames are never runtime-valid."""
    if frame.source != SOURCE_AUTHORITATIVE_RUNTIME:
        return CaptureDecision("INVALID", ("synthetic_input_rejected",), frame.sequence, None, frame.session_id, 1)
    return CaptureDecision("INVALID", ("synthetic_probe_cannot_establish_runtime_authority",), frame.sequence, None, frame.session_id, 1)
