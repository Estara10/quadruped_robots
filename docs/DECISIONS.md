# Architecture Decisions

Only accepted decisions belong here. Open questions remain in `GAP_MATRIX.md`.

## DEC-001 — Priority Order

- Status: Accepted on Day 0
- Decision: Correctness > Stability > Observability > Safety > Performance > Paper Speed.

## DEC-002 — Real ABS/RL Gate

- Status: Accepted on Day 0
- Decision: Formal real-robot ABS/RL experiments are NO-GO until Phase 1 Acceptance and the Phase 2 safety gates are satisfied.

## DEC-003 — Experimental Variants

- Status: Accepted on Day 0
- Decision: `paper-faithful` and `stabilized` behavior must be separate configurations and separate reported results. Engineering additions cannot be silently attributed to ABS.

## DEC-004 — Current-State Source of Truth

- Status: Accepted on Day 0
- Decision: `docs/CURRENT_STATE.md` is the only authoritative current project status. README and compatibility-agent files must point to it rather than maintain competing status copies.

## DEC-005 — Rolling Planning

- Status: Accepted on Day 0
- Decision: Only the next executable task has a detailed file under `docs/exec-plans/`. Future tasks stay at roadmap level until their dependencies are complete.

## DEC-006 — Historical Evidence

- Status: Accepted on Day 0
- Decision: Any historical run that does not satisfy the current `EXPERIMENT_PROTOCOL.md` is retained as `LEGACY / NON-ACCEPTANCE` and cannot enter formal Acceptance statistics.

## DEC-007 — Ray Frame Timing Contract

- Status: Accepted on 2026-08-24 (P1-01F)
- Decision: A runtime ray frame is valid only when the producer and consumer agree on the versioned shared-memory header, a sequence-consistent 11-beam snapshot, and a `steady_clock` nanosecond completion timestamp within the configured freshness threshold. Missing, stale, incoherent, or non-finite frames fail closed. Simulation-only fault injection must be explicitly armed and is disabled by the real-robot launch.
