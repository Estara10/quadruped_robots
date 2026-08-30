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

## DEC-008 — Project-owned MuJoCo Bridge Thread

- Status: Accepted on 2026-08-28 (P1-09O implementation; independent review later found an incomplete model-reload boundary)
- Decision: Any long-lived MuJoCo bridge callback that accesses `mjModel` or
  `mjData` must be owned by the project as a stoppable, joinable thread. The SDK
  detached `RecurrentThread` is not a lifecycle owner. Main must request bridge
  stop and join all m/d-accessing bridge workers before releasing final m/d.

## DEC-009 — Fail-closed MuJoCo Model Reload Barrier

- Status: Accepted on 2026-08-29
- Decision: While an m/d-accessing RobotBridge worker is active, MuJoCo must not
  directly replace `mjModel` or `mjData`. A reload must first enter the same
  stop-and-join barrier as teardown. Until a safe, reviewed rebind/restart
  protocol exists, a reload that cannot complete this barrier is refused
  fail-closed. This is a lifecycle safety decision only; it does not change ABS
  policy, thresholds, switching, solver, dynamics, or experiment criteria.

## DEC-010 — P1-01 Scope Alignment and Acceptance Reconciliation

- Status: Accepted on 2026-08-30 by the Director / project owner (explicitly
  approved; not an inference from missing evidence).
- Decision: P1-01's objective is **current deployment mapping correctness and
  simulation operational validity**, not forensic reconstruction of the
  historical training environment. Historical training reproducibility
  (config, seed, command, Git revision, export invocation, raw RA rollout
  dataset, episode count, independent shell/job logs) is **deferred
  reproducibility**, **not** a P1-01 Acceptance condition or blocker. The
  project owner declares that Agile was trained by the owner and that RA was
  trained using Agile checkpoint `model_4000.pt`; this is recorded as
  `OPERATOR_DECLARED`, not as independently immutable historical proof. Real
  Go2 `foot_force[0..3]` slot semantics are **Phase 2 hardware-only**. The
  declared policy order `FL,FR,RL,RR` → documented action remap → controller /
  MuJoCo actuator order `FR,FL,RR,RL` is the operational mapping, accepted
  conditionally on the operator-declared training order and existing asymmetric
  contract evidence; it is not claimed to be independently recovered historical
  artifact metadata. This decision does not change the underlying controller
  remap and does not reopen P1-09. The independent Reviewer accepted P1-01
  with known issues on 2026-08-30; Phase 1 remains NOT ACCEPTED.
