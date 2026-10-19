# P1-10 Stage-A Common-Start Alignment — Offline Implementation Evidence

## Scope and status

This increment implements the Director-approved minimum startup-alignment
mechanism for a future flat Stage-A replay. It is an observability and launch
boundary change only. No MuJoCo, ROS2, controller, A/B replay, benchmark,
FormalRun, or later P1 task was started. No pair was created by this
increment.

The `replay_pair_20260907_stage_a_current_instrumented` was completed before
this increment and remains historical and non-retryable. Its preserved
diagnosis records the Run A/Run B results and `difference_count=66604`; the
2026-09-05 pair and all earlier failed pairs remain non-reusable.

Evidence-integrity note: while running an existing offline regression, an old
fixture helper removed the real 2026-09-07 pair's `run_A`, `run_B`, and
comparison-output files before its setup failed. The pair manifest and the
preserved read-only diagnosis remain, but the raw artifact set is now
incomplete. No runtime files were fabricated and no pair retry is permitted;
the replacement test helper is non-mutating. This incident is reported here
so the pair is not treated as a current comparison input.

Status: **P1-10 Stage-A COMMON-START IMPLEMENTATION — IMPLEMENTED / AWAITING
INDEPENDENT REVIEW**.

## Frozen offline execution identity

The independent common-start execution manifest is
[`stage_a_common_start_execution_manifest_20260907.json`](stage_a_common_start_execution_manifest_20260907.json).

| Item | Frozen value |
|---|---|
| Manifest SHA-256 | `ba68c1b5458e02d3038efa894da36e0e864b9261c7d4ea453dc9c45fa27222ae` |
| Canonical identity SHA-256 | `da4a27bd5e730291ddcee99de5b54f0fef5349393fed1e9bff378fc9dc98fb9d` |
| Execution kind | `P1-10_STAGE_A_FLAT_REPLAY_COMMON_START` |
| Scenario / variant | `flat_goal_forward` / `stabilized` |
| Seed / window | `20260902` / `25.0 s` |
| Initial state | `scene_default / mj_makeData:qpos0` |
| Current instrumented executable | `c076456703bc35bfb68acd670df4fb0e24eba0f3bc008b3b407cd556fe37ca65`, 4,220,480 bytes |
| Controller plugin | `418cef686967f4973fec6e331780fb26380d5d0a40044b2fe2aeb9ae5a8946bf` |
| P1-08 canonical identity | `59dd13fed5ebd026ec519f2659643237502be8e4d8df5174a65b7d35ceb4f7e0` (parent/provenance only) |

The current executable is a distinct Stage-A instrumented artifact, not the
accepted P1-08 executable. The common-start identity contains no absolute
path, timestamp, or runtime capture ID. The capture ID is created once by the
harness for each future launch and is transmitted only through the bound
launch environment and saved run artifacts.

## Common-start gate contract

`common/abs_stage_a_common_start_contract.h` and
`scripts/p1_10_common_start.py` define the versioned 264-byte shared record.
The simulator creates the record after `mj_makeData` and `mj_forward`, binds
the harness-generated capture ID and initial-qpos SHA, and publishes
`WAITING_FOR_RELEASE` plus `INITIAL_READY`.

In the controlled `main.cc::PhysicsLoop` paths:

1. Before release, only `mj_forward` is reachable; `mj_step` is not called.
2. The harness waits for the simulator-owned readiness record, then launches
   ROS/controller and waits for the named controller active signal, RL-active
   producer frame, and the declared goal command sequence.
3. `GateClient.release()` is one-shot and capture-ID-bound. A wrong, repeated,
   malformed, missing, or timed-out release fails closed.
4. Only after the durable `RELEASED` state is visible may the first controlled
   PhysicsLoop `mj_step` execute. The simulator records its physics step,
   monotonic time, and simulation time in the same contract record.

The shared record uses a seqlock with a CAS writer claim so simulator and
controller producers cannot silently overwrite one another's anchor fields.
The UI `simulate.cc` reset, keyframe, step-forward, and teleoperation paths are
not part of formal Stage-A authority and are prohibited by the operator
instructions.

## Producer-side anchor and record binding

`StateRL::enter()` records the existing internal RL session and producer-side
RL-enter edge. `StateRL::writeRtFrame()` records the first post-release runtime
frame after the first released physics step. The resulting anchor contains:

- capture ID and initial-qpos SHA;
- gate state and release monotonic timestamp;
- RL session, RL step, frame sequence, and RL monotonic timestamp;
- first PhysicsLoop step, monotonic timestamp, and simulation time;
- first formal runtime-frame sequence/step/time and complete flags.

The harness validates this anchor before sampling, saves
`common_start_anchor.json`, stores the same anchor in
`process_facts.json`, and passes it to `RunRecordRecorder`. The gated record
stores it in terminal `start_anchor`; the first LIVE frame carries
`start_anchor_ref`. Record validation rejects a missing, malformed, stale,
wrong-capture, wrong-qpos, or otherwise incomplete anchor. Legacy records with
no `common_start_required` field remain readable, but are never relabeled as
common-start records.

The anchor is a start/observability contract. It does not change policy input,
RA/Agile/Recovery/switching logic, thresholds, physics, timestep, PD, solver,
scene geometry, or control values. Collision instrumentation may be present in
the current binary, but this flat Stage-A identity does not claim obstacle
collision evidence.

## Offline validation performed

- Common-start Python gate/anchor tests: **6/6 PASS**.
- Run-record regression: **54/54 PASS**.
- Historical Stage-A identity boundary tests: **4/4 PASS**; stale identity is
  rejected rather than rebound to rebuilt artifacts.
- Historical Stage-B identity boundary tests: **2/2 PASS**; stale identity is
  rejected rather than rebound to rebuilt artifacts.
- P1-08 harness regression: **96 checks PASS**.
- Simulator common-start C++ contract test: **PASS**.
- CTest `p1_08_sim_clock_test` and
  `p1_10_stage_a_common_start_contract_test`: **2/2 PASS**.
- Simulator target and controller target: **offline build PASS**.

The old comparator test was corrected to avoid deleting or recreating a real
historical pair; full comparator filesystem fixtures remain temporary and
non-pair-backed. No runtime success was manufactured by these tests.

## Remaining boundary

This is implementation and offline contract evidence only. The common-start
manifest remains `FROZEN_OFFLINE_PENDING_INDEPENDENT_REVIEW`; no fresh common-
start pair exists. Independent Reviewer review must pass before Director may
freeze a new pair and issue a separate, controlled A/B authorization. P1-10 is
not accepted; flat replay remains only the Stage-A infrastructure/
repeatability sub-gate; P1-11/P1-12/P1-13 remain unauthorized.
