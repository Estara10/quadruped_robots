# P1-10 Stage-A Common-Start Alignment Investigation — 2026-09-07

## Scope and status

This is a read-only design investigation for a future flat Stage-A pair. It
does not modify the comparator, runtime, scene, model, configuration, pair, or
saved run artifacts, and it starts no MuJoCo, ROS2, controller, A/B, benchmark,
FormalRun, or later P1 task.

The input pair is
[`replay_pair_20260907_stage_a_current_instrumented/`](replay_pair_20260907_stage_a_current_instrumented/).
It is completed runtime evidence: Run A and Run B individually have valid
records and clean process facts, while the exact saved-record comparison is
`FAIL` (`difference_count=66604`). It is complete and non-retryable. This
investigation does not create a replacement pair.

## Source trace

The conclusions below are based on the current source and the saved pair
diagnosis:

- MuJoCo model/data creation and the transition into `PhysicsLoop`:
  `unitree_mujoco/simulate/src/main.cc:566-594`.
- PhysicsLoop stepping and sim-clock/collision publication:
  `unitree_mujoco/simulate/src/main.cc:300-555`.
- Simulator default run state: `unitree_mujoco/simulate/src/mujoco/simulate.h:217-219`.
- Bridge initial-ready wait, which does not gate PhysicsLoop:
  `unitree_mujoco/simulate/src/main.cc:596-640` and
  `unitree_mujoco/simulate/src/bridge_lifecycle.h:73-84`.
- UI-only pause/reset/step paths:
  `unitree_mujoco/simulate/src/mujoco/simulate.cc:1668-1686` and
  `2039-2048`.
- Capture launch order, recording order, and fixed window:
  `scripts/p1_08_baseline_capture.py:1241-1319`.
- Controller-active and RL-active polling:
  `scripts/p1_08_baseline_capture.py:1372-1401`.
- Recorder meta/start boundary:
  `scripts/run_record.py:343-359`.
- Controller lifecycle/FSM update and activation:
  `quadruped_ros2_control_humble/controllers/rl_quadruped_controller/src/RlQuadrupedController.cpp:68-166,253-343`.
- RL entry/session and authoritative frame production:
  `quadruped_ros2_control_humble/controllers/rl_quadruped_controller/src/FSM/StateRL.cpp:280-430,888-909`.
- ROS launch/spawner chain:
  `quadruped_ros2_control_humble/controllers/rl_quadruped_controller/launch/mujoco.launch.py:52-99`.

## 1. Current start and readiness semantics

### Physics start

`PhysicsThread` loads the XML, calls `mj_makeData`, calls `mj_forward`, marks
the bridge's internal `initialReady`, and immediately enters `PhysicsLoop`.
`sim.run` defaults to `1`. On the first running loop, the initial sync branch
calls `mj_step`; later in-sync iterations call `mj_step` repeatedly. The clock
and collision snapshot are published after those controlled PhysicsLoop steps.

`initialReady` is only consumed by the Unitree bridge thread before DDS/bridge
construction. It is not a physics-start barrier: PhysicsLoop has no wait on
controller-manager state, controller activation, or RL entry.

### Actual signals in the current harness

| Boundary | Current signal | What it proves | What it does not prove |
|---|---|---|---|
| MuJoCo data ready | `mj_makeData` + `mj_forward`, then internal `initialReady` | model/data are available to bridge setup | physics is paused, controller is ready, or a capture anchor exists |
| Sim clock advancing | `wait_sim_clock_advance()` observes a changing `/mujoco_sim_clock` sequence | at least one simulator step has already occurred | common initial state or controller readiness |
| ROS/controller ready | `ros2 control list_controllers` returns rc 0 and output contains `rl_quadruped_controller` and `active` | the named controller is reported active by the external query | exact `on_activate`/FSM/RL transition timestamp or shared physics sequence |
| RL ready | a polled `/mujoco_rt_frame` decodes as authoritative and has `rl_active == 1` | a producer frame has already declared RL active | the exact producer edge time before polling, or qpos0 at that edge |
| Runtime-record start | `RunRecordRecorder.start()` writes meta after the RL poll | the consumer record has begun | the first physics step or exact RL-enter edge was recorded atomically |

The current `RuntimeFrame` contains `monotonic_ns`, `session_id`, frame
`sequence`, `rl_step`, `controller_active`, `rl_entered`, and `rl_active`.
Those are useful partial anchors. Its frame sequence is not the sim-clock
sequence, and the two streams have no atomic cross-stream start marker. The
recorder starts after the harness has observed RL active, so the current
`created_at_ns` is a consumer-side boundary rather than the RL producer edge.

## 2. Why the first captured frame is already 12–14 seconds into simulation

The current order is:

1. launch MuJoCo;
2. wait only until the sim-clock sequence advances
   (`wait_sim_clock_advance(20.0)` uses `20.0` as a timeout, not as a required
   simulation-time advance);
3. launch ROS2;
4. wait for the controller-active CLI result;
5. publish the fixed `2 -> 2 -> 3 -> 3` command sequence with `0.5 s` and
   `4.0 s` sleeps;
6. poll for `rl_active`; only then start the runtime recorder.

Physics is unpaused throughout these stages because `sim.run` starts at `1`.
The saved logs therefore show normal asynchronous pre-roll:

| Run | Controller active (log) | RL active / record start (log) | First LIVE sim time | First LIVE `rl_step` / frame source sequence |
|---|---:|---:|---:|---:|
| A | 14:15:09 | 14:15:19 | 13.948 s | 96 / 194 |
| B | 14:17:14 | 14:17:23 | 12.384 s | 95 / 192 |

The exact saved evidence supports `startup alignment` as the minimum root-cause
classification. It does not support an unbound random source. A scheduler or
process-timing contribution is possible (both logs contain the same non-fatal
FIFO scheduling warning), but causality is not isolated by this pair. Fixed
scene/config/executable/model hashes agree.

The original `mj_makeData:qpos0` is therefore only the simulator's initial
state at allocation. It is not the state at the first captured LIVE frame.

## 3. Existing mechanisms and candidate designs

### A. Startup pause until controller ready

There is no current harness-controlled, non-UI startup pause. The `sim.run`
flag is the relevant simulator switch, but this source exposes it through the
simulator/UI path; the only automatic assignment found is the model-warning
pause in `LoadModel`, not a readiness contract. The existing bridge lifecycle
condition variable does not stop PhysicsLoop.

A minimal future interface would be a one-shot, harness-controlled startup
gate in the simulator/PhysicsLoop. It must publish an initial-state-ready
record, keep `mjData` at the verified `mj_makeData:qpos0` state until release,
and record a release acknowledgement before any formal capture step. The
release must not be a key press or an operator UI action.

Impact:

- Initial-state authenticity: strongest of the candidates; no pre-RL physics
  drift if the gate is held before the first `mj_step`.
- Control semantics: startup scheduling changes. The controller must be tested
  to ensure it can reach its declared ready state while physics is held, or the
  protocol must define the exact release-to-RL transition. This is not proven
  by the current static source.
- Auditability: high if the gate state, release event, qpos/source binding,
  controller-ready observation, and RL edge are all saved.
- Experiment behavior: changes the startup boundary and removes the current
  uncontrolled pre-roll; it does not change policy equations, RA/Agile/
  Recovery logic, thresholds, or physical parameters.

### B. Reset to qpos0 after controller ready

MuJoCo's `mj_resetData` plus `mj_forward` exists in `simulate.cc`, but the
current path is a UI pending-reset operation. There is no harness API or
protocol that coordinates it with controller/FSM state, bridge ownership,
history, or recording. Resetting `mjData` after controller activation would
also require proving what happens to controller-held observations, actions,
timers, and DDS state. It is not a safe existing mechanism and is not the
minimum recommendation.

Impact:

- Initial-state authenticity: can restore model data to its default state only
  after a new reset contract is proven; it does not automatically restore all
  controller state.
- Control semantics: materially changes the episode boundary and can discard
  state already observed by the controller.
- Auditability: possible but requires reset-before/after snapshots and a
  controller/FSM reset acknowledgement.
- Experiment behavior: more intrusive than a pre-step gate; it can change the
  first control action and trajectory even without changing the ABS algorithm.

### C. Anchor at the actual RL enter edge

The existing frame is the closest non-UI mechanism. `StateRL::enter()` creates
the session and sets `running_`; `writeRtFrame()` publishes the session,
monotonic timestamp, RL step, and RL flags. `wait_rl_active()` observes that
frame. This is a useful producer-side candidate, but the current recorder
starts only after polling and does not preserve a dedicated edge event joined
to the first physics step.

Impact:

- Initial-state authenticity: does not restore qpos0 or remove pre-roll.
- Control semantics: none if used only for recording.
- Auditability: materially improves the RL/record boundary; a producer-side
  edge record or an atomic shared anchor is still needed to join it to the
  corresponding physics step.
- Experiment behavior: no ABS algorithm change.

## 4. Recommended minimum design

No existing mechanism safely satisfies all four requested conditions. The
minimum complete contract should be a coordinated startup protocol with two
small pieces:

1. a non-UI simulator startup gate held before the first controlled
   `PhysicsLoop::mj_step`, released once the harness has recorded the frozen
   context and a real controller-ready result; and
2. a producer-side RL-enter anchor, retained in the existing runtime-frame
   contract or its versioned event companion, containing at least capture/run
   identity, internal session, frame sequence, RL step, monotonic time, and a
   physics-step/sim-clock reference. The recorder must be armed before the
   release/edge and must retain the first edge frame.

The protocol must explicitly decide whether RL entry occurs while the gate is
held or immediately after release. Static inspection cannot assume that the
current controller update loop can complete the `2 -> 2 -> 3` transition while
physics is paused. That interaction needs a focused offline interface test and
one separately authorized runtime smoke validation before a formal pair.

Do not use post-ready `mj_resetData` as a shortcut. If controller-ready cannot
be reached while paused, the minimum safe alternative is a formally defined
release-to-RL handshake that proves the qpos0 snapshot at the RL anchor; a
plain sleep or a recorder-side timestamp is insufficient.

This design requires new interfaces; it is not implemented by this
investigation. The comparator threshold and exact/numeric/excluded rules remain
unchanged. The next Director decision is whether to authorize this focused
startup-gate/anchor design, or to define a different pre-registered
behavioral-repeatability metric.

## Status and boundaries

- `replay_pair_20260907_stage_a_current_instrumented` is completed evidence and
  must not be retried.
- Run A and Run B individually succeeded; exact saved-record comparison failed.
- This task added no runtime evidence and created no pair.
- P1-10 remains **IMPLEMENTED / AWAITING INDEPENDENT REVIEW**.
- Flat replay remains only a Stage-A infrastructure/repeatability sub-gate;
  P1-10 is not accepted and no later P1 task is authorized.

