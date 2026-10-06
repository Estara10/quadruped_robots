# P1-09F — StateRLRec Thread Teardown Stability

Date: 2026-08-28  
Scope: `StateRLRec` worker-thread lifetime only. No ROS2, MuJoCo, benchmark,
pilot, formal recorder run, or real-robot process was started.

## Observed failure mechanism

P1-09E captured the complete HUD evidence before shutdown, then controller-manager
teardown aborted with `terminate called without an active exception` / SIGABRT
(exit `-6`). Its captured stack relationship was controller plugin deletion →
`StateRLRec::~StateRLRec()`.

The source chain explains that failure:

1. `LeggedGymController::on_activate()` owns `StateRLRec` in
   `state_list_.rlRec` ([`RlQuadrupedController.cpp`](../../../quadruped_ros2_control_humble/controllers/rl_quadruped_controller/src/RlQuadrupedController.cpp),
   lines 331–336); plugin destruction releases that state.
2. Before P1-09F, `StateRLRec` constructed a `std::thread` in its constructor
   with `while (true)`, and had no destructor or join. `exit()` only disabled
   model execution, leaving the thread joinable.
3. Destruction of a joinable `std::thread` calls `std::terminate`, matching the
   P1-09E shutdown symptom. No P1-09E HUD value is invalidated by this
   post-capture teardown failure, but a recorder run cannot be considered
   stably ended while it exists.

## Old → new lifecycle

| Point | Before | P1-09F |
|---|---|---|
| Controller activation | `StateRLRec` construction immediately created a permanent worker. | Construction loads the model only; no worker exists yet. |
| `enter()` | Sets `running_`; old worker already exists. | Initializes the same Recovery state, sets `running_`, then starts one worker. |
| Worker loop | `while (true)`; only inference was gated by `running_`. | Stops when `StoppableThread::stopRequested()` is true; periodic wait is woken by a condition variable. |
| `exit()` | Sets `running_ = false`, but leaves a joinable worker. | Requests stop and joins before resetting counters/zeroing the same Kp/Kd/tau fields. |
| Re-entry | Reuses a permanently running worker. | Starts one fresh worker only after the old one is joined. |
| Destructor/plugin unload | `std::thread` could still be joinable, triggering terminate. | Calls the same idempotent stop-and-join as `exit()`. No detach path exists. |

Implementation anchors:

- [`StateRLRec.cpp`](../../../quadruped_ros2_control_humble/controllers/rl_quadruped_controller/src/FSM/StateRLRec.cpp):
  `startRlThread()` / `stopRlThread()` (lines 62–102), `enter()` start (lines
  104–147), and `exit()` join-before-command cleanup (lines 167–181).
- [`StoppableThread.h`](../../../quadruped_ros2_control_humble/controllers/rl_quadruped_controller/include/rl_quadruped_controller/common/StoppableThread.h):
  join-only owner, stop notification, and start/stop serialization (lines
  19–90).

The normal active-state inference body remains `runModel()` at the existing
`decimation / frequency` period, with the pre-existing priority request
unchanged. P1-09F did not modify Recovery observation/action handling,
thresholds, solver, gains, dynamics, or configuration behavior.

## Mechanical validation

| Command | Result |
|---|---|
| `source /opt/ros/humble/setup.bash && colcon build --packages-select rl_quadruped_controller --cmake-args -DBUILD_TESTING=ON` | **PASS** — controller shared library and lifecycle executable rebuilt. |
| `./build/rl_quadruped_controller/p1_09f_stoppable_thread` | **PASS** — unstarted destruction, started-thread stop/join, repeated stop, restart, and stopped-thread destruction. The test has a 10 s periodic wait and requires stop/join under 500 ms, proving stop wakes the wait rather than waiting for its timeout. |
| `python3 scripts/test_abs_rt_frame.py` | **PASS** — 24 tests. |
| `python3 scripts/test_abs_live_hud.py` | **PASS** — 17 tests. |
| `python3 scripts/test_formal_runtime_adapter.py` | **PASS** — 16 tests. |
| `python3 scripts/test_formal_experiment_contract.py` | **PASS** — 22 tests. |
| `git diff --check` | **PASS**. |

The compiled helper is the production owner used by `StateRLRec`; its test does
not instantiate ROS2 or Torch. It verifies the worker lifecycle contract, not
the full controller-manager unload path.

## Remaining runtime evidence

`UNKNOWN`: this offline build/test evidence cannot prove that a controller
plugin unload in a live ROS2 + MuJoCo process now exits without SIGABRT. A later,
separately authorized, bounded simulation-only shutdown test must confirm an
exit code of zero and no `terminate`/SIGABRT. Until that capture exists, this
fix must not be used to classify a recorder run as stably ended or as a formal
`VALID` run.

P1-09 remains **EXECUTING**. This is not P1-09 Acceptance or Phase 1 evidence.
