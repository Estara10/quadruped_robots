# P1-09O — Joinable RobotBridge implementation

## Scope

Lifecycle-only implementation. No MuJoCo, ROS2, benchmark, formal run, pilot,
or real-robot execution was performed.

## Evidence chain

P1-09N established from the resolved Unitree SDK 2.0.0 archive that
`RecurrentThread` creates a detached native pthread and its destructor does not
provide a join barrier. Therefore a bridge callback reading `mjModel`/`mjData`
could not be proven complete before those objects were released.

## Old and new model

| Concern | Old | New |
|---|---|---|
| Bridge callback owner | SDK `unitree::common::RecurrentThread` | `RobotBridge::thread_`, a project-owned `JoinableThread` |
| Stop | SDK lifetime/`Wait()` semantics not sufficient as a completion proof | idempotent atomic stop flag plus condition-variable wakeup |
| Completion | detached; no join proof | `stopAndJoin()` joins the moved worker |
| Destruction | base destructor was non-virtual and did not establish derived cleanup | virtual base destructor; derived thread member is destroyed only after explicit stop/join |

`unitree_sdk2_bridge.h` no longer uses `RecurrentThread` for the m/d-accessing
bridge callback. The callback checks the owned stop flag, runs the existing
`RobotBridge::run()` body, and waits with a bounded condition-variable timeout.
The control computations and callback body were not changed.

## Ownership and shutdown order

The final shutdown sequence is:

```text
RUNNING
  -> RenderLoop returns (UI/window/exitrequest/signal)
  -> bridge_stop_request.requestStop()
  -> UnitreeSdk2BridgeThread wakes
  -> RobotBridge::requestStop()
  -> RobotBridge::thread_.stopAndJoin()
  -> bridge thread joins
  -> physics thread joins
  -> main releases ctrlnoise, mjData, mjModel
  -> main returns
```

The physics thread no longer frees the final `m/d` objects or calls `exit(0)`.
This makes main the final owner for final shutdown. The bridge wrapper waits on
a condition variable rather than a sleep-only loop. Signal handlers only set
the MuJoCo signal flag; the render loop converts it to the existing exitrequest
path. UI/window/exitrequest and signal therefore converge at main's bridge stop
request.

## Mechanical validation

Added `joinable_thread.h` and `test/p1_09o_joinable_thread.cpp`. The test covers
unstarted destruction, start, duplicate start rejection, repeated stop, join,
restart, and stopped destruction. It passed. The simulator target also built.

Static checks found no `RecurrentThread`, `pthread_exit()`, `detach`, or normal
path `exit(0)` in the modified bridge/main/helper sources.

## Status and limits

- Build and offline lifecycle test: PASS.
- `git diff --check`: PASS.
- Runtime clean shutdown: UNKNOWN; explicitly not run by task scope.
- Existing MuJoCo model reload path and all external SDK/DDS lifetime behavior:
  UNKNOWN and outside this minimal change.
- P1-09 and Phase 1 remain NOT ACCEPTED. Independent Reviewer review is required.
