# P1-09L — MuJoCo Graceful-Shutdown Lifecycle Design

Date: 2026-08-28  
Status: **DESIGN ONLY — implementation is pending independent design review.**

## Scope and evidence boundary

This is a source-based lifecycle design. No MuJoCo, ROS2, controller, benchmark,
or real-robot process was started, and no runtime source/configuration was
modified.

The requested `unitree_mujoco/simulate/src/simulate.cc` path does not exist in
this checkout. The directly used render implementation is
`unitree_mujoco/simulate/src/mujoco/simulate.cc`; this report uses that resolved
path.

P1-09K runtime evidence confirms that the tested MuJoCo child had
`SigIgn=0x6`, including ignored SIGINT, then required SIGTERM cleanup and
exited `143`. That is a failed clean-shutdown result, not an implementation
claim for the design below.

## Current lifecycle — evidence table

| Execution context | Creation / owner | Loop or blocking condition | Current exit path | Join / detach state | Classification |
|---|---|---|---|---|---|
| Main / render | `main.cc:621-637`; main calls `RenderLoop` at 633 | `RenderLoop` runs while window is open and `exitrequest` is false (`mujoco/simulate.cc:2791`) | After render return, main joins physics only then calls `pthread_exit(NULL)` (`main.cc:634-636`) | Does not join bridge thread; no detach shown | **CONFIRMED** |
| Physics worker | `std::thread(PhysicsThread, ...)` at `main.cc:630-631` | `PhysicsLoop` continues while `!sim.exitrequest.load()` (`main.cc:288-495`) | `PhysicsThread` frees model/data then calls `exit(0)` (`main.cc:500-533`, especially 532) | Main attempts `physics_thread.join()` at 634, but worker `exit(0)` can end the whole process before that ownership path completes | **CONFIRMED** |
| Unitree SDK bridge worker | `std::thread(UnitreeSdk2BridgeThread, ...)` at `main.cc:628` | First waits forever for `d`; after bridge construction it sleeps forever (`main.cc:535-568`) | No stop predicate or return condition | No join and no detach shown | **CONFIRMED** |
| GLFW/UI internals | GLFW/UI implementation | Window/event-loop behavior is external to this source | UI Quit writes `sim->exitrequest.store(1)` (`mujoco/simulate.cc:1466-1468`); GLFW close makes `ShouldCloseWindow()` true (`mujoco/glfw_adapter.cc:172-173`) | Internal thread ownership is not established by the audited code | **UNKNOWN** |

Additional confirmed facts:

- `Simulate::exitrequest` is an atomic state at `mujoco/simulate.h:218-222`.
- The render loop stores `exitrequest=2` after it exits (`mujoco/simulate.cc:2883`).
- The `unitree_mujoco` executable is built from the lifecycle sources by
  `unitree_mujoco/simulate/CMakeLists.txt:17-35`; no signal-handling source is
  listed there.
- The existing shell launcher sends its background jobs the default signal via
  `kill %1 %2` (`scripts/launch_abs_sim.sh:35-42`); it is not an application
  shutdown protocol.

## Proposed minimal lifecycle contract

The intended process contract is:

```text
RUNNING
  └─ RequestStop(reason) ─> STOP_REQUESTED
                                └─ all owned workers return and main joins them
                                   ─> THREADS_JOINED
                                          └─ main releases remaining resources and returns 0
                                             ─> PROCESS_EXITED
```

`RequestStop(reason)` is idempotent: the first reason is retained for evidence,
and later requests only reaffirm stopping. It must be the only route that
starts process teardown. No source is allowed to call `exit()`, `pthread_exit()`,
or detach a worker as a substitute for this sequence.

### Unified request sources

| Source | Proposed handling | Classification |
|---|---|---|
| Render-loop window close | On detecting GLFW close, call the normal request path before returning from the render loop. | **PROPOSED** |
| Existing UI Quit / `exitrequest` | Replace direct policy of "Quit means local atomic store only" with a named normal request method that records the reason and sets the shared stop request. | **PROPOSED** |
| SIGINT / SIGTERM | Install `sigaction` handlers early in `main`. Handler does only `volatile sig_atomic_t` flag assignment; render/main normal code polls that flag and invokes `RequestStop`. Do not call logging, mutexes, C++ atomics, MuJoCo, GLFW, or `exit()` in the handler. | **PROPOSED** |
| Internal fatal initialization failure | Request stop, join every started worker, then return a nonzero code from main. | **PROPOSED** |

Installing a `sigaction` handler is required to replace the observed ignored
SIGINT disposition. Whether the ignored disposition originated from the parent
shell is still **LIKELY**, not confirmed; the required behavior is independent
of that origin.

### Worker stop and ordered reclamation

1. `RequestStop` marks an explicit coordinator stop flag and makes
   `sim.exitrequest` true so existing physics/render conditions observe it.
2. The bridge worker checks the same stop request in both its pre-`d` wait and
   its post-construction loop. It returns rather than sleeping indefinitely.
3. `PhysicsLoop` already observes `exitrequest`; `PhysicsThread` must return
   normally after its local cleanup, not call `exit(0)`.
4. After `RenderLoop` returns, `main` invokes/affirms `RequestStop`, joins the
   bridge worker **before** freeing MuJoCo model/data it may observe, then joins
   the physics worker. Main then returns normally; it must not call
   `pthread_exit()`.

The bridge-before-physics ordering is the conservative proposal because the
bridge construction receives `m`/`d`; its exact post-construction ownership and
destructor behavior were not established in this audit. Any implementation must
verify that no bridge callback can access freed MuJoCo objects. This ordering is
therefore **LIKELY** safe, not confirmed.

## Minimal file-level implementation boundary (not implemented)

| File | Minimal intended change | Risk / proof required |
|---|---|---|
| `unitree_mujoco/simulate/src/main.cc` | Add a small shutdown coordinator; install async-signal-safe handlers; make bridge loops stoppable; remove worker `exit(0)` and main `pthread_exit`; join every created `std::thread`. | Signal safety, join ordering, and no use-after-free of `m`/`d` must be reviewed. |
| `unitree_mujoco/simulate/src/mujoco/simulate.h` | Expose only the minimal named normal exit-request API/state needed by render/UI and main. | Must preserve current UI behavior and not introduce controller semantics. |
| `unitree_mujoco/simulate/src/mujoco/simulate.cc` | Route UI Quit/window-close observation through that normal request API; poll pending signal request from normal render-loop code. | Render loop must return promptly without calling shutdown logic from a signal handler. |
| `unitree_mujoco/simulate/CMakeLists.txt` | Add a lifecycle-only unit test target only if existing test layout supports it. | No required runtime/configuration behavior change. |

No controller, policy, threshold, gain, solver, dynamics, scene, ROS launch, or
hardware file is in scope.

## Validation plan for a future implementation

### Mechanical / build checks

- Compile `unitree_mujoco` and any isolated shutdown-coordinator test.
- Test idempotent first-reason stop request, stop-before-bridge-initialization,
  stop-after-bridge-initialization, and join of only actually-started workers.
- Source/compile check that `PhysicsThread` has no `exit(0)`, main has no
  `pthread_exit`, and every started `std::thread` is joined on every normal
  return path.

### Strict bounded runtime checks (separate authorization required)

- MuJoCo-only flat-scene run: record PID/PGID, `/proc/<pid>/status`, signal
  timestamp, stop reason, per-thread stop/join events, final wait result, and
  raw logs.
- One SIGINT case and one SIGTERM case: each must exit without TERM/KILL
  escalation and with the declared clean main return (`rc=0`).
- One actual UI Quit/window-close case only after an identified window target:
  prove both render-loop return and whole-child exit (`rc=0`).
- Controller/plugin shutdown validation remains a separate scoped run; do not
  conflate it with an algorithm, formal episode, benchmark, or performance run.

Any timeout, nonzero key process exit, `terminate`, SIGABRT, detached orphan,
or TERM/KILL escalation is a failure, not clean shutdown.

## Out of scope

- StateRL/StateRLRec policy behavior, recovery solver, switching, thresholds,
  gains, control frequency, model I/O, or dynamics.
- ROS2 runtime changes, real-robot operation, benchmarks, formal runs, and
  perception work.
- Claiming that P1-09, Phase 1, UI-close behavior, or the existing child
  shutdown path is accepted.
- Determining Unitree SDK bridge callback/destructor guarantees without a
  separately justified direct dependency read.

## Open design risks

| Question | Status | Required closure before implementation acceptance |
|---|---|---|
| Does `UnitreeSDK2BridgeBase` own callbacks/threads that touch `m` or `d` after its loop returns? | **UNKNOWN** | Directly inspect the bridge dependency or instrument its destruction in a lifecycle-only test. |
| Can GLFW polling be delayed indefinitely by a render/driver call after a signal flag is set? | **UNKNOWN** | Bounded runtime test with logs around signal poll and render-loop exit. |
| Exact provenance of the ignored SIGINT disposition | **LIKELY** inherited from asynchronous launch | Not required for the repair; handler installation and runtime disposition check are required. |
| Is join order bridge then physics free of all shared-data races? | **LIKELY** conservative | Review ownership and verify a shutdown run under sanitizers if compatible. |

## Reviewer recommendation

**Yes — require an independent Reviewer to approve this lifecycle design before
implementation.** The repair changes signal disposition, cross-thread ownership,
and MuJoCo model/data lifetime. Reviewer approval must specifically cover
async-signal safety, idempotent request semantics, bridge/model ownership,
join ordering, failure paths, and the requirement that TERM/KILL never be
reported as graceful shutdown.

P1-09 and Phase 1 remain **NOT ACCEPTED**.
