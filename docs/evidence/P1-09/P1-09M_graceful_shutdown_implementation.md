# P1-09M — MuJoCo Unified Controlled-Exit Lifecycle Implementation

Date: 2026-08-28  
Status: **BLOCKED — implementation not started**

## Blocking decision

The task's implementation precondition was not satisfied at the start of P1-09M.
P1-09N subsequently resolved the installed SDK archive and its binary
implementation, but the resulting contract still does not provide the required
safe completion proof. `RecurrentThread` declares a destructor and `Wait()` in
`rl_sar/src/rl_sar/library/thirdparty/robot_sdk/unitree/unitree_sdk2/include/unitree/common/thread/recurrent_thread.hpp:52-60`,
while `Thread` declares its destructor in `thread.hpp:35` and exposes no stop or
join operation in `thread.hpp:35-54`. The now-resolved binary implementation
creates detached pthreads and cancels without joining; therefore it remains
impossible to prove that the `RecurrentThread` created by `RobotBridge` stops
and is reclaimed before `mjModel`/`mjData` are freed. See the superseding
P1-09N contract report.

Per the P1-09M hard constraint, no implementation, test target, build, or
runtime was started. This is not a partial code fix and must not be reported as
clean-shutdown evidence.

No P1-09L Reviewer conclusion file was present in the local
`docs/evidence/P1-09/` directory; the design-review prerequisite is therefore
also **UNKNOWN**, not treated as approval.

## Evidence chain

| Claim | Evidence | Classification |
|---|---|---|
| MuJoCo creates the outer bridge worker | `unitree_mujoco/simulate/src/main.cc:628` | **CONFIRMED** |
| The bridge constructs a Unitree `RecurrentThread` | `unitree_sdk2_bridge.h:546-556` | **CONFIRMED** |
| `RecurrentThread` starts its native thread in its constructor | `recurrent_thread.hpp:16-49` calls `Run(...)` | **CONFIRMED** |
| `RecurrentThread` has a quit field but no visible public stop method | `recurrent_thread.hpp:52-60`; `mQuit` is private at 60 | **CONFIRMED** |
| Destructor/Wait implementation stops and joins the native thread | No implementation in checkout; only declarations | **UNKNOWN** |
| `RobotBridge` destructor explicitly stops/joins the recurrent thread | No destructor or stop/join call in `unitree_sdk2_bridge.h:534-657` | **CONFIRMED: absent in audited source** |
| `m`/`d` are freed while the current bridge lifetime is not proven stopped | `main.cc:526-532` frees them in `PhysicsThread`; bridge receives them at `main.cc:546` | **CONFIRMED risk; ordering safety UNKNOWN** |
| Current physics worker exits the process | `main.cc:532`, `exit(0)` | **CONFIRMED** |
| Current main terminates through pthread exit | `main.cc:636`, `pthread_exit(NULL)` | **CONFIRMED** |

## Current ownership and required order

The current source does not establish a safe owner for `m` and `d` after they
are loaded by `PhysicsThread`. The proposed lifecycle design treats main as the
eventual process coordinator, but the following ownership facts remain open:

```text
main constructs Simulate
  ├─ physics std::thread: loads/owns cleanup of m,d and ctrlnoise
  └─ UnitreeSdk2BridgeThread: passes m,d into RobotBridge/RecurrentThread

required before mj_deleteData/mj_deleteModel:
  stop request
    → bridge outer thread returns
    → RobotBridge/RecurrentThread is proven stopped and reclaimed
    → physics thread returns after its cleanup
    → main joins all started threads
    → only then release remaining m/d ownership
    → main returns
```

The exact safe order cannot be implemented until the Unitree thread library
semantics and bridge destructor behavior are available. The earlier design's
bridge-before-physics join order remains a **LIKELY** conservative proposal,
not a proven implementation rule.

## Special exit paths classified

| Location | Path | Classification and disposition |
|---|---|---|
| `unitree_sdk2_bridge.h:65` | `exit(EXIT_FAILURE)` for unsupported joystick | **UNKNOWN risk** for graceful cleanup. It occurs in the bridge worker after the outer worker has been created; do not classify as safe pre-thread failure without proving all m/d users are stopped. |
| `unitree_sdk2_bridge.h:84` | `std::exit(EXIT_FAILURE)` when a requested ray test fault is not explicitly enabled for simulation | **UNKNOWN risk** in current ownership context; it is initialization-time validation but executes inside the bridge worker. Must be reviewed in implementation. |
| `unitree_sdk2_bridge.h:145` | `std::_Exit(EXIT_SUCCESS)` for the explicit `exit` ray fault injection | **CONFIRMED intentional abrupt test path**, not a normal lifecycle path; it must remain excluded from graceful-shutdown claims and real-robot mode. |
| `main.cc:532` | Physics worker `exit(0)` | **CONFIRMED normal-path defect**; must be replaced by return only after cleanup ownership is proven. |
| `main.cc:590` | `std::exit(1)` after macOS Rosetta diagnostic | **LIKELY startup failure before simulation threads**; not part of the normal running lifecycle, but implementation should preserve it only if it is before thread creation. |
| `main.cc:636` | `pthread_exit(NULL)` | **CONFIRMED normal-path defect**; must be replaced by ordered joins and main return. |

## Implementation decision

**No files were modified.** In particular, no signal handler, stop flag,
condition variable, bridge loop, physics return path, join order, or test was
added. Implementing before the missing `RecurrentThread` contract is resolved
would risk freeing `m/d` while an SDK callback still accesses them.

## Unblock requirements

Before implementation resumes, obtain one of:

1. the exact Unitree SDK source for `Thread::~Thread`,
   `RecurrentThread::~RecurrentThread`, `Wait()`, `CreateThreadNative`, and
   any native join/stop behavior; or
2. a versioned binary/source contract plus a standalone lifecycle probe that
   proves destruction stops and joins the callback thread before return.

Then an independent Reviewer must approve the P1-09L design and specifically
verify bridge destructor behavior, m/d ownership, and the final join order.

## Planned validation after unblock (not executed)

- Compile-only lifecycle tests for idempotent `RequestStop`, all request-source
  convergence, bridge/physics/render stop propagation, join order, and no
  detach.
- Static checks that normal paths contain no `exit(0)`, `pthread_exit()`, or
  detach, and signal handlers contain only `sig_atomic_t` writes.
- Build the simulator target.
- Only after those checks, one separately authorized simulation-only run per
  exit source, with no TERM/KILL escalation accepted as graceful evidence.

P1-09 and Phase 1 remain **NOT ACCEPTED**.
