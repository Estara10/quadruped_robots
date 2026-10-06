# P1-09P — Fail-closed MuJoCo model reload barrier

## Scope and status

This is a lifecycle-only correction. No MuJoCo, ROS2, benchmark, formal run,
pilot, or real-robot execution was performed. P1-09 and Phase 1 remain
unaccepted.

## Old → new reload behavior

Previously, `PhysicsLoop` loaded `mnew/dnew` and deleted the global `m/d`
without consulting the active RobotBridge. The bridge worker could therefore
continue reading cached pointers after deletion.

Now `BridgeLifecycle::reloadAllowed()` is checked before every old `m/d`
delete/replace path (`droploadrequest` and `uiloadrequest`). The main thread
reserves the bridge lifecycle before starting worker threads. While that
reservation is active, reload is rejected, the newly loaded candidate is
deleted, the existing `m/d` are retained, and a diagnostic reason is emitted.
There is intentionally no automatic rebind/restart protocol.

## Lifecycle and concurrency boundary

```text
BRIDGE_RESERVED/ACTIVE
  -> reloadAllowed() == false
  -> reject candidate reload; retain current m/d

shutdown request
  -> bridge worker stop + join
  -> bridge lifecycle inactive
  -> physics join
  -> final m/d release
```

`JoinableThread::start()` refuses to start while either the old native worker is
joinable or `join_in_progress_` is true. `stopAndJoin()` moves the worker only
under the lifecycle mutex, joins outside it, then clears the in-progress flag;
concurrent starts cannot enter during the join window. It does not detach.

## DDS / external-thread boundary

The direct RobotBridge callback is now project-owned and joined before final
m/d cleanup. The Unitree ChannelFactory/DDS implementation may own additional
threads, but no local source or SDK binary evidence proves that those threads
retain or access `mjModel`/`mjData`. This remains UNKNOWN; the barrier does not
claim to shut down unrelated DDS internals.

## Validation

- `p1_09p_reload_barrier_test`: PASS. Covers inactive allowance, active
  rejection, stop request, and inactive-after-stop state.
- `p1_09o_joinable_thread_test`: PASS.
- `unitree_mujoco` simulator target build: PASS.
- Static inspection: both `droploadrequest` and `uiloadrequest` replacement
  paths check `reloadAllowed()` before deleting old m/d; no detach or exit-based
  normal shutdown introduced.
- `rtk git diff --check`: PASS.

## Remaining limitations

- The active reservation is deliberately conservative: no model reload is
  allowed while the bridge lifecycle exists.
- Thread safety of the pre-existing global `m/d` fields and MuJoCo render/data
  synchronization outside these replacement sites remains UNKNOWN.
- External DDS thread ownership remains UNKNOWN.
- A bounded runtime proof of reload refusal and clean shutdown is still needed;
  this task did not run it.
