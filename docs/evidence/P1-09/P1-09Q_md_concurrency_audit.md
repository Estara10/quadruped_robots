# P1-09Q — MuJoCo m/d 并发访问与模型交接定向审计

## Scope and status

This is a read-only static audit. No MuJoCo, ROS2, benchmark, formal run,
reload runtime test, or real robot was run.

The supplied P1-09P Reviewer disposition is recorded as **ACCEPT WITH KNOWN
ISSUES**. It accepts the fail-closed reload barrier as an offline lifecycle
increment, while retaining High UNKNOWN/risks for m/d concurrency and DDS
lifetime. P1-09 remains **EXECUTING / NOT ACCEPTED**.

## Direct accessors

| Accessor | Thread | m/d operation | Current synchronization | Finding |
|---|---|---|---|---|
| `PhysicsThread` initial load | physics `std::thread` | creates `m/d`, calls `mj_forward` | no shared lock during initial load | **CONFIRMED**: before bridge can observe `d`, but global publication is not an atomic ownership protocol |
| `PhysicsLoop` stepping | physics `std::thread` | writes `d` through `mj_step`, control/noise/elastic-band updates; reads `m`/`d` | `sim.mtx` around the simulation block | **CONFIRMED**: protected against render-side `Sync` operations that use the same mutex; not protected against RobotBridge |
| `PhysicsLoop` paused path | physics `std::thread` | `mj_forward(m,d)` | `sim.mtx` | **CONFIRMED**: not protected against RobotBridge |
| `PhysicsLoop` reload | physics `std::thread` | creates candidate, deletes/replaces global `m/d` | P1-09P `reloadAllowed()` barrier | **CONFIRMED**: active bridge reload is rejected; inactive/released path is only safe if no m/d worker can be started again |
| `RobotBridge::run` | project-owned bridge worker | reads `sensordata`, `xpos/xmat/contact`; writes `ctrl`; ray/collision reads | `lowcmd->mutex_` only protects lowcmd message, not m/d | **CONFIRMED data-race risk** with PhysicsLoop; no common m/d mutex is present |
| `RenderLoop::Sync` and UI callbacks | main/render thread | reads/writes `sim.m_/d_`, calls `mj_forward`, reset, history/key operations | `sim.mtx` for `Sync` and event polling path | **CONFIRMED**: uses the render/physics mutex; `Render()` itself primarily renders `scn`, while `Sync` prepares it |
| `RenderLoop::LoadOnRenderThread` | main/render thread | installs `sim.m_/d_`, copies model/data metadata and creates scene | render `sim.mtx` via the request path | **CONFIRMED**: render-side model handoff is synchronized with `Simulate::Load`; it does not synchronize the bridge's cached pointers |
| Unitree DDS/ChannelFactory | SDK-owned/unknown threads | DDS message transport updates LowCmd/LowState objects; no direct m/d access in inspected project code | SDK internals not source-available here | **UNKNOWN** whether SDK-owned threads retain or access m/d |

## Concurrency conclusions

The bridge callback and PhysicsLoop access the same `mjData` concurrently.
`RobotBridge::run()` writes `mj_data_->ctrl[]` and reads sensor/contact arrays;
PhysicsLoop writes the same `mjData` during `mj_step`, `mj_forward`, noise and
elastic-band updates. The bridge does not acquire `sim.mtx`, and MuJoCo does
not make ordinary C++ unsynchronized field access race-free. This is therefore
a **CONFIRMED data-race risk**, not merely an UNKNOWN based on missing crashes.

The bridge also caches `mjModel*` and `mjData*` in
`UnitreeSDK2BridgeBase` (`unitree_sdk2_bridge.h:51-52, 509-510`). P1-09P blocks
replacement while the bridge lifecycle is reserved, which closes the specific
use-after-free replacement window. It does not by itself make ordinary
same-object reads/writes data-race-free.

## Reload candidate leak boundary

Both reload branches in `main.cc` have the form `LoadModel` → `mj_makeData` →
conditional replacement:

- `droploadrequest` around `main.cc:308-338`;
- `uiloadrequest` around `main.cc:348-376`.

If `mnew` succeeds but `mj_makeData(mnew)` returns null, the current `else`
path calls `LoadMessageClear()` without `mj_deleteModel(mnew)`. The exact leak
is the candidate `mjModel` allocation and its owned model storage for that
failed reload attempt. This is a **CONFIRMED leak boundary**. It is separate
from the P1-09P active-bridge rejection path, where `dnew` and `mnew` are both
explicitly deleted.

## Access classification

### May be concurrent

- DDS message transport and project bridge message locking may proceed
  concurrently with unrelated simulator work, subject to SDK guarantees that
  are not available here.
- Render GPU submission can occur after `Sync` has prepared the scene; direct
  m/d access in the inspected `Render()` body is not the primary operation.

### Must be mutually exclusive

- Any bridge read/write of `mjData` must not overlap PhysicsLoop's `mj_step`,
  `mj_forward`, reset, control/noise, or elastic-band writes.
- Any m/d replacement or deletion must be mutually exclusive with every m/d
  accessor, including bridge, physics and render handoff.
- Model-derived metadata and render scene rebuild must remain synchronized with
  the model pointer used by the render thread.

## DDS boundary

The inspected `main.cc` only calls
`unitree::robot::ChannelFactory::Instance()->Init(...)`; the direct bridge code
constructs SDK LowCmd/LowState publishers/subscriber objects and its own worker
invokes `run()`. No inspected project-side DDS callback receives an m/d pointer.
That is insufficient to prove SDK-owned DDS threads do not retain or access
m/d. Conclusion: **UNKNOWN**, not a claim of safety and not a blocker to the
already-proven project-worker join boundary.

## Minimal repair design (not implemented)

1. Keep DEC-009's fail-closed reload refusal and make the barrier state explicit
   as a non-rebind lifecycle state. Never stop/restart the bridge implicitly for
   a reload.
2. Introduce one authoritative m/d access boundary. Preferred design is a
   physics-owned m/d with bridge command/state snapshots: bridge copies command
   input into a synchronized snapshot, PhysicsLoop applies it and publishes a
   synchronized sensor snapshot; the bridge no longer dereferences m/d.
3. If a direct bridge access is retained temporarily, use one common mutex for
   every bridge, PhysicsLoop and render/UI m/d operation, with a documented
   lock order and short critical sections. Do not hold that mutex while doing
   DDS publication or other potentially blocking work.
4. Add a failed-`mj_makeData` cleanup helper or equivalent ownership guard so a
   successful `mnew` is always deleted unless ownership is transferred.

Snapshot ownership has better real-time and deadlock properties but changes the
bridge data path and requires freshness/latency tests. A common mutex is a
smaller interim change but can increase control/render contention and risks
deadlock if DDS or UI callbacks are called while held. Neither design is
implemented in P1-09Q.

## ROADMAP decision

No Roadmap Change Request is needed yet. The findings refine the existing
P1-09 lifecycle work and should be handled as the next scoped lifecycle task.
A Roadmap Change Request becomes appropriate only if the Director chooses to
promote snapshot ownership into a new phase/task or change P1-09 acceptance
boundaries.
