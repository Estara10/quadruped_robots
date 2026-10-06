# P1-09J — MuJoCo SIGINT / Exit-Path Static Audit

Date: 2026-08-28  
Status: **STATIC AUDIT COMPLETE — no runtime was started.**

## Scope

This audit read only the files directly on the P1-09I shutdown path. It did not
run MuJoCo, ROS2, a benchmark, a formal episode, or a real robot, and it did
not modify source, configuration, or launch scripts.

Files inspected:

- `unitree_mujoco/simulate/src/main.cc`
- `unitree_mujoco/simulate/src/mujoco/simulate.cc`
- `unitree_mujoco/simulate/src/mujoco/simulate.h`
- `unitree_mujoco/simulate/src/mujoco/glfw_adapter.cc`
- `unitree_mujoco/simulate/src/mujoco/platform_ui_adapter.h`
- `unitree_mujoco/simulate/CMakeLists.txt`
- `scripts/launch_abs_sim.sh`
- `quadruped_ros2_control_humble/controllers/rl_quadruped_controller/launch/mujoco.launch.py`
- P1-09G/P1-09I reports and P1-09I raw logs

## 1. What the P1-09I signal evidence proves

P1-09I recorded MuJoCo as PID `116590`, PGID `116590`, SID `116590`, launched
with `setsid`. The orchestrator then recorded:

```text
11:08:59.729Z SIGNAL label=mujoco signal=SIGINT target_pid=116590 target_pgid=116590
11:09:04.972Z SIGINT_RESULT label=mujoco result=TIMEOUT wait_s=5
11:09:04.978Z SIGNAL label=mujoco signal=SIGTERM reason=SIGINT_TIMEOUT target_pid=116590 target_pgid=116590
11:09:05.099Z PROCESS_EXIT label=mujoco pid=116590 rc=143
```

The `kill(2)` call returned no recorded error and targeted the process group
whose PGID equalled the MuJoCo child PID. Therefore:

| Question | Classification | Evidence / limit |
|---|---|---|
| Was a SIGINT delivery attempt made to the MuJoCo process group? | **CONFIRMED** | P1-09I orchestrator signal record and exact PGID |
| Did the MuJoCo application catch SIGINT and set an internal stop flag? | **UNKNOWN** | No application signal log or runtime signal-state capture exists |
| Did the process act on SIGINT within 5 seconds? | **CONFIRMED: NO** | The child remained waitable until SIGTERM; final `rc=143` followed SIGTERM |
| Was SIGTERM the cause of the final termination? | **CONFIRMED** | SIGTERM was sent after the timeout and wait returned `143` |

Thus “SIGINT was sent to the correct target” must not be rewritten as “the
application handled SIGINT”.

## 2. Signal-handler audit

No `signal`, `sigaction`, `signalfd`, `pthread_sigmask`, or equivalent handler
appears in the inspected MuJoCo source. The entry path in
`main.cc:582-637` constructs the simulator, starts the bridge and physics
threads, runs `RenderLoop()`, and then joins only the physics thread. The CMake
target is the ordinary `unitree_mujoco` executable at
`simulate/CMakeLists.txt:17-35`; no signal wrapper or shutdown library is
added by the target.

If SIGINT has the default disposition, the operating system's default action
would terminate the process (normally represented as shell status 130). The
source itself does not convert SIGINT into `Simulate::exitrequest`. The fact
that P1-09I did not terminate after SIGINT therefore points to runtime signal
disposition or process-launch context, not to a source-level SIGINT handler.

The P1-09I runner launched the binary asynchronously from a shell with
`setsid ... &`. It is **LIKELY**, but not statically proven here, that SIGINT
was inherited as ignored by an asynchronous shell child. A future runtime-only
check of `/proc/<pid>/status` or `sigaction` state is required to confirm this
specific explanation.

## 3. Confirmed process-lifecycle defects

### 3.1 Permanent Unitree bridge thread

`main.cc:628` creates `std::thread unitree_thread` running
`UnitreeSdk2BridgeThread`. Its implementation at `main.cc:535-568` has two
unbounded loops:

- `main.cc:538-546`: waits for `d` with `usleep(500000)` and has no stop
  condition;
- `main.cc:564-567`: after bridge construction, executes `while (true)` with
  `sleep(1)` and no stop condition.

There is no `join()` or `detach()` for `unitree_thread` in `main.cc`. This is a
**CONFIRMED** process-lifetime defect. Even if the UI loop exits normally, this
thread has no source-level path to finish.

### 3.2 Main-thread termination is not an orderly join

After `RenderLoop()` returns, `main.cc:634` joins only
`physicsthreadhandle`, then `main.cc:636` calls `pthread_exit(NULL)`.
`unitree_thread` is not joined before that call. This is **CONFIRMED** as an
incomplete shutdown protocol; `pthread_exit` does not provide the missing
bridge-thread stop/join.

### 3.3 Physics thread calls process-wide `exit(0)`

`PhysicsThread` runs `PhysicsLoop` and then frees model state at
`main.cc:527-530`, but calls `exit(0)` at `main.cc:532` from the worker thread.
This bypasses normal return-to-main ownership and thread-join sequencing. It is
a **CONFIRMED** unsafe shutdown design, even though it is not the same symptom
as SIGTERM `rc=143`.

## 4. Render / GLFW / wait-path audit

The render loop is explicit:

- `simulate.cc:2791`: loops while `!ShouldCloseWindow()` and
  `!exitrequest.load()`;
- `simulate.cc:2803`: calls `PollEvents()`;
- `simulate.cc:2862-2863`: renders;
- `simulate.cc:2876-2883`: frees render state and stores `exitrequest=2` on loop
  exit.

The UI “Quit” action at `simulate.cc:1466-1468` sets `exitrequest=1`. GLFW's
`ShouldCloseWindow()` delegates to `glfwWindowShouldClose` at
`glfw_adapter.cc:172-173`. These are existing normal UI-close mechanisms, but
there is no signal bridge into them.

The physics loop checks `exitrequest` at `main.cc:299`. Its ordinary pacing is a
bounded `sleep_for(1ms)` at `main.cc:362-370`, followed by mutex-protected
simulation work. The render loop has no source-visible `glfwWaitEvents` or
unbounded render sleep. Therefore an indefinite GLFW sleep is **not confirmed**
as the primary cause. A mutex holder or an in-flight GLFW/OpenGL call at the
exact signal moment remains **UNKNOWN** without a runtime stack capture.

The most direct confirmed exit blocker is the permanent bridge thread and the
non-orderly worker-thread `exit(0)`/`pthread_exit` design. The likely SIGINT
disposition issue explains why the external SIGINT did not initiate the normal
exit path in P1-09I, but requires runtime confirmation.

## 5. Existing normal shutdown mechanisms

| Mechanism | Exists | Assessment |
|---|---|---|
| MuJoCo UI Quit / close window | Yes: `UiEvent` and `ShouldCloseWindow()` | **CONFIRMED**, but not process-safe while the bridge thread is infinite |
| ROS2 lifecycle/service/topic for MuJoCo process shutdown | Not found in the inspected launch/source path | **UNKNOWN** beyond this scope; `mujoco.launch.py` only starts ROS nodes and spawners |
| Keyboard/UI event | UI event path exists; exact automated key path not used | **CONFIRMED** as an internal UI path, not validated as process-wide clean shutdown |
| Process-group SIGINT | P1-09I targets the correct PGID | **CONFIRMED delivery attempt**, not a clean-shutdown mechanism because the child did not exit |
| Existing shell wrapper | `scripts/launch_abs_sim.sh:35-42` traps signals and `kill`s jobs | **CONFIRMED**, but it uses default `kill` (SIGTERM) and does not prove clean MuJoCo exit; it was not the exact P1-09I command |

`mujoco.launch.py:30-99` starts RViz, robot-state-publisher,
`ros2_control_node`, and spawners. Its `OnProcessExit` handlers sequence
spawners only; it contains no MuJoCo shutdown operation. The P1-09I direct
runner therefore correctly treated MuJoCo as an independently owned process.

## 6. Signal-target assessment

The P1-09I target was structurally correct: `setsid` created a new session and
process group, and the recorded MuJoCo PGID equalled its PID. Sending
`SIGINT` to `-$pgid` was therefore the correct group target for that runner.
The target choice is **CONFIRMED CORRECT**.

This does not prove signal handling. The missing distinction is:

```text
correct PGID target
    ≠ application caught SIGINT
    ≠ application completed orderly shutdown
```

## 7. Root-cause classification

| Finding | Classification |
|---|---|
| No MuJoCo signal handler bridges SIGINT to `exitrequest` | **CONFIRMED** |
| MuJoCo did not exit after P1-09I SIGINT and ended after SIGTERM `rc=143` | **CONFIRMED** |
| Permanent bridge thread has no stop condition and is never joined | **CONFIRMED** |
| Physics worker uses process-wide `exit(0)` | **CONFIRMED** |
| Main uses `pthread_exit` without joining the bridge thread | **CONFIRMED** |
| P1-09I signal target/PGID was wrong | **REJECTED — static evidence says target was correct** |
| SIGINT was inherited as ignored from the asynchronous shell launch | **LIKELY**, runtime signal disposition not captured |
| GLFW/render/mutex blocking was the primary cause | **UNKNOWN**, not supported as primary by source inspection |
| A ROS2 lifecycle mechanism already cleanly shuts down MuJoCo | **UNKNOWN / no such path found in inspected files** |

## 8. Candidate next validation (not executed)

One future Director-authorized validation should isolate exit behaviour without
running ROS2 or a formal experiment:

1. Launch the existing MuJoCo binary and `scene_flat.xml` once in a controlled
   process session, recording PID/PPID/PGID and `/proc/<pid>/status` signal
   dispositions before sending a signal.
2. Send SIGINT to the exact MuJoCo PID/PGID and wait without sending a fallback
   signal for a declared bound; record whether it exits and its raw wait status.
3. Separately test the existing UI Quit/window-close path and record whether the
   bridge thread prevents process termination.
4. Do not send SIGTERM in the first observation window if the purpose is to
   measure natural SIGINT behaviour; if cleanup is required afterward, label it
   forced termination and keep the run FAIL for clean-shutdown acceptance.

This candidate is not executed by P1-09J.

## 9. If a code repair is required

The smallest safe repair boundary is limited to MuJoCo process lifecycle:

- install an explicit, async-signal-safe stop request for SIGINT/SIGTERM and
  bridge it to the existing render/physics stop condition;
- make `UnitreeSdk2BridgeThread` stop-aware in both its pre-`d` wait and its
  post-bridge loop;
- remove worker-thread `exit(0)` and return control to `main`;
- stop and join `unitree_thread` before `main` returns.

Risks are signal-safety mistakes, races with `d`/`m`, and changing the existing
Unitree bridge shutdown ordering. Validation must therefore include a compile
test, repeated UI-close and SIGINT-only simulations, explicit thread/process
exit codes, no abort signatures, and confirmation that the ROS2 bridge remains
functional during normal operation. No such repair was made in P1-09J.

P1-09 and Phase 1 remain unaccepted. This audit is evidence and diagnosis, not
an implementation or runtime validation result.
