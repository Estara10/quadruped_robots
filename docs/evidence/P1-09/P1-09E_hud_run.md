# P1-09E — Single Bounded MuJoCo HUD Authenticity Verification

Date: 2026-08-28
Scope: one Director-authorized, strictly-bounded simulation-only run
(MuJoCo + `StateRL` + read-only HUD). **This is not a benchmark, not a formal
experiment, not a Phase 1 Gate run, and not an Acceptance claim.** P1-09 remains
**EXECUTING**.

## 1. Actual launch commands and simulation-only evidence

Replicated `scripts/launch_abs_sim.sh` simulation-only commands exactly (no
real-robot path):

```
cd /home/lidio/quadruped_robots/unitree_mujoco
./simulate/build2/unitree_mujoco -s scene_flat.xml          # safe flat-ground scene

cd /home/lidio/quadruped_robots/quadruped_ros2_control_humble
ros2 launch rl_quadruped_controller mujoco.launch.py simulation_test:=0
```

- Scene `scene_flat.xml` (flat ground), the **only** scene used; no batch scenes,
  no variant comparison.
- `unitree_mujoco/simulate/config.yaml`: `robot: go2`, `robot_scene:
  scene_flat.xml`, `domain_id: 1`, **`interface: "lo"`** (loopback = simulation
  mode; the real-robot interface `enp7s0` is never used).
- `mujoco.launch.py` is the simulation launch (spawns `ros2_control_node` +
  spawners + rviz); `real_go2.launch.py` was never invoked. `simulation_test:=0`
  (fault injection disabled).
- RL auto-entry used the controller's own `/control_input` with the same
  `command=2,2,3` sequence as `launch_abs_sim.sh`, published with
  `--qos-reliability reliable` (deterministic delivery) and confirmed by the
  controller's own `Switched from ... to ...` log lines:
  `passive → fixed down → fixed stand → rl`.

## 2. Raw evidence (10 consecutive LIVE frames)

Read-only sampler `scripts/abs_live_hud_sampler.py` (calls only
`abs_rt_frame.read_shm_frame`/`classify_frame`; never writes the frame, no formal
artifact) over `/dev/shm/mujoco_rt_frame` at 50 Hz. Raw log:
`docs/evidence/P1-09/P1-09E_capture_raw.jsonl` (877 records). First 10 distinct
LIVE frames (`source=1` = `AUTHORITATIVE_RUNTIME`, `policy_state=0` = AGILE,
`session_id=12087183097814`):

| t_monotonic_ns | status | source | session_id | sequence | monotonic_ns | rl_step |
|---|---|---|---|---|---|---|
| 12087497358997 | LIVE | 1 | 12087183097814 | 30 | 12087483527963 | 14 |
| 12087517555222 | LIVE | 1 | 12087183097814 | 32 | 12087503046986 | 15 |
| 12087538202807 | LIVE | 1 | 12087183097814 | 34 | 12087525851418 | 16 |
| 12087558591758 | LIVE | 1 | 12087183097814 | 36 | 12087543354545 | 17 |
| 12087578977459 | LIVE | 1 | 12087183097814 | 38 | 12087563318826 | 18 |
| 12087599964332 | LIVE | 1 | 12087183097814 | 40 | 12087586253117 | 19 |
| 12087620446080 | LIVE | 1 | 12087183097814 | 42 | 12087602997647 | 20 |
| 12087640678747 | LIVE | 1 | 12087183097814 | 44 | 12087623770591 | 21 |
| 12087661180175 | LIVE | 1 | 12087183097814 | 46 | 12087645246902 | 22 |
| 12087681459728 | LIVE | 1 | 12087183097814 | 48 | 12087663832798 | 23 |

Sampler summary (`P1-09E_sampler_summary.txt`): **RESULT=OK** — 877 reads,
**301 LIVE**, **287 distinct LIVE frames**, session consistent (one session_id),
**sequence strictly increasing**, **monotonic_ns strictly increasing**,
**all source == AUTHORITATIVE_RUNTIME**, all policy states valid. RL LIVE
observation window ≈ **6.2 s** (first to last LIVE read), within the ≤ 15 s
budget.

## 3. HUD LIVE evidence

Human terminal HUD (`abs_live_hud.py --iters 3`) rendered during the window
(`P1-09E_hud_live.txt`):

```
[ LIVE ] authoritative runtime frame
  session_id           = 12087183097814     # same session as the sampler
  rl_step              = 116 → 141          # advancing
  frame age            = 10.7 ms            # fresh
  policy_state         = AGILE
  ra_value             = -0.8266
  lin_vel (actual)     =  2.0718  0.2948  0.1139
  command (target)     =  4.9881 -0.5542 -0.2709
  ray_valid            = 1
  ray_age_ns           = 758383
  ray2d[11]            = 2.5850 × 11        # 11 rays present
```

The controller log confirms the authoritative producer:
`[StateRL]: [RtFrame] Shared memory initialized: /mujoco_rt_frame
session_id=12087183097814` (matches the frame), plus `[RL] Ray2d: shm connected,
policy active`.

## 4. Post-exit invalidation evidence

The controller was exited from RL by `command=1` (emergency stop), which the
controller handles through its **explicit exit path** — `[HARD-STOP] command=1 ->
forcing PASSIVE` → `current_state_->exit()` = `StateRL::exit()` →
`invalidateRtFrame()` (magic=0/version=0/stable even sequence). The read-only HUD
then:

- Sampler: **LIVE → INVALID** transition, gap **20.32 ms = one refresh cycle**
  (last LIVE seq=648 at t=12093678506851; first INVALID at t=12093698827039),
  followed by **576 consecutive INVALID** reads — no residual LIVE, no STALE
  window, no last-frame values retained.
- Human HUD after exit (`P1-09E_hud_after.txt`):
  `[ INVALID ] no live simulation data` … `previous frame values are NOT shown
  (no residual data).`

The captured values come **directly from `/dev/shm/mujoco_rt_frame`** — nothing
was hand-written, zero-filled, replayed, substituted, or fabricated. A prior
deliberate smoke check with no controller running correctly reported `FAIL`
(zero LIVE frames) rather than a fabricated PASS.

## 5. Process start / stop / errors

- Started and stopped cleanly: `unitree_mujoco` (pid 78267), `ros2 launch
  rl_quadruped_controller` (pid 78335), controller manager, spawners, rviz, the
  sampler, and the human HUD. `ORCH_EXIT=0`. No leftover processes after
  teardown.
- **Observed teardown anomaly (pre-existing, not from this task's changes):** at
  controller-manager shutdown the process aborted with
  `terminate called without an active exception` (SIGABRT, exit code −6) inside
  `controller_manager::~ControllerManager()` → controller plugin deletion →
  `StateRLRec::~StateRLRec()`. `StateRLRec` declares `std::thread rl_thread_` and
  has no destructor/`join()` in `StateRLRec.cpp` — a pre-existing teardown bug in
  the manual-recovery FSM state. It occurred **after** all P1-09E evidence was
  captured (at cleanup kill), is in code this task did not modify, and does not
  affect the frame evidence. Flagged for Reviewer awareness.

## 6. Explicit statements

- This was **not a benchmark**, **not a formal experiment**, **not a pilot**,
  and **not a Phase 1 Gate** run. No success, arrival, collision-free,
  performance, or success-rate metric was computed or reported.
- No HUD output was wired to the formal recorder; **no formal VALID run** was
  produced.
- This does **not** constitute P1-09 or Phase 1 Acceptance; P1-09 remains
  **EXECUTING** and awaits the next independent Reviewer re-review.
- Remaining `UNKNOWN` (unchanged): `collision` (bridge-side only, no producer in
  the frame); `torque_saturated` (no per-joint saturation computed); measured
  cadence and active ray-source mode (frame carries `monotonic_ns`/`rl_step` so
  cadence is measurable, but no measured period or effective `MUJOCO_RAY_SOURCE`
  is claimed here); formal-recorder binding of the frame to the P1-02
  `FormalRunWriter` (future, separately-authorized).
- The runtime processes were started and stopped under this bounded run; no
  real-robot path was entered (`interface: "lo"`, `mujoco.launch.py` only).

## 7. Verification commands (all offline, re-run after the run)

| Command | Result |
|---|---|
| `rtk python3 scripts/test_abs_rt_frame.py` | **24/24 PASS** |
| `rtk python3 scripts/test_abs_live_hud.py` | **17/17 PASS** |
| `rtk python3 scripts/test_formal_runtime_adapter.py` | **16/16 PASS** |
| `rtk python3 scripts/test_formal_experiment_contract.py` | **22/22 PASS** |
| `rtk git diff --check` | **clean** |

P1-09 status: **EXECUTING**. Not ACCEPTED.
