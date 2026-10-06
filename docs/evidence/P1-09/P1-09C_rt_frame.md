# P1-09C — Real-Time Data Link + Minimal Read-Only HUD

Date: 2026-08-28
Scope: offline, additive implementation increment of P1-09. No runtime process was run.
Status: P1-09 remains **EXECUTING**; this increment is not an Acceptance claim.

## What was implemented

A single versioned real-time observation frame that the authoritative runtime
producer (the `StateRL` controller) publishes each RL step, plus the fail-closed
Python consumer side (frame classifier + minimal read-only HUD state model).

- `common/abs_rt_frame_contract.h` — the fixed-layout, versioned frame contract
  (`FrameHeader` + `RuntimeFrame`, 424 bytes, magic/version/seqlock, source enum).
- `quadruped_ros2_control_humble/.../FSM/StateRL.h/.cpp` — additive instrumentation:
  opens/owns `/mujoco_rt_frame` shm in `enter()`, publishes a frame at the end of
  `runModel()` and a faulted frame in `safetyVeto()`, cleans up in the destructor.
- `scripts/abs_rt_frame.py` — frame layout mirror, fail-closed `classify_frame`,
  `read_shm_frame` (runtime-only reader), and the `HudState` display model.
- `scripts/test_abs_rt_frame.py` — 22 offline synthetic tests.

No change to control semantics, policy I/O, thresholds, gains, switching, solver,
dynamics, or config. The frame write is observational only: it reads the
already-computed command chain (`policy_actions`, `clamped_actions`,
`output_dof_pos_`, `output_torques`) and copies it to shared memory.

## The frame (single data link)

One frame is published at the end of each successful `runModel()` and carries:

| Group | Fields | Order / note |
|---|---|---|
| header | `magic`, `version`, `sequence`, `monotonic_ns` | seqlock: odd while writing, even = stable |
| identity | `session_id` (monotonicNowNs at `enter()`), `rl_step`, `source` | `source = AUTHORITATIVE_RUNTIME` |
| state | `controller_active`, `rl_entered`, `rl_active`, `safety_faulted`, `policy_state` (AGILE/RECOVERY/FAULTED) | |
| ray | `ray_origin`, `ray_valid`, `ray_age_ns`, `ray2d[11]` | `ray_origin = SHM_RUNTIME` only when ray shm mapped |
| unavailable | `collision_origin`, `torque_saturated_computed` | both 0 today (bridge-side / not computed) |
| scalars | `ra_value`, `lin_vel[3]`, `command[3]`, `world_pose[3]` | body-frame velocity, command, world pose |
| command chain | `action_raw[12]`, `action_clipped[12]`, `joint_target_rad[12]`, `torque_nm[12]`, `torque_saturated[12]` | see order note below |

Joint-order note (documented, never remapped here): `action_raw` is policy order
(ROS1 FL,FR,RL,RR); `action_clipped`, `joint_target_rad`, `torque_nm` are
controller order (FR,FL,RR,RL) — identical to the existing
`logSymmetryDebug`/`policyToCtrlDofOrder` convention. `torque_saturated` is not
computed anywhere and is always written 0.0 with `torque_saturated_computed = 0`.

## Real-data boundary (fail-closed)

`classify_frame(data, now_ns, stale_timeout_ns)` returns exactly one of:

| Status | Condition |
|---|---|
| `MISSING` | empty/absent input |
| `INVALID` | wrong size, wrong magic/version, unarmed (seq 0) or odd sequence, inconsistent flags (`rl_active` with `safety_faulted`/without `rl_entered`, `controller_active=0`), non-finite payload, `monotonic_ns=0`, or `now < monotonic_ns` |
| `UNKNOWN_ORIGIN` | `source` unset or unrecognized |
| `LEGACY` | `source == LEGACY_ONLY` (rejected) |
| `SYNTHETIC` | `source == SYNTHETIC_TEST` (never live) |
| `STALE` | authoritative, coherent, finite, but timestamp older than timeout |
| `LIVE` | authoritative + coherent + finite + consistent + fresh |

Only `source == AUTHORITATIVE_RUNTIME` is a LIVE candidate. A complete,
well-formed `SYNTHETIC_TEST` or `LEGACY_ONLY` frame is classified accordingly and
never shown as live simulation data. This mirrors the P1-09B adapter input-origin
boundary at the frame level.

A faulted frame (`policy_state=FAULTED`, `rl_active=0`, `safety_faulted=1`) is
real, fresh, authoritative data and is therefore classified LIVE — the HUD then
surfaces the fault and suppresses the command chain. A fault is real evidence and
must not be hidden.

## HUD state model (`HudState`)

`HudState.update(...)` classifies the latest snapshot; `HudState.display()`
returns only real live values and refuses to fabricate:

- non-LIVE status → only `{"status", "live": False}`; no data fields.
- `torque_saturated` → always `None` (not computed; `torque_saturated_computed=0`).
- `collision` → always `None` (collision shm is bridge-side, not in this frame).
- `ray2d` → `None` when `ray_valid=0`.
- `action_raw/action_clipped/joint_target_rad/torque_nm` → `None` when faulted.

## Validation results (all offline)

| Command | Result |
|---|---|
| `python3 scripts/test_abs_rt_frame.py` | **22/22 PASS** |
| `python3 scripts/test_formal_runtime_adapter.py` | **16/16 PASS** (P1-09B green) |
| `python3 scripts/test_formal_experiment_contract.py` | **22/22 PASS** (P1-02 green) |
| `g++ -fsyntax-only -std=c++17 -I common/ common/abs_rt_frame_contract.h` | **PASS** |
| layout check (sizeof/offsets vs Python `<7Q11I81f`) | **MATCH** (424 bytes; header.sequence @ 16) |
| `python3 -m py_compile abs_rt_frame.py test_abs_rt_frame.py` | **PASS** |
| `colcon build --packages-select rl_quadruped_controller --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DTorch_DIR=.../Torch` | **PASS (BUILD_EXIT=0)**, `librl_quadruped_controller.so` produced, `StateRL.cpp.o` rebuilt, no warnings/errors |
| `git diff --check` | **clean** |

## Synthetic-only fields (test fixtures, not runtime evidence)

Every `test_abs_rt_frame.py` frame is built in Python with `struct.pack` and is
therefore synthetic. In particular: `session_id`, `rl_step`, `monotonic_ns`,
`ra_value`, `lin_vel`, `command`, `world_pose`, `ray2d`, and the 5×12 command
chain are all fixture numbers, not captured runtime values.

## Synthetic fixture source fix (old → new)

The first P1-09C test suite defaulted its Python-built fixtures to
`source = AUTHORITATIVE_RUNTIME`, so a synthetic frame could (incorrectly) be
classified LIVE by the classifier under test.

**Old behavior:** `_pack_frame()` defaulted to `source=AUTHORITATIVE_RUNTIME`;
only the origin-rejection tests overrode it. A fully well-formed synthetic frame
was therefore treated as authoritative and classified LIVE.

**New behavior:** `_pack_frame()` defaults to `source=SYNTHETIC_TEST`, so no
synthetic fixture is ever mistaken for real runtime data. Tests that verify the
authoritative *parse branches* (LIVE / STALE / INVALID / faulted) opt in
explicitly via `_authoritative_fixture(...)` (= `_pack_frame(source=
AUTHORITATIVE_RUNTIME, ...)`) and are named/documented as `synthetic-test-only`
format-and-branch verification, never runtime evidence. Added guards:

- `test_default_fixture_is_synthetic_not_authoritative` — the default fixture is
  `SYNTHETIC`, never LIVE.
- `test_synthetic_fixture_never_displayed_live` — a finite/fresh/coherent
  synthetic frame is shown by `HudState` as `live=False` with no data fields.

`abs_rt_frame.py` (classifier/HUD) needed no change: `classify_frame` already
returns `SYNTHETIC` for `source=SYNTHETIC_TEST` and `LEGACY` for
`source=LEGACY_ONLY` before any freshness/finiteness check, so the HUD never
shows synthetic/legacy input as live data.

## Fields that still lack an authoritative runtime producer

The frame publisher (`StateRL::writeRtFrame`) authoritatively produces the
in-memory controller fields, but the following remain `UNKNOWN`/unavailable and
are signalled by flags, never fabricated:

- `collision` — bridge-side only; `collision_origin = UNAVAILABLE` (a future
  increment may read `/dev/shm/mujoco_collision`, which is real MuJoCo data, but
  this frame does not carry it).
- `torque_saturated_*` (12) — no per-joint saturation flag is computed anywhere;
  `torque_saturated_computed = 0`.
- measured cadence / active ray source mode — the frame records `monotonic_ns`
  and `rl_step` (so cadence is *measurable* by a consumer), but the frame itself
  does not claim a measured period or the effective `MUJOCO_RAY_SOURCE` mode;
  both remain to be captured by a future authoritative increment.
- `session_id` is a per-`enter()` runtime identity, not the P1-02 formal `run_id`
  lineage; the P1-02 `run_id` allocation remains in the P1-02 `FormalRunWriter`.

## Relationship to P1-09B

P1-09B established the offline adapter boundary with an explicit input-origin
classification (`SYNTHETIC_TEST` writable; `LEGACY_ONLY`/`AUTHORITATIVE_RUNTIME`/
missing/unrecognized rejected). P1-09C adds the **producer-side** transport: the
`source` field in the frame carries the same four-way origin classification, and
`StateRL::writeRtFrame` always writes `AUTHORITATIVE_RUNTIME`. The frame is the
future input a recorder would bind to `FormalRuntimeAdapter`; that binding is a
later, separately-authorized increment. No P1-02 schema or Acceptance change.

## Compile verification (controller target)

The controller target containing `StateRL.cpp` was compiled with the project's
own build method, without starting any ROS2 node / MuJoCo / simulation / pilot /
benchmark / real-robot process:

```
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --packages-select rl_quadruped_controller --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DTorch_DIR=/home/lidio/Libraries/libtorch-cpu-2.0.1/share/cmake/Torch
```

Result: **BUILD_EXIT=0** after a full recompile (`colcon build ...
rl_quadruped_controller` took 23.0 s and rebuilt the object, not a no-op).
`librl_quadruped_controller.so` produced
(`build/rl_quadruped_controller/librl_quadruped_controller.so`, mtime
2026-08-28T15:23:17) and `StateRL.cpp.o` rebuilt (mtime 2026-08-28T15:23:12). The
object was verified to contain the new instrumentation — `nm -C .../StateRL.cpp.o`
shows `StateRL::writeRtFrame(...)` and `StateRL::writeRtFaultedFrame()` — and the
build log contains **no warnings and no errors** under `-Wall -Wextra -Wpedantic`.
The header
`common/abs_rt_frame_contract.h` resolves through the package's existing private
include path `${CMAKE_CURRENT_SOURCE_DIR}/../../../common` (the same path that
already serves `abs_ray2d_shm_contract.h`), so no `CMakeLists.txt` change was
required.

## Confirmation

No ROS2 node, MuJoCo, simulation, benchmark, pilot, formal episode, or real-robot
process was run. This increment is (1) offline Python code + synthetic tests, and
(2) a compile-only controller build of `StateRL.cpp`; the shared library was
compiled but never executed.
