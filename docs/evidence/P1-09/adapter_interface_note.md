# P1-09A — Minimal Adapter-Interface Note (design only)

Date: 2026-08-27
Scope: read-only evidence and interface-design increment of P1-09. No implementation
code is produced in this increment. No controller/policy/threshold/solver/config
behavior may be changed by any future adapter; the adapter is observational and
additive only.

This note proposes the smallest surface that would connect the authoritative
runtime sources documented in
[`field_to_runtime_source.md`](../P1-02/field_to_runtime_source.md) to the accepted
P1-02 formal contract (`abs-go2-formal-run/v1`).

## Proposed observational inputs (read-only)

These already exist in the allowed scope and require no control change to read:

- `FormalRunWriter` (`scripts/formal_experiment_contract.py`) — allocates `run_id`,
  writes manifest/telemetry/events/summary/plots and binds them by hash.
- Git identity: `scripts/run_abs_eval.py:_git_text` (commit/branch/status-short).
- Model hashes: `scripts/run_abs_eval.py:model_provenance` (`sha256_file`).
- Effective config: `scripts/run_abs_eval.py:write_effective_abs_config` (add hash).
- Scene identity/hash: `scripts/run_abs_eval.py:write_session_manifest` snapshots.
- Pose: `/dev/shm/mujoco_qpos` (19 doubles) and the odometer state interfaces.
- Rays + v2 header: `/dev/shm/mujoco_ray2d` + `abs_ray2d_shm::FrameHeader`
  (`unitree_sdk2_bridge.h:281-289`); validated snapshot in `StateRL::updateRay2d`.
- Collision counters: `/dev/shm/mujoco_collision` (5 int32).
- Ray source mode: `MUJOCO_RAY_SOURCE` env (`unitree_sdk2_bridge.h:69-74`).

## Proposed formal artifacts/events emitted

The adapter would emit, per run, through `FormalRunWriter`:

- `manifest.json` — identity, git, model hashes, effective-config hash, scenario
  hash, perception source/version/hash, thresholds, `hardware_mode=simulation`.
- `telemetry.csv` — per-sample row: run_id, sequence, `simulation_time_s`,
  `monotonic_time_ns`, `telemetry_fresh`, base pose (roll/pitch/yaw), policy_state,
  ra_value + thresholds, command/actual velocity, recovery twist, `ray_valid` +
  `ray_age_ns`, controller/rl active, collision available/collision, fall,
  arrival_candidate, 11 `ray_log2_*`, and the 5×12 command chain.
- `events.jsonl` — `episode_start`, `controller_active`, `rl_entered`,
  `valid_ready`, recovery ENTER/EXIT, `collision_start`/`collision_end`, `fall`,
  `arrival_accepted`, `timeout`, `terminal`, `shutdown`, plus any
  sensor/perception/controller invalidation.
- `summary.json` + 5 data-bound plots (RA/switching, trajectory/obstacles,
  command tracking, stability, recovery markers).

## Source fields requiring a future additive instrumentation hook

These are computed inside `StateRL`/bridge but not currently exposed, so a future
observational hook (read-only emission) is required:

1. **Simulation time + per-row sequence + monotonic clock** — no producer exists;
   requires a `simulation_time_s` source (MuJoCo `data.time`) and a run-local
   monotonic/sequence counter bridged from the simulator loop.
2. **5×12 command chain** — `action_raw`/`action_clipped`/`joint_target_rad`/
   `torque_nm` already exist in `StateRL::runModel` memory (`policy_actions`,
   `clamped_actions`, `output_dof_pos_`, `output_torques`) and only need an
   additive per-step exporter; `torque_saturated` is not computed at all and needs
   a new per-joint saturation flag. No change to the PD/target computation.
3. **`recovery_constraint_margin`** — not currently computed; requires a new
   read-only derived value from the final RA/twist check.
4. **Structured safety events** — `rl_entered` and recovery ENTER/EXIT already
   have in-memory transition flags (`running_`, `in_recovery_`) and only need an
   emitter; `collision_start`/`collision_end` have raw counters in the bridge; but
   `fall`, `arrival_accepted`, and `terminal` have no authoritative runtime
   producer (legacy evaluator only) and need a separate authoritative source.
5. **`ray_valid` / `ray_age_ns`** — already computed in `StateRL::updateRay2d`;
   require additive emission into the telemetry row. **`telemetry_fresh`** is a
   separate row-level freshness flag with no computed producer (distinct from
   ray-specific `ray_valid`); it needs a row-level monotonic timestamp + threshold.
6. **Measured rates and active ray source mode** — require a runtime capture hook
   (measured callback/runModel period and the effective `MUJOCO_RAY_SOURCE` value);
   never inferred from static config or startup log echoes.

## Boundary reminders

- No change to control semantics, policy I/O, thresholds, gains, switching, solver,
  dynamics, or runtime config behavior.
- No benchmark, pilot, formal Acceptance run, or real-robot execution.
- `UNKNOWN` and `LEGACY_ONLY` fields stay explicit and make the episode `INVALID`
  until an authoritative producer is connected.
