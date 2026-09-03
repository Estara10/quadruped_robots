# P1-08 — MuJoCo Model, Effective Timing, and Dynamics Baseline (2026-09-01)

Status: **NOT an accepted P1-08 baseline** — this document records the
2026-09-01 capture, which used the **v1 sim-clock contract and unhashed timing**,
and is therefore historical/non-acceptance evidence. The P1-08 repair
([`P1-08_repair_20260901.md`](P1-08_repair_20260901.md)) defines the v2 contract
and the canonical baseline identity; an accepted baseline requires a new
Director-authorized recapture. Phase 1 remains **NOT ACCEPTED**.

## Baseline identity (superseded, NOT accepted)

**`bdd47a0d1414a0e8642c2e51cda3fef303ab32288366c5451988047251ea0734`** (old,
pre-repair aggregate identity — superseded). A canonical identity
(`build_p1_08_baseline_identity.py`, v1.0) now binds raw timing hashes; the
demonstration identity recomputed from this old capture + refreshed manifest is
`99b995b0…` and is **not** an accepted baseline.

## Historical measurement record (v1 capture, non-acceptance)

"Freeze" = record and bind the actual values used by the controlled
simulation-only run. It is **not** Sim-to-Real equivalence, tuning, benchmark,
paper equivalence, or an ABS result. All timing values come from **one
authoritative runtime capture**, not comments.

## Model closure (hash-bound)

- Launched scene: `unitree_mujoco/unitree_robots/go2/scene_flat.xml` (includes
  `go2.xml`); interface `lo`; `mujoco_simulate.robot_scene=scene_flat.xml`.
- Closure SHA-256 **`8d9218de0dc02978fc0ef4ba1c790fa3b968fbdbfdb945e14522436a2574ea07`**
  over scene XML + recursive includes + referenced mesh assets: 18 present files,
  **no missing asset**.

## Effective static model facts (MuJoCo probe, authoritative)

| Fact | Value |
|---|---|
| MuJoCo version | 3.3.3 |
| opt.timestep | **0.002 s** |
| integrator | Euler |
| cone | elliptic |
| solver | Newton |
| iterations / ls_iterations | 100 / 50 |
| tolerance / ls_tolerance | 1e-8 / 1e-2 |
| gravity | (0, 0, −9.81) |
| impratio | 100 |
| dims | nq19 nv18 nu12 njnt13 nbody18 ngeom57 nsite5 nsensor45 nmesh16 |
| actuator ctrlrange | hip/thigh ±23.7 N·m, knee ±45.43 N·m (12 actuators) |
| joint ranges | FR/FL hip ±1.0472, thigh −1.5708…3.4907, calf −2.7227…−0.83776; RR/RL back-hip −0.5236…4.5379 |

Probe 10-step arithmetic: advance 0.02 s = 10 × 0.002 s → **confirmed**.

## Effective static controller facts

| Key | Value | Source |
|---|---|---|
| controller_manager update_rate | 1000 Hz | robot_control.yaml |
| rl_quadruped_controller update_rate | 200 Hz | robot_control.yaml |
| decimation | 4 | abs/config.yaml |
| model_folder / use_rl_thread | abs / false | robot_control.yaml |
| **switching_mode** | **stabilized_switch** (P1-07 default, unchanged) | abs/config.yaml |
| ra_threshold / recovery_hold_steps | −0.05 / 30 | abs/config.yaml |
| twist params (λ,lr,τ,ε,ranges) | 10, 0.5, 0.05, 0.05, ±[1.5,0.3,3.0] | abs/config.yaml |
| robot_scene | scene_flat.xml | simulate/config.yaml |
| domain_id / interface | duplicate-key **static ambiguity** | simulate/config.yaml |

## Observed timing (one authoritative capture, 2026-09-01 14:02:38→14:03:20)

Capture `docs/evidence/P1-08/capture_20260901_rerun/`: 1251 rt_frame samples +
12500 sim_clock samples over a 25.0 s window; SIGINT-only shutdown, both
processes **exit rc=0**, `shutdown_complete=true`, `forced_termination=false`.

| Quantity | Observed value | Source |
|---|---|---|
| Physics timestep (sim-time per step) | **0.002 s** exactly (mean 0.0020000000000000755; min/max/p95/p99 all 0.002) | sim_clock |
| Physics wall-clock period | mean 2.0 ms → **500 Hz** | sim_clock |
| Policy (RL-step) tick period | **50.0 Hz** (mean 19.999 ms, median 20.001 ms; min 3.342 ms, P95 21.099 ms, P99 27.947 ms, max 47.836 ms; 1250 periods) | rt_frame |
| RA tick period | **= policy tick 50 Hz** (runRAModel per RL step, source-verified) | rt_frame |
| Recovery tick | **not active during capture** (0 recovery-active samples, 0 transitions) → not measurable | rt_frame policy_state |
| Controller callback period | declared 200 Hz; derived 5.0 ms = observed policy period / 4 under periodic-callback assumption; direct per-callback timestamps UNKNOWN | static + derived |

The **min/max policy-period outliers are dropped/duplicated-callback facts**:
one 3.34 ms "fast" frame and one 47.8 ms "slow" frame (missed-cycle jitter),
P99 27.9 ms. Policy/RA effective rate is now **proven ≈50 Hz** from runtime
evidence (GAP_MATRIX "Runtime rates" ambiguity resolved for the policy rate).

## Instrumentation (observability-only, behavior-neutral)

- `common/abs_sim_clock_contract.h`: single-writer `/mujoco_sim_clock`
  {sequence, monotonic_ns(steady_clock), sim_time} published after each
  `mj_step` in `main.cc` (2 sites). Never touches scheduling/thresholds/policy/
  optimizer/model/controller. Mechanical test `p1_08_sim_clock_test` **PASS**.
- Reused `/mujoco_rt_frame` (StateRL `writeRtFrame`, once per RL step) — no
  controller change.
- Recorded as observability, not a performance improvement.

## Known UNKNOWN / boundaries

- Recovery tick period: not measurable in this capture (never active).
- Direct 200 Hz controller-callback timestamps: no authoritative source; derived
  only under the periodic-callback assumption.
- `domain_id`/`interface` duplicate keys: static ambiguity (parser precedence
  not proven at runtime).
- Goal shaping, policy artifact order: unchanged P1-01/P1-04 variants.
- Not claimed: paper equivalence, Sim-to-Real equivalence, benchmark, formal
  experiment, performance tuning, or an ABS success/failure result.
- Single run (no retry); the first 2026-09-01 attempt that ran a stale binary is
  recorded separately in `P1-08_run_blocked_20260901.md`.

## Evidence (immutable)

- `P1-08_baseline_manifest.json` (schema v1, full SHA-256 inventory)
- `P1-08_model_probe_scene_flat.txt`
- `P1-08_simulation_baseline.json` (baseline identity + merged facts)
- `capture_20260901_rerun/`: `mujoco_raw.log`, `ros2_launch_raw.log`,
  `orchestrator_raw.log`, `process_facts.json`, `rt_frame_timing.jsonl`,
  `sim_clock_timing.jsonl`, `timing_stats.json`
- `P1-08_run_blocked_20260901.md` (the pre-reauthorization failed attempt)

## Orchestrator exit-code caveat (recorded, data unaffected)

The `p1_08_baseline_capture.py` process exited `1` on the successful run due to
a **post-save logging defect**: the final `== DONE ==` log line was emitted after
`LOG_F.close()`. All evidence (process_facts, raw logs, timing JSONLs) was
written **before** that point, and both target processes exited `rc=0` with
`shutdown_complete=true` — the capture data is complete and valid. The defect
was fixed in the script (`log` moved before `LOG_F.close()`); the fix is not a
re-run.

Phase 1 remains **NOT ACCEPTED**; P1-10 not started.
