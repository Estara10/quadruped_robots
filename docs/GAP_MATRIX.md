# ABS Paper-to-Code Gap Matrix

Baseline: Day 0 audit. A function or model file is not completion evidence by itself. `UNKNOWN` means the evidence does not currently exist or was not recoverable; it must not be guessed.

P1-01 disposition: **ACCEPTED WITH KNOWN ISSUES** by independent review on
2026-08-30 under DEC-010. Historical reproducibility remains deferred, and real
Go2 `foot_force[0..3]` semantics remain Phase 2 hardware-only; neither is a
P1-01 blocker. See [`REVIEW_2026-08-30_SCOPE_ALIGNMENT.md`](evidence/P1-01/REVIEW_2026-08-30_SCOPE_ALIGNMENT.md).

| Requirement | Paper Definition | Current Implementation | Evidence | Gap | Severity | Required Change | Validation Method |
|---|---|---|---|---|---|---|---|
| Agile I/O | 61 observations → 12 joint targets | P1-04 **ACCEPTED WITH KNOWN ISSUES**: production-linked parity and all scoped live ray/non-finite fault replays pass | [`POLICY_IO_CONTRACT.md`](POLICY_IO_CONTRACT.md), [`p1_01f_local_contract.json`](evidence/P1-01/p1_01f_local_contract.json), [`P1-04 parity matrix`](evidence/P1-04/p1_04_parity_matrix_20260830.md), [`P1-04 review`](evidence/P1-04/REVIEW_2026-08-31_PARITY.md) | Per DEC-010, artifact order is the declared operational mapping (not independent historical metadata); goal shaping remains an intentional variant | High | Keep P1-01 contract regression + P1-04 parity matrix; goal shaping separate paper-faithful/stabilized mode | Asymmetric artifact fixture plus source lineage |
| 61-D observation | Contacts, angular velocity, gravity, goal, timer, q, qdot, previous action, 11 rays | P1-04 (2026-08-31) **ACCEPTED WITH KNOWN ISSUES**: contact/ang_vel/gravity/dof_vel/prev-action/ray-value **MATCH** element-wise vs real training oracle; goal & timer **INTENTIONAL_VARIANT**; dof_pos nominal MATCH, training bias-distribution UNKNOWN; ray validity fail-closed variant; deployment global `±100` clamp and training `LeggedRobot.step()` clip documented | [`POLICY_IO_CONTRACT.md`](POLICY_IO_CONTRACT.md), [`p1_04_parity_matrix_20260830.md`](evidence/P1-04/p1_04_parity_matrix_20260830.md), [`P1-04 review`](evidence/P1-04/REVIEW_2026-08-31_PARITY.md) | Goal shaping is an intentional engineering variant; training bias distribution unobservable; in-domain clamp parity is supported, but the complete downstream policy-library chain was not inspected; live integration evidence remains | High | Separate paper-faithful/stabilized modes and run fault replay | Fixed-state vectors plus live fault replay |
| Policy artifact provenance | Deployed policy must bind to source checkpoint, config, seed, commit and export | Exact Agile checkpoint→export→deployed chain **CONFIRMED** directly (2026-08-30 snapshot re-audit: checkpoint `model_4000.pt` `iter=4000` → export → deployed, weight-equal 8/8 + byte-equal SHA `5a87d6…`) | [`manifest.yaml`](../artifacts/manifest.yaml), [`POLICY_IO_CONTRACT.md`](POLICY_IO_CONTRACT.md), [`P1-01_server_snapshot_reaudit_20260830.md`](evidence/P1-01/P1-01_server_snapshot_reaudit_20260830.md), [`P1-01_scope_decision_20260830.md`](evidence/P1-01/P1-01_scope_decision_20260830.md) | Weight chain **CONFIRMED**; historical config snapshot/seed/training commit/export invocation = **deferred reproducibility** (Director-approved 2026-08-30 scope decision; NOT a P1-01 blocker) | (resolved by scope decision) | (retain hash/weight chain) | Source→export→deployment hash/weight chain |
| Joint/contact/action order | Must match the exported policy | Declared policy order `FL,FR,RL,RR` → documented action remap `[3,4,5,0,1,2,9,10,11,6,7,8]`/contact `[1,0,3,2]` → controller/MuJoCo `FR,FL,RR,RL`; Isaac Gym training order and ROS2/MuJoCo order are independently proven; current remap is bijective | [`isaac_gym_asset_order.json`](evidence/P1-01/isaac_gym_asset_order.json), [`POLICY_IO_CONTRACT.md`](POLICY_IO_CONTRACT.md), [`P1-01_scope_decision_20260830.md`](evidence/P1-01/P1-01_scope_decision_20260830.md) | **Operational mapping accepted conditionally** on the recorded operator-declared training order (`FL,FR,RL,RR`) and existing asymmetric contract evidence; it is **not** claimed to be independently recovered historical artifact metadata | (resolved by scope decision) | (do not change the underlying remap) | Asymmetric artifact input/action fixture and source lineage |
| Timer | Normalized time-left | ROS2 defaults to explicit 9-s rolling time-left; `legacy_fixed` is named only | [`POLICY_IO_CONTRACT.md`](POLICY_IO_CONTRACT.md) | Rolling timer has no physical episode reset | Medium | Record/compare deployment semantics in a scoped experiment | Timer boundary/replay test |
| Goal observation | Body-frame relative x/y/heading | Runtime adds radial compression and path shaping | [`StateRL.cpp`](../quadruped_ros2_control_humble/controllers/rl_quadruped_controller/src/FSM/StateRL.cpp) | Agile and RA inputs differ from raw training command | High | Separate raw ABS goal from navigation shaping | Golden coordinate cases and on/off ablation |
| Path tracking | Not part of paper ABS core | Enabled by default with tunable gains | [`config.yaml`](../quadruped_ros2_control_humble/descriptions/unitree/go2_description/config/abs/config.yaml) | Performance may be misattributed to ABS | High | Separate paper-faithful and stabilized variants | Paired fixed-seed on/off evaluation |
| Contact semantics | Vertical contact threshold with previous-step OR | Deployment now applies the same current OR prior policy-cycle filter | [`POLICY_IO_CONTRACT.md`](POLICY_IO_CONTRACT.md), [`P1-01_final_closure_20260830.md`](evidence/P1-01/P1-01_final_closure_20260830.md) | Real `foot_force[0..3]` semantics = **Phase 2 hardware-only UNKNOWN** — no verifiable official slot contract (local + official unitree_sdk2 LowState has none; MuJoCo bridge FR,FL,RR,RL is simulation convention; unitree_ros2 claim via third-party unverified) | High | Authorized real-hardware single-foot loading capture | Asymmetric single-foot touchdown/liftoff/scrape replay |
| Training termination contact | Base contact terminates failed episodes | Go2 config filters for `"base"`; runtime body is named `trunk`, producing an empty list | [`isaac_gym_asset_order.json`](evidence/P1-01/isaac_gym_asset_order.json), [`go2_pos_config.py`](../ABS/training/legged_gym/legged_gym/envs/go2/go2_pos_config.py) | Training may not terminate on base/trunk contact as intended | High | Confirm intended body filter and correct only in a scoped training-contract task | Runtime name capture plus constructed trunk-contact episode |
| Action and PD | `q_default + scale*a`, 12 targets, Kp30/Kd0.65 | Controller→motor trace and policy permutation are bijective; inline path adds action/target clipping | [`POLICY_IO_CONTRACT.md`](POLICY_IO_CONTRACT.md), [`validate_p1_01_contract.py`](../scripts/validate_p1_01_contract.py) | Policy-side order: operational mapping accepted conditionally on the operator-declared training order (DEC-010; not independently recovered historical metadata); manual Recovery lacks inline path clamps; saturation impact unrecorded | Medium | Test both execution paths and saturation (technical); historical provenance is deferred reproducibility | Asymmetric target trace and joint-wise saturation statistics |
| Torque limits | Consistent robot/training/execution limits | URDF, MJCF and runtime YAML values differ | [`go2.xml`](../unitree_mujoco/unitree_robots/go2/go2.xml), [`config.yaml`](../quadruped_ros2_control_humble/descriptions/unitree/go2_description/config/abs/config.yaml) | Safety and dynamics semantics inconsistent | High | Define training/sim/hardware/safety limits separately | Joint step tests and actual saturation point |
| Runtime rates | Policy/RA/Recovery 50 Hz; PD 200 Hz | `StateRL` runs model inference every four controller callbacks; static config contains both controller-manager 1000 Hz and controller-local 200 Hz, while comments also state 125 Hz | [`formula_parameter_trace.yaml`](evidence/P1-03/formula_parameter_trace.yaml), [`robot_control.yaml`](../quadruped_ros2_control_humble/descriptions/unitree/go2_description/config/robot_control.yaml) | Actual callback cadence and which static rate governs it are `UNKNOWN`; candidate policy/RA/Recovery rates are 50 Hz or 250 Hz, not proven 50 Hz | Medium | Capture effective scheduling configuration and measured periods; do not infer cadence from comments | Mean/P95/P99, missed-cycle report and effective-controller-manager capture |
| RA observation | Twist6 + goal2 + ray11 = 19 | P1-05 **ACCEPT WITH KNOWN ISSUES** (2026-08-31): 19-D matrix order matches paper Eq.14 (`cat(lin_vel, ang_vel, commands[0:2], ray2d)`); lin_vel/ang_vel body-frame **MATCH**; goal x/y = **shaped** commands (goal-shaping variant); rays = log2 from MuJoCo shm, **fail-closed** validity variant | [`StateRL.cpp`](../quadruped_ros2_control_humble/controllers/rl_quadruped_controller/src/FSM/StateRL.cpp), [`P1-05_ra_19d_matrix_20260830.json`](evidence/P1-05/P1-05_ra_19d_matrix_20260830.json), [`REVIEW_2026-08-31_RA_SEMANTICS.md`](evidence/P1-05/REVIEW_2026-08-31_RA_SEMANTICS.md) | Goal shaping remains a stabilized variant; ray fail-closed handling is a deployment variant | High | Keep P1-05 19-D matrix; separate paper-faithful goal mode | Deterministic 19-D field/order fixture plus source trace |
| RA target | Paper Eq.16–18, gamma=.999999, paper reach signal | P1-05 **ACCEPT WITH KNOWN ISSUES** (2026-08-31): `g=+1 collision/−1`, `l=tanh(log2(d/0.65+1e-8))`, Bellman `γ=0.999999` max/min operator, 10-step soft collision, terminal bootstrap 1.0, loss ×100. Eq.16–17 **MATCH** (structural/source-level); Eq.18 reach **INTENTIONAL_VARIANT/UNKNOWN** (log2+0.65+1e-8); collision definition/terminal/loss **INTENTIONAL_VARIANT/UNKNOWN**. Switching three-way: paper `RA>=−0.05`; recovered testbed `recovery=(v_pred>-twist_eps=-0.05)` strict, immediate, no hold (`testbed.py:64,324`); deployed ENTER `ra>−0.05`, EXIT `ra<−0.08`+30-step hold (**stabilized variant**); no paper equivalence claimed | [`testbed.py`](../ABS/training/legged_gym/legged_gym/scripts/testbed.py), [`ABS_PAPER_NOTES.md`](../ABS_PAPER_NOTES.md), [`P1-05_ra_semantics_20260830.md`](evidence/P1-05/P1-05_ra_semantics_20260830.md) | Paper Eq.18 exact log base/σ and loss/terminal constants not recorded locally (UNKNOWN); no label numeric fixture; stabilized switching differs from paper | High | Keep P1-05 label/arithmetic evidence; no behavior change | Hand-calculated targets and fixed validation set |
| RA provenance | Policy-conditioned value from ~200k Agile episodes | Exact RA named→JIT→deployed chain **CONFIRMED** directly (2026-08-30 snapshot re-audit: named RA → JIT → deployed, weight-equal 6/6 + byte-equal SHA `05c40f…`; naming mechanism `policy_name[:-3]+"_ra.pt"` source-verified in `testbed.py`) | [`manifest.yaml`](../artifacts/manifest.yaml), [`POLICY_IO_CONTRACT.md`](POLICY_IO_CONTRACT.md), [`P1-01_server_snapshot_reaudit_20260830.md`](evidence/P1-01/P1-01_server_snapshot_reaudit_20260830.md), [`P1-01_scope_decision_20260830.md`](evidence/P1-01/P1-01_scope_decision_20260830.md) | **Operator-declared linkage**: project owner declares RA used Agile `model_4000.pt` (`OPERATOR_DECLARED`); independent historical execution record is **absent** and is **deferred reproducibility** (not a P1-01 blocker) | (resolved by scope decision) | (retain weight chain) | Source/export golden outputs plus dataset manifest |
| Switching threshold | Recovery when RA >= −0.05; Agile below | P1-07 (2026-09-01) **ACCEPT WITH KNOWN ISSUES — final independent review**: `abs.switching_mode` = `paper_faithful_switch` (`RA >= -0.05` enter / `RA < -0.05` exit, no hysteresis, no hold) or `stabilized_switch` (**default**; strict `>` enter, `< -0.08` + 30-step hold exit, byte-for-byte pre-P1-07); invalid mode rejected at init; helper `RASwitchingLogic.hpp` | [`StateRL.cpp`](../quadruped_ros2_control_humble/controllers/rl_quadruped_controller/src/FSM/StateRL.cpp), [`RASwitchingLogic.hpp`](../quadruped_ros2_control_humble/controllers/rl_quadruped_controller/include/rl_quadruped_controller/FSM/RASwitchingLogic.hpp), [`P1-07`](evidence/P1-07/P1-07_switching_modes.md), [`REVIEW_2026-09-01_FINAL.md`](evidence/P1-07/REVIEW_2026-09-01_FINAL.md) | Switching separated into paper-faithful vs stabilized; runtime behavior of either mode unmeasured (**UNKNOWN**) | High | Runtime switching evidence later (deferred; P1-07 accepted) | Offline truth-table test **PASS (292 checks)** + runtime ENTER/EXIT timestamp assertion |
| Recovery hold timing | No 30-step forced hold in paper definition | P1-07 (2026-09-01) **ACCEPT WITH KNOWN ISSUES — final independent review**: 30-step forced hold applies only to `stabilized_switch` (default); `paper_faithful_switch` has **no** forced hold; comments assume 8 ms but actual tick is 20 ms | [`config.yaml`](../quadruped_ros2_control_humble/descriptions/unitree/go2_description/config/abs/config.yaml), [`robot_control.yaml`](../quadruped_ros2_control_humble/descriptions/unitree/go2_description/config/robot_control.yaml) | Actual hold ≈0.60 s, not claimed 0.24 s (hold duration unmeasured at runtime) | **Critical** | Measure actual hold duration and configure in seconds (runtime task, out of P1-07 scope) | ENTER/EXIT timestamp assertion |
| Recovery I/O | 49 observations, no exteroception, 12 joint targets | Hash, `49→12` architecture, deterministic model output and field slices verified | [`p1_01_contract.json`](../artifacts/p1_01_contract.json), [`POLICY_IO_CONTRACT.md`](POLICY_IO_CONTRACT.md) | Source provenance/order = **deferred reproducibility** (DEC-010; not a P1-01 blocker); manual and inline target-clamp paths differ (technical gap) | High | Execute an implementation-level 49-D parity fixture on both paths; provenance is deferred reproducibility | 49-D golden vector and asymmetric motor-target trace |
| Safe twist update | Re-evaluate safety during Recovery | Twist optimized on entry and cached | [`StateRL.cpp`](../quadruped_ros2_control_humble/controllers/rl_quadruped_controller/src/FSM/StateRL.cpp), [`P1-06_recovery_optimizer_parity.md`](evidence/P1-06/P1-06_recovery_optimizer_parity.md) | P1-06 **ACCEPT WITH KNOWN ISSUES** (2026-08-31, two-dimensional verdicts): cached twist may become unsafe; no per-tick re-optimization; not fixed | **Critical** | Deferred (separate paper-faithful solver work; not auto-fixed by P1-06) | Per-tick final RA constraint margin |
| Eq.22 displacement | Includes yaw-coupled second-order terms | Runtime uses only vx*dt and vy*dt (`StateRL.cpp:623-624`); reference `get_pos_integral` (`testbed.py:55-61`) matches paper — **MISMATCH** (paper↔reference MATCH; deployment MISMATCH; first-order goal-penalty consequence MISMATCH), recorded by P1-06, not fixed | [`StateRL.cpp`](../quadruped_ros2_control_humble/controllers/rl_quadruped_controller/src/FSM/StateRL.cpp), [`testbed.py`](../ABS/training/legged_gym/legged_gym/scripts/testbed.py), [`P1-06 matrix`](evidence/P1-06/P1-06_recovery_optimizer_matrix.json) | Mathematical mismatch (yaw-coupled terms omitted) | High | Deferred (paper-faithful solver; numeric fixture proves the difference, no auto-fix) | Python/ROS1/C++ value and gradient parity |
| Safe twist constraint | Final predicted RA below −0.05 | Fixed ReLU penalty (`10*max(ra+0.10,0)`); no final feasibility check (paper UNKNOWN; testbed↔deployment both absent); grad-clip per-element vs reference L2-norm **MISMATCH** (not an approved stabilized variant) | [`StateRL.cpp`](../quadruped_ros2_control_humble/controllers/rl_quadruped_controller/src/FSM/StateRL.cpp), [`P1-06 matrix`](evidence/P1-06/P1-06_recovery_optimizer_matrix.json) | Output safety is unproven; paper λ/ε/feasibility/fallback UNKNOWN | High | Deferred (record final value and define infeasible fallback in a separate task) | State replay and satisfaction-rate report |
| Recovery failure | Safe fallback in dangerous state | Non-finite observation/RA/action/target and ray/writer faults invoke finite nominal command plus latched PASSIVE; all scoped live replays pass | [`POLICY_IO_CONTRACT.md`](POLICY_IO_CONTRACT.md), [`p1_01f_local_contract.json`](evidence/P1-01/p1_01f_local_contract.json) | This is controller-level fail-closed only, not the Phase-2 independent Safety Supervisor | Medium | Retain regression; build independent supervisor only in Phase 2 | Clock-synchronized fault replay to final command |
| Ray geometry | 11 rays, ±45°, 0.1–6 m, log distance | Default geometric writer uses the declared origin/angles/range and writes `log2(m)`, but uses exact boxes, skips dynamic/terrain/mesh geoms, and is disabled in external/ray-pred mode | [`formula_parameter_trace.yaml`](evidence/P1-03/formula_parameter_trace.yaml), [`unitree_sdk2_bridge.h`](../unitree_mujoco/simulate/src/unitree_sdk2_bridge.h) | Default writer differs from training circle-query semantics; actual external-vs-geometric mode remains `UNKNOWN` without effective-run capture | High | Offline ray oracle and registered geometry; record effective source mode | Cylinder/box/low obstacle/terrain comparison plus source-mode capture |
| Ray validity | Identified fresh perception at runtime | Versioned sequence-consistent stamped side-channel validates presence, freshness and finite beams; normal/freeze/exit/NaN/Inf live replays pass | [`POLICY_IO_CONTRACT.md`](POLICY_IO_CONTRACT.md), [`p1_01f_local_contract.json`](evidence/P1-01/p1_01f_local_contract.json) | External predictor does not yet implement the v2 header; hardware perception adapter remains future work | High | Require v2 header in external source before using it; add P2 adapter later | Normal-writer soak plus clock-synchronized freeze/exit/NaN/Inf tests |
| MuJoCo source/assets | Exact simulator and robot model must be reproducible | Day 0 adds required source/Go2 assets; prior history was incomplete | [`REPOSITORY_BASELINE.md`](REPOSITORY_BASELINE.md) | Clean-checkout build and exact external versions still unverified | High | Build from clean checkout and record dependency versions | Rebuild plus binary/model hashes |
| Recovery training artifact reference | Exported policy provenance must be portable and tied to an immutable artifact | Tracked candidates do not match deployed Recovery; absolute symlink target is a different artifact; server checkpoint is documented but absent | [`manifest.yaml`](../artifacts/manifest.yaml), [`recover_v4_twist.pt`](../ABS/training/legged_gym/resources/policy/recover_v4_twist.pt) | Exact source/export relationship = **deferred reproducibility** (DEC-010; not a P1-01 blocker); the absolute symlink target differing from the tracked candidate is a technical artifact-portability note | High | Retain the weight chain; replace the absolute reference only after a tracked immutable artifact is available (deferred reproducibility follow-up) | Fresh-checkout source/export/deployment weight chain |
| MuJoCo config key uniqueness | Runtime configuration must have one unambiguous effective value per key | `domain_id` and `interface` occur more than once in the tracked simulator YAML | [`config.yaml`](../unitree_mujoco/simulate/config.yaml) | Parser-dependent effective values and operator intent are not explicitly proven | Medium | Normalize only after confirming runtime precedence and intended values | YAML parser fixture plus runtime effective-config report |
| Timestep/dynamics | Simulator deviations quantified | MJCF lacks explicit timestep; PhysX/MuJoCo parameters differ | [`go2.xml`](../unitree_mujoco/unitree_robots/go2/go2.xml), [`legged_robot_config.py`](../ABS/training/legged_gym/legged_gym/envs/base/legged_robot_config.py) | Dynamics gap not quantified | High | Freeze solver/timestep and perform system-ID tests | Step, drop, contact and velocity tracking tests |
| Collision | True registered contact events | Only selected static primitives; sustained contact increments each tick | [`unitree_sdk2_bridge.h`](../unitree_mujoco/simulate/src/unitree_sdk2_bridge.h) | Incomplete geometry and wrong event-count semantics | **Critical** | Edge-counted named contact events with duration | Constructed contact truth cases |
| Fall | Clear paper/training/deployment definitions | Evaluator height/tilt differs from training base-contact termination | [`run_abs_eval.py`](../scripts/run_abs_eval.py) | Cross-result fall rate is not equivalent | High | Report each criterion separately and preregister primary rule | Labelled fall/non-fall replay set |
| Success | Stable, upright, collision-free arrival | P1-02's **accepted fixture-level** validator rejects Success for any collision/fall telemetry transition, structured safety event, or terminal contradiction; legacy evaluator is unchanged | [`REVIEW_2026-08-26_FINAL.md`](evidence/P1-02/REVIEW_2026-08-26_FINAL.md), [`formal_experiment_contract.py`](../scripts/formal_experiment_contract.py) | Authoritative runtime event producer/arrival-hold source is not connected; no formal runtime episode is accepted and legacy outputs remain unsafe for Acceptance | **Critical** | Connect safety-first reducer to authoritative structured events | Fixture safety-alignment/SUCCESS-veto PASS; runtime integration required |
| Seed/Monte Carlo | Fixed reproducible seeds and multi-seed statistics | P1-02's **accepted fixture-level** contract has deterministic root-seed derivation, internally allocated run IDs, duplicate-run-ID comparison rejection and paired-key validation; runtime generator/evaluator propagation remains absent | [`REVIEW_2026-08-26_FINAL.md`](evidence/P1-02/REVIEW_2026-08-26_FINAL.md), [`field_to_runtime_source.md`](evidence/P1-02/field_to_runtime_source.md) | Runs are not controlled Monte Carlo; seed propagation into formal runtime episodes is unimplemented, and cross-process uniqueness is not a distributed registry | **Critical** | Connect every random source in P1-10 | Fixture seed/pairing PASS; run-ID collision and runtime replay required |
| Experiment outputs | Manifest, timeline, events, trajectory, summary and plots | P1-02's **accepted fixture-level** `abs-go2-formal-run/v1` schema/validator verifies nested fields, run identity, duplicate run-ID rejection, 11-ray/5×12 command vectors, hash-bound artifacts and data-driven plots | [`REVIEW_2026-08-26_FINAL.md`](evidence/P1-02/REVIEW_2026-08-26_FINAL.md), [`p1_02_mechanical_tests.json`](evidence/P1-02/p1_02_mechanical_tests.json) | Runtime adapter does not emit complete formal telemetry/events/plots; no formal runtime episode or benchmark is accepted, and existing results remain `LEGACY / NON-ACCEPTANCE` | High | Connect authoritative runtime sources without bypassing validator | 22 fixture-level schema/validator tests PASS; runtime integration evidence required |
| HUD | Key policy/RA/safety state visible | No verified ABS HUD | MuJoCo source audit | Operational observability incomplete | Medium | Minimal HUD without replacing logs | Screenshot and state-change test |
| StateRLRec / MuJoCo teardown | Controller plugin and simulator process must release resources without aborting or forced termination | P1-09E observed `terminate`/SIGABRT at plugin teardown because `StateRLRec` owned an unjoined permanent inference thread; P1-09F repaired the worker lifecycle. P1-09G/I reached `[REC-ENTER]` and `[REC-EXIT]`, normal controller-manager/plugin shutdown and ROS launch `rc=0`, with no captured abort signature; MuJoCo received SIGINT, timed out, then received SIGTERM and exited `rc=143`. P1-09J statically confirmed an unjoined permanent MuJoCo bridge thread, worker-thread `exit(0)`, incomplete main shutdown, and no SIGINT handler. P1-09K runtime confirmed `SigIgn=0x6`, including SIGINT; UI-close delivery was not exercised because no window target was found | [`P1-09E_hud_run.md`](evidence/P1-09/P1-09E_hud_run.md), [`P1-09F_thread_teardown.md`](evidence/P1-09/P1-09F_thread_teardown.md), [`P1-09G_controlled_shutdown.md`](evidence/P1-09/P1-09G_controlled_shutdown.md), [`P1-09I_controlled_shutdown.md`](evidence/P1-09/P1-09I_controlled_shutdown.md), [`P1-09J_mujoco_shutdown_audit.md`](evidence/P1-09/P1-09J_mujoco_shutdown_audit.md), [`P1-09K_signal_quit_diagnosis.md`](evidence/P1-09/P1-09K_signal_quit_diagnosis.md) | StateRLRec live-unload/controller-plugin path is observed PASS, but clean MuJoCo shutdown is **BLOCKED / REJECTED EVIDENCE**: SIGINT is ignored in the tested context, does not terminate the child, and SIGTERM is required (`rc=143`). UI-close whole-process behavior remains **UNKNOWN** | High | Director-authorized lifecycle-only repair or validated existing UI-close path; retain join-only ownership and do not call TERM cleanup clean | SIGINT disposition evidence, MuJoCo exit `0` after SIGINT or documented UI close, controller/plugin cleanup, no `terminate`/SIGABRT |
| MuJoCo graceful-shutdown lifecycle design | One stop request must stop owned workers, join them, then let main return normally | P1-09L documents a proposed `RUNNING → STOP_REQUESTED → THREADS_JOINED → PROCESS_EXITED` coordinator, signal-safe pending flag, stoppable bridge loops, no worker `exit(0)`, and join-only ownership | [`P1-09L_graceful_shutdown_design.md`](evidence/P1-09/P1-09L_graceful_shutdown_design.md) | **Design only**; bridge destructor/model-data ownership, render-loop signal responsiveness, and join ordering remain `UNKNOWN`/`LIKELY`; no runtime proof exists | High | Obtain independent design review before a lifecycle-only implementation; then execute bounded SIGINT/SIGTERM/UI-close proof without TERM/KILL escalation | Code review + build/lifecycle tests + one bounded MuJoCo-only exit per request source, final `rc=0` |
| MuJoCo controlled-exit implementation | Stop all m/d users, join them, then let main return normally | P1-09M could not begin: `RecurrentThread` destructor/stop/join implementation is absent locally; current `exit(0)`/`pthread_exit()` and unjoined bridge remain | [`P1-09M_graceful_shutdown_implementation.md`](evidence/P1-09/P1-09M_graceful_shutdown_implementation.md), `recurrent_thread.hpp`, `main.cc` | **BLOCKED**: cannot prove SDK callback thread stops before `mjModel`/`mjData` release; no implementation or test evidence | High | Recover exact SDK thread semantics, obtain independent design review, then implement lifecycle-only repair | SDK contract/probe + compile/static lifecycle tests + bounded runtime clean exit |
| RecurrentThread binary lifecycle contract | Bridge callback must finish before RobotBridge and `m/d` destruction | Effective SDK 2.0.0 archive contains `pthread_create` with detached state; `Wait()` sets `mQuit`; destructor calls `pthread_cancel` but no `pthread_join`; RobotBridge does not call `Wait()` | [`P1-09N_recurrent_thread_contract.md`](evidence/P1-09/P1-09N_recurrent_thread_contract.md), `/home/lidio/Libraries/unitree_sdk2/lib/libunitree_sdk2.a` SHA256 recorded there | **BLOCKED**: stop request exists only as `Wait()`, and completion before `m/d` free is not proven | High | Obtain completion barrier/source contract or redesign ownership boundary, then independent review | Version-pinned lifecycle probe or authoritative SDK source + teardown test |
| D435 adapter | Real depth maps to the same 11-ray contract | Prototype has stream/FOV/invalid-depth issues | [`realsense_ray_predictor.py`](../scripts/realsense_ray_predictor.py) | ±45° visibility, preprocessing and latency unverified | **Critical** | Intrinsics-based Perception Adapter | Calibration target, disconnect and edge-ray tests |
| Independent Safety Supervisor | Policy-independent final command veto | Protections are distributed across FSM/controller/hardware | [`RlQuadrupedController.cpp`](../quadruped_ros2_control_humble/controllers/rl_quadruped_controller/src/RlQuadrupedController.cpp) | Controller crash/stall safety unproven | **Critical** | Independent heartbeat, sensor health and latched veto | kill -9, stall, DDS and camera fault injection |
| Real launch | Explicit fail-fast sim/real mode | Real script/parameter path may still select simulation defaults | [`launch_abs_real.sh`](../scripts/launch_abs_real.sh), [`real_go2.launch.py`](../quadruped_ros2_control_humble/controllers/rl_quadruped_controller/launch/real_go2.launch.py) | Hardware mode and parameter propagation unproven | **Critical** | Explicit xacro/launch hardware parameters | Expanded URDF and mock initialization checks |

## Evidence Status Summary

### P1-09O implementation status

P1-09O replaces the MuJoCo bridge's m/d-accessing SDK detached callback with a
project-owned `JoinableThread`. Main requests bridge stop, joins the bridge and
physics threads, and only then releases final m/d. Offline lifecycle/build
evidence is PASS, but independent review **REJECTED** it as a complete repair:
PhysicsLoop reload can replace m/d while RobotBridge retains cached pointers.
Real process shutdown, reload lifetime, concurrent start/stop behavior, and
external SDK/DDS teardown remain UNKNOWN. DEC-009 approves a fail-closed reload
safety barrier; its lifecycle-only implementation and independent review are
still required. See
[`P1-09O_joinable_bridge_implementation.md`](evidence/P1-09/P1-09O_joinable_bridge_implementation.md).

### P1-09Q concurrency audit

P1-09P is recorded as **ACCEPT WITH KNOWN ISSUES**. P1-09Q confirms a High
bridge–PhysicsLoop same-`mjData` data-race risk because RobotBridge reads and
writes m/d without the PhysicsLoop/render mutex. It also confirms that both
reload branches leak `mnew` when `mj_makeData(mnew)` fails. SDK DDS thread
ownership and any hidden m/d access remain **UNKNOWN**. See
[`P1-09Q_md_concurrency_audit.md`](evidence/P1-09/P1-09Q_md_concurrency_audit.md).

P1-09S closes the reviewed lock-boundary and initial-publication gaps offline:
post-`sim.Load()` m/d replacement is guarded, bridge start waits for initial m/d
readiness, constructor I/O is outside the guard, and armed `ray_exit` is
immediate. Runtime lock/contention behavior and DDS-owned thread lifetime
remain **UNKNOWN**. See
[`P1-09S_md_lock_boundary_closure.md`](evidence/P1-09/P1-09S_md_lock_boundary_closure.md).

P1-09S remains **REJECTED** for lock-internal reload diagnostics, restart
ambiguity, and insufficient mechanical coverage. P1-09T is **REJECTED** because
separate active/terminal flags retained a transition race and its coverage was
not a real concurrent test. P1-09U replaces that pair with one mutex-protected
`INITIAL → RESERVED → ACTIVE → STOPPING → TERMINAL` state machine, reserves
before worker creation, rejects restart after terminal, and adds a real
64-iteration concurrent test. It is awaiting independent review; runtime clean
shutdown, DDS lifetime, and full runtime concurrency remain UNKNOWN. See
[`P1-09U_single_lifecycle_state.md`](evidence/P1-09/P1-09U_single_lifecycle_state.md).

P1-09V closes the terminal predecessor invariant and constructor-path m/d
access gap offline: `completeTerminal()` accepts only `STOPPING`, main joins
and destroys the bridge before terminal completion, and `mj_name2id`/`m->nu` are
guarded by `sim.mtx` with local copies. Static and lifecycle tests pass; runtime
clean shutdown, DDS lifetime, and full runtime concurrency remain UNKNOWN.
P1-09V is awaiting independent review. See
[`P1-09V_terminal_invariant_closure.md`](evidence/P1-09/P1-09V_terminal_invariant_closure.md).

P1-09W consolidates the remaining offline lifecycle corrections: `beginStop()`
accepts only RESERVED/ACTIVE, terminal completion is main-owned after bridge
join/reset, and constructor scene metadata is copied under `sim.mtx` before
printing. R/S/T/U/V/W mechanical checks and O/P/U/V lifecycle tests pass;
runtime clean shutdown, DDS lifetime, and full runtime concurrency remain
UNKNOWN. P1-09W is **ACCEPT WITH KNOWN ISSUES**; its bounded runtime clean
shutdown validation is tracked by P1-09X. See
[`P1-09W_lifecycle_consolidated_closure.md`](evidence/P1-09/P1-09W_lifecycle_consolidated_closure.md).

P1-09X was attempted once in the MuJoCo-only boundary and was **BLOCKED** by
GLFW initialization failure before the simulator reached runtime. It provides
no clean-shutdown or bridge/physics lifecycle evidence. See
[`P1-09X_clean_shutdown_run.md`](evidence/P1-09/P1-09X_clean_shutdown_run.md).

P1-09Y confirms the current GLFW blocker is an unreachable configured X11
display (`DISPLAY=:0`); `xdpyinfo` and `xset q` both fail, while no Xvfb or
project headless backend is available. The exact cause behind the unreachable
socket is UNKNOWN. See
[`P1-09Y_glfw_preflight.md`](evidence/P1-09/P1-09Y_glfw_preflight.md).

P1-09Z's authorized retry was **BLOCKED** at the required X11 preflight:
`DISPLAY=:0` with `/run/user/1000/gdm/Xauthority` yielded `xdpyinfo` rc=1.
MuJoCo was not launched, so no additional clean-shutdown evidence exists. See
[`P1-09Z_clean_shutdown_retry.md`](evidence/P1-09/P1-09Z_clean_shutdown_retry.md).

P1-09AA defines the proposed fail-closed `/mujoco_rt_frame` to P1-02 binding,
but does not close runtime integration. `run_id`/session binding, simulation
time, collision/fall, seed/config provenance, active ray-source provenance,
measured cadence and complete shutdown remain unavailable or UNKNOWN. See
[`P1-09AA_formal_recorder_binding_design.md`](evidence/P1-09/P1-09AA_formal_recorder_binding_design.md).

P1-09AB implements the fixed-source offline recorder boundary and rejection
tests. Session/sequence checks and missing-authority fail-closed behavior pass,
but no runtime producer is connected and no formal VALID run is possible.
Simulation time, seed/config, collision/fall, saturation, active ray source,
measured cadence and clean shutdown remain UNKNOWN. See
[`P1-09AB_recorder_binding.md`](evidence/P1-09/P1-09AB_recorder_binding.md).

P1-09AC closes the offline sequence/origin boundary: source sequence and
`rl_step` allow sampling gaps but reject rollback, while formal telemetry
sequence is contiguous only for truly eligible samples; rejected samples use a
separate rejection index. Synthetic/legacy/unknown-origin input remains
ineligible. This does not close runtime authority or produce formal VALID data.
See [`P1-09AC_sequence_and_origin_closure.md`](evidence/P1-09/P1-09AC_sequence_and_origin_closure.md).

P1-09AD first blocked at the mandatory graphical preflight (`xdpyinfo` rc=1);
in the user-confirmed graphical terminal the preflight succeeded and MuJoCo
3.3.3 started and exited `rc=0` after Ctrl+C with no TERM/KILL. The raw log
proves startup and the final exit code only; bridge stop/join, physics join,
and m/d release are not logged, so the result is **PARTIAL PASS**, not full
clean-shutdown proof. See
[`P1-09AD_clean_shutdown_graphical.md`](evidence/P1-09/P1-09AD_clean_shutdown_graphical.md).

P1-09AE completes the offline runtime data chain: the full payload of the real
`/mujoco_rt_frame` is saved into one per-run JSONL record (session/step/time
kept; flag-0 fields unavailable; no mock fill), and a post-run summary is
computed from that saved record only. simulation_time_s, reached-goal, timeout,
collision and fall remain `UNKNOWN` (no authoritative source); SUCCESS is never
produced. The record is fail-closed after Reviewer REJECT fixes: any present
non-LIVE/malformed/non-authoritative frame, continuity break, run-identity
mismatch, duplicate/misplaced terminal, negative duration, or malformed process
fact invalidates the whole record; frame status is whitelisted to
`{LIVE, MISSING}` (unknown/null/wrong-type statuses fail closed) and every LIVE
payload is fully schema-validated (missing field, wrong type/length, NaN/Inf,
malformed nested structure → INVALID) before any statistics; a MISSING frame is
a legal gap only when it carries no payload and no availability (non-empty →
`malformed_missing_frame` → INVALID); the recorder runs a two-phase capture →
stop_sampling → finalize lifecycle (duplicate finalize and post-stop/post-
finalize frame writes rejected; missing process facts stay UNKNOWN); the
summary never raises on a malformed record. See
[`P1-09AE_runtime_record_completion.md`](evidence/P1-09/P1-09AE_runtime_record_completion.md).

P1-09AE closes the process-level SIGINT-to-exit observation for one bounded
MuJoCo-only graphical run: PID/PGID and signal disposition were captured,
SIGINT delivery returned 0, no TERM/KILL escalation occurred, and `wait`
returned `rc=0`. The simulator has no internal join/release/DDS lifecycle log,
so those facts remain UNKNOWN and P1-09 is not accepted. See
[`P1-09AE_clean_shutdown_run.md`](evidence/P1-09/P1-09AE_clean_shutdown_run.md).

P1-09AE 的两阶段 runtime record contract 已获独立 Reviewer PASS：完整
LIVE payload 在 CAPTURE 阶段保存，STOP 后拒绝退出 INVALID 帧，FINALIZE
以实际 process facts 写唯一 terminal，summary 只读同一 record。尚未发生
完整的 MuJoCo + StateRL runtime capture；任务 outcome 仍因 goal/timeout/
collision/fall 权威来源缺失而只能为 UNKNOWN。P1-09 未接受。

P1-09AE 于 2026-08-30 完成一次环境修正后的真实 runtime capture（本运行唯一
改动为启动环境：所有子进程继承与 `launch_abs_sim.sh` 一致的
`LD_LIBRARY_PATH`，含 unitree_sdk2/lib + libtorch，解决上一次 11:28 失败的
`libddsc.so.0` 缺失）。数据链完整：真实 MuJoCo + StateRL → HUD LIVE（同一
session `8049381969251`）→ 两阶段 recorder（CAPTURE 306 LIVE / 176 MISSING →
STOP 未写 terminal → FINALIZE）→ 真实 process facts（ros launch `rc=0`、
MuJoCo `rc=0`、无 TERM/KILL、无残留）→ `post_run_summary`（record
VALID、authoritative true、`outcome=UNKNOWN`，非 SUCCESS）。该运行关闭了
启动环境阻塞与"真实 capture 尚未发生"的空档；`reached_goal`/`timeout`/
`collision`/`fall`/`simulation_time_s` 仍无权威来源而保持 UNKNOWN，内部
bridge/physics join 与 DDS teardown 顺序仍 UNKNOWN，P1-09 未接受。证据：
[`P1-09AE_record_capture_20260830.md`](evidence/P1-09/P1-09AE_record_capture_20260830.md)。

P1-09AE runtime-record subchain (2026-08-30) is **ACCEPT WITH KNOWN ISSUES**:
one real controlled MuJoCo + StateRL → HUD → authoritative JSONL record →
two-phase finalize → real process facts → post-run summary chain is verified.
P1-09 original task is **ACCEPTED**. The independent Reviewer accepted both
former formal blockers in the 2026-08-30 formal-closure run: (1) an
authoritative structured safety/terminal-event source exists
(`scripts/formal_runtime_binding.py` single reducer over the authoritative frame
and orchestrator wait facts; no text-log parsing); and (2) a representative real
episode was classified by the P1-02 validator as **INVALID**
(`validator_completed=true`, reasons = missing authoritative
`simulation_time_s`/collision/fall/goal/timeout sources, never a fabricated
SUCCESS/VALID; formal `run-0cbf4a…` ↔ runtime record `26861e6c…` ↔ session
`15818838355107`). See
[`P1-09C_formal_closure_20260830.md`](evidence/P1-09/P1-09C_formal_closure_20260830.md).
Other UNKNOWNs remain deferred to P1-01 provenance, P1-08 reproducibility, P1-02
follow-up, Phase-2 hardware, or future observability/benchmark; none is a
third P1-09 blocker. Phase 1 remains NOT ACCEPTED.

- Architecture present: Agile, RA, Recovery and switching data flow.
- Paper-equivalent correctness: **not yet proven**.
- Formal Phase 1 evidence: **not accepted**.
- Formal real ABS/RL: **NO-GO**.
- P1-03 paper-to-code trace: **ACCEPTED / COMPLETED** (offline traceability only) — [`formula_parameter_trace.yaml`](evidence/P1-03/formula_parameter_trace.yaml), [final review](evidence/P1-03/REVIEW_2026-08-27_FINAL.md). Not paper-equivalence or runtime proof; effective policy/RA/Recovery cadence (50 vs 250 Hz) and the active ray source mode remain `UNKNOWN` without authoritative runtime capture.
