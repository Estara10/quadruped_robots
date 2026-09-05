# P1-06 — Recovery Eq.21/Eq.22 and Safe-Twist Optimizer Parity (2026-08-30, REJECT-corrected)

Offline source trace + deterministic numeric validation. No code/config/model/
threshold/solver/state-machine change; no MuJoCo/ROS2/training/export/benchmark/
hardware. Reviewer REJECT (2026-08-31) was a **classification overclaim**, not a
discovered algorithm mismatch; this document corrects every verdict to be
two-dimensional (`paper relationship` vs `testbed ↔ deployment relationship`)
and never treats a paper-undefined item as paper MATCH.

Sources:
- Paper: `docs/ABS_PAPER_NOTES.md:108-131` (Recovery twist ranges, Eq.21, Eq.22).
- Recovered testbed: `ABS_fuwuqi/ABS/training/legged_gym/legged_gym/scripts/testbed.py:55-72,337-348` (identical in `ABS/` for these audited lines; **source integrity PARTIAL — only the audited lines, not whole-file identity**).
- Deployment: `quadruped_ros2_control_humble/controllers/rl_quadruped_controller/src/FSM/StateRL.cpp:577-659` (`computeRecoveryTwist`).

## 1. Eq.21 — safe-twist optimizer three-way matrix (two-dimensional verdicts)

| Item | Paper relationship | Recovered testbed | Deployment | testbed ↔ deployment |
|---|---|---|---|---|
| Variables vx/vy/wz | paper implies twist vx/vy/wz (Eq.21 "twist") | `[vx,vy,wz]` from `cat(base_lin_vel[0:2], base_ang_vel[2:3])` (:337) | `{lin_vel[0:2], ang_vel[2]}` (:587-590) | **MATCH** |
| Objective structure | paper: minimize predicted goal deviation subject to RA below threshold (Eq.21) | `10*max(ra+2*0.05,0)+0.02*((x-goal_x)^2+(y-goal_y)^2)` (:343) | `10*clamp(ra+2*0.05,0)+0.02*((pos_x-cmd_x)^2+(pos_y-cmd_y)^2)` (:627-628) | **MATCH** (structure) |
| Objective constants (λ=10, ε=0.05, goal coeff 0.02) | **UNKNOWN** (paper does not specify numeric constants) | λ=10, ε=0.05, coeff=0.02 (:63-66,343) | λ=10, ε=0.05, coeff=0.02 (config) | **MATCH** |
| RA penalty | **UNKNOWN** for λ/ε (paper unspecified) | `10*max(ra+0.10,0)` (:343) | `10*clamp(ra+0.10,0)` (:627) | **MATCH** |
| Goal-deviation term | paper: predicted goal deviation | `0.02*((x-goal_x)^2+(y-goal_y)^2)` with yaw-coupled displacement (:343) | `0.02*((pos_x-cmd_x)^2+(pos_y-cmd_y)^2)` with first-order displacement (:628) | **MISMATCH** (Eq.22 term feeds this) |
| Gradient algorithm | paper: gradient descent | SGD `twist -= lr*clip_grad(grad,1)` (:346) | SGD `twist -= lr*clamp(grad,-1,1)` (:633-634) | **MATCH** (both SGD) |
| Gradient clip | **UNKNOWN** (paper unspecified) | **L2-norm** `_clip_grad`: `grad*thres/max(norm,thres)` (:70-72) | **per-element** `torch::clamp(grad,-1,1)` (:633) | **MISMATCH** (L2 vs per-element; not an approved stabilized variant — no decision record) |
| Initialization | paper: "from the current twist" (Eq.21) | current base lin_vel[0:2] + ang_vel[2] (:337) | current lin_vel[0:2] + ang_vel[2] (:587-590) | **MATCH** |
| Learning rate | **UNKNOWN** (paper unspecified) | `twist_lr = 0.5` (:66) | `params_.twist_lr = 0.5` (:595) | **MATCH** |
| Iterations | paper: within/at most 5 iterations (ABS_PAPER_NOTES:121) | **10** `for _iter in range(10)` (:339) — exceeds paper ≤5, not paper MATCH | **3** `for iter < 3` (:606) — only satisfies the paper upper-bound interpretation | **MISMATCH** (10 vs 3); paper: testbed 10 **not MATCH**, deployment 3 **upper-bound MATCH only** |
| Twist bounds | paper: vx ±1.5, vy ±0.3, wz ±3 (ABS_PAPER_NOTES:110-114) | `twist_min/max = ±[1.5,0.3,3.0]` (:67-68,347) | `±[vx_m,vy_m,wz_m] = ±[1.5,0.3,3.0]` (config :75-79) | **MATCH**; paper **MATCH** (all three identical) |
| Output clamp (type) | **UNKNOWN** (paper unspecified whether component-wise) | component-wise `clip(min,max)` (:347) | component-wise `index_put_ clamp` (:636-638) | **MATCH** |
| Final feasibility check | **UNKNOWN** (paper unspecified) | none | none | **MATCH** (both absent — this is NOT paper MATCH) |
| Infeasible fallback | **UNKNOWN** (paper unspecified) | none | none | **MATCH** (both absent — NOT paper MATCH) |
| Candidate twist → 19-D RA input | **UNKNOWN** (paper does not give the full concatenation) | `cat([vx,vy, lin_vel_z, ang_vel x,y, wz, goal_x,y, rays])` 19-D (:340) | `cat([twist vx,vy, lin_vel_z, ang_vel x,y, twist wz, cmd x,y, rays])` 19-D (:610-617) | **MATCH** |

Eq.21 overall: testbed↔deployment **MISMATCH** on iteration count, gradient
clip, and the Eq.22-displacement-fed goal-deviation term; testbed↔deployment
**MATCH** on objective structure/constants, RA penalty, bounds, init, lr, clamp
type, feasibility/fallback absence, and RA-input concatenation. Paper
relationship is **UNKNOWN** wherever the paper does not specify (constants,
λ/ε, clip method, lr, feasibility, fallback, RA-input concatenation), and
**MATCH** on variables, goal-deviation concept, gradient descent, current-twist
init, ≤5-iteration bound (deployment only), and twist bounds ±[1.5,0.3,3.0].

## 2. Eq.22 — displacement three-way matrix

| Item | Paper Eq.22 | Recovered testbed (get_pos_integral) | Deployment (StateRL.cpp) | testbed ↔ deployment |
|---|---|---|---|---|
| dt | 0.05 s | `tau = 0.05` (:63) | `tau = 0.05` (config :72) | **MATCH** |
| delta_x | `vx*dt - 0.5*vy*wz*dt^2` | `vx*tau - 0.5*vy*wz*tau^2` (:59) | `pos_x = vx*tau` (:623) — second-order term omitted | **MISMATCH** (deployment omits) |
| delta_y | `vy*dt + 0.5*vx*wz*dt^2` | `vy*tau + 0.5*vx*wz*tau^2` (:60) | `pos_y = vy*tau` (:624) — second-order term omitted | **MISMATCH** |
| theta | yaw rotation (implied) | `theta = wz*tau` (:58) | not used in position penalty | **INTENTIONAL_VARIANT / UNKNOWN** (deployment drops theta; not part of pos penalty) |
| coordinate frame | planar x/y | planar body-frame | planar body-frame | **MATCH** |

Eq.22: paper ↔ reference **MATCH**; deployment **MISMATCH** (both yaw-coupled
second-order terms omitted; ABS_PAPER_NOTES:131 records this as a gap). The
first-order goal-penalty consequence (deployment pos_pen uses `vx*tau`/`vy*tau`)
is preserved as **MISMATCH** — the deployment omits the yaw-coupled displacement.

## 3. Deterministic numeric fixture

`scripts/test_p1_06_recovery_optimizer.py` — independent Python arithmetic
oracle (every formula cites the audited source). Result: **14/14 PASS (exit 0)**.
This proves **independent arithmetic only** — it does not execute `StateRL` or
testbed and is not runtime parity.

- Eq.22 nonzero yaw (`vx=1.2, vy=-0.4, wz=2.0, tau=0.05`): reference
  `dx=0.061, dy=-0.017`; deployment `dx=0.060, dy=-0.020` → DIFFER (yaw-coupling
  proven); matches P1-03 recorded fixture.
- Eq.22 zero-yaw: reference == first-order (`dx=0.06, dy=-0.02`).
- Eq.21 objective: `ra=-0.03, goal=(0.5,-0.2), est=(0.06,-0.017)` →
  `ra_pen=0.70`, `pos_pen=0.004542`, `loss=0.704542`; safe `ra=-0.2` → `ra_pen=0`.
- Clamp boundary: `(5,-5,0.1) → (1.5,-0.3,0.1)`.
- Gradient clip: `(3,4)` → L2-norm `(0.6,0.8)` (reference) vs per-element
  `(1,1)` (deployment) → DIFFER.

## 4. Classification summary (REJECT-corrected)

| Difference | Paper relationship | testbed ↔ deployment |
|---|---|---|
| Eq.22 yaw-coupled terms omitted in deployment | **MISMATCH** (vs paper Eq.22) | **MISMATCH** |
| First-order goal-penalty consequence | **MISMATCH** (deployment omits yaw-coupled displacement) | **MISMATCH** |
| Iteration count (10 vs 3) | testbed 10 **not MATCH**; deployment 3 **upper-bound MATCH only** (paper ≤5) | **MISMATCH** |
| Gradient clip (L2 vs per-element) | **UNKNOWN** (paper unspecified) | **MISMATCH** (not an approved stabilized variant) |
| Objective constants / RA λ,ε | **UNKNOWN** (paper unspecified) | **MATCH** |
| Twist bounds ±[1.5,0.3,3.0] | **MATCH** (documented ABS_PAPER_NOTES:110-114) | **MATCH** |
| Output clamp / feasibility / fallback | **UNKNOWN** (paper unspecified) | **MATCH** (both absent/component-wise — NOT paper MATCH) |
| Candidate twist → 19-D RA input | **UNKNOWN** (paper lacks full concatenation) | **MATCH** |
| Init / lr / SGD / variables / RA-penalty structure | init "current twist" + SGD **MATCH**; lr **UNKNOWN** | **MATCH** |

No paper-undefined constant, clip, feasibility, fallback, or full candidate
input construction is described as paper MATCH.

## 5. Remaining UNKNOWN

- Paper exact λ/ε, goal-deviation coefficient, learning rate, gradient-clip
  method, output-clamp type, feasibility/fallback, and full candidate-twist RA
  concatenation (not in local paper materials).
- Whether the deployment 3-iteration / simplified-displacement / per-element
  clip behavior is an approved stabilized variant (no decision record exists).

## 6. Commands run

- `python3 scripts/test_p1_06_recovery_optimizer.py` → **RESULT: PASS (14/14), exit 0**.
- Source reads (Read tool): `testbed.py:55-72,337-348`,
  `StateRL.cpp:577-659`, `ABS_PAPER_NOTES.md:108-131`,
  `docs/evidence/P1-03/formula_parameter_trace.yaml` (ABS-EQ-021/022),
  `abs/config.yaml:70-79`.
- No MuJoCo/ROS2/training/export/benchmark/hardware run.
- `git diff --check` → PASS.

P1-06 status: **IMPLEMENTED / AWAITING INDEPENDENT REVIEW**; not self-accepted.
Phase 1 remains NOT ACCEPTED.
