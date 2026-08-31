# P1-05 — RA Label, Model Semantics, and Agile Operational Binding (2026-08-30)

Offline evidence audit. No code/config/model/threshold/state-machine change; no
MuJoCo/ROS2/training/export/benchmark/hardware. Both training evidence roots
searched: `ABS/` and `ABS_fuwuqi/ABS/` (their `testbed.py` RA-label lines
197–570 are byte-identical). DEC-010 boundary: RA ↔ Agile `model_4000.pt` is
**OPERATOR_DECLARED** operational binding only; historical seed/command/Git/
dataset are deferred reproducibility, not blockers.

## 1. Recovered RA training label (testbed.py, server snapshot)

All expressions from
`ABS_fuwuqi/ABS/training/legged_gym/legged_gym/scripts/testbed.py` (identical in
`ABS/`).

### RA observation (19-D) — testbed.py:498
```python
ra_obs = torch.cat([env.base_lin_vel, env.base_ang_vel, obs[:,10:12], obs[:,-11:]], dim=-1)
```
= `[base_lin_vel(3), base_ang_vel(3), goal x/y(2), log rays(11)]` = 19. Matches
paper Eq.14 order (ABS_PAPER_NOTES).

### Label signals — testbed.py:501-503
```python
# ls <= 0: reach target; gs > 0: failure
gs = collision.float() * 2 - 1                       # +1 collision, -1 not
ls = torch.tanh(torch.log2(torch.norm(obs[:,10:12], dim=-1) / 0.65 + 1e-8))
```
- `g` (avoid) = `+1` on collision, `-1` otherwise.
- `l` (reach) = `tanh(log2(‖goal_xy‖ / 0.65 + 1e-8))` (base-2 log; `‖goal_xy‖`
  from the Agile obs command x/y; threshold `0.65` m; `1e-8` numerical epsilon).

### Collision definition — testbed.py:362-371
```python
collision = any(norm(contact_forces[termination_contact_indices], dim=-1) > 1.)   # base contact
hor_footforce = norm(contact_forces[feet_indices[:2], 0:2])                        # front-feet horizontal
ver_footforce = abs(contact_forces[feet_indices[:2], 2])                           # front-feet vertical
foot_hor_col = any(hor_footforce > 2 * ver_footforce + 10.0)                       # horizontal-shear heuristic
collision = collision OR foot_hor_col
_near_obj = (any(minobjdist < 0.95) AND base_twist_norm > 0.5) OR (contact_forces[0] > 1.)
collision = collision AND _near_obj                                                # filter simulator noise
```
Collision = (base contact `>1 N` OR front-foot horizontal-shear `> 2·vertical + 10 N`) AND near-object
filter (`<0.95 m` at `>0.5 m/s`, or base contact).

### Bellman / hindsight — testbed.py:513-545
```python
gamma = 0.999999
hindsight = 10
# collision softened over the last 10 timesteps before a gs>0 terminal:
new_values = gs; new_values -= (span[1]-t) * 2/hindsight   # -> g_soft = 1 - 2*d/10, d=1..10
vs_new = ra_vf(s[t+1]) * (~done[t+1]) + 1.0 * done[t+1]     # terminal bootstrap = 1.0
vs_discounted = gamma * max(g_soft[t+1], min(l[t], vs_new)) + (1-gamma) * max(l[t], g_soft[t+1])
v_loss = 100 * mean((vs_old - vs_discounted)**2)             # weight_end=0 (all samples equal)
```

Hand-computable form of the Bellman target:
`V*(s_t) = γ·max(g̃(s_{t+1}), min(l(s_t), V*(s_{t+1}))) + (1−γ)·max(l(s_t), g̃(s_{t+1}))`,
`γ = 0.999999`, `l = tanh(log2(d/0.65 + 1e-8))`, `g̃ = +1·collision / −1`, softened to
`1 − 2·d/10` over the 10 pre-terminal steps, terminal bootstrap `V=1.0` on `done`,
loss scaled ×100.

### Paper comparison (ABS_PAPER_NOTES Eq.16–18)

| Item | Paper (Eq.16–18) | Recovered training (testbed.py) | Classification |
|---|---|---|---|
| Bellman regression | discounted reach-avoid, Eq.16–17, `γ=0.999999` | `V' = γ·max(g̃',min(l,V_next)) + (1−γ)·max(l,g̃')`, `γ=0.999999` | **MATCH** (structural, γ identical) |
| Collision softening | softened over the last ten timesteps | linear soften `1 − 2·d/10` over 10 steps | **MATCH** |
| Reach signal | Eq.18 `tanh(log(d_goal / sigma_tight))` | `tanh(log2(d/0.65 + 1e-8))` — base-2 log, `σ=0.65`, `+1e-8` | **INTENTIONAL_VARIANT** (log2 base + `0.65` + `1e-8`; paper's exact base/σ value not recorded locally → also **UNKNOWN** for the exact paper constants) |
| Collision definition | base/body contact (paper termination) | base contact `>1N` **plus** front-foot horizontal-shear heuristic `>2·vert+10N` **plus** near-object filter | **INTENTIONAL_VARIANT** (extra training heuristics) |
| Terminal bootstrap | paper terminal value not recorded locally | `V=1.0` on any `done` (collision/reach/timeout) | **INTENTIONAL_VARIANT / UNKNOWN** |
| Loss scale | paper loss scale not recorded | ×100 | **INTENTIONAL_VARIANT / UNKNOWN** |

No MISMATCH: no value/operator that directly contradicts the recorded paper
spec.

## 2. Deployed RA 19-D input matrix

Deployment: `StateRL::computeRAObservation()`
(`StateRL.cpp:1120-1127`) → `abs_observation::ra()`
(`AbsObservationContract.cpp:29-32`):

```cpp
ra = cat({ in.lin_vel, in.ang_vel, in.commands.index({.., Slice(0,2)}), in.ray2d }, 1);
```

| Slot | Field | Training source | Deployment source | Frame | Unit/scale | Order | Classification |
|---|---|---|---|---|---|---|---|
| 0:3 | base lin vel | `env.base_lin_vel` (body) | `obs_.lin_vel = quatRotateInverse(base_quat, odom_vel_world)` (`StateRL.cpp:1380`) | body | m/s × 1 | n/a | **MATCH** (both body-frame linear velocity; source differs odom-vs-env) |
| 3:6 | base ang vel | `env.base_ang_vel` (body) | `obs_.ang_vel` = IMU gyro (`StateRL.cpp:1363`) | body | rad/s × 1 | n/a | **MATCH** |
| 6:8 | goal x/y | `obs[:,10:12]` = Agile commands[0:2] (raw body-frame relative) | `obs_.commands[0:2]` = **shaped** body_x/body_y (radial `min(1,5/(d+0.01))`, path correction, arrival stand-still) (`StateRL.cpp:1455-1467,1522`) | body | m (compressed) | n/a | **INTENTIONAL_VARIANT** (same upstream goal shaping as Agile [10:13]; P1-01F documented) |
| 8:19 | rays | `obs[:,-11:]` = 11 log2 rays | `obs_.ray2d` = 11 log2 from MuJoCo shm, fail-closed (`StateRL.cpp:732`, `AbsObservationContract.cpp:44-50`) | planar 2-D | log2(distance) | 11 ordered | **MATCH** (value) + **INTENTIONAL_VARIANT** (fail-closed validity) |

Total 19 slots; order matches paper Eq.14 and the training `ra_obs`.

## 3. Deployed RA output and StateRL switching

- **Shape**: `ra_value.pt` is `19 → [64,64,1] → Tanh` (P1-01 verified; arch
  `[19,64,64,1]`). Input 19, output scalar.
- **Range / direction**: Tanh output ∈ `(−1, +1)`. Training checkpoint-save gate
  requires `die_v > 0.2` (high risk at the die standard obs), `start_v < −0.1`,
  `turn_v < −0.1` (testbed.py:561) → **higher RA value = higher risk (collision-prone); lower = safer**. The policy is conditioned on the Agile policy
  (`OPERATOR_DECLARED` `model_4000.pt`).
- **StateRL comparisons** (`StateRL.cpp:1557-1592`):
  ```cpp
  ra_entry_thr = params_.ra_threshold;            // -0.05 (config ra_threshold)
  ra_exit_thr  = params_.ra_threshold - 0.03;     // -0.08 (hysteresis margin)
  if (!in_recovery_ && ra_value_ > ra_entry_thr)  { in_recovery_=true; rec_hold_left_=REC_HOLD_STEPS; computeRecoveryTwist(); }
  else if (in_recovery_) { rec_hold_left_--; if (rec_hold_left_ <= 0 && ra_value_ < ra_exit_thr) in_recovery_=false; }
  ```
  - ENTER: `ra_value_ > −0.05` (strict `>`, no `>=`).
  - EXIT: `ra_value_ < −0.08` **and** the forced hold (`REC_HOLD_STEPS =
    recovery_hold_steps_`, config = 30) has expired.
- **Paper vs recovered-testbed vs deployed switching**:
  - Paper (ABS_PAPER_NOTES): `RA < −0.05 → Agile`; `RA >= −0.05 → optimize safe
    twist → Recovery`. No `−0.08` exit and no 30-step hold are defined.
  - Recovered testbed (`testbed.py:64,324`, both `ABS/` and `ABS_fuwuqi/ABS/`):
    `twist_eps = 0.05`; `recovery = (v_pred > -twist_eps).squeeze(-1)` i.e. strict
    `v_pred > −0.05` → recovery. **Immediate test/testbed branch**: no `−0.08`
    exit hysteresis and no 30-step forced hold.
  - Current stabilized deployment: ENTER `> −0.05` (strict; paper `>=` — a minor
    operator variant at exactly `−0.05`); EXIT `< −0.08` after a 30-step forced
    hold (stabilized variant; the paper does not define them).
  - Classification: the strict `>` entry (shared by recovered testbed and
    deployment, vs paper `>=`) and the deployment `−0.08` exit hysteresis +
    30-step hold are **INTENTIONAL_VARIANT** — recorded, not changed; paper
    equivalence is not claimed.

## 4. Three-way comparison summary

| Item | Paper | Recovered training | Deployed | Classification |
|---|---|---|---|---|
| RA obs 19-D order | Eq.14 (lin vel, ang vel, goal, rays) | `cat(lin_vel, ang_vel, obs[10:12], rays)` | `cat(lin_vel, ang_vel, commands[0:2], ray2d)` | **MATCH** (order/frame) |
| Reach label | Eq.18 `tanh(log(d/σ))` | `tanh(log2(d/0.65+1e-8))` | n/a (label is training-only) | **INTENTIONAL_VARIANT / UNKNOWN** (log base/σ) |
| Bellman | Eq.16–17 `γ=0.999999`, soft collision | `γ=0.999999`, `max(g̃,min(l,V))` operator, soft-10 | n/a | **MATCH** |
| Collision | base contact | base + front-foot shear + near-obj filter | n/a | **INTENTIONAL_VARIANT** |
| Goal input | raw body rel x/y | raw | **shaped** | **INTENTIONAL_VARIANT** (goal shaping) |
| Switching | `< −0.05`/`>= −0.05`, no hold | `recovery = (v_pred > -twist_eps) = (v_pred > −0.05)`, strict `>`, immediate, no `−0.08` exit, no 30-step hold (`testbed.py:64,324`) | `> −0.05` enter; `< −0.08` + 30-step hold exit | **INTENTIONAL_VARIANT** (strict `>` vs paper `>=`; deployment `−0.08` exit hysteresis + 30-step hold not in paper/testbed) |
| RA output range | ~(−1,1) | Tanh (−1,1) | Tanh (−1,1) | **MATCH** |
| RA ↔ Agile binding | — | `OPERATOR_DECLARED` `model_4000.pt` | weight chain CONFIRMED; execution binding deferred | **OPERATOR_DECLARED / UNKNOWN (deferred)** |

No MISMATCH. No unclassified index shift / threshold / default.

## 5. Commands run

- `diff` of `testbed.py:197-570` between `ABS/` and `ABS_fuwuqi/ABS/` →
  **IDENTICAL** (RA label code consistent across both evidence roots).
- Source reads (Read tool): `testbed.py`, `ABS_PAPER_NOTES.md`,
  `StateRL.cpp:1120-1127,1361-1541,1557-1592`, `AbsObservationContract.cpp:29-50`.
- No MuJoCo/ROS2/training/export/benchmark/hardware run.
- `git diff --check` → PASS.

## 6. Remaining UNKNOWN / deferred

- Paper Eq.18 exact log base, `σ_tight` value, loss scale, terminal bootstrap:
  not recorded locally → UNKNOWN (training uses log2, 0.65, ×100, 1.0).
- RA training dataset/seed/historical execution: **deferred reproducibility**
  (DEC-010), not a P1-05 blocker.
- RA ↔ Agile exact executed binding: `OPERATOR_DECLARED` only.

P1-05 status: **IMPLEMENTED / AWAITING INDEPENDENT REVIEW**; not self-accepted.
