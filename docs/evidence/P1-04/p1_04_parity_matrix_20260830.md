# P1-04 — Agile 61-D Observation Parity Matrix (2026-08-30)

Oracle: real `LeggedRobotPos.compute_observations()`
(`ABS/training/legged_gym/legged_gym/envs/base/legged_robot_pos.py:217-255`).
Deployment: `StateRL::computeObservation()`
(`quadruped_ros2_control_humble/controllers/rl_quadruped_controller/src/FSM/StateRL.cpp:1109-1118`)
→ `abs_observation::agile()`
(`.../src/FSM/AbsObservationContract.cpp:21-24`).

Element-wise parity is proven by `scripts/test_p1_01_local_contract.py`
(Agile61 PASS, mismatches=[]; RA19 PASS; Recovery49 PASS; semantic helpers PASS;
exit 0) using the compiled production adapter
`build/rl_quadruped_controller/p1_01_abs_observation_adapter`. The DEC-010
declared policy order (`FL,FR,RL,RR` → remap → controller/MuJoCo `FR,FL,RR,RL`)
is used only as the declared operational mapping.

## Global post-assembly clamp [0:61] — KNOWN ISSUE (declared-equivalent limit)

After the 61 slots are assembled, deployment applies
`torch::clamp(obs, -clip_obs, clip_obs)` with `clip_obs=100`
(`AbsObservationContract.cpp:14-16,21-28`). The recovered training Go2
configuration declares `clip_observations = 100.`
(`legged_robot_pos_config.py:95`). The parity fixture values are within this
range, so the clamp is a no-op for the element-wise oracle result.

Independent review confirmed that the training environment
`LeggedRobot.step()` applies `torch.clip` to `obs_buf` before returning the
observation. The fixture remains inside this range, so it still does not prove
the behavior of every downstream policy-library call outside the tested domain.
This is recorded as a **KNOWN ISSUE / declared-equivalent limit**, not as a
value-level `MATCH` outside the tested domain and not as an unclassified
mismatch. It applies uniformly to slots `[0:61]` and introduces no slot shift,
permutation, default value, or non-finite-value masking.

## Block-by-block matrix

### contact [0:4] — MATCH
| Aspect | Training | Deployment |
|---|---|---|
| Source | `contact_forces[:, feet_indices, 2] > 1.` then `contact_filt = logical_or(contact, last_contacts)` (`legged_robot_pos.py:135-137`) | `foot_force > abs_contact_threshold` then `temporalContact(current, last_contacts_)` (`StateRL.cpp:1535-1541`) |
| Transform | `contact_filt.float()*2 - 1.0` (`legged_robot_pos.py:232`) | `temporalContact` = `logical_or(current,previous).float()*2-1` (`AbsObservationContract.cpp:39-43`) then `controllerToPolicyContact` `{1,0,3,2}` (`AbsObservationContract.cpp:20,22`) |
| Frame | vertical contact force, threshold 1 N | vertical foot-force, threshold `abs_contact_threshold` (config = 1.0) |
| Unit/scale | ±1 (no scale) | ±1 (no scale) |
| Order | training `FL,FR,RL,RR` | controller `FR,FL,RR,RL` → remap `{1,0,3,2}` → policy `FL,FR,RL,RR` |
| Validity | assumes fresh forces each step | fail-closed only at the real-source boundary (Phase 2 hardware-only for real `foot_force` slots) |
| Evidence | test fixture contact + `contact_touch/liftoff/next` helpers PASS | same |

Real Go2 `foot_force[0..3]` slot semantics are Phase 2 hardware-only (DEC-010);
this does not affect the observation transform.

### angular velocity [4:7] — MATCH
| Aspect | Training | Deployment |
|---|---|---|
| Source/transform | `base_ang_vel * obs_scales.ang_vel` (`legged_robot_pos.py:233`) | `in.ang_vel * s.ang_vel` (`AbsObservationContract.cpp:22`); `in.ang_vel` = IMU gyroscope (`StateRL.cpp:1363`) |
| Frame | body | body (IMU gyro) |
| Unit/scale | rad/s × `obs_scales.ang_vel` | rad/s × `params_.ang_vel_scale` (config) |
| Order | n/a (3-vector) | n/a |
| Validity | n/a | finite-checked at the observation boundary |
| Evidence | test fixture `ang=[.11,-.22,.37]` matches element-wise | same |

### gravity [7:10] — MATCH
| Aspect | Training | Deployment |
|---|---|---|
| Source/transform | `projected_gravity` (`legged_robot_pos.py:234`) | `quatRotateInverse(obs_.base_quat, [0,0,-1])` (`StateRL.cpp:1113`) |
| Frame | body | body |
| Unit/scale | world `[0,0,-1]` in body frame | same |
| Order | n/a | n/a |
| Validity | n/a | finite-checked |
| Evidence | test fixture `gravity=[.41,-.52,-.73]` matches | same |

### goal / command [10:13] — INTENTIONAL_VARIANT (upstream goal shaping)
| Aspect | Training | Deployment |
|---|---|---|
| Source/transform | `commands[:, :3]`; `commands[:,:2] = quat_rotate_inverse(yaw_quat(base_quat), pos_diff)[:,:2]`; `commands[:,2] = wrap_to_pi(heading_target - heading)` (`legged_robot_pos.py:128-133,166-169,235`) — **raw body-frame relative x/y + heading error, no scaling** | `obs_.commands = {body_x, body_y, heading_cmd}` where `body_x,body_y = world-goal→body-frame` then **radially scaled `min(1, 5/(dist+0.01))`**, optional path-tracking lateral correction, arrival stand-still (`dist<0.5`→0), joystick trim; `heading_cmd = atan2(body_y,body_x)` (`StateRL.cpp:1448-1471,1522`) |
| Frame | body | body |
| Unit/scale | meters × 1 + radians | meters × `min(1,5/(dist+0.01))` (compressed) + radians |
| Order | n/a (3-vector) | n/a |
| Validity | assumes fresh commands | n/a |
| Evidence | `legged_robot_pos.py:166-169` (no scaling) | `StateRL.cpp:1455-1462` (`scale = min(1, 5/(dist+0.01))`), `1464-1467` (path correction), `1474-1480` (arrival stand-still) |

Exact code path of the difference: `StateRL.cpp:1458-1462` applies radial
distance compression; training `legged_robot_pos.py:166-169` does not. This is
the documented P1-01F "goal shaping" **INTENTIONAL ENGINEERING VARIANT** — not
parity. The existing test feeds the same command value to both sides, so the
assembly-level transform (identity concatenation at [10:13]) matches; the
**upstream** command computation is the variant.

### timer [13:14] — INTENTIONAL_VARIANT (rolling timer, no physical reset)
| Aspect | Training | Deployment |
|---|---|---|
| Source/transform | `timer_left / max_episode_length_s` (`legged_robot_pos.py:236`); `timer_left` starts at `episode_length_s`, decrements by `dt`, resets per episode to `episode_length_s - randomize_timer_minus*rand` (`:59-60,93,126`) — **linear 1→0 per episode** | `rollingTimeLeftNormalized(elapsed, horizon) = 1 - fmod(max(0,elapsed),horizon)/horizon` (`AbsObservationContract.cpp:34-38`), via `normalizedTimer()` (`StateRL.cpp:678-682`; `legacy_fixed`→0.5) — **rolling 1→0→1, no physical reset** |
| Frame | episode time | continuous elapsed time |
| Unit/scale | normalized 0..1 | normalized 0..1 |
| Order | n/a (1 scalar) | n/a |
| Validity | per-episode reset | continuous fmod roll |
| Evidence | `legged_robot_pos.py:236,59-60,93` | `StateRL.cpp:678-682`; `AbsObservationContract.cpp:34-38`; adapter helper `timer:[1.0,.5,1.0]` |

Values coincide at horizon start (1.0) and mid-run (remaining fraction); they
differ at the end-of-horizon boundary (deployment rolls back to 1.0; training
would reach 0.0 before an episode reset). This is the documented P1-01F "no
physical reset" **INTENTIONAL VARIANT** — not parity at the reset boundary.

### joint position [14:26] — MATCH (nominal); bias distribution INTENTIONAL_VARIANT / UNKNOWN
| Aspect | Training | Deployment |
|---|---|---|
| Source/transform | `(dof_pos - default_dof_pos - dof_bias) * obs_scales.dof_pos` (`legged_robot_pos.py:237`); `dof_bias` **randomized ±0.08** (`domain_rand.max_dof_bias`, `:79-80`) | `(controllerToPolicyDof(dof_pos) - controllerToPolicyDof(default_dof_pos) - in.dof_bias) * s.dof_pos` (`AbsObservationContract.cpp:11-13,23`); `in.dof_bias` = config nominal (zero default), remapped (`StateRL.cpp:1115`) |
| Frame | joint position (rad) | joint position (rad) |
| Unit/scale | rad × `obs_scales.dof_pos` | rad × `params_.dof_pos_scale` |
| Order | training `FL,FR,RL,RR` | controller `FR,FL,RR,RL` → remap `{3,4,5,0,1,2,9,10,11,6,7,8}` → policy `FL,FR,RL,RR` |
| Validity | randomized bias per episode (unobservable distribution) | nominal zero bias |
| Evidence | test fixture `bias=[0]*12` matches element-wise | same |

The transform is **MATCH** at the nominal-bias contract (element-wise equal).
The **training bias distribution** (±0.08 random per episode, `legged_robot_pos.py:79-80`) is not reproduced in deployment (explicit nominal zero): this is an
**INTENTIONAL_VARIANT** (nominal calibration) and the exact per-sample training
bias is **UNKNOWN** (unobservable random distribution) — it is not filled with a
default.

### joint velocity [26:38] — MATCH
| Aspect | Training | Deployment |
|---|---|---|
| Source/transform | `dof_vel * obs_scales.dof_vel` (`legged_robot_pos.py:238`) | `controllerToPolicyDof(in.dof_vel, order) * s.dof_vel` (`AbsObservationContract.cpp:23`) |
| Frame | joint velocity (rad/s) | joint velocity (rad/s) |
| Unit/scale | rad/s × `obs_scales.dof_vel` (0.2) | rad/s × `params_.dof_vel_scale` (0.2) |
| Order | training `FL,FR,RL,RR` | controller → remap `{3,4,5,0,1,2,9,10,11,6,7,8}` → policy `FL,FR,RL,RR` |
| Validity | n/a | finite-checked |
| Evidence | test fixture `dof_vel` asymmetric matches | same |

### previous action [38:50] — MATCH
| Aspect | Training | Deployment |
|---|---|---|
| Source/transform | `actions` (`legged_robot_pos.py:239`) | `controllerToPolicyDof(in.actions, order)` (`AbsObservationContract.cpp:23`) |
| Frame | previous raw actions | previous raw actions |
| Unit/scale | raw (action units) | raw |
| Order | training `FL,FR,RL,RR` | controller → remap `{3,4,5,0,1,2,9,10,11,6,7,8}` → policy `FL,FR,RL,RR` |
| Validity | n/a | n/a |
| Evidence | test fixture `actions` asymmetric matches | same |

### ray2d [50:61] — MATCH (value); validity INTENTIONAL_VARIANT (fail-closed)
| Aspect | Training | Deployment |
|---|---|---|
| Source/transform | `log2(ray2d_obs) * obs_scales.ray2d` (`legged_robot_pos.py:247-251`); `ray2d_obs` = 11 fresh distances (log2 applied) | `in.ray2d` = 11 log2 distances from MuJoCo shm (`StateRL.cpp:732`); no extra transform (`AbsObservationContract.cpp:23`) |
| Frame | planar 2-D rays | planar 2-D rays (MuJoCo) |
| Unit/scale | log2(distance) × `obs_scales.ray2d` | log2(distance) (already log2 in shm) |
| Order | 11 ordered rays | 11 ordered rays (same geometry) |
| Validity | assumes fresh valid rays each step | **fail-closed**: `rayFrameValid` rejects missing/stale/NaN/Inf (`StateRL.cpp:684-748`; `AbsObservationContract.cpp:44-50`); invalid → safety veto / stale handling |
| Evidence | test fixture `ray` (log2 values) matches element-wise | adapter helper `ray_valid:[1,0,0,0]` |

The **value transform** is MATCH (log2, element-wise). The **validity behavior**
is an **INTENTIONAL_VARIANT**: deployment fails closed on missing/stale/
non-finite rays (P1-01F fail-closed contract), while training assumes
freshly-generated valid rays each step.

## Summary

| Block | Classification |
|---|---|
| contact [0:4] | MATCH |
| angular velocity [4:7] | MATCH |
| gravity [7:10] | MATCH |
| goal / command [10:13] | INTENTIONAL_VARIANT (upstream goal shaping) |
| timer [13:14] | INTENTIONAL_VARIANT (rolling timer, no physical reset) |
| joint position [14:26] | MATCH (nominal); bias distribution INTENTIONAL_VARIANT / UNKNOWN |
| joint velocity [26:38] | MATCH |
| previous action [38:50] | MATCH |
| ray2d [50:61] | MATCH (value); validity INTENTIONAL_VARIANT (fail-closed) |

No MISMATCH. No unclassified index shift, joint permutation, default-filled
value, or silent non-finite value remains.

## Evidence

- `scripts/test_p1_01_local_contract.py` + compiled production adapter
  `build/rl_quadruped_controller/p1_01_abs_observation_adapter` (Agile61/RA19/
  Recovery49 PASS; semantic helpers PASS).
- Source: `legged_robot_pos.py:217-255`, `StateRL.cpp:1109-1118,1361-1541`,
  `AbsObservationContract.cpp:11-50`, `test_p1_01_local_contract.py`.
- Machine-readable: `p1_04_parity_matrix_20260830.json`.

Independent Reviewer disposition (2026-08-31): **ACCEPT WITH KNOWN ISSUES**.
See [`REVIEW_2026-08-31_PARITY.md`](REVIEW_2026-08-31_PARITY.md). Phase 1
remains NOT ACCEPTED.
