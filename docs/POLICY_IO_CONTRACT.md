# ABS Go2 Policy I/O Contract

Task baseline: P1-01F, 2026-08-24. Status: **PARTIAL — local tensor assembly and corrected deployment contract are proven; deployed artifact lineage remains UNKNOWN (training server unavailable)**.

This document separates four evidence levels:

1. `RUNTIME-CAPTURED`: returned by Isaac Gym or logged by the running ROS 2 stack;
2. `SOURCE-VERIFIED`: traced through executable code/configuration;
3. `CONDITIONAL`: correct only if the deployed artifact is proven to descend from the captured training environment;
4. `UNKNOWN`: evidence required by the contract is unavailable.

Names, comments and visually plausible locomotion are not order evidence.

## Deployed Artifact Contract

| Artifact | Immutable deployed identity | Executable I/O | Source/export evidence | Provenance status |
|---|---|---|---|---|
| Agile | `5a87d6…e0b7cf`, 801,726 B | `61 → 512 → 256 → 128 → 12` | Archive root `05_27_15-53-31_model_4000`; generic export exists in `helpers.py` | Source checkpoint, seed, training commit and exact export are **UNKNOWN** |
| RA | `05c40f…a90b7`, 32,011 B | `19 → 64 → 64 → 1 → Tanh` | `testbed.py` names RA from the loaded Agile run; converter defaults to `05_27_15-53-31_model_4000_ra.pt` | Dataset, source hash, seed and exact Agile binding are **UNKNOWN** |
| Recovery | `e3047a…b0171`, 775,715 B | `49 → 512 → 256 → 128 → 12` | Export script documents `06_04_22-43-20_/model_15000.pt`; current binary and script entered together in commit `84beae2` | Source checkpoint/hash, seed, training commit and exact export are **UNKNOWN** |

The installed deployment paths are symlinks to these tracked source-tree artifacts, and their hashes match. Full fields are in [`artifacts/manifest.yaml`](../artifacts/manifest.yaml); deterministic model outputs are in [`p1_01_contract.json`](../artifacts/p1_01_contract.json).

The three artifacts are runtime-bound by `StateRL`: Agile and RA load from `config/abs`, and Recovery loads from `config/rec`. That proves what runs together, not that they were trained together.

## Authoritative Training Runtime Order

The Go2 asset was loaded through the Isaac Gym runtime API with the same asset options used by `LeggedRobot`. The preserved capture is [`isaac_gym_asset_order.json`](evidence/P1-01/isaac_gym_asset_order.json).

`RUNTIME-CAPTURED` DOF order:

```text
0 FL_hip    1 FL_thigh    2 FL_calf
3 FR_hip    4 FR_thigh    5 FR_calf
6 RL_hip    7 RL_thigh    8 RL_calf
9 RR_hip   10 RR_thigh   11 RR_calf
```

`RUNTIME-CAPTURED` feet order and rigid-body indices:

```text
FL_foot(4), FR_foot(8), RL_foot(12), RR_foot(16)
```

The training action vector directly indexes this DOF vector. Training contacts are read through these feet indices. Therefore the current Go2 training implementation is FL, FR, RL, RR for actions, joint observations and contacts.

This does **not** by itself prove that either deployed policy artifact was exported from that implementation. Deployed Agile and Recovery order remains `UNKNOWN` until source checkpoint/export lineage or equivalent independent model evidence is recovered.

The same runtime capture found no rigid-body name containing `base`, so `terminate_after_contacts_on = ["base"]` creates an empty termination-contact list for this asset. This is an observed gap, not an accepted intended behavior.

## ROS 2, Unitree and MuJoCo Order

The controller order is `RUNTIME-CAPTURED` and `SOURCE-VERIFIED`:

```text
FR hip/thigh/calf, FL hip/thigh/calf,
RR hip/thigh/calf, RL hip/thigh/calf
```

`RlQuadrupedController` sorts all joint interfaces by this YAML order. A legacy runtime log captured the complete identity motor map at `logs/abs_eval/20260709_140058/scene_obstacle/run_001/runtime.log` (SHA-256 `12f9db…f694`) and the current source retains the same logic. The mapping excerpt and source references are preserved in [`ros2_motor_map.json`](evidence/P1-01/ros2_motor_map.json); the raw log remains non-Acceptance data.

Current controller index to final actuator mapping:

| Controller index | Joint | Unitree motor | MuJoCo actuator |
|---:|---|---:|---|
| 0 | FR hip | 0 | FR_hip |
| 1 | FR thigh | 1 | FR_thigh |
| 2 | FR calf | 2 | FR_calf |
| 3 | FL hip | 3 | FL_hip |
| 4 | FL thigh | 4 | FL_thigh |
| 5 | FL calf | 5 | FL_calf |
| 6 | RR hip | 6 | RR_hip |
| 7 | RR thigh | 7 | RR_thigh |
| 8 | RR calf | 8 | RR_calf |
| 9 | RL hip | 9 | RL_hip |
| 10 | RL thigh | 10 | RL_thigh |
| 11 | RL calf | 11 | RL_calf |

No software sign inversion occurs between controller joint targets and these motor slots.

## Current Remap Contract

The deployed configuration selects `policy_joint_order: ros1_fl_fr_rl_rr`. Both Agile/inline-Recovery and manual Recovery use:

```text
DOF index_select = [3,4,5, 0,1,2, 9,10,11, 6,7,8]
contact index_select = [1,0,3,2]
```

The DOF permutation is its own inverse and is bijective. It maps the captured training candidate order FL,FR,RL,RR to the proven controller order FR,FL,RR,RL.

Conclusion:

- remap mathematics and implementation direction: **PASS**;
- remap required if deployed policies descend from the captured Go2 training environment: **PASS (CONDITIONAL)**;
- remap necessity/correctness for the actual deployed Agile and Recovery files: **UNKNOWN**, because artifact provenance is not closed.

Removing or changing the remap is therefore prohibited until the remaining evidence is recovered.

## Action-to-Motor Trace

Under the conditional FL-first policy order, every policy index has one destination:

| Policy index | Candidate policy joint | Controller/motor index |
|---:|---|---:|
| 0 | FL hip | 3 |
| 1 | FL thigh | 4 |
| 2 | FL calf | 5 |
| 3 | FR hip | 0 |
| 4 | FR thigh | 1 |
| 5 | FR calf | 2 |
| 6 | RL hip | 9 |
| 7 | RL thigh | 10 |
| 8 | RL calf | 11 |
| 9 | RR hip | 6 |
| 10 | RR thigh | 7 |
| 11 | RR calf | 8 |

Agile and inline Recovery apply:

```text
policy output
→ conditional policy-to-controller permutation
→ hip scale (currently 1.0)
→ action clamp (currently ±4)
→ q_offset = 0.25 * action
→ q_target = q_default + q_offset
→ Go2 position-limit clamp
→ controller joint command
→ identity Unitree/MuJoCo motor slot
```

Manual `StateRLRec` uses the same permutation, scale and default pose, but currently lacks the inline path's final ±4 action clamp and Go2 position-limit clamp. This difference is recorded only; P1-01 does not change it.

## Contact Trace

MuJoCo touch sensors, DDS slots and controller contacts are `FR, FL, RR, RL`. The current candidate-policy mapping is:

| Candidate policy contact | Controller/DDS index | MuJoCo sensor |
|---|---:|---|
| FL | 1 | FL_touch |
| FR | 0 | FR_touch |
| RL | 3 | RL_touch |
| RR | 2 | RR_touch |

Simulation contact mapping is `SOURCE-VERIFIED`. Two limitations remain:

- foot-force interfaces are not explicitly sorted or runtime-asserted like joint interfaces;
- the local Unitree IDL does not label real Go2 `foot_force[0..3]` semantics.

Therefore real-robot contact order is `UNKNOWN` and must be captured asymmetrically before Phase 2 RL.

## Observation Contracts

All three deployed TorchScripts reject a wrong last dimension through their first linear layer. Exact fields are frozen in [`p1_01_contract.json`](../artifacts/p1_01_contract.json).

### Agile 61

| Slice | Field | Training contract | Current ROS 2 contract | Status |
|---|---|---|---|---|
| 0:4 | contact | FL,FR,RL,RR; ±1; vertical force `>1`, current OR previous | Same threshold and current OR prior policy-cycle contact, then conditional remap | PASS in MuJoCo/controller order; real Go2 slot order UNKNOWN |
| 4:7 | angular velocity | body frame, rad/s, scale 1 | IMU gyro assumed body frame, scale 1 | Frame validity not asserted at runtime |
| 7:10 | gravity | world `[0,0,-1]` projected into body frame | same quaternion inverse operation | Static parity |
| 10:13 | goal command | body-frame relative x/y/heading, scale 1 | body-frame goal with additional path/radial shaping | Semantic mismatch already tracked |
| 13:14 | timer | remaining time divided by 9 s | default `paper_faithful_rolling`: `1-fmod(elapsed,9)/9`; explicit `legacy_fixed` mode remains available | rolling phase is defined; no physical reset is performed |
| 14:26 | joint position | `q - q_default - dof_bias`, candidate FL-first, scale 1 | explicit configured nominal controller-order bias, remapped before subtraction | nominal contract PASS; random training sample remains unobservable distribution gap |
| 26:38 | joint velocity | candidate FL-first, scale 0.2 | conditional remap, scale 0.2 | Order conditional |
| 38:50 | previous action | candidate FL-first raw previous action | controller-stored action remapped back conditionally | Round-trip passes conditionally |
| 50:61 | rays | 11 × `log2(m)`, 0.1–6 m, −45°…+45° | MuJoCo shared memory already contains `log2(m)` | Missing/stale source is not validly distinguished |

Training adds noise before inference; deployment does not. Both sides clamp/limit observations according to their configured path, but source validity is not equivalent.

### RA 19

| Slice | Field | Runtime source and scale | Validity |
|---|---|---|---|
| 0:3 | body linear velocity | odometry/estimator transformed into body frame, scale 1 | source freshness not part of tensor contract |
| 3:6 | angular velocity | IMU gyro, scale 1 | frame/freshness not asserted |
| 6:8 | goal x/y | same current Agile command x/y | includes current shaping deviations |
| 8:19 | rays | same 11 log2 rays | stale/missing identity unavailable |

The model shape is proven. Exact RA dataset normalization and binding to the deployed Agile model remain `UNKNOWN`.

### Recovery 49

| Slice | Field | Training/current runtime contract | Status |
|---|---|---|---|
| 0:4 | contact | same candidate contact contract as Agile | Same temporal filter; real Go2 slot order remains UNKNOWN |
| 4:7 | angular velocity | body gyro, scale 1 | Layout known; frame/freshness/finite validity not asserted |
| 7:10 | gravity | projected gravity | Layout known; quaternion/finite validity not asserted |
| 10:13 | safe twist | `[vx, vy, wz]`, m/s,m/s,rad/s, scale 1 | Static layout; solver correctness is out of P1-01 |
| 13:25 | joint position | training uses `q - q_default - dof_bias`; deployment uses explicit nominal bias, scale 1 | nominal contract PASS; randomized training sample remains unobservable |
| 25:37 | joint velocity | candidate policy order, scale 0.2 | Conditional order |
| 37:49 | previous action | candidate policy order | Conditional round-trip |

Recovery intentionally has no ray input.

## P1-01 Contract Result

### Local golden parity — 2026-08-24

`scripts/test_p1_01_local_contract.py` calls the real training methods `LeggedRobotPos.compute_observations()` and `LeggedRobotRec.compute_observations()` and compares their output element-by-element with `p1_01_abs_observation_adapter`. The adapter calls `AbsObservationContract`, which is also used by production `StateRL`/`StateRLRec`. The fixture is deliberately asymmetric in every leg, joint, velocity, action, ray and command. Result: **Agile 61 PASS; RA 19 PASS; Recovery 49 PASS; 0 mismatching indices**.

The pass is conditional on semantically equivalent upstream values and does not erase these observed differences:

| Topic | Paper/training | Current ROS2 runtime | Result |
|---|---|---|---|
| Timer | `timer_left / 9`, decremented each simulation step | default rolling time-left phase; named `legacy_fixed` compatibility mode | **PASS (deployment contract)**; no physical reset remains explicit |
| Goal | raw body-frame position target/heading command | radial distance compression plus optional lateral/heading path shaping before both Agile and RA | **INTENTIONAL ENGINEERING VARIANT**; paper-faithful status is not established |
| Contact | vertical force `>1`, `current OR previous-frame` | same temporal filter in controller order before policy remap | **PASS** in simulation/controller mapping; real Go2 slots UNKNOWN |
| Recovery/Agile `dof_bias` | `q-q_default-dof_bias`; Go2 configs randomize ±0.08 | explicit nominal zero calibration | **PASS (nominal contract)**; randomized distribution remains unobservable |
| Ray validity | generated fresh distance in [0.1,6], then `log2` | stamped frame freshness plus finite-beam check; invalid fails closed | **PASS (local contract)**; live replay pending |
| NaN/Inf at policy boundary | no deployment contract in paper | finite checks through final motor command and PASSIVE veto | **PASS (code/helper test)**; live replay pending |

RA is assembled in the training testbed as `[body_lin_vel(3), body_ang_vel(3), Agile_command[:2], Agile_ray2d(11)]`; the local test uses this training-side assembly, not the ROS2 formula as its oracle. RA goal therefore receives the shaped Agile x/y at ROS2 runtime.

The controller, simulator, motor chain, current Isaac Gym runtime order, dimensions, field slices and current permutations have reproducible evidence. All three policies are confirmed to use the original ABS training implementation; server-side run/checkpoint/export proof is nevertheless **UNKNOWN, blocked by training server availability**. The contract cannot assert the true deployed policy/contact order until that lineage closes. P1-01 therefore remains blocked; this document must not promote the conditional candidate order to a confirmed artifact fact.

### P1-01F — Deployment Contract Corrections

- **Timer:** default is now `paper_faithful_rolling`; it exposes the training time-left phase over each 9-second deployment horizon. It intentionally does not claim a physical robot reset. `legacy_fixed` is explicit and non-default.
- **Contact:** Agile, inline Recovery and manual Recovery retain the current raw controller-order contact for one prior policy cycle, exactly as training's `logical_or(current,last_contacts)` does. Entry initializes prior contacts false.
- **Bias:** both configurations declare a controller-order nominal `dof_bias` and both policy observations remap/subtract it. The zero default is a deliberate nominal calibration, not an invented replay of training's random per-episode ±0.08 value.
- **Rays:** `/mujoco_ray2d_stamp` is now a versioned header: magic, version, odd/even sequence, and a `steady_clock` nanosecond completion timestamp. MuJoCo computes rays privately, then marks the frame odd only during the bounded 11-float copy and publishes it even after the matching stamp. ROS2 accepts only a stable, matching snapshot that is fresh (default 200 ms) and finite. Invalid, missing, stale or partial-invalid data invokes `safetyVeto`; a genuine fresh all-6m frame remains valid.
- **Non-finite values:** observation, RA observation/output, Agile/Recovery output, joint target and final motor command are finite-checked. A failure zeros gains/torque, writes a finite nominal target and latches PASSIVE. This is a local contract veto, not a Phase-2 Safety Supervisor.

`test_p1_01_local_contract.py` now injects single-foot touch/liftoff, timer boundary, missing writer, stale writer and Inf-ray helper faults in addition to 61/19/49 parity. The compiled controller package passes. Full live ROS2/MuJoCo fault injection remains required before Phase 1 acceptance.

Recorded local evidence: [`p1_01f_local_contract.json`](evidence/P1-01/p1_01f_local_contract.json).

### P1-01F live ROS2 + MuJoCo fault injection — 2026-08-24

All timings below use the same `std::chrono::steady_clock` nanosecond domain in the writer, controller and test hooks. The normal run measured the writer at 1.0022 ms mean (min/max 1.0000/1.0470 ms) and the deployed ray freshness check at 20.0001 ms mean (min/max 19.9823/20.0221 ms). The 200 ms threshold is unchanged; the acceptance tolerance is 220 ms, one measured 50 Hz check interval.

| Case | Result | Injection → detection | Safety command evidence |
|---|---|---:|---|
| Normal writer | **PASS** — no false-positive veto during measured RL run | n/a | finite RL commands observed |
| Writer freeze | **PASS** | 211.706 ms | finite target; Kp/Kd/torque all zero |
| Writer abrupt exit | **PASS**; old shared-memory stamp remained and became stale | 205.353 ms | finite target; Kp/Kd/torque all zero |
| Ray NaN / Inf | **PASS** | 20.759 / 18.862 ms | finite target; Kp/Kd/torque all zero |
| Observation NaN | **PASS** | 0.048 ms | finite target; Kp/Kd/torque all zero |
| RA output NaN | **PASS** | 0.117 ms | finite target; Kp/Kd/torque all zero |
| Policy action NaN | **PASS** | 0.131 ms | finite target; Kp/Kd/torque all zero |
| Joint target NaN | **PASS** | 0.105 ms | finite target; Kp/Kd/torque all zero |
| Final command NaN / Inf | **PASS** | 0.063 / 0.052 ms | non-finite candidate vetoed before interface write; finite zero-gain command emitted |

Each fault log contains `[ABS-LIVE-FAULT] event=injected`, `[ABS-CONTRACT] event=detected`, `[ABS-LIVE-CMD] event=safety_veto`, `[ABS-LIVE-CMD] event=passive_command_write`, and `Switched from rl to passive`. `ABS_TEST_FAULT` and `MUJOCO_RAY_TEST_FAULT` require explicit simulation-only environment guards; `real_go2.launch.py` forces `ABS_SIMULATION_TEST=0`.

The earlier 4.7–4.9 s readings are superseded: they compared independently orchestrated wall/log timings rather than one clock domain. The earlier normal-writer false positive is explained by the old writer marking a frame in-progress across the full geometry calculation; the revised bounded publish window and sequence-consistent reader eliminate that failure in the current normal run.

This does not alter goal shaping (`INTENTIONAL ENGINEERING VARIANT`), server-dependent provenance `UNKNOWN`, real Go2 foot-force order `UNKNOWN`, or the Phase 2 NO-GO gate.
