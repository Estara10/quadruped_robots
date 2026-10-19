# S1-02 Execution Report

> 归档路径说明（2026-10-08）：本文历史引用中的 `artifacts/manifest.yaml` 现位于根目录 `archive/artifacts/manifest.yaml`。原核查结论与行号引用保留，不代表本次重新验证模型。

## Status

**COMPLETE — conditional Director approval.** The short, non-formal base diagnostic and targeted interface review are accepted. The 12-dimensional joint semantics of the deployed Agile and Recovery artifacts remain UNKNOWN; this is a retained interface risk that must be discussed before formal experiments and is not considered verified. No S1-03 work was started or authorized.

## Interface Findings

Dimensions below are the deployed model declarations in `artifacts/manifest.yaml`, consistent with the already completed shape probe; no model probe was repeated. Observation construction and output handling are from the current controller source. DOF-related indices in the observation and action paths use the controller's declared permutation, but that does not establish the artifacts' learned output semantics.

| Model | Input order, dimensions, source, preprocessing and units | Output and controller path | Evidence |
|---|---|---|---|
| **Agile** (`61 → 12`) | `contact(4)` from foot-force threshold plus one-frame temporal OR, encoded `{-1,+1}`, policy order; `ang_vel(3)` body gyro ×1, rad/s; `gravity_vec(3)` projected gravity, dimensionless; `commands(3)` body-relative goal x/y in metres after dimensionless distance-dependent scaling, plus heading angle in rad; `timer(1)` rolling time-left / 9 s, dimensionless; `dof_pos(12)` `(q−default−bias)×1`, rad; `dof_vel(12)×0.2`, scaled rad/s; previous controller action `(12)` converted to policy order, dimensionless; `ray2d(11)` log2(range in metres), dimensionless. Concatenated in this order and clipped to ±100. | 12 dimensionless policy outputs. Controller assumes policy order `FL,FR,RL,RR`, permutes to controller order `FR,FL,RR,RL`, clips to ±4, multiplies by 0.25, adds default joint angles, clips joint targets to per-type limits, then writes position/Kp/Kd command interfaces. Named-joint meaning of each artifact output index is UNKNOWN (see below). | `AbsObservationContract.cpp:11–23`; `StateRL.cpp:1146–1155,1573–1593,1690–1738`; `abs/config.yaml:6–10,34–48,58–65`; `artifacts/manifest.yaml:7` |
| **RA** (`19 → 1`) | `lin_vel(3)` body-frame linear velocity `[vx,vy,vz]`, m/s; `ang_vel(3)` body-frame angular velocity `[wx,wy,wz]`, rad/s; `commands(2)` body-relative goal x/y in metres after dimensionless distance-dependent scaling; `ray2d(11)` log2(range in metres), dimensionless. Concatenated as `[vx,vy,vz,wx,wy,wz,cmd_x,cmd_y,ray2d...]`; no additional input scale is applied in `ra()`. | One scalar RA Value. It is a learned score; physical units are UNKNOWN. It is compared against the switching threshold; it does not map to a joint target. | `AbsObservationContract.cpp:29–32`; `StateRL.cpp:1157–1177`; `artifacts/manifest.yaml:46` |
| **Recovery** (`49 → 12`) | `contact(4)` same encoding/order as Agile; `ang_vel(3)` ×1, rad/s; `gravity_vec(3)` projected gravity, dimensionless; safe `twist(3)` `[vx,vy,wz]`, m/s, m/s, rad/s, passed into the `commands` segment without another scale; `dof_pos(12)` `(q−default−bias)×1`, rad; `dof_vel(12)×0.2`, scaled rad/s; prior controller action `(12)` converted to policy order, dimensionless. Concatenated in this order and clipped to ±100; no timer or ray segment. | Same controller path as Agile: presumed policy-order permutation, ±4 action clip, ×0.25, add default positions, joint-limit clipping, then position/Kp/Kd command-interface writes. Artifact output index-to-joint semantics remain UNKNOWN. | `AbsObservationContract.cpp:11–19,25–27`; `StateRL.cpp:682–697,1657–1670,1698–1738`; `rec/config.yaml:5–10,22–37`; `artifacts/manifest.yaml:87` |

The controller permutation is explicitly `{3,4,5,0,1,2,9,10,11,6,7,8}` from controller order to its assumed policy order `FL,FR,RL,RR` (and the same permutation is self-inverse). That is a **controller assumption**, reflected in `policy_joint_order: ros1_fl_fr_rl_rr`; it is not direct artifact evidence. Agile and Recovery outputs are consumed through this assumption.

### 12-D action joint order

| Evidence category | Finding |
|---|---|
| Training implementation convention | The recovered Legged Gym training implementation obtains `dof_names` from the loaded asset and applies each action vector index to the corresponding indexed DOF (`legged_robot.py:80–98,377–398,762–766`). This shows training actions use the environment's DOF index order. It does not disclose which named order was bound to the exact runs below. |
| Controller assumption | `StateRL.cpp:204–232` and both deployment configs assume policy order `FL,FR,RL,RR`, then map to controller order `FR,FL,RR,RL`. |
| Deployed Agile artifact | **UNKNOWN.** Manifest links the current artifact to checkpoint `go2_pos_rough/05_27_15-53-31_/model_4000.pt` and records matching actor weights/export, but says the run config/order snapshot and exact export invocation are unavailable; it explicitly leaves `expected_joint_order` UNKNOWN (`artifacts/manifest.yaml:21–44`). |
| Deployed Recovery artifact | **UNKNOWN.** Manifest links the current artifact to checkpoint `go2_rec_rough/06_04_22-43-20_/model_15000.pt` and records matching actor weights/export, but says the run config/order snapshot and export execution log are unavailable; it explicitly leaves `expected_joint_order` UNKNOWN (`artifacts/manifest.yaml:102–125`). |

The directly relevant source checked for this question was the existing artifact manifest and its named Agile/Recovery run/export metadata, plus the recovered training implementation establishing its DOF-index convention. This did not establish named-joint order for either exact artifact. No URDF ordering, YAML comment, observed locomotion, or newly run model probe is used as proof. The impact is that deployment can be shown to write 12 mapped targets, but it cannot yet be shown that each artifact's learned output index is assigned to its trained named joint; this affects interpreting motion and any later policy comparison. The smallest future verification is to obtain an immutable `dof_names` snapshot bound to each exact training run/checkpoint and export, or—if that historical record cannot be recovered—conduct a separately authorized labelled action-channel isolation check and report that it verifies the deployment channel map only, not the artifact's training semantics.

## Runtime and Timing Findings

Retained from the existing S1-02 evidence; no new capture was made in this supplement:

- The earlier legacy-candidate diagnostic used `stabilized_switch`, `scene_obstacle.xml`, 200 Hz controller update, decimation 4, entry threshold `-0.05`, and `recovery_hold_steps=30`. It loaded Agile, RA and Recovery from the installed `go2_description/config/{abs,rec}` paths recorded in `short_diag_20261007/run_context.json`; that run ended at RL step 1978 with `monotonic_clock_order` safety veto. This is not a formal group result.
- The later short diagnostic used `paper_faithful_switch` with `scene_obstacle.xml`, 1000 Hz manager, 200 Hz controller update, decimation 4, entry threshold `-0.05`, configured hold 30 but hold disabled by the candidate. Models loaded from `/tmp/s1_02_overlay/share/go2_description/config/{abs,rec}` and were byte-compared equal to repository deployment artifacts. This is a legacy A-like candidate, not a formal A result (`run_context.json:17–45`).
- Recorder preflight was ready before launch. It captured 1,848 continuous policy frames (`rl_step` 0–1847, no gaps) and 27,406 simulator-clock samples, including one stale sample before the current clock segment. There were 1,847 measured intervals: mean 19.943 ms, median 20.009 ms, P95 26.745 ms, minimum 1.939 ms, maximum 58.509 ms. Aggregate rate was 50.14 Hz; per-interval reciprocal median was 49.98 Hz, P5–P95 37.39–78.93 Hz, extremes 17.09–515.84 Hz. Code invokes RA once per policy decision cycle, so measured policy/RA cadence is the same. No 125 Hz / 8 ms annotation is treated as measurement.
- The 1,848 frames each contain the RA value, `mode_after`, action source/payload and a shared steady-monotonic timestamp. `mode_before` is reconstructed from the preceding continuous frame; it is UNKNOWN for the first frame. Thus RA, selected mode and action are associated with the same `rl_step` by the single runtime-frame write. There were 14 mode edges (7 Agile→Recovery and 7 Recovery→Agile), with 35 Recovery frames.
- All 1,848 frames were bracketed by adjacent MuJoCo clock samples using shared monotonic timestamps, without interpolation. The simulator-time span was 18.896–55.734 s; bracket width median 2.346 ms, P95 3.048 ms, maximum 7.808 ms. This establishes the recorded association interval, not that the controller consumes `sim_time` for hold logic.
- 295 command-interface telemetry records matched the nearest prior policy-frame target at printed precision; five corresponded to Recovery targets. This proves command-interface writes only. MuJoCo consumption/application remains UNKNOWN.
- The later run had no safety veto, including no `monotonic_clock_order` veto. It was manually stopped; process and shared-memory cleanup were checked (`cleanup_check.json`). No repeated run was performed.

Evidence: `evidence/S1-02/short_diag_20261007/summary.json`, `run_context.json`, `policy_frames.jsonl`, `policy_frames_simtime_bracket.jsonl`, `sim_clock.jsonl`, `collector_ready.json`, `ros_launch.log`, and `cleanup_check.json`.

## Files Changed

- `docs/thesis_project/evidence/S1-02/S1-02_EXECUTION_REPORT.md` — consolidated S1-02 report and this targeted interface/order supplement.
- `docs/thesis_project/CURRENT_STATE.md` — replaced the stale claim that S1-02 technical work had not begun; records execution, review status and unresolved artifact-order condition.
- No controller, model, runtime configuration, or evidence capture was changed in this supplement.

## Remaining UNKNOWN / Blockers

- Exact trained named-joint order for the deployed Agile and Recovery artifact outputs. Direct run-bound DOF-order snapshots are missing. Until resolved, policy-to-joint semantic correctness and motion attribution remain UNKNOWN; do not claim those artifacts' joint ordering is verified.
- `sim_time` is recorded alongside policy cycles but is not consumed by the controller's hold timing. Hold-time integration/verification remains future S1 work.
- Recovery target consumption by MuJoCo remains UNKNOWN; existing telemetry proves write-out only.

## S1-02 Completion Conditions

Director 的有条件通过接受下表中已明确保留的 UNKNOWN 和未满足项；因此本任务状态为 COMPLETE，但动作关节顺序仍未验证，正式实验前必须讨论。

| Condition | Judgment |
|---|---|
| Key model dimensions, observation order, units and action mapping have direct evidence; conflicts are explicit | **PARTIAL.** Input dimensions/order and controller mapping are documented. Units/semantics that lack proof are marked UNKNOWN. Agile and Recovery artifact output joint order is UNKNOWN, so the condition is not met. |
| Current policy/RA frequency measured or concrete reproducible blocker documented | **MET.** 50.14 Hz aggregate over 1,847 adjacent intervals; distribution above. |
| One diagnostic explains RA/mode/action cycle association; missing transitions are explicit | **MET for observed cycle association.** 14 edges observed; `mode_before` for initial frame and physical command consumption remain UNKNOWN. |
| MuJoCo time source, controller consumption and minimum integration boundary are clear | **PARTIAL.** Clock samples are paired with every frame; controller hold does not consume this clock, and actual time-based hold remains unimplemented/unverified. |
| No formal experiment, training, real-robot test, or premature B/C/D implementation | **MET.** This supplement was read-only except report/current-state edits. |

## Recommended Next Step

No next Active Task is currently authorized. Before formal experiments, discuss the UNKNOWN Agile/Recovery artifact joint order. If a future task is authorized to investigate it, first seek the exact run-bound `dof_names` snapshot for the two linked checkpoints. If unavailable, decide separately whether a labelled deployment action-channel check is useful; it can verify the deployment channel map only, not historical training semantics. This report does not authorize S1-03.
