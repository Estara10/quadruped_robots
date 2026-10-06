# P1-02 Schema Field → Runtime Source Map (P1-09A audit, 2026-08-27)

This is the authoritative field-to-runtime-source map produced by the P1-09A
read-only audit. Every required P1-02 formal field (`abs-go2-formal-run/v1`) is
assigned exactly one status. `UNKNOWN` and `LEGACY_ONLY` must make a future formal
episode `INVALID` until an authoritative producer is connected.

## Status legend

| Status | Meaning |
|---|---|
| `AUTHORITATIVE_NOW` | An exact capture point exists and is already written by the P1-02 `FormalRunWriter`/validator, or is an immutable constant with a single producer. |
| `AVAILABLE_BUT_ADAPTER_NEEDED` | A source exists in the allowed code scope but is not emitted into the formal artifact; a minimal additive adapter/hook is required. |
| `UNKNOWN` | No authoritative source exists in the allowed scope. Must remain `UNKNOWN`; never inferred from filename/comment/static config. |
| `LEGACY_ONLY` | Present only in the legacy evaluator or text-log output and forbidden from formal Acceptance. |

Producer clock/domain notes:

- `steady_clock_ns` — `std::chrono::steady_clock` monotonic nanoseconds; the ray
  v2 header, ray check, safety veto, and command telemetry all use this domain.
- `python monotonic` — `time.monotonic()` in `run_abs_eval.py` (legacy evaluator).
- `simulation_time_s` — no producer exists anywhere in the allowed scope.
- `wall-clock` — `datetime.now()` in `run_abs_eval.py` (session manifest only).

## Manifest fields

| Field | Status | Source (path:symbol:line) | Clock/domain & identity | Capture w/o control change? | Missing info & fail-closed consequence |
|---|---|---|---|---|---|
| `schema_version` | AUTHORITATIVE_NOW | `scripts/formal_experiment_contract.py:SCHEMA_VERSION:37` (from `schemas/formal_experiment_run_v1.json:$id`) | immutable const | yes | none |
| `run_id` | AUTHORITATIVE_NOW | `scripts/formal_experiment_contract.py:FormalRunWriter.allocate_run_id:247-254` | process-local UUID4 registry | yes | none |
| `created_at` | AVAILABLE_BUT_ADAPTER_NEEDED | `scripts/run_abs_eval.py:write_session_manifest:964` (`datetime.now().isoformat()`) | wall-clock | yes | not in formal manifest; adapter must write wall-clock timestamp. Missing → schema `required` fails. |
| `variant` | UNKNOWN | `scripts/run_abs_eval.py:ABLATION_MODES:44-50` | n/a | yes (label only) | legacy labels `full`/`agile_only`/`no_recovery_hold`/`early_recovery`/`late_recovery` are not `paper-faithful`/`stabilized`/`agile-only`. No runtime producer of a formal variant label. |
| `pairing_key` | AUTHORITATIVE_NOW | `scripts/formal_experiment_contract.py:pairing_key:178-192` | canonical SHA-256 | yes | derived from scenario/seed/config/model hashes; missing inputs propagate as `missing_pairing_key_input`. |
| `git.commit` / `git.branch` | AVAILABLE_BUT_ADAPTER_NEEDED | `scripts/run_abs_eval.py:_git_text:945-958` → `write_session_manifest:997-1000` | wall-clock (subprocess `git rev-parse`/`branch`) | yes | session-level only; adapter must move to per-run manifest. Missing → schema `required` fails. |
| `git.dirty_state` | AVAILABLE_BUT_ADAPTER_NEEDED | `scripts/run_abs_eval.py:_git_text:945-958` (`git status --short`) | wall-clock | yes | `status_short` text is captured but not reduced to `clean`/`dirty`; adapter must derive it. |
| `git.dirty_patch_sha256` | UNKNOWN | none | n/a | n/a | no dirty-patch hash is captured anywhere. Must stay `UNKNOWN` or add a capture hook. |
| `models.*.path` / `models.*.sha256` | AVAILABLE_BUT_ADAPTER_NEEDED | `scripts/run_abs_eval.py:model_provenance:871-884` (`sha256_file`) | wall-clock | yes | SHA-256 computed for the 3 deployed models but written only into `session_manifest.json`, not the formal per-run manifest. |
| `models.*.source_provenance` | UNKNOWN | none | n/a | n/a | P1-01 lineage; no authoritative source. Must remain `UNKNOWN`. |
| `effective_config.path` | AVAILABLE_BUT_ADAPTER_NEEDED | `scripts/run_abs_eval.py:write_effective_abs_config:907-912` | wall-clock | yes | patched YAML written; adapter must emit path into manifest. |
| `effective_config.sha256` | AVAILABLE_BUT_ADAPTER_NEEDED | `scripts/run_abs_eval.py:write_effective_abs_config:907-912` (no hash computed) | wall-clock | yes | file written but no hash; add `sha256_file()`. Runtime-effective confirmation (controller actually loaded it) is UNKNOWN. |
| `environment.mujoco_binary_path` | AVAILABLE_BUT_ADAPTER_NEEDED | `scripts/run_abs_eval.py:MUJOCO_BIN:34` | n/a (constant) | yes | path known; adapter must emit it. |
| `environment.mujoco_binary_sha256` | UNKNOWN | none | n/a | n/a | no binary hash capture. |
| `environment.mujoco_version` | UNKNOWN | none | n/a | n/a | no version capture. |
| `environment.timestep_s` | UNKNOWN | none | n/a | n/a | MJCF lacks explicit timestep (see GAP_MATRIX timestep/dynamics row). |
| `environment.solver` | UNKNOWN | none | n/a | n/a | no solver capture. |
| `environment.go2_mjcf_path` | AVAILABLE_BUT_ADAPTER_NEEDED | `scripts/run_abs_eval.py:CONFIG_PATH/scene dir:39,1010-1014` | n/a | yes | scene/MJCF path known; adapter must emit + hash. |
| `environment.go2_mjcf_sha256` / `go2_assets_sha256` | AVAILABLE_BUT_ADAPTER_NEEDED | `scripts/run_abs_eval.py:write_session_manifest:1006-1014` (copies, no hash) | wall-clock | yes | files copied to `input_snapshots/` without hash; add `sha256_file()`. |
| `environment.hardware_mode` | AVAILABLE_BUT_ADAPTER_NEEDED | `scripts/run_abs_eval.py:run_episode:689-700` (launches `unitree_mujoco` + `mujoco.launch.py`) | n/a | yes | fixed `simulation` constant; no runtime confirmation but launch is simulation-only. |
| `scenario.id` / `scenario.path` | AVAILABLE_BUT_ADAPTER_NEEDED | `scripts/run_abs_eval.py:args.scenes:1040-1043` | n/a | yes | scene name/path from CLI; adapter must emit. |
| `scenario.sha256` | AVAILABLE_BUT_ADAPTER_NEEDED | `scripts/run_abs_eval.py:write_session_manifest:1010-1014` (copies, no hash) | wall-clock | yes | no scene hash; add `sha256_file()`. |
| `scenario.metadata.obstacle_count` | AVAILABLE_BUT_ADAPTER_NEEDED | `scripts/generate_test_scenes.py:92-132` / `scripts/run_abs_eval.py:analyze_scene_clearance:417-471` | n/a | yes | obstacle count derivable from scene XML; not recorded formally. |
| `seeds.root_seed` | UNKNOWN | none (only `scripts/generate_test_scenes.py:SEED=42:13`, `np.random.seed:135`) | n/a | n/a | fixed 42 in scene gen only; evaluator/controller have no root seed. Controller uses unseeded `rand()` (`StateRL.cpp:1260-1261`). |
| `seeds.sources.*` (scene/controller/perception/evaluator) | UNKNOWN | none | n/a | n/a | no downstream seed derivation or recording; `derive_seed` exists in writer but no runtime source is wired. |
| `perception.source` | AVAILABLE_BUT_ADAPTER_NEEDED | `unitree_mujoco/simulate/src/unitree_sdk2_bridge.h:69-74` (`MUJOCO_RAY_SOURCE` env) | n/a | yes | mode switch read from env; adapter must record the effective mode. Active mode is UNKNOWN without a runtime capture. |
| `perception.version` | UNKNOWN | none | n/a | n/a | no perception version capture. |
| `perception.sha256` | UNKNOWN | none (candidate `scripts/launch_abs_ray_pred.sh:RAY_PRED_MODEL:22`) | n/a | n/a | ray-pred model path exists but no hash capture in the evaluator. |
| `perception.frame_contract_version` | AVAILABLE_BUT_ADAPTER_NEEDED | `unitree_mujoco/simulate/src/unitree_sdk2_bridge.h:286` (`abs_ray2d_shm::kVersion`) | `steady_clock_ns` | yes | v2 header version written by geometric writer only; external writer (`scripts/ray_predictor.py`) writes raw floats with no header. |
| `rates_hz.*` (controller/pd/policy/ra/perception) | UNKNOWN | static only: `robot_control.yaml:4,152`; `StateRL.cpp:374-387` (decimation) | n/a | yes (read-only) | static `update_rate`/`decimation` are config, not measured cadence. P1-03 accepts this as CONFLICT/UNKNOWN; requires runtime capture. |
| `thresholds.arrival_region_m` | AVAILABLE_BUT_ADAPTER_NEEDED | `scripts/run_abs_eval.py:arrival_threshold:1068`; `StateRL.cpp:1241` (hardcoded 0.5) | n/a | yes | two producers (evaluator arg + controller constant) must be reconciled; adapter must record the effective value. |
| `thresholds.arrival_hold_s` | UNKNOWN | none | n/a | n/a | no arrival-hold timer exists; only a distance threshold. |
| `thresholds.fall_height_m` / `fall_angle_rad` | AVAILABLE_BUT_ADAPTER_NEEDED | `scripts/run_abs_eval.py:1085-1086` | n/a | yes | evaluator args; adapter must record. |
| `thresholds.collision_definition_id` | UNKNOWN | none | n/a | n/a | no registered collision definition ID exists. |
| `thresholds.ra_entry_threshold` / `ra_exit_threshold` | AVAILABLE_BUT_ADAPTER_NEEDED | `config/abs/config.yaml:ra_threshold`; `StateRL.cpp:1327-1328` | n/a | yes | entry = `ra_threshold`; exit = `ra_threshold - 0.03` (derived). Adapter must record both. |

## Telemetry fields (`x-abs-telemetry`)

| Field | Status | Source (path:symbol:line) | Clock/domain & identity | Capture w/o control change? | Missing info & fail-closed consequence |
|---|---|---|---|---|---|
| `run_id` | AUTHORITATIVE_NOW | `scripts/formal_experiment_contract.py:FormalRunWriter.write_telemetry:296-306` | n/a | yes | writer binds every row to run ID. |
| `sequence` | UNKNOWN | none | n/a | n/a | legacy `step` (text-log counter) is not a formal per-run sequence. Requires a monotonic sequence producer. |
| `simulation_time_s` | UNKNOWN | none | n/a | n/a | no simulation-time producer. `[EVAL] t` is `elapsed_s = rl_step_count*decimation/frequency` (`StateRL.cpp:424`), a derived counter, not MuJoCo sim time. |
| `monotonic_time_ns` | UNKNOWN | none | n/a | n/a | `steady_clock_ns` exists in ray header / veto / command telemetry but is not emitted as a per-row telemetry column. |
| `telemetry_fresh` | UNKNOWN | none | n/a | n/a | no row-level freshness flag/timestamp is computed; distinct from ray-specific `ray_valid`. Requires a row-level monotonic timestamp + threshold (both absent). |
| `base_x_m/base_y_m/base_z_m` | AVAILABLE_BUT_ADAPTER_NEEDED | `unitree_mujoco/simulate/src/unitree_sdk2_bridge.h:127-133` (`qpos_shm_ptr_`); `scripts/run_abs_eval.py:313-337` reads `/dev/shm/mujoco_qpos` | raw shm, no clock/identity | yes | qpos shm has 19 doubles (x,y,z,quat,12 joints) but no timestamp/sequence/run identity → not fresh. |
| `roll_rad/pitch_rad/yaw_rad` | AVAILABLE_BUT_ADAPTER_NEEDED | `scripts/run_abs_eval.py:quat_wxyz_to_rpy:167-184`; `StateRL.cpp:1167` | raw shm | yes | derived from qpos quat; no timestamp. |
| `policy_state` | AVAILABLE_BUT_ADAPTER_NEEDED | `StateRL.cpp:in_recovery_:1337,1353` | `steady_clock_ns` (in-process) | yes | `in_recovery_` bool exists in memory; not emitted as a column. |
| `ra_value` / `ra_entry_threshold` / `ra_exit_threshold` | AVAILABLE_BUT_ADAPTER_NEEDED | `StateRL.cpp:ra_value_:425`; `:1327-1328` | `steady_clock_ns` | yes | `ra_value_`, `ra_threshold`, and derived exit exist in memory; not emitted. |
| `command_vx_mps/command_vy_mps/command_wz_rps` | AVAILABLE_BUT_ADAPTER_NEEDED | `StateRL.cpp:1289` (`obs_.commands` = body_x,body_y,heading_cmd) | `steady_clock_ns` | yes | computed in memory; not emitted as columns. |
| `actual_vx_mps/actual_vy_mps/actual_wz_rps` | AVAILABLE_BUT_ADAPTER_NEEDED | `StateRL.cpp:1133-1147` (`odom_state_interface_` → `obs_.lin_vel`) | `steady_clock_ns` | yes | body-frame velocity from odometer; not emitted. |
| `recovery_vx_mps/recovery_vy_mps/recovery_wz_rps` | AVAILABLE_BUT_ADAPTER_NEEDED | `StateRL.cpp:cached_rec_*:1340-1342,1364-1366` | `steady_clock_ns` | yes | cached twist in memory; not emitted. |
| `recovery_constraint_margin` | UNKNOWN | none | n/a | n/a | `computeRecoveryTwist` (`StateRL.cpp:515`) computes a penalty loss but no explicit final constraint margin is stored. |
| `ray_valid` | AVAILABLE_BUT_ADAPTER_NEEDED | `StateRL.cpp:updateRay2d:622-689` (`ray2d_valid_`) | `steady_clock_ns` | yes | validity computed; not emitted as a column. |
| `ray_age_ns` | AVAILABLE_BUT_ADAPTER_NEEDED | `StateRL.cpp:updateRay2d:656` (`last_ray_age_ns_`) | `steady_clock_ns` | yes | age computed; not emitted. |
| `controller_active` | AVAILABLE_BUT_ADAPTER_NEEDED | `StateRL.cpp:running_:355`; `scripts/run_abs_eval.py:wait_for_controller:257-265` | `steady_clock_ns` / wall-clock | yes | `running_` in memory; evaluator returns ephemeral boolean. No persisted event/column. |
| `rl_active` | AVAILABLE_BUT_ADAPTER_NEEDED | `StateRL.cpp:777` (`running_ && !safety_faulted_`) | `steady_clock_ns` | yes | derivable in memory; not emitted. |
| `collision_available` | AVAILABLE_BUT_ADAPTER_NEEDED | `unitree_mujoco/simulate/src/unitree_sdk2_bridge.h:updateCollisionTelemetry:125`; `scripts/run_abs_eval.py:COLLISION_PATH:79` | raw shm | yes | 5-int32 shm exists; no timestamp/sequence/identity. |
| `collision` | AVAILABLE_BUT_ADAPTER_NEEDED | `scripts/run_abs_eval.py:293-311` (reads collision flag/count) | raw shm | yes | counters exist; no edge event and real slot semantics remain UNKNOWN. |
| `fall` | LEGACY_ONLY | `scripts/run_abs_eval.py:monitor_episode:329-337` (height/tilt rule) | python monotonic | yes | proxy evaluator rule only; `StateRL::checkBodySafety` (`:781-801`) exists but emits no structured fall event. |
| `arrival_candidate` | AVAILABLE_BUT_ADAPTER_NEEDED | `StateRL.cpp:arrived:1287` (`dist_to_goal < arrival_threshold`) | `steady_clock_ns` | yes | `arrived` bool exists in `StateRL::runModel` memory; not emitted as a column. Distinct from the legacy evaluator goal-error proxy. |
| `ray_log2_*` (11) | AVAILABLE_BUT_ADAPTER_NEEDED | `unitree_mujoco/simulate/src/unitree_sdk2_bridge.h:281-289` (v2 write); `StateRL.cpp:670` (`obs_.ray2d`) | `steady_clock_ns` | yes | 11 log2 values exist; adapter must emit the validated snapshot with validity/freshness. |
| `action_raw_*` (12) | AVAILABLE_BUT_ADAPTER_NEEDED | `StateRL.cpp:policy_actions:1396/1371` | `steady_clock_ns` | yes (read) | computed in `StateRL::runModel` memory but not emitted into formal telemetry. |
| `action_clipped_*` (12) | AVAILABLE_BUT_ADAPTER_NEEDED | `StateRL.cpp:clamped_actions:1399/1373` | `steady_clock_ns` | yes (read) | computed in `StateRL::runModel` memory but not emitted into formal telemetry. |
| `joint_target_rad_*` (12) | AVAILABLE_BUT_ADAPTER_NEEDED | `StateRL.cpp:output_dof_pos_:1419` | `steady_clock_ns` | yes (read) | computed in `StateRL::runModel` memory but not emitted into formal telemetry. |
| `torque_nm_*` (12) | AVAILABLE_BUT_ADAPTER_NEEDED | `StateRL.cpp:output_torques:1417` | `steady_clock_ns` | yes (read) | computed in `StateRL::runModel` memory but not emitted into formal telemetry. |
| `torque_saturated_*` (12) | UNKNOWN | none | n/a | n/a | no per-joint saturation boolean is computed anywhere; `checkTorqueSafety` (`StateRL.cpp:803`) only vetoes on a limit, it does not emit a saturation flag. |

## Event fields (`x-abs-events`)

| Field | Status | Source (path:symbol:line) | Clock/domain & identity | Capture w/o control change? | Missing info & fail-closed consequence |
|---|---|---|---|---|---|
| `episode_start` | UNKNOWN | none | n/a | n/a | no structured event emitter exists. |
| `controller_active` | AVAILABLE_BUT_ADAPTER_NEEDED | `StateRL.cpp:running_:355`; `scripts/run_abs_eval.py:wait_for_controller:257-265` | `steady_clock_ns` / wall-clock | yes | no persisted event; text/log boolean only. |
| `rl_entered` | AVAILABLE_BUT_ADAPTER_NEEDED | `scripts/run_abs_eval.py:auto_enter_rl:268-276` (publishes `/control_input`); `StateRL.cpp:enter` (RL entry) | wall-clock | yes | publish is not FSM-entry confirmation; needs a `rl_entered` event from StateRL entry. |
| `valid_ready` | UNKNOWN | none | n/a | n/a | no preflight readiness event. |
| `terminal` | LEGACY_ONLY | `scripts/run_abs_eval.py:determine_result:599-612` | python monotonic | yes | legacy ordering chooses `SUCCESS` before `FALL` (line 604 vs 608); forbidden from formal Acceptance. |
| `shutdown` | UNKNOWN | none | n/a | n/a | process stop is not an emitted event. |
| `recovery_enter` / `recovery_exit` | AVAILABLE_BUT_ADAPTER_NEEDED | `StateRL.cpp:in_recovery_:1337` (enter), `:1353` (exit) | `steady_clock_ns` | yes | `in_recovery_` transition flags exist in memory; not emitted as structured events. Text log (`[RA-REC]`) is a side effect only. |
| `collision_start` / `collision_end` | AVAILABLE_BUT_ADAPTER_NEEDED | `unitree_mujoco/simulate/src/unitree_sdk2_bridge.h:updateCollisionTelemetry:125` | raw shm | yes | counters exist but no edge event with sequence/time. |
| `fall` | LEGACY_ONLY | `scripts/run_abs_eval.py:monitor_episode:329-337` | python monotonic | yes | proxy rule only. |
| `arrival_accepted` / `timeout` | LEGACY_ONLY | `scripts/run_abs_eval.py:321-325,612` | python monotonic | yes | legacy evaluator only; no structured producer. |

## Summary and plots

| Field | Status | Source (path:symbol:line) | Capture w/o control change? | Missing info & fail-closed consequence |
|---|---|---|---|---|
| `summary.validity` / `invalid_reasons` | AUTHORITATIVE_NOW | `scripts/formal_experiment_contract.py:validate_run:642-662` | yes | validator classifies `VALID`/`INVALID`/`LEGACY / NON-ACCEPTANCE`. |
| `summary.terminal_outcome` | LEGACY_ONLY | `scripts/run_abs_eval.py:determine_result:599-612` | yes | wrong ordering; must be replaced by the formal terminal reducer. |
| `summary.metrics` | LEGACY_ONLY | `scripts/run_abs_eval.py:summarize_from_rows:527-573` | yes | legacy aggregate; not the formal metric set. |
| `summary.artifacts` / `artifact_hashes` | AUTHORITATIVE_NOW | `scripts/formal_experiment_contract.py:FormalRunWriter.write_summary:320-341` | yes | writer binds paths + SHA-256. |
| fixed plots (5) | UNKNOWN | `scripts/formal_experiment_contract.py:write_data_plot:343-359` (writer exists); `scripts/analyze_abs_eval.py` (legacy aggregate) | yes | no runtime producer of per-episode data-bound SVG; legacy aggregate report is forbidden. |

## Current conclusion

The P1-02 writer/validator is authoritative for `run_id`, `schema_version`,
`pairing_key`, summary artifact hashing, and validator classification. Everything
else requires either a minimal additive adapter (`AVAILABLE_BUT_ADAPTER_NEEDED`) or
remains `UNKNOWN`/`LEGACY_ONLY`. In particular:

- Simulation time, per-row sequence/monotonic clock, and `telemetry_fresh` have no
  producer anywhere → telemetry cannot yet be `VALID`.
- The 5×12 command chain: `action_raw`/`action_clipped`/`joint_target_rad`/
  `torque_nm` are computed in `StateRL::runModel` memory but not emitted
  (`AVAILABLE_BUT_ADAPTER_NEEDED`); `torque_saturated` is not computed at all
  (`UNKNOWN`).
- Measured rates and the active ray source mode remain `UNKNOWN` (P1-03 accepted
  finding); static config is not measurement.
- Existing `run_abs_eval.py` output remains `LEGACY / NON-ACCEPTANCE`; its
  `determine_result` ordering (Success before Fall) is forbidden from formal use.
