# P1-02 Schema Field → Runtime Source Map

Status is the evidence status of the current implementation, not a claim that an existing run is formal evidence. `UNKNOWN` fields must make a future formal episode `INVALID` until an authoritative producer is connected.

| Schema field/group | Current source or candidate | Status | P1-02 handling |
|---|---|---|---|
| `run_id`, schema version, artifact layout | `FormalRunWriter` in `scripts/formal_experiment_contract.py` | PASS | Writer binds manifest, every telemetry row, every event, summary and plot provenance to one run ID. |
| Git commit/branch/dirty state | `scripts/run_abs_eval.py::_git_text()` and `write_session_manifest()` | PARTIAL | Existing session manifest has commit/branch/status text; formal adapter must provide per-run dirty patch hash or explicit dirty-state value. |
| Deployed model path/SHA-256 | `model_provenance()` in `run_abs_eval.py` | PARTIAL | SHA-256 can be captured; P1-01 source checkpoint/export/order lineage remains `UNKNOWN`. |
| Effective config snapshot/hash | `write_effective_abs_config()` in `run_abs_eval.py` | PARTIAL | Snapshot exists; formal writer requires canonical hash and runtime-effective confirmation. |
| MuJoCo binary/version/timestep/solver | `MUJOCO_BIN` in `run_abs_eval.py`; simulator source | UNKNOWN | No verified version/timestep/solver capture path was found in the allowed source scope. |
| Go2 MJCF/assets hash | scene/MJCF paths in `run_abs_eval.py` | PARTIAL | Paths/snapshots exist; formal adapter must compute and record immutable hashes. |
| Scenario ID/file/hash/metadata | `args.scenes`, `write_session_manifest()`, `generate_test_scenes.py` | PARTIAL | Existing names and snapshots lack per-episode hash/metadata provenance. |
| Root and downstream seeds | `generate_test_scenes.py` has fixed `np.random.seed(42)` | UNKNOWN | Current evaluator has no declared root-seed propagation. Formal writer provides deterministic `derive_seed`; P1-10 must connect every runtime source. |
| Variant label | `run_abs_eval.py::ABLATION_MODES` | FAIL for formal variants | Existing labels/overrides are not the required `paper-faithful`/`stabilized`/`agile-only` contract. P1-02 validator rejects undeclared labels; P1-07 owns behavior separation. |
| Controller active | `wait_for_controller()` | PARTIAL | It returns an ephemeral boolean, not a persisted lifecycle event tied to one run. |
| RL entered | `auto_enter_rl()` publishes control input | UNKNOWN | Publish is not confirmation that the FSM entered RL. Formal `rl_entered` requires an authoritative runtime event. |
| Simulation time/fresh pose | `mujoco_qpos` in `unitree_sdk2_bridge.h` | UNKNOWN | QPOS shared memory has no timestamp/sequence/run identity. Formal telemetry requires `simulation_time_s` and `telemetry_fresh`; absent values are INVALID. |
| Ray values/validity | v2 ray frame/header in `unitree_sdk2_bridge.h` and `StateRL::updateRay2d()` | PARTIAL | Controller validates v2 freshness, but `run_abs_eval.py` reads only legacy 11-float ray memory without its header. Formal telemetry cannot claim ray freshness yet. |
| Policy state/RA/recovery twist | `StateRL::logEvalTelemetry()` / `[EVAL]` parsing | PARTIAL | Text output has a subset at a throttled rate; no authoritative structured event stream. |
| 11 rays, raw/clipped actions, joint target, torque/saturation | `StateRL.cpp` computes sources; formal schema requires 11 + 5×12 finite fields | UNKNOWN | No structured evaluator export in the allowed code path; any absent/non-finite vector makes the episode INVALID. |
| Contact | `StateRL.cpp` temporal contact handling | PARTIAL | Runtime state exists; no formal event/telemetry export and real slot semantics stay `UNKNOWN`. |
| Collision event | `updateCollisionTelemetry()` in `unitree_sdk2_bridge.h` | PARTIAL | Contact counters/geom IDs exist, but no time/sequence/run identity or formal edge event. Formal validator requires collision telemetry false→true edge alignment to `collision_start`. |
| Fall | `monitor_episode()` qpos height/tilt rules | PARTIAL | It has a proxy evaluator rule; no registered formal event stream and definition differs from training termination. Formal validator requires fall telemetry false→true edge alignment to `fall`. |
| Arrival/outcome ordering | `run_abs_eval.py::determine_result()` | FAIL | Existing ordering can choose Success before Fall. The formal reducer rejects safety-at-arrival Success; a runtime adapter is still required. |
| Fixed plots | `analyze_abs_eval.py` aggregate reports | UNKNOWN | No per-episode fixed formal plot writer currently exists. |

## Current conclusion

The P1-02 library and validator are implemented and tested, but the current runtime adapter is intentionally incomplete. Therefore existing `run_abs_eval.py` outputs remain `LEGACY / NON-ACCEPTANCE`; they must not be passed to the validator as formal `VALID` evidence.

The strict validator now rejects wrong-run telemetry/events, incomplete or non-finite 11-ray/12-D command-chain vectors, and summaries/plots without a verifiable manifest/data hash chain. These mechanical protections do not create a runtime source where none exists.

It also rejects non-finite/malformed event clocks and any telemetry/event/terminal safety contradiction. Formal comparison aggregation additionally requires the three schema-valid paired variants through the comparison CLI.
