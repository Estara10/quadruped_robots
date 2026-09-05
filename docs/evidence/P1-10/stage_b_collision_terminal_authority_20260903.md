# P1-10 Stage B Minimal Collision/Terminal Authority

Date: 2026-09-03  
Scope: offline authority implementation and tests only  
Scenario scope: `obstacle_test1` only

## Verdict

The minimum collision authority source is implemented in the simulator physics
path and is consumed by the existing saved runtime recorder. It is not runtime
validated in this task. `obstacle_test1` therefore remains
`IMPLEMENTED / AWAITING RUNTIME VALIDATION`; the five-map candidate suite is not
an accepted P1-10 suite. Overall P1-10 remains
`IMPLEMENTED / AWAITING INDEPENDENT REVIEW`.

No MuJoCo, ROS2, controller, A/B replay, benchmark, FormalRun, or later P1 task
was started. No accepted P1-08 baseline, historical capture, historical pair,
XML, model, policy, controller algorithm, physics parameter, or control behavior
was changed.

## Authority contract and dataflow

`common/abs_collision_contract.h` defines the versioned 392-byte
`/mujoco_collision_v2` snapshot. The snapshot carries magic/version, an
acquire/release sequence, monotonic timestamp, `mjData::time`, one-based
physics-step identity, authority and invalidity fields, contact class counts,
last classified geom IDs, and the exact `obstacle_test1` scene root and closure
hashes.

`unitree_mujoco/simulate/src/obstacle_collision_authority.h` is called after
each existing `mj_step` in `main.cc`. It reads `mjData::ncon` and the current
`mjContact` geom pair. It first binds the model by the seven XML-derived
type/position/size signatures and then classifies contacts. The legacy
`/mujoco_collision` five-integer bridge telemetry remains untouched for
compatibility and is not read as formal authority.

The existing `RunRecordRecorder` reads the versioned snapshot beside each
authoritative runtime-frame sample, validates it without a live-shm fallback,
and stores the structured `collision_snapshot` in `runtime_record.jsonl`.

## Repair amendment: capture binding and runtime model fingerprint

The independent-review REJECT blockers were repaired offline. Snapshot version
2 now carries the harness-generated `capture_id` and a deterministic
`runtime_model_fingerprint`. The capture ID has the structured form
`p1-10-capture-` plus 32 lowercase hexadecimal characters, is created by the
harness with cryptographic randomness, is never a CLI input, and is archived in
the resolved context, process facts, runtime-record meta/frame/terminal lines,
and collision snapshot. The recorder requires the expected ID; missing,
malformed, stale, old-version, or mismatched snapshots remain UNKNOWN/INVALID,
never collision=false. Existing P1-08/P1-09 records without collision authority
remain readable as historical UNKNOWN; old collision snapshot bytes are not
silently accepted as v2.

The runtime fingerprint is computed from the actually loaded `mjModel` by the
shared `common/abs_collision_model_fingerprint.h` specification and by the
offline initial-state probe. Canonical bytes are schema plus NUL, little-endian
u32 body/geom counts, then every geom in ID order containing geom ID, type,
body ownership, group, contype, conaffinity, binary64 pos/quat/size, geom name,
and body name (length-prefixed strings), hashed with SHA-256. The full model is
covered, including non-obstacle geoms; the seven obstacle signatures remain
classification metadata only. Closure SHA-256 identifies the preflight file
closure, while the runtime fingerprint identifies the loaded MuJoCo contact
model. Authority requires both identities and the expected capture binding to
match.

Formal authority scope is only the two harness-controlled `main.cc`
`PhysicsLoop` paths: each such `mj_step` publishes one collision snapshot.
`simulate.cc` UI step-forward is interactive debugging only and is outside the
formal P1-10 capture scope; the Operator runbook prohibits UI reset, keyframe,
teleop, and similar intervention. No “all mj_step” guarantee is made.
Malformed/invalid snapshots invalidate the record; missing, stale, unknown, or
non-authoritative snapshots remain UNKNOWN. The saved-record summary reports
coverage and never turns a final sampled `false` into an episode-wide
collision-free conclusion without a complete episode coverage boundary.

## Binding values

- scenario: `obstacle_test1`
- root XML SHA-256:
  `e12a69fa5463e723d115696b8872c27c71b03a9d029a9ef933343ae93ba6dd5e`
- full closure SHA-256:
  `6ca5da14be6909815ac9c41bf6db0f8108e07082aea5aba22c91e833e6181746`
- formal snapshot version: 1
- formal snapshot size: 264 bytes
- physics source: `mjData::ncon`, `mjData::contact[]`, `mjData::time`

Pre-repair candidate-suite binding after the offline authority update:

- suite: `scenarios/p1_10/obstacle_candidate_suite_manifest.json`
- suite SHA-256:
  `4552fc5b408855174337c7b2d73acf49a92a5b44622a540e0d1b90082ab58cb5`
- `obstacle_test1.json` SHA-256:
  `75cd42879dfb124a81b8028b42b5f3a4a889cd5428b6c68cce37e24c171be4df`

The candidate scenario status remains `UNSUPPORTED` so it cannot enter the
capture suite. Its `collision_authority` sub-binding is now
`IMPLEMENTED / AWAITING RUNTIME VALIDATION`; the four other candidate
collision-authority sub-bindings remain `UNSUPPORTED`. This distinguishes an
implemented source contract from a runtime-validated/capture-eligible
scenario.

The seven obstacle signatures are derived from the frozen `scene_test1.xml`
inventory. `scene_obstacle.xml` is its byte-identical alias and is not a second
collision identity.

## Contact classification

| Contact | Formal classification | Counts as obstacle collision |
|---|---|---|
| robot geom ↔ bound obstacle geom | `RobotObstacle` | Yes, only when model binding succeeds |
| robot geom ↔ `floor` | `Ground` | No |
| robot geom ↔ robot geom | `Self` | No |
| other known non-obstacle pair | `Other` | No |
| out-of-range, missing, or unclassified geom | `Unknown` | No; authority is not used for a false claim |

Robot identification follows the existing model collision-geom identity (group
3), while obstacle identification follows all seven frozen XML-derived
signatures, not a guessed geom index. A model identity mismatch, non-finite
simulation time, invalid physics-step identity, invalid snapshot sequence, or
unknown contact classification fails closed.

## Goal, fall, and timeout status

| Signal | Status | Reason |
|---|---|---|
| goal arrival | `AUTHORITATIVE_SOURCE_TRACE_ONLY` / not formalized as a terminal source | Existing controller evaluation/log path is not an accepted structured terminal producer; no new arrival threshold was invented |
| fall | `UNKNOWN` | No approved deployed structured fall authority was found; no height/angle threshold was added |
| timeout | `UNKNOWN` | The fixed 25 s capture window is not a controller timeout and is not relabeled as one |
| collision event | `AUTHORITATIVE_IMPLEMENTED` for future bound snapshots | Requires successful model binding and valid saved snapshot coverage; no runtime event is claimed here |

Collision contact is an event/outcome observation and does not cause a new stop,
control, switching, or terminal policy. `collision_events=true` is written only
when a valid robot↔bound-obstacle contact is observed. An episode-wide
`collision_events=false` is intentionally not asserted by the current recorder
because its samples do not carry a complete episode coverage boundary.

## Tests and static verification

Offline results completed for this increment:

- `python3 scripts/test_p1_10_collision_authority.py`: **11 PASS**
- `python3 scripts/test_run_record.py`: **54/54 PASS**
- `cmake --build unitree_mujoco/simulate/build2 -j2`: **PASS**, including the
  modified `unitree_mujoco` target; the executable was not launched
- `ctest --test-dir unitree_mujoco/simulate/build2 --output-on-failure`: **1/1
  PASS**
- `test_p1_10_obstacle_inventory.py`: **10 PASS**
- `test_p1_10_scenario_suite.py`: **12/12 PASS**
- `test_p1_10_saved_record_compare.py`: **17/17 PASS**
- `test_formal_runtime_binding.py`: **12/12 PASS**
- `test_formal_experiment_contract.py`: **22/22 PASS**
- `test_formal_runtime_adapter.py`: **16/16 PASS**
- `test_formal_rt_frame_recorder.py`: **12/12 PASS**
- `test_p1_08_sim_clock.py`: **32 checks PASS**
- `test_p1_08_baseline_identity.py`: **21 checks PASS**
- `test_p1_08_harness.py`: **93 checks PASS**
- py-compile of the changed authority/recorder and related test/binding
  modules: **PASS**
- JSON validation of the obstacle candidate suite, historical inventory, frozen
  pair manifest, and accepted P1-08 manifests: **PASS**
- `git diff --check`: **PASS**

The repair-specific offline checks cover capture-ID mismatch/missing/old
version/malformed/stale data, recorder capture binding, deterministic Python
fingerprint mutation sensitivity, the shared v2 snapshot layout, the
no-runtime initial-state probe fingerprint, full obstacle-candidate inventory
regeneration, and preservation of the accepted flat suite/frozen pair. The
instrumented simulator target was compiled but not launched; the accepted
P1-08 executable remains the separate `1e9b330f...` identity. Any future
obstacle capture must use an independently recorded Stage-B manifest/identity
for the newly instrumented executable and may not reuse the accepted P1-08
binary identity.

The collision tests cover positive robot↔obstacle, floor/self/unknown contact
domains, model/version/identity/sequence/time rejection, stale/malformed
snapshots, real filesystem snapshot consumption by the recorder, physics-step
coverage gaps, and the no-false collision-free rule. Historical records without
the optional snapshot remain explicitly UNKNOWN.

## Remaining gate

Independent Reviewer must review this repaired source/contract change before any
Operator is authorized to run `obstacle_test1`. A future runtime validation must
demonstrate the actual bound scene, snapshot identity, contact classification,
coverage, and saved record. This evidence does not establish ABS obstacle
effectiveness, collision occurrence, Recovery behavior, benchmark validity,
paper equivalence, sim-to-real validity, or Phase 1 acceptance.

Current status: **P1-10 Stage B AUTHORITY IMPLEMENTATION — REPAIRED / AWAITING
INDEPENDENT REVIEW**. `obstacle_test1` remains
`IMPLEMENTED / AWAITING RUNTIME VALIDATION`; goal, fall, and timeout remain
UNKNOWN/source-trace-only as documented. No obstacle runtime, A/B replay,
benchmark, FormalRun, P1-11, P1-12, or P1-13 was started, and no Operator
authorization has been issued.
