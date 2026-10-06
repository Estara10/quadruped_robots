# P1-10 Historical Five-Map Formalization — Offline Evidence

Date: 2026-09-03. Role: Execution. This is an offline preparation record. It
does not claim an obstacle runtime, collision occurrence, Recovery occurrence,
ABS obstacle effectiveness, benchmark, FormalRun, or P1-10 acceptance.

## Inventory and deduplication

The complete machine-readable inventory, including every closure file's path,
SHA-256 and byte size, is
[`historical_five_map_inventory_20260903.json`](historical_five_map_inventory_20260903.json).
The parser recursively follows XML includes and file-backed mesh/hfield/
texture/skin references and records failures closed.

| Historical root | Root SHA-256 | Closure SHA-256 | Static obstacle geoms | Formal identity | Static count only |
|---|---|---|---:|---|---|
| `scene_test1.xml` | `e12a69fa5463e723d115696b8872c27c71b03a9d029a9ef933343ae93ba6dd5e` | `6ca5da14be6909815ac9c41bf6db0f8108e07082aea5aba22c91e833e6181746` | 7 | `obstacle_test1` | low/medium |
| `scene_test2.xml` | `aa39cda3361e63338dfdab5800ee19c6595ef04a853632e1e51b377385d0a480` | `59d73776386b14a5c45e72bc887cfe0effacf114931a6238692cae131978f73a` | 8 | `obstacle_test2` | medium |
| `scene_test3.xml` | `d779d66e53eaa4d7c085d9f40fd1d8178b5852adf020261252df5485df9347b4` | `3f19ba623c72e4ea10f1bec2684ebd4c12ca464871f5a6cb100c14eb19ea9077` | 10 | `obstacle_test3` | highest count |
| `scene_test4.xml` | `8c7cf3c8655cae0d577c3c9faea3e459ca21b223641f13d6cb59d4647e9005ba` | `fd7a3467366d2d00b21c435d177b1931237dd101c102e62c7e0dbf717e67813b` | 6 | `obstacle_test4` | lowest count |
| `scene_test5.xml` | `638902e9cab1163d9c637aa7a6061906fc39f9095c8ea1b28796a6a4138fe37b` | `54fa37ed13d495ed4007b27f61d7aeebaf22e9b1e620a9d54e587b0a7afe8260` | 9 | `obstacle_test5` | high count |

`scene_obstacle.xml` is byte-identical to `scene_test1.xml` (root SHA
`e12a69fa5463e723d115696b8872c27c71b03a9d029a9ef933343ae93ba6dd5e`) and is therefore an alias/duplicate of `obstacle_test1`,
not a sixth map. `scene_terrain.xml` is separately recorded as a future /
generalization candidate (root SHA `90b744c3c6bec809ba830d572b2ca38d9373fa07b7cbefad6efc579e720463b2`, closure SHA
`bfdf31a5919a7e6f4a8753b6bd89ef94b526086000eebf3aae3644fda47d8ede`) and is not included in the five-map count.

The obstacle list in each candidate is derived from root `worldbody/geom`
element order with a stable XML path. Each entry records type, raw and parsed
pos/size/quat, and collision-related attributes. The historical root obstacle
geoms omit `contype`, `conaffinity`, `group`, `condim`, `margin`, `gap`, and
`priority`; omission is recorded as `XML_OMITTED` / runtime `UNKNOWN`, not
replaced with an asserted MuJoCo default.

## Candidate suite status

The independent candidate suite is
[`obstacle_candidate_suite_manifest.json`](../../../scenarios/p1_10/obstacle_candidate_suite_manifest.json)
and its five scenario files are `scenarios/p1_10/obstacle_test1.json` through
`obstacle_test5.json`. The candidate suite manifest SHA-256 is
`427cd676f3e9d4c67f53900dcaa2e2c516152298d4f580b684a935a8b6a9d0f8`.
Every candidate is explicitly hash-bound but currently
`UNSUPPORTED`: the existing accepted flat capture suite remains unchanged,
formal collision/terminal authority is unavailable, and no obstacle
repeatability evidence exists. None is `CAPTURE_ELIGIBLE`, and no Operator
launch is authorized.

### Stage B authority amendment

After the offline Stage B authority implementation, the current candidate-suite
manifest was regenerated with SHA-256
The pre-repair candidate-suite manifest was
`4552fc5b408855174337c7b2d73acf49a92a5b44622a540e0d1b90082ab58cb5`.
The scenario status for all five candidates remains `UNSUPPORTED` and no
candidate is capture eligible. Only the nested `collision_authority` binding
for `obstacle_test1` changed to
`IMPLEMENTED / AWAITING RUNTIME VALIDATION`; the other four remain
`UNSUPPORTED`. The earlier hash above is retained as the pre-amendment
inventory snapshot, not as the current suite identity.

Stage B prioritizes `obstacle_test1` because it is the canonical representative
after deterministic de-duplication and exercises the smallest first formal
candidate while preserving the existing geometric ray source trace. This is a
priority choice, not a claim of behavioral success or difficulty. Stage C
expands the remaining four only after Stage B authority and runtime evidence
are independently closed.

All candidates bind `stabilized/stabilized_switch`, root seed `20260902` as
pairing/provenance only, `25.0 s`, `scene_default / mj_makeData:qpos0`, fixed
goal `[7.0, 0.0]`, and the accepted P1-08 baseline manifest SHA
`2667ed37a854f85e5a7c493e7d4a8b1871a84ce95d3e3b0742801d383f8dc915`, baseline
identity-document SHA `6c3563c25d45cc275db6b083f9f0fc0cc2067b48bc8f4a93dcace9f6d42817ea`,
and canonical identity `59dd13fed5ebd026ec519f2659643237502be8e4d8df5174a65b7d35ceb4f7e0`.

## First-map source trace: `obstacle_test1`

| Segment | Classification | Offline finding |
|---|---|---|
| MuJoCo obstacle geoms → ray producer | `MATCH / IMPLEMENTED SOURCE TRACE` | `unitree_sdk2_bridge.h::computeRay2d` reads the model and applies the geometric candidate filter. Boxes/cylinders/spheres/capsules/ellipsoids can be considered; floor, plane/hfield/mesh, robot groups 2/3, and dynamic bodies are excluded. |
| Ray producer → StateRL | `MATCH / IMPLEMENTED SOURCE TRACE` | Versioned `/mujoco_ray2d` data are read by `StateRL::updateRay2d`; missing, stale, incoherent, nonfinite data fail closed. |
| Ray → RA 19-D / Agile 61-D | `MATCH / IMPLEMENTED SOURCE TRACE` | `computeRAObservation`/`runRAModel` and the Agile observation path consume the ray features. |
| RA → stabilized switching | `MATCH / IMPLEMENTED SOURCE TRACE` | `RASwitchingLogic::stepSwitching` applies the existing stabilized thresholds/hold logic. No threshold was changed. |
| switching enter edge → Recovery | `MATCH / IMPLEMENTED SOURCE TRACE` | The enter edge invokes the existing Recovery twist/observation path. This source trace is not a runtime event record. |
| MuJoCo contact → structured runtime record | `IMPLEMENTED SOURCE TRACE` | The Stage B versioned `/mujoco_collision_v2` snapshot is written after the existing physics step, bound to obstacle_test1 model identity, and consumed by `RunRecordRecorder`; runtime validation remains pending. The legacy `/mujoco_collision` is diagnostic only. |
| goal/fall/timeout → terminal/outcome | `MISSING IMPLEMENTATION / UNSUPPORTED` | The formal reducer has no verified authoritative producer for these fields and correctly preserves `UNKNOWN`; no event is fabricated. |

The current ray contract is 11 rays at -45° through 45°, range 0.1–6.0 m,
body-yaw frame with the documented -0.05 m x-origin, and `log2(distance_m)`
storage. A no-hit is max-range encoding; missing/stale/incoherent/nonfinite
frames fail closed. Effective obstacle source/frame validity remains `UNKNOWN`
until an authorized obstacle capture proves it. The XML omission of collision
attributes and the exclusion rules are explicit risks to close before runtime;
the inventory does not infer that every declared geom is sensed.

The collision authority implementation is behavior-neutral and limited to the
simulator's authoritative `mjData` contact scan → versioned,
timestamped/session-bound snapshot → existing recorder/reducer boundary.
Goal, fall, and controller-timeout authority remain UNKNOWN or source-trace-only
as stated above; no new terminal threshold or event is fabricated.

## Offline verification and state

`test_p1_10_obstacle_inventory.py` covers five-map parsing, closure/hash
recalculation, XML metadata binding, root/asset/goal/qpos/metadata mutation
rejection, mechanical de-duplication, unsupported capture rejection, flat
suite/frozen-pair preservation, ray-contract fail-closed boundaries, and
terrain exclusion. Result: **10 offline obstacle-inventory tests PASS**.

The accepted flat suite SHA remains the frozen pair's recorded value
`eb81d60742864fe9c870e957ba3ab601e80da3e64bc48a42c26f849570f3152d`.
The latest flat saved-record pair remains
`FROZEN_OFFLINE_PENDING_INDEPENDENT_REVIEW`, manifest SHA
`86ae55914db294d269d6f70909bfad1878c287c644f5a85c4075fa758f923a6c`, under
`docs/evidence/P1-10/replay_pair_20260903_saved_record_closure/`. Historical
failed pairs remain `FAILED_FOR_THIS_PAIR` and were not retried.

No MuJoCo, ROS2, controller, A/B replay, benchmark, FormalRun, P1-11, P1-12,
or P1-13 runtime was started. P1-10 remains **IMPLEMENTED / AWAITING
INDEPENDENT REVIEW**; the final state of this preparation is:

**P1-10 Stage B/C PREPARATION — IMPLEMENTED / AWAITING INDEPENDENT REVIEW**

The subsequent Stage B authority increment is recorded separately in
[`stage_b_collision_terminal_authority_20260903.md`](stage_b_collision_terminal_authority_20260903.md).
The current task state is therefore
**P1-10 Stage B AUTHORITY IMPLEMENTATION — IMPLEMENTED / AWAITING INDEPENDENT REVIEW**;
the earlier Stage B/C preparation label above is retained as the historical
state of this inventory evidence.
