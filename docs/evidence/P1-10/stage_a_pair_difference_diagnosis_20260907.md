# P1-10 Stage-A Pair Difference Diagnosis — 2026-09-07

## Scope and disposition

This is a read-only diagnosis of the already executed pair:

`docs/evidence/P1-10/replay_pair_20260907_stage_a_current_instrumented/`

The pair is completed runtime evidence and is not retryable. Run A and Run B
were each executed once before this diagnosis; neither run was started by this
task. The pair manifest was not changed. The existing comparator output was not
changed.

The pair manifest remains the frozen manifest with SHA-256
`769e7f6bd148ad3111213af2261cb357a89510b511453c8f406b7f5c3a4edc8c`.
The existing saved-record comparator result is:

- status: `FAIL`;
- difference_count: `66604`;
- canonical projection SHA-256:
  `4f190d6b6d50a0becd1ae003f5abf491b6d7c5311baa02958b6c46eeaad47c16`;
- every reported difference has kind `value`;
- all 1251 LIVE frame indices have at least one reported numeric difference.

## Saved evidence read

For both `run_A/` and `run_B/`, this diagnosis read:

- `runtime_record.jsonl`;
- `sim_clock_timing.jsonl`;
- `rt_frame_timing.jsonl`;
- `preflight_evidence.json`;
- `process_facts.json`;
- `mujoco_raw.log`;
- `ros2_launch_raw.log`;
- `orchestrator_raw.log`;
- `diff_report.json` at the pair root.

No live shared memory, process launch, comparator rerun, or runtime command was
used.

## Run facts

| Field | Run A | Run B |
|---|---:|---:|
| run_id | `1fdea48e1725437c8a12676888778cf2` | `0c761e333de346a9832486ea6b4a7f54` |
| capture_id | `p1-10-capture-1e94a0334a026d78bf342485fab377d4` | `p1-10-capture-d6f1c5dbad11c18ea747ca6a1d5511cf` |
| record format | 2 | 2 |
| record lines | 1253 | 1253 |
| LIVE frames | 1251 | 1251 |
| terminal records | 1, last | 1, last |
| process exit_code | 0 | 0 |
| required child wait rc | MuJoCo 0; ROS launch 0 | MuJoCo 0; ROS launch 0 |
| shutdown_complete | `true` | `true` |
| normal_shutdown | `true` | `true` |
| forced_termination | `false` | `false` |
| shutdown_request_source | `SIGINT` | `SIGINT` |
| cleanup escalation/errors | none | none |
| orphan inventory state | `none` | `none` |
| terminal termination_reason | `FRAMES_ENDED_RC0` | `FRAMES_ENDED_RC0` |
| terminal outcome fields | goal/fall/timeout/collision `UNKNOWN` | goal/fall/timeout/collision `UNKNOWN` |

The terminal records have `fact_validation_errors=[]`. The saved facts therefore
support individual Run A and Run B success as capture executions. They do not
support an exact replay match or any obstacle/effectiveness conclusion.

## First and last LIVE frames

The `sim_time` values below are the embedded collision-snapshot diagnostic
values. Both runs mark that snapshot `available=false`, `authoritative=false`,
and `status=UNKNOWN`; they are reported for alignment diagnosis, not as a
formal collision-time authority.

| Field | Run A first | Run B first |
|---|---:|---:|
| `rl_step` | 96 | 95 |
| `source_sequence` | 194 | 192 |
| `sim_time` | 13.948000000001327 | 12.384000000000805 |
| `monotonic_ns` | 6901813846903 | 7025889748223 |
| `world_pose` | `[1.4004426002502441, 0.31052249670028687, -0.05990318953990936]` | `[1.3966840505599976, 0.25299566984176636, -0.027406426146626472]` |
| `ra_value` | -0.8249688148498535 | -0.8274896144866943 |
| `policy_state` | 0 / `AGILE` | 0 / `AGILE` |

| Field | Run A last | Run B last |
|---|---:|---:|
| `rl_step` | 1346 | 1345 |
| `source_sequence` | 2694 | 2692 |
| `sim_time` | 38.94200000000163 | 37.377999999999716 |
| `monotonic_ns` | 6926809454896 | 7050885603253 |
| `world_pose` | `[7.137818813323975, -0.12471076846122742, 0.177810400724411]` | `[7.121745586395264, -0.11767122894525528, -0.07365522533655167]` |
| `ra_value` | -0.9988781809806824 | -0.9988783001899719 |
| `policy_state` | 0 / `AGILE` | 0 / `AGILE` |

The first LIVE frames are already different in `world_pose`, action values,
RA, joint targets, and torque. The comparator's first reported paths are
`$.frames[0].numeric.action_clipped[...]`; it reports numeric differences over
all 1251 frames. The numeric difference counts by field are:

| Field | Difference count |
|---|---:|
| `action_clipped` | 15012 |
| `action_raw` | 15012 |
| `torque_nm` | 15012 |
| `joint_target_rad` | 14986 |
| `world_pose` | 3753 |
| `ra_value` | 1220 |
| `command` | 357 |
| Total numeric differences | 65352 |

The remaining 1252 entries are exact-sequence value differences: 1251 entries
under `$.rl_step_sequence[...]` plus one initial anchor under
`$.recovery_entry_exit_sequence[0].rl_step`. The two `rl_step` sequences are
not equal because A starts at 96 and B at 95, although each individual sequence
is internally contiguous with unit deltas. Thus `diff_report.json` contains
65352 numeric value differences and 1252 exact-sequence differences.

## Ordering and stream coverage

The orchestrator logs establish the same logical order in both runs:

`RL active` → `runtime record started` → fixed 25-second sampling → cleanup.

The human-readable log timestamps are both at second resolution. In the saved
record, the meta-to-first-recorded-LIVE delay is 378093 ns for A and 507735 ns
for B. The first LIVE frame timing is therefore not an identical capture
boundary across the two launches.

Policy and frame coverage facts:

- both runs have exactly 1251 LIVE frames;
- A `rl_step`: 96..1346, contiguous, 1251 unique values;
- B `rl_step`: 95..1345, contiguous, 1251 unique values;
- A `source_sequence`: 194..2694, unique, delta 2 per frame;
- B `source_sequence`: 192..2692, unique, delta 2 per frame;
- both policy-state label sequences contain only `AGILE` and have the same
  state-label pattern;
- neither run contains a Recovery state/event;
- a transition list keyed by the initial `rl_step` differs only in that initial
  anchor (A=96, B=95), not in observed policy-state behavior;
- both `rt_frame_timing.jsonl` streams contain 1251 samples and match their
  respective first/last LIVE frame timestamps;
- A sim-clock stream has 12500 accepted rows, B has 12497; observed adjacent
  sim-time increments are nominally 0.002 s, with one stride-4 sample gap in A
  and three in B. This is a stream-coverage difference, not evidence that
  either run's runtime record was invalid.

The logs show controller active before RL active in both runs. Run A reached
RL active/record start at 14:15:19 after its first sampled sim time of about
13.948 s. Run B reached RL active/record start at 14:17:23 after its first
sampled sim time of about 12.384 s. Both runs also contain the same FIFO real-
time scheduling warning (`Operation not permitted`); both nevertheless ended
with rc=0 and no forced cleanup. The warning alone does not prove a causal
numeric divergence.

## Binding and physics consistency

The two preflight/context/process artifact sets agree on the frozen bindings:

- scenario `flat_goal_forward`, scenario SHA-256
  `beba99ed4e6f6c8f84eb1ac514f2da4b6e910c1587fdf91f5e95ac6bc639e092`;
- suite SHA-256
  `eb81d60742864fe9c870e957ba3ab601e80da3e64bc48a42c26f849570f3152d`;
- variant `stabilized`, binding SHA-256
  `2f0dfc4e8bf5237a578d99030facc38459fd5f899af49b508e48e29b7e8a4e1c`;
- root seed `20260902`, fixed window `25.0 s`;
- scene `scene_flat.xml`, root SHA-256
  `9ce83b3e61c722a523d0359536cee803f17610f95d2275fc32e96801ec3c1908`;
- model closure SHA-256
  `8d9218de0dc02978fc0ef4ba1c790fa3b968fbdbfdb945e14522436a2574ea07`;
- initial-state binding SHA-256
  `18dc81bbb45a5c4addb5a681928e45a708cada8a70d2c265f68e907332643f97`;
- initial qpos SHA-256
  `a604dd11dc57ea655bf6d746dcf068a91e80a0a1eddc73d20c1a3800468f59d8`;
- instrumented executable SHA-256
  `e4602a19c60ae8072648c8f113770b8d15a8cc3a0fe5b32ffcfc6e63cb40bc32`;
- simulator config SHA-256
  `86fcfab9ecdf888901340697ef9c99fcc72bbd3d88c86f2178b4ce2ab2c88b95`;
- robot-control config SHA-256
  `59c61ad4f29c2b37b3741236b35ce11c773fb6cfe16ed8d58b92757febf8bf7c`;
- ABS config SHA-256
  `1cd42c4bb29baad1873bd55e7f2f1d82fb0c8ec8bf35fb8e4eff61338758c586`;
- controller plugin SHA-256
  `2b31e558471227a385906239a4fd20d1f9cb759a4960c743b6f5065f13fe6d4e`.

Both records carry the same runtime model fingerprint
`7a516b363665a9e7108b07d3b6383d180a709c783bb095f177864cc452d23f86`.
The saved evidence therefore shows no scene/config/executable/hash drift.
The observed sim-clock streams are consistent with the same nominal physics
timestep of 0.002 s, but their sample counts and gap counts differ as recorded
above.

## Minimum root-cause classification

**Minimum supported classification: `startup alignment`.** The first captured
simulation phase differs (`13.948` s versus `12.384` s), and the first policy
step differs (96 versus 95), while fixed bindings and model identity agree.
The capture starts after asynchronous launch/controller/RL progression rather
than at a shared recorded simulation-step anchor. This is sufficient to explain
why exact frame-0 comparison fails without alleging a model or scene change.

`scheduler/process timing` is a possible contributing mechanism because the
two launches have different startup/capture timing and both show a non-fatal
FIFO scheduling warning, but the saved evidence does not isolate it as the
causal mechanism. `unbound random source` is not supported: the frozen context
declares no consumed random producer for this fixed scene/default-state/fixed-
goal path, and no binding drift is present. No weaker `unknown` classification
is needed for the observed comparator failure; exact scheduler causality remains
unproven.

## Exact comparison rule and next measurement

The current all-numeric exact-equality rule is appropriate for bit-exact replay
only when both launches have a demonstrably identical synchronization boundary,
stream start, and scheduling contract. It is not sufficient as the sole
reasonable repeatability metric for two independently scheduled physical-model
launches. The current FAIL is a true statement that the saved projections are
not identical; it is not by itself an algorithm-failure or P1-10 acceptance
verdict.

Before changing the comparator, Director approval is needed for evidence that
would support a minimum replacement contract:

1. an authoritative common alignment marker (for example a recorded RL-entry
   or simulation-step marker) and launch-to-marker timing in both runs;
2. frame/step coverage and gap rules after alignment;
3. pre-registered numeric metrics such as per-field max/mean/RMSE/quantiles with
   thresholds justified by repeated controlled runs, rather than invented here;
4. exact comparison for categorical policy/recovery event sequences and
   terminal/process validity;
5. multiple independent same-binding runs, with any seed and random-source
   audit completed before interpreting the metric as behavioral repeatability.

The present pair does not contain enough evidence to choose numeric tolerances
or to distinguish scheduler timing from a more specific startup mechanism.
The next decision is therefore either to repair and measure a common startup
alignment, or to define and approve a bounded behavioral-repeatability metric
based on the evidence above. No comparator threshold was changed in this task.

## Status

- pair: completed, retained, and non-retryable;
- Run A: individual capture success;
- Run B: individual capture success;
- exact saved-record replay comparison: `FAIL`;
- P1-10: **IMPLEMENTED / AWAITING INDEPENDENT REVIEW**;
- flat replay remains a Stage-A infrastructure/repeatability sub-gate, not
  P1-10 final acceptance;
- no obstacle runtime, benchmark, FormalRun, P1-11, P1-12, or P1-13 was
  started by this diagnosis.
