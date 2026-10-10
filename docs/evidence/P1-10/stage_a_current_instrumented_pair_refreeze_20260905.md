# P1-10 Stage A Current-Instrumented Flat Pair Re-Freeze — Offline Evidence

Date: 2026-09-05

## Scope and status

This evidence records an offline-only Stage-A identity and pair freeze. No
MuJoCo, ROS2, controller, Run A, Run B, benchmark, FormalRun, or later P1 task
was started. No capture data were produced. Operator execution remains
prohibited until independent review accepts this closure.

The final status is:

`P1-10 Stage A PREPARATION — IMPLEMENTED / AWAITING INDEPENDENT REVIEW`

## Old-pair disposition

The prior saved-record closure pair
`replay_pair_20260903_saved_record_closure/` is marked
`SUPERSEDED_NOT_RUN`. It was bound to the unavailable P1-08 executable
`1e9b330f2b6c39dabaaa8424ee53c41d3be08ea00eb3e69ba71f332de50654e2`, while
the current `build2` executable is a different instrumented artifact. The old
pair is retained for historical readability and was not retried or rewritten
as a runtime failure.

## New frozen identity

| Item | Frozen value |
|---|---|
| Pair ID | `P1-10-STAGE-A-REPLAY-20260905-flat_goal_forward-stabilized-current-instrumented` |
| Pair directory | `docs/evidence/P1-10/replay_pair_20260905_stage_a_current_instrumented/` |
| Pair manifest SHA-256 | `f463e1c3f0e33cfcc23deeddd255522857907995e87ac0bae43af2f2a3af508b` |
| Pair status | `FAILED_FOR_THIS_PAIR` |
| Stage-A manifest | `docs/evidence/P1-10/stage_a_execution_manifest_20260905.json` |
| Stage-A manifest SHA-256 | `550e07f0958f46a0a1ec3336736347aa07d25c40f287373af47ea07d4ee09920` |
| Canonical identity SHA-256 | `2b9852526491ff2ac52fcf6eebe0e8e8ba6584eb7486ba4e8a94a6951b84382f` |
| Scenario / variant | `flat_goal_forward` / `stabilized` |
| Root seed / window | `20260902` / `25.0 s` |
| Initial state | `scene_default / mj_makeData:qpos0` |
| Current executable SHA-256 / size | `e4602a19c60ae8072648c8f113770b8d15a8cc3a0fe5b32ffcfc6e63cb40bc32` / `4,214,888` bytes |
| Parent provenance only | P1-08 canonical identity `59dd13fed5ebd026ec519f2659643237502be8e4d8df5174a65b7d35ceb4f7e0` |

The canonical identity contains only repository-relative paths and frozen
content identities. The absolute executable path is retained only as a runtime
locator in the manifest and is excluded from the canonical identity; no
timestamp or capture ID is included. Collision-v2 instrumentation may be
present in the current binary, but this flat pair does not interpret or claim
obstacle-collision evidence.

## Preflight-to-comparator contract

The future harness invocation must include the explicit
`--stage-a-execution-manifest` argument. The harness generates the per-run
capture ID internally and writes it into the resolved context and preflight
evidence; it is not a CLI input and is not part of the frozen canonical pair
identity.

The dedicated
`scripts/p1_10_stage_a_saved_record_compare.py` accepts only `--pair-dir` and
derives the pair manifest and exactly `run_A`/`run_B` from it. Each run must
contain these five files:

- `preflight_evidence.json`
- `runtime_record.jsonl`
- `process_facts.json`
- `scenario_resolved_manifest.json`
- `p1_10_context.json`

The comparator requires the preflight evidence to bind the Stage-A manifest,
pair identity, current executable hash/size, scene root/full closure,
config/plugin records, scenario/variant/seed/window/qpos, and the production
context. It revalidates the resolved manifest and context through the reviewed
full-binding contract. Missing, malformed, drifted, substituted, symlinked, or
externally located artifacts reject closed. Comparison input remains saved
`run_A/runtime_record.jsonl` and `run_B/runtime_record.jsonl` only; live shared
memory is never read. Deterministic projection/diff outputs refuse overwrite.

## Offline verification

| Check | Result |
|---|---|
| Stage-A identity generation and independent recomputation | PASS |
| Current instrumented binary hash/size binding | PASS |
| P1-08 accepted executable cannot be Stage-A identity | PASS |
| Production preflight/context writer → five-artifact fixture → comparator | PASS |
| Preflight missing/binary/closure/config/scenario/variant/seed/window/qpos/pair drift | REJECT as required |
| Wrong pair path, pair/run/artifact symlink, output overwrite | REJECT as required |
| Stage-A execution identity tests | PASS; 6 tests |
| Stage-A saved-record comparator filesystem tests | PASS; 4 tests |
| Existing historical comparator regression | PASS; 17 test methods |
| JSON validation / Python compilation / `git diff --check` | PASS |

The fixture tests create only temporary synthetic offline files under the new
pair directory and remove them after each test. Run A subsequently stopped
before child launch on the residual-process inspection defect; the pair is
`FAILED_FOR_THIS_PAIR` and must not be retried. A new fresh pair and new
Director authorization are required. The pair has no `run_A`, `run_B`, or
runtime output.

## Boundaries

This is identity/preflight preparation, not flat behavior evidence, ABS
effectiveness evidence, P1-08 baseline replay, benchmark, FormalRun, or Phase
1 acceptance. Obstacle runtime remains unauthorized. P1-10 remains not finally
accepted, and P1-11/P1-12/P1-13 do not start automatically.
