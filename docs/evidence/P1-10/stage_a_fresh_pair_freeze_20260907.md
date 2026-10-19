# P1-10 Stage-A Fresh Pair Freeze — 2026-09-07

## Freeze result

A new empty Stage-A pair was frozen offline at:

`docs/evidence/P1-10/replay_pair_20260907_stage_a_current_instrumented/`

Pair ID: `P1-10-STAGE-A-REPLAY-20260907-flat_goal_forward-stabilized-current-instrumented`

Pair manifest SHA-256:
`769e7f6bd148ad3111213af2261cb357a89510b511453c8f406b7f5c3a4edc8c`

Status: `FROZEN_OFFLINE_PENDING_INDEPENDENT_REVIEW`.
The directory contains only the frozen `pair_manifest.json`; `run_A/` and
`run_B/` do not exist. No capture ID is included in the canonical execution
identity.

The pair binds `flat_goal_forward`, `stabilized`, root seed `20260902`, fixed
window `25.0 s`, `scene_default / mj_makeData:qpos0`, the flat scene/closure,
current config and plugins, and the current instrumented executable
`e4602a19c60ae8072648c8f113770b8d15a8cc3a0fe5b32ffcfc6e63cb40bc32`
(`4,214,888` bytes). The reused Stage-A execution manifest is
`docs/evidence/P1-10/stage_a_execution_manifest_20260905.json`, manifest
SHA-256 `550e07f0958f46a0a1ec3336736347aa07d25c40f287373af47ea07d4ee09920`,
canonical identity
`2b9852526491ff2ac52fcf6eebe0e8e8ba6584eb7486ba4e8a94a6951b84382f`.
Its canonical identity remains independent of pair locator, date, and runtime
capture ID; only its validated binding is reused.

P1-08 canonical identity
`59dd13fed5ebd026ec519f2659643237502be8e4d8df5174a65b7d35ceb4f7e0` is
parent/provenance only. The current executable is not the accepted P1-08
executable. Collision-v2 instrumentation may exist in the binary, but this
flat pair is not obstacle-collision evidence.

## Disposition and controls

The prior pair
`docs/evidence/P1-10/replay_pair_20260905_stage_a_current_instrumented/`
remains `FAILED_FOR_THIS_PAIR` with its original manifest unchanged. It failed
before child launch during the old residual-process preflight defect and is
not a retry target; it was not overwritten or deleted.

The Stage-A pair tool now accepts an explicit new repository-relative pair
directory/ID and refuses to overwrite an existing pair manifest. The capture
preflight binds evidence to the already-frozen parent pair manifest. The
dedicated comparator accepts only the new frozen `--pair-dir`, derives both run
directories from its manifest, validates saved preflight provenance, and
refuses external paths, symlinks, and output overwrite.

No MuJoCo, ROS2, controller, Run A, Run B, benchmark, FormalRun, or later P1
task was started. The next action is Independent Reviewer review of this
offline freeze; until approval, the Operator must not run the commands in the
runbook.
