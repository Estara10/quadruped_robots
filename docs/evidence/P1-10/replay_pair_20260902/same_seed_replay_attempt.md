# P1-10 Director-Authorized Same-Seed Replay Attempt

Date: 2026-09-02

## Scope and decision

Pair: `P1-10-REPLAY-20260902-flat_goal_forward-stabilized`.

The pair manifest was frozen before Run A. Run A was attempted exactly once.
Its P1-10 scenario context resolved successfully, but the harness stopped at
the residual-process preflight: the broad `pgrep` expression matched the
active Codex sandbox command line because the invocation contained the
MuJoCo path. This occurred before any runtime prerequisite or child launch.

The pair is therefore `FAILED_FOR_THIS_PAIR`. Run B was not attempted and no
retry was made. This is not a replay PASS and does not authorize a future
attempt.

## Frozen binding

| Item | Value |
|---|---|
| pair manifest | `pair_manifest.json`, SHA-256 `a5d5de6c58629121a4316719f1fb9ed10d1a3e2e221c40ce57b1236af89fb080` |
| scenario / hash | `flat_goal_forward` / `beba99ed4e6f6c8f84eb1ac514f2da4b6e910c1587fdf91f5e95ac6bc639e092` |
| suite hash | `eb81d60742864fe9c870e957ba3ab601e80da3e64bc48a42c26f849570f3152d` |
| root seed / P1-02 pairing | `20260902` / `6ac5d17dd81862e91dbf8cdbaacf141ca4d0e7b553bf8717f0f722d58b073d96` |
| scenario pairing key | `fc887e89ce542e5062ba6669611cb459c4ffe7de35198c59df29cedc9b2bed29` |
| variant / binding | `stabilized` / `2f0dfc4e8bf5237a578d99030facc38459fd5f899af49b508e48e29b7e8a4e1c` |
| switching | `stabilized_switch` |
| baseline manifest / identity | `2667ed37a854f85e5a7c493e7d4a8b1871a84ce95d3e3b0742801d383f8dc915` / `59dd13fed5ebd026ec519f2659643237502be8e4d8df5174a65b7d35ceb4f7e0` |
| initial state | `scene_default / mj_makeData:qpos0`, qpos hash `a604dd11dc57ea655bf6d746dcf068a91e80a0a1eddc73d20c1a3800468f59d8`, binding `f7907a927c31d3d6a5d497ab274b3d913bf4fc8ccb0e9713a9dbd1e182d0a9a0` |
| window / goal resampling / obstacles | `25.0 s` / `false` / explicit empty |

## Run facts

| Run | Result | PID/PGID | Signals | wait rc | forced/orphan |
|---|---|---|---|---|---|
| A | preflight FAIL at residual-process check | none | none | n/a | no runtime child; post-attempt process check found none |
| B | not run by policy | none | none | n/a | not applicable |

No runtime record, sim-clock timing record, reader stats, process facts,
child logs, or canonical projection exists because launch was never reached.

## Archived raw files

| File | SHA-256 | Bytes |
|---|---|---:|
| `pair_manifest.json` | `a5d5de6c58629121a4316719f1fb9ed10d1a3e2e221c40ce57b1236af89fb080` | 5163 |
| `run_A_preflight_fail.json` | `8e131fce2b1d25a598caec0b412713ee39e4ef67876c5c620d963211844bde0f` | 14485 |
| `pair_result.json` | `d0c242caebb8cdee4536489a148043bb01a68de5db160e7cf93eb461655aac73` | 3299 |
| `raw_inventory.json` | `c9ba45db13222b01be34e57653f5d96a8deb27ab52b6b971462a13a6d9d07649` | 957 |

`pair_result.json` is the machine-readable terminal result. The comparison
status is `NOT_RUN`: both successful runtime records are required, and no live
shared memory was read for comparison.

## Comparison boundary

The frozen protocol would require exact canonical equality for binding fields,
strict `rl_step` sequence, controller/RL/safety/discrete event fields and
terminal success semantics. The numeric projection would include world pose,
command, RA, actions, joint targets, torque and rays; any nonzero float delta
would fail. `run_id`, `session_id`, monotonic/wall-clock timestamps, PID/PGID
and reader polling cadence are excluded. None of these comparisons were
reached in this failed pair.

This attempt produced no formal VALID episode, benchmark, pilot, multi-seed
statistics, obstacle result, paper-faithful/agile-only result, or Phase 1
acceptance. P1-11/P1-12/P1-13 were not started. No commit or push was made.
