# P1-10 Offline Verification

Date: 2026-09-02

## Result

The offline implementation remains **P1-10 — IMPLEMENTED / AWAITING INDEPENDENT
REVIEW**, but the Director-authorized same-seed replay pair is
**FAILED FOR THIS PAIR**. This file records the offline contract and the
separate replay attempt; it does not claim replay success.

## Exact bindings

- Suite: `scenarios/p1_10/scenario_suite_manifest.json`
  SHA-256 `eb81d60742864fe9c870e957ba3ab601e80da3e64bc48a42c26f849570f3152d`.
- Accepted P1-08 v2 baseline identity:
  `59dd13fed5ebd026ec519f2659643237502be8e4d8df5174a65b7d35ceb4f7e0`.
- Accepted P1-08 baseline manifest SHA-256:
  `2667ed37a854f85e5a7c493e7d4a8b1871a84ce95d3e3b0742801d383f8dc915`.
- Go2 flat model closure:
  `8d9218de0dc02978fc0ef4ba1c790fa3b968fbdbfdb945e14522436a2574ea07`.

## Checks

| Test | Result |
|---|---|
| suite index and all scenario files parse/hash/validate | PASS |
| actual MuJoCo construction path qpos/pose/yaw extraction | PASS; no `mj_step` |
| same scenario + root seed gives byte-identical resolved manifest | PASS |
| different root seed changes pairing identity; no runtime random consumer claimed | PASS |
| scenario/scene/override/seed-registry mutation changes hash input | PASS |
| missing field, path escape, unregistered override, unsupported source | PASS / fail closed |
| `resample_goal_on_arrival=true` | PASS / rejected |
| actual `--scene`, `--manifest`, `--window-s`, initial-state-source mismatch | PASS / rejected |
| paired labels and cross-pair seed rejection | PASS |
| stabilized consumed variant binding; paper-faithful/agile-only rejection | PASS |
| P1-02 formal validator/seed/pairing regression | 22/22 PASS |
| P1-08 harness regression, including residual-process identity tests | 93 checks PASS |

The offline fixture root seed is `20260902`; for the fixed flat scenarios it is
pairing/provenance identity only. `derived_seeds` is empty and the Python
orchestrator source is `DECLARED_NOT_CONSUMED`; no random producer is claimed
without a real consumer. The resolved manifest's `formal_context` carries the
initial-state and consumed-variant bindings.

Resolved evidence file hashes: forward
`9b67bbd30dd363cc035a9dc3896e195027fb18ae8170f36b60648b47073cea42`;
lateral `84688059c7234d13e73d9b19869ec6ec9cd5f0d54598acf1fbe8714794ee127b`.

## Director-authorized replay pair result

- Pair: `P1-10-REPLAY-20260902-flat_goal_forward-stabilized`.
- Frozen pair manifest SHA-256:
  `a5d5de6c58629121a4316719f1fb9ed10d1a3e2e221c40ce57b1236af89fb080`.
- Run A was attempted exactly once and failed before child launch: the
  broad residual-process `pgrep` matched the current Codex sandbox command
  line because the invocation contained the MuJoCo path.
- No MuJoCo or ROS2 process was started, no PID/PGID or signal/wait facts
  exist, and no runtime record was created. Run B was not attempted and no
  retry was made, as required by the authorization.
- Raw preflight evidence:
  `run_A_preflight_fail.json` SHA-256
  `8e131fce2b1d25a598caec0b412713ee39e4ef67876c5c620d963211844bde0f`.
- Machine-readable result:
  `pair_result.json` SHA-256
  `d0c242caebb8cdee4536489a148043bb01a68de5db160e7cf93eb461655aac73`.
- Raw inventory: `raw_inventory.json`; runtime raw logs and records are
  empty because launch was never reached.

## Boundary

The authorized replay attempt did not reach MuJoCo/ROS2 runtime capture, so no
same-seed runtime replay was demonstrated. No formal `VALID` episode,
benchmark, pilot, multi-seed statistic, or real-robot result exists.
P1-11/P1-12/P1-13 were not started, and no commit or push was performed.
Runtime event-sequence replay and every producer not covered by the current
flat path remain `UNKNOWN`/deferred. The pair is terminally failed pending a
new Director decision; this evidence does not authorize a retry.

## Residual-process preflight correction

The pair failure root cause was the prior `pgrep -af` substring rule: the
capture invocation itself contained `unitree_mujoco`, so the active Codex
sandbox command line was reported as a residual runtime process. The harness
now uses `/proc` executable identity plus exact argv attribution. Exact
MuJoCo executable identity, the capture's ROS launch argv, and
`ros2_control_node` with the Go2 controller config remain hard failures.
Self/ancestor PIDs and unrelated shells that merely mention a simulator path
are excluded. `/proc` inspection errors, malformed records, and ambiguous
controller attribution return `uncertain` and remain fail-closed.

Focused offline tests pass as part of the **93-check** P1-08 harness suite.
This task performs no replay and does not authorize a retry of the failed pair.
Independent Reviewer approval and a new Director authorization are required
before any future replay attempt.
