# P1-10 Same-Seed Replay Pair — 2026-09-03

Pair ID: `P1-10-REPLAY-20260903-flat_goal_forward-stabilized`  
Outcome: **FAILED_FOR_THIS_PAIR**

The pair manifest was frozen before Run A. It binds the current accepted
`flat_goal_forward` scenario, root seed `20260902`, supported `stabilized`
variant, accepted P1-08 v2 baseline, and `scene_default / mj_makeData:qpos0`.

- Pair manifest SHA-256:
  `b86a19887dee8a441c7a5643eca698ea4a092a92bd549e972d41b1067c8f049e`
- Resolved manifest SHA-256:
  `9b67bbd30dd363cc035a9dc3896e195027fb18ae8170f36b60648b47073cea42`
- Scenario SHA-256:
  `beba99ed4e6f6c8f84eb1ac514f2da4b6e910c1587fdf91f5e95ac6bc639e092`
- Suite SHA-256:
  `eb81d60742864fe9c870e957ba3ab601e80da3e64bc48a42c26f849570f3152d`

Run A was attempted exactly once. Residual-process identity preflight passed
with no matches. X11 preflight then failed with `xdpyinfo rc=1`; no ldd or
runtime child was launched. PID/PGID, signals, waits, runtime records, timing
records, process facts, logs, and canonical replay projections therefore do
not exist and are recorded as not applicable. Run B was not attempted and no
retry was made.

The original
`P1-10-REPLAY-20260902-flat_goal_forward-stabilized` remains
`FAILED_FOR_THIS_PAIR` and is unchanged. P1-10 remains
`IMPLEMENTED / AWAITING INDEPENDENT REVIEW`. This result is not runtime
replay proof, benchmark, pilot, multi-seed evidence, FormalRun acceptance, or
Phase 1 acceptance.
