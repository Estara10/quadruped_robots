# Independent Review — P1-10 Residual-Process Preflight Repair

Date: 2026-09-02  
Review scope: residual-process preflight detection only.

## Decision

**ACCEPT WITH KNOWN ISSUES / NEW REPLAY AUTHORIZATION MAY BE REQUESTED**

This decision accepts the residual-process preflight repair sub-gate only. It
does not self-accept or accept the overall P1-10 task.

## Accepted facts

- Residual detection uses `/proc` process-identity classification rather than
  broad command-line substring matching.
- The detector excludes its own PID and the ancestor execution chain.
- Exact MuJoCo and attributable ROS launch/controller identities are rejected
  as live residual runtime processes.
- Uncertainty, process-inspection errors, malformed identity data, ambiguous
  controller attribution, and relevant zombie state fail closed.
- Focused offline coverage passes, including the corrected
  `test_p1_08_harness.py` result of **93 checks**.

## Known issue and boundary

The earlier evidence count was stale (`92` checks); it has been corrected to
`93` checks. This is a documentation correction only.

This review does not accept P1-10, does not create runtime replay evidence,
and does not change the original
`P1-10-REPLAY-20260902-flat_goal_forward-stabilized` pair. That original pair
remains `FAILED_FOR_THIS_PAIR` and was not retried. No same-seed replay is
authorized by this record alone; any new replay requires the separately
recorded Director authorization.

## Referenced evidence

- [`residual_process_preflight_closure_20260902.md`](residual_process_preflight_closure_20260902.md)
- [`residual_process_preflight_closure_20260902.json`](residual_process_preflight_closure_20260902.json)
- [`P1-10.md`](../../exec-plans/P1-10.md)
