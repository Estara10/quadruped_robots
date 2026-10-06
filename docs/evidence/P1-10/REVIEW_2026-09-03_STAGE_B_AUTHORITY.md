# P1-10 Stage B Collision Authority — Final Independent Review

## Decision

**ACCEPT WITH KNOWN ISSUES / STAGE-B OBSTACLE RUNTIME PREPARATION MAY PROCEED**

This accepts only the offline collision-authority implementation for the
canonical `obstacle_test1` candidate.  It does not accept an obstacle runtime
run, the five-map formal suite, P1-10 as a whole, or any later P1 task.

## Accepted offline scope

- A harness-generated capture identity is propagated to the collision v2
  snapshot, runtime record, process facts, and resolved context; mismatches
  fail closed.
- The loaded MuJoCo contact model is bound by the deterministic
  `abs-go2-collision-model-fingerprint/v1` fingerprint, while XML/asset
  closure remains a separate preflight file identity.
- Formal collision publishing is limited to the two harness-controlled
  `main.cc` PhysicsLoop paths.  UI step-forward is interactive debugging and
  is explicitly outside the formal capture scope.
- Only robot-to-bound-obstacle contact is an obstacle collision.  Ground,
  self, other, unavailable, stale, malformed, and unknown contacts do not
  become a false collision-free conclusion.
- The collision observation does not change policy, RA, switching, Recovery,
  terminal behavior, or physics.

## Identity boundary

The accepted P1-08 executable identity is `1e9b330f...`.  The current
instrumented Stage-B executable is a distinct artifact (`e4602a19...` at the
time of review) and must receive its own Stage-B execution manifest/identity.
It must never be represented as the P1-08 accepted executable.  The accepted
P1-08 baseline remains read-only historical evidence.

## Retained known issues / UNKNOWN

- `obstacle_test1` has not had a runtime run; no real contact, coverage, or
  saved obstacle record exists.
- Goal arrival is source-trace-only; fall and controller timeout remain
  `UNKNOWN`.
- Episode-wide collision-free status is not asserted without complete
  coverage.
- The five historical obstacle maps are not an accepted formal scenario suite.
- Historical Stage-B descriptions using the former v1/264-byte snapshot are
  superseded by the current v2/392-byte contract and must not be used as the
  pre-run contract.

## Mechanical evidence reviewed

- Collision authority tests: 11 PASS.
- Model-fingerprint mutation target: 9/9 PASS.
- Scenario, inventory, recorder, comparator, P1-02 and P1-08 regressions:
  PASS as recorded in the Stage-B authority evidence.
- Simulator build and CTest: PASS.

No MuJoCo, ROS2, controller, obstacle pair, benchmark, FormalRun, or later P1
task was run in this review.

## Phase status

P1-10 remains **IMPLEMENTED / AWAITING INDEPENDENT REVIEW** overall.  Stage-B
runtime preparation may proceed only after its independent execution identity
and preflight contract are frozen.  P1-11/P1-12/P1-13 do not start
automatically.  Phase 1 remains **NOT ACCEPTED**.
