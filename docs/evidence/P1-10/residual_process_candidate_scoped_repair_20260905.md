# P1-10 Residual-Process Candidate-Scoped Preflight Repair — 2026-09-05

## Disposition

The current-instrumented Stage-A pair
`replay_pair_20260905_stage_a_current_instrumented` is
`FAILED_FOR_THIS_PAIR`. Run A stopped before child launch and must not be
retried. A future attempt requires a new fresh pair and a new Director
authorization.

This change repairs only residual-process preflight identity recognition. It
does not reopen or modify P1-08 evidence, and it does not produce runtime
evidence.

## Observed failure

The failed preflight recorded:

`ProcessInspectionError: pid=1: unable to read identity: Permission denied: /proc/1/exe`

The previous scan treated one unrelated system PID inspection failure as a
global `uncertain` result, although no MuJoCo, ROS launch, or Go2 controller
match was present and no child was launched.

## Contract repair

`p1_08_baseline_capture.py` now retains per-PID partial `/proc` evidence. A
permission or read failure for a PID with no readable runtime-shaped candidate
evidence is recorded as `uninspectable_non_candidate` and scanning continues.
The rule is identity-based and does not special-case PID 1.

If the readable `exe`/`cmdline` already identifies the exact MuJoCo executable,
the attributable ROS launch, or the Go2 `ros2_control_node` candidate, any
remaining identity failure is `uncertain` and preflight rejects. Exact runtime
residuals, controller attribution ambiguity, zombie candidates, malformed
identity, and self/ancestor exclusion behavior remain fail-closed. Shell
command lines that merely mention a MuJoCo path remain non-candidates.

No broad substring `pgrep` was restored.

## Verification

- `scripts/test_p1_08_harness.py`: **96 checks PASS**.
- Added coverage for unrelated `/proc/<pid>/exe` `PermissionError`, runtime-shaped
  partial identity failure, identified MuJoCo with `stat` failure, exact
  residuals, shell/path-only mentions, zombie candidates, and self/ancestor
  exclusion.
- `py_compile`: PASS.
- JSON validation: PASS.
- `git diff --check`: PASS.

No MuJoCo, ROS2, controller, Run A/B, benchmark, FormalRun, or later P1 task
was started. The failed pair remains non-reusable; this evidence does not
authorize a new pair or Operator runtime.
