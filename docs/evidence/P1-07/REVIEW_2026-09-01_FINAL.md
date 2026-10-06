# P1-07 — Final Independent Review (2026-09-01)

Decision: **P1-07 — ACCEPT WITH KNOWN ISSUES**.
Final independent review date: **2026-09-01**.

This records the accepted Final Reviewer result for P1-07 (Separate and Test
Paper-Faithful vs Stabilized Switching) into the source of truth. No algorithm,
test, model, config, threshold, hold, helper, CMake, or runtime behavior was
changed by this review.

## 1. Acceptance scope — switching-only

- **Explicit mode separation**: exactly two valid `abs.switching_mode` values —
  `paper_faithful_switch` and `stabilized_switch` — with a single pure
  dependency-free state-machine boundary (`RASwitchingLogic.hpp`).
- **Truth tables** tested offline at all threshold equalities
  (`p1_07_switching.cpp`, **292 checks PASS**, exit 0): paper `RA >= -0.05`
  enters (equality enters), `RA < -0.05` exits immediately (no hold can delay);
  stabilized `RA > -0.05` enters, `RA == -0.05` does not, `RA < -0.08` + 30-step
  hold exits, `RA == -0.08` does not, `-0.05 < RA < -0.08` boundary matches
  current code.
- **Default compatibility**: `stabilized_switch` is the default (member init +
  config key), reproducing the pre-P1-07 ENTER/EXIT byte-for-byte; regression
  over a 32-sample deterministic series against the literal pre-P1-07 reference
  logic PASS.
- **Invalid mode** values fail at initialization (`RCLCPP_FATAL` +
  `std::invalid_argument` in `loadYaml`); no silent fallback.
- **Non-finite RA** (NaN/±Inf) is fail-closed: the helper returns
  `invalid=true` and never applies a transition; the existing `runRAModel`
  fail-closed path is unchanged. Covered in both modes.

## 2. Known issues (low severity; recorded, not fixed)

- **CTest not registered**: `p1_07_switching` is added as an executable target
  only (`add_executable` in `BUILD_TESTING`); it is **not** registered with
  `add_test()` into ament ctest, so it must be run directly. Recorded as a
  low-severity known issue; no fix applied (no `add_test()` added).
- **`paper_faithful_switch` is NOT full paper-faithful ABS**: it selects per the
  paper rule but still executes the deployment Recovery optimizer (entry-edge
  optimize + cached twist); it is not claimed to be paper-equivalent.

## 3. Retained UNKNOWN

- **MuJoCo/ROS2 runtime switching** behavior of either mode is **not measured**
  and remains `UNKNOWN` (no runtime/benchmark/training/hardware run).
- Whether `paper_faithful_switch` is a useful experimental mode: out of scope.

## 4. Explicit boundaries

- P1-07 does **not** resolve the P1-06 Eq.21/Eq.22 optimizer MISMATCH
  (yaw-coupled displacement terms omitted, first-order goal-penalty consequence,
  per-element vs L2 gradient clip, iteration count 3 vs 10, no per-tick
  safe-twist re-optimization). Those remain recorded and **outside** this task.
- There is currently **no active engineering task**.
- **P1-08 has not started**.
- **Phase 1 remains NOT ACCEPTED.**

## Evidence

- `docs/evidence/P1-07/P1-07_switching_modes.md`
- `docs/evidence/P1-07/P1-07_switching_decision_table.json`
- `docs/exec-plans/P1-07.md`
- `quadruped_ros2_control_humble/controllers/rl_quadruped_controller/test/p1_07_switching.cpp`
