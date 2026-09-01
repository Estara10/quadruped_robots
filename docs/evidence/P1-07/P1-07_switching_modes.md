# P1-07 — Paper-Faithful vs Stabilized Switching (2026-09-01)

Status: **IMPLEMENTED / AWAITING INDEPENDENT REVIEW** (not self-accepted).
Phase 1 remains **NOT ACCEPTED**.

Scope: **switching semantics only**. This is evidence for
"paper-faithful switching", **not** full paper-faithful ABS. P1-06 Recovery
optimizer / Eq.22 mismatches are recorded and remain **outside** this task
(see §5). No MuJoCo/ROS2 runtime, benchmark, training, export, formal run, or
hardware was executed.

## 1. Mode contract and decision table

Exactly two valid modes, selected by `abs.switching_mode` (default
`stabilized_switch`):

| Mode | State on each policy step | ENTER Recovery | EXIT Recovery | Forced hold |
|---|---|---|---|---|
| `paper_faithful_switch` | stateless | `RA >= -0.05` (equality **enters**) | `RA < -0.05` (immediate, no hold, no hysteresis) | none |
| `stabilized_switch` (default) | held | `RA > -0.05` (strict, equality does **not** enter) | `RA < -0.08` **and** `recovery_hold_steps` (30) hold expired | 30 policy steps |

Truth tables (RA sequence vs resulting state; both modes tested offline with
the **same** RA sequences):

| Condition | paper_faithful_switch | stabilized_switch |
|---|---|---|
| `RA == -0.05`, from Agile | → Recovery | → stays Agile |
| `RA > -0.05`, from Agile | → Recovery | → Recovery (hold := 30) |
| `RA < -0.05`, from Agile | → Agile | → stays Agile (not `>`) |
| `RA < -0.05`, in Recovery, hold active | → Agile **immediately** (no hold can delay) | → stays Recovery (hold governs) |
| `-0.05 < RA < -0.08`, in Recovery, hold expired | → Agile (below `-0.05`) | → stays Recovery (not `< -0.08`) |
| `RA == -0.08`, in Recovery, hold expired | → Agile (below `-0.05`) | → stays Recovery (strict `<` fails) |
| `RA < -0.08`, in Recovery, hold expired | → Agile | → exits Recovery |
| `RA` non-finite (NaN/±Inf) | **invalid=true — no transition** (fail-closed path is authoritative) | **invalid=true — no transition** |

Key boundary facts proven by tests: paper equality at `-0.05` enters;
stabilized equality at `-0.05` does not enter; stabilized equality at `-0.08`
does not exit; paper has no hold state that can delay an exit.

## 2. Implementation

Single state-machine boundary: new pure header
`controllers/rl_quadruped_controller/include/rl_quadruped_controller/FSM/RASwitchingLogic.hpp`
(dependency-free: no torch/ROS/YAML). `StateRL::runModel()` delegates the
inline ENTER/EXIT decision to `abs_switching::stepSwitching(...)`; on
`enter_edge` it runs the **unchanged** `computeRecoveryTwist()` + cache + log.
`loadYaml()` parses `abs.switching_mode` and **fails initialization**
(`RCLCPP_FATAL` + `std::invalid_argument`) on any value other than the two
valid names — no silent fallback. `config.yaml` declares the key with the
default value. `StateRL.h` member default = `stabilized_switch`.

Non-finite handling: `runRAModel()` already fail-closed on a non-finite RA
observation/output via `safetyVeto` (unchanged, `StateRL.cpp`); the helper
additionally returns `invalid=true` for NaN/±Inf and leaves the state
untouched, so a non-finite RA can never become an Agile/Recovery transition or
bypass the fail-closed path.

## 3. Tests and reproducible commands

`quadruped_ros2_control_humble/controllers/rl_quadruped_controller/test/p1_07_switching.cpp`
— offline, deterministic, no libtorch/ROS/MuJoCo. Coverage:

- **A paper**: `RA==-0.05` enters; `RA>-0.05` enters; `RA<-0.05` selects/exits
  to Agile immediately, including from in-Recovery with an active hold; no hold
  state can delay exit; stay/re-enter edges.
- **B stabilized**: `RA==-0.05` does not enter; `RA>-0.05` enters and arms the
  30-step hold; safe `RA` cannot exit before hold expiry; `RA==-0.08` does not
  exit after expiry; `RA<-0.08` exits after expiry; `-0.05<RA<-0.08` boundary
  sequences match current code; regression over a 32-sample deterministic
  series reproduces the **literal pre-P1-07 reference** state sequence
  (strict `>`, unconditional hold decrement, `< -0.08` exit) for every step.
- **C safety/config**: invalid mode strings rejected (parse returns false,
  value unchanged); the two valid names round-trip; default selection is
  stabilized; NaN/±Inf produce `invalid=true` with no transition in **both**
  modes; finite RA always yields `invalid=false`. The same RA series runs under
  both modes against the paper oracle.

Reproduce:

```bash
cd quadruped_ros2_control_humble/controllers/rl_quadruped_controller
g++ -std=c++17 -Wall -Wextra -Wpedantic -I include test/p1_07_switching.cpp -o /tmp/p1_07_switching && /tmp/p1_07_switching
# RESULT: PASS (292 checks), exit 0

# full package build (verifies StateRL.cpp + header integration):
cd /home/lidio/quadruped_robots/quadruped_ros2_control_humble
source /opt/ros/humble/setup.bash
colcon build --packages-select rl_quadruped_controller --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DTorch_DIR=/home/lidio/Libraries/libtorch-cpu-2.0.1/share/cmake/Torch
./build/rl_quadruped_controller/p1_07_switching   # RESULT: PASS (292 checks)
```

Results: standalone compile **exit 0 / PASS (292 checks)**; `colcon build`
**exit 0**; built `p1_07_switching` **PASS (292 checks)**; existing
`p1_09f_stoppable_thread` regression **exit 0**; `p1_01_abs_observation_adapter`
passes with the system `libstdc++` (bare-shell failure is a pre-existing
anaconda `libstdc++` shadow, unrelated to P1-07). `git diff --check` PASS.

## 4. Default compatibility

- Default mode = `stabilized_switch`, matching the pre-P1-07 deployed ENTER/EXIT
  semantics byte-for-byte (verified by the regression test over a 32-sample
  sequence against the literal reference logic).
- Existing config files that omit the new key keep the stabilized default.
- The checked-in config explicitly declares `switching_mode: "stabilized_switch"`,
  so existing launch behavior is unchanged.
- Initial members are unchanged: `in_recovery_=false`, `rec_hold_left_=0`.

## 5. P1-06 optimizer differences that remain OUTSIDE this task (recorded, not fixed)

This task changes **only** the switching rule. The following P1-06 recorded
differences are **not** addressed and remain as recorded:

- Eq.22 deployment omits yaw-coupled second-order terms (`pos_x=vx*tau` /
  `pos_y=vy*tau` vs paper/reference) → **MISMATCH**.
- First-order displacement goal-penalty consequence → **MISMATCH**.
- Gradient clip: deployment per-element `torch::clamp` vs reference L2-norm →
  **MISMATCH** (not an approved stabilized variant).
- Iterations: deployment 3 vs recovered-testbed 10 (paper ≤5) →
  testbed↔deployment **MISMATCH**.
- Safe-twist **re-optimization cadence**: both modes keep the current behavior —
  twist is optimized on the entry edge and cached for the recovery dwell; the
  paper does not specify per-tick re-optimization and the recorded P1-06 gap
  ("cached twist may become unsafe; no per-tick re-optimization") is unchanged.

`paper_faithful_switch` therefore selects/switches per the paper rule but still
executes the deployment Recovery optimizer; it is **not** full paper-faithful
ABS and is not claimed to be paper-equivalent.

## 6. Remaining UNKNOWN / boundaries

- Runtime behavior (MuJoCo/ROS2) of either mode: **not measured** (no runtime
  authorized).
- Whether `paper_faithful_switch` is a useful experimental mode: out of scope.
- No benchmark, safety-performance, or Phase 1 Acceptance claim.
- P1-08 and later tasks do not start automatically.
