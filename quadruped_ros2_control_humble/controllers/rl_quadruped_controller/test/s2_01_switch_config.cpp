#include "rl_quadruped_controller/FSM/AbsSwitchConfig.hpp"
#include "rl_quadruped_controller/FSM/RASwitchingLogic.hpp"

#include <cmath>
#include <cstdio>
#include <limits>
#include <stdexcept>
#include <string>

namespace {
int checks = 0;
int failures = 0;

void check(bool condition, const char* label)
{
    ++checks;
    if (!condition) {
        ++failures;
        std::printf("FAIL: %s\n", label);
    }
}

abs_switching::SwitchConfig parse(const std::string& yaml)
{
    return abs_switching::parseSwitchConfig(YAML::Load(yaml)["abs"]);
}

bool rejects(const std::string& yaml)
{
    try {
        (void)parse(yaml);
        return false;
    } catch (const std::exception&) {
        return true;
    }
}
}  // namespace

int main()
{
    using abs_switching::SwitchMode;
    using abs_switching::SwitchState;
    const auto config = parse("abs:\n  switching:\n    group: A\n    entry_threshold: -0.05\n");
    check(std::abs(config.entry_threshold + 0.05) < 1e-12, "legal A entry threshold");
    check(config.exit_threshold == config.entry_threshold, "A exit derived from entry");
    check(!config.hysteresis_enabled && !config.hold_enabled, "A hysteresis and hold disabled");

    const auto deployed = abs_switching::parseSwitchConfig(YAML::LoadFile(
        "quadruped_ros2_control_humble/descriptions/unitree/go2_description/config/abs/config.yaml")["abs"]);
    check(std::abs(deployed.entry_threshold + 0.05) < 1e-12 &&
          deployed.exit_threshold == deployed.entry_threshold &&
          !deployed.hysteresis_enabled && !deployed.hold_enabled,
          "actual deployed YAML is consumed as A with derived exit and no hold");

    check(rejects("abs: {}\n"), "missing switching rejected");
    check(rejects("abs:\n  switching:\n    group: A\n"), "missing entry rejected");
    check(rejects("abs:\n  switching:\n    entry_threshold: -0.05\n"), "missing group rejected");
    check(rejects("abs:\n  switching:\n    group: B\n    entry_threshold: -0.05\n"), "B missing exit rejected");
    check(rejects("abs:\n  switching:\n    group: C\n    entry_threshold: -0.05\n"), "known unsupported C rejected");
    check(rejects("abs:\n  switching:\n    group: D\n    entry_threshold: -0.05\n"), "D missing exit and hold rejected");
    check(rejects("abs:\n  switching:\n    group: E\n    entry_threshold: -0.05\n"), "unknown group rejected");
    check(rejects("abs:\n  switching:\n    group: A\n    entry_threshold: -0.05\n    exit_threshold: -0.08\n"), "A disallowed exit field rejected");
    check(rejects("abs:\n  switching:\n    group: A\n    entry_threshold: -0.05\n    recovery_min_hold_s: 0.2\n"), "A disallowed hold field rejected");
    check(rejects("abs:\n  switching:\n    group: A\n    entry_threshold: .nan\n"), "non-finite entry rejected");
    check(rejects("abs:\n  switching:\n    group: A\n    entry_threshold: .inf\n"), "positive infinite entry rejected");
    check(rejects("abs:\n  switching:\n    group: A\n    entry_threshold: -.inf\n"), "negative infinite entry rejected");
    check(rejects("abs:\n  switching:\n    group: A\n    entry_threshold: 1.0\n"), "out-of-range entry rejected");
    check(rejects("abs:\n  ra_threshold: -0.05\n  switching:\n    group: A\n    entry_threshold: -0.05\n"), "conflicting legacy threshold rejected");
    check(rejects("abs:\n  switching_mode: paper_faithful_switch\n  switching:\n    group: A\n    entry_threshold: -0.05\n"), "legacy mode rejected");
    check(rejects("abs:\n  recovery_hold_steps: 30\n  switching:\n    group: A\n    entry_threshold: -0.05\n"), "legacy hold rejected");
    check(rejects("abs:\n  exit_threshold: -0.08\n  switching:\n    group: A\n    entry_threshold: -0.05\n"), "misplaced exit field rejected");

    const auto b_config = parse("abs:\n  switching:\n    group: B\n    entry_threshold: -0.05\n    exit_threshold: -0.08\n");
    check(b_config.group == "B" && b_config.mode == SwitchMode::hysteresis_switch,
          "legal B selects explicit hysteresis mode");
    check(std::abs(b_config.entry_threshold + 0.05) < 1e-12 &&
          std::abs(b_config.exit_threshold + 0.08) < 1e-12,
          "B consumes explicit E and X");
    check(b_config.hysteresis_enabled && !b_config.hold_enabled,
          "B enables hysteresis and disables hold");
    check(rejects("abs:\n  switching:\n    group: B\n    entry_threshold: -0.05\n    exit_threshold: -0.05\n"), "B X equal to E rejected");
    check(rejects("abs:\n  switching:\n    group: B\n    entry_threshold: -0.05\n    exit_threshold: -0.04\n"), "B X greater than E rejected");
    check(rejects("abs:\n  switching:\n    group: B\n    entry_threshold: -0.05\n    exit_threshold: .nan\n"), "B non-finite X rejected");
    check(rejects("abs:\n  switching:\n    group: B\n    entry_threshold: -0.05\n    exit_threshold: -0.08\n    recovery_min_hold_s: 0.2\n"), "B hold field rejected");

    const double be = b_config.entry_threshold;
    const double bx = b_config.exit_threshold;
    const double middle = (be + bx) / 2.0;
    auto a = abs_switching::stepSwitching(SwitchMode::paper_faithful_switch,
                                           SwitchState{false, 0}, be, be, be, 0);
    auto b = abs_switching::stepSwitching(SwitchMode::hysteresis_switch,
                                           SwitchState{false, 0}, be, be, bx, 0);
    check(a.state.in_recovery && b.state.in_recovery, "A and B both enter at RA == E");
    a = abs_switching::stepSwitching(SwitchMode::paper_faithful_switch,
                                      a.state, middle, be, be, 0);
    b = abs_switching::stepSwitching(SwitchMode::hysteresis_switch,
                                      b.state, middle, be, bx, 0);
    check(!a.state.in_recovery && b.state.in_recovery,
          "only hysteresis exit condition differs inside X-to-E band");
    b = abs_switching::stepSwitching(SwitchMode::hysteresis_switch,
                                      b.state, bx, be, bx, 0);
    check(b.state.in_recovery && b.state.hold_left == 0,
          "RA == X retains Recovery without hold");
    b = abs_switching::stepSwitching(SwitchMode::hysteresis_switch,
                                      b.state, bx - 1e-6, be, bx, 0);
    check(!b.state.in_recovery && b.state.hold_left == 0,
          "RA < X exits in the same cycle without hold");
    auto b_fluctuating = SwitchState{};
    b_fluctuating = abs_switching::stepSwitching(SwitchMode::hysteresis_switch,
                                                  b_fluctuating, be, be, bx, 0).state;
    b_fluctuating = abs_switching::stepSwitching(SwitchMode::hysteresis_switch,
                                                  b_fluctuating, middle, be, bx, 0).state;
    b_fluctuating = abs_switching::stepSwitching(SwitchMode::hysteresis_switch,
                                                  b_fluctuating, bx, be, bx, 0).state;
    check(b_fluctuating.in_recovery, "repeated B fluctuations inside hysteresis band retain Recovery");
    b_fluctuating = abs_switching::stepSwitching(SwitchMode::hysteresis_switch,
                                                  b_fluctuating, bx - 1e-6, be, bx, 0).state;
    b_fluctuating = abs_switching::stepSwitching(SwitchMode::hysteresis_switch,
                                                  b_fluctuating, be, be, bx, 0).state;
    check(b_fluctuating.in_recovery, "B re-enters at E after an earlier exit");
    const auto b_invalid = abs_switching::stepSwitching(SwitchMode::hysteresis_switch,
        SwitchState{true, 0}, std::numeric_limits<double>::infinity(), be, bx, 0);
    check(b_invalid.invalid && b_invalid.state.in_recovery,
          "non-finite B RA leaves state unchanged for the existing safety veto");

    const auto c_config = parse("abs:\n  switching:\n    group: C\n    entry_threshold: -0.05\n    recovery_min_hold_s: 0.30\n");
    check(c_config.group == "C" && c_config.mode == SwitchMode::minimum_hold_switch,
          "legal C selects minimum-hold mode");
    check(c_config.entry_threshold == c_config.exit_threshold &&
          c_config.recovery_min_hold_s == 0.30 && !c_config.hysteresis_enabled && c_config.hold_enabled,
          "C derives exit from entry and consumes a seconds-based hold only");
    check(rejects("abs:\n  switching:\n    group: C\n    entry_threshold: -0.05\n"), "C missing hold rejected");
    check(rejects("abs:\n  switching:\n    group: C\n    entry_threshold: -0.05\n    recovery_min_hold_s: 0\n"), "C zero hold rejected");
    check(rejects("abs:\n  switching:\n    group: C\n    entry_threshold: -0.05\n    recovery_min_hold_s: -.inf\n"), "C infinite hold rejected");
    check(rejects("abs:\n  switching:\n    group: C\n    entry_threshold: -0.05\n    exit_threshold: -0.08\n    recovery_min_hold_s: 0.30\n"), "C explicit exit rejected");
    check(rejects("abs:\n  switching:\n    group: C\n    entry_threshold: -0.05\n    recovery_min_hold_s: 0.30\n    recovery_hold_steps: 30\n"), "C legacy hold rejected");

    const double ce = c_config.entry_threshold;
    const double ch = c_config.recovery_min_hold_s;
    auto c = abs_switching::stepSwitching(SwitchMode::minimum_hold_switch,
        SwitchState{}, ce, ce, ce, 0, 1.0, false, 7, ch);
    check(!c.state.in_recovery && c.hold_status == abs_switching::kHoldClockInvalid,
          "C does not enter when RA is high but sim clock is invalid");
    c = abs_switching::stepSwitching(SwitchMode::minimum_hold_switch,
        SwitchState{}, ce, ce, ce, 0, 1.0, true, 7, ch);
    check(c.state.in_recovery && c.enter_edge && c.state.recovery_enter_sim_time_s == 1.0 &&
          c.state.recovery_clock_segment == 7 && c.hold_status == abs_switching::kHoldActive,
          "C enters at RA == E on a valid clock and starts H in that decision cycle");
    auto c_before = abs_switching::stepSwitching(SwitchMode::minimum_hold_switch,
        c.state, ce - 0.01, ce, ce, 0, 1.299, true, 7, ch);
    check(c_before.state.in_recovery && c_before.hold_status == abs_switching::kHoldActive &&
          std::abs(c_before.hold_elapsed_s - 0.299) < 1e-9,
          "C blocks low-risk exit before H expires");
    auto c_due = abs_switching::stepSwitching(SwitchMode::minimum_hold_switch,
        c.state, ce - 0.01, ce, ce, 0, 1.3, true, 7, ch);
    check(!c_due.state.in_recovery && c_due.hold_status == abs_switching::kHoldExpired,
          "C permits low-risk exit exactly at elapsed == H");
    auto c_high_due = abs_switching::stepSwitching(SwitchMode::minimum_hold_switch,
        c.state, ce, ce, ce, 0, 1.3, true, 7, ch);
    check(c_high_due.state.in_recovery && c_high_due.state.recovery_enter_sim_time_s == 1.0 &&
          c_high_due.hold_status == abs_switching::kHoldExpired,
          "C remains Recovery after expiry when RA >= E and does not restart H");
    auto c_late_exit = abs_switching::stepSwitching(SwitchMode::minimum_hold_switch,
        c_high_due.state, ce - 0.01, ce, ce, 0, 1.4, true, 7, ch);
    check(!c_late_exit.state.in_recovery && std::abs(c_late_exit.hold_elapsed_s - 0.4) < 1e-9,
          "C exits at first later low-risk cycle using original entry time");
    auto c_invalid_clock = abs_switching::stepSwitching(SwitchMode::minimum_hold_switch,
        c.state, ce - 0.01, ce, ce, 0, 2.0, false, 7, ch);
    check(c_invalid_clock.state.in_recovery && c_invalid_clock.hold_status == abs_switching::kHoldClockInvalid,
          "C blocks conversion on invalid clock without losing entry time");
    auto c_segment_reset = abs_switching::stepSwitching(SwitchMode::minimum_hold_switch,
        c.state, ce - 0.01, ce, ce, 0, 2.0, true, 8, ch);
    check(c_segment_reset.state.in_recovery && c_segment_reset.state.recovery_enter_sim_time_s == 1.0 &&
          c_segment_reset.hold_status == abs_switching::kHoldClockInvalid,
          "C does not reset or extend t_enter across a clock segment change");

    const auto d_config = parse("abs:\n  switching:\n    group: D\n    entry_threshold: -0.05\n    exit_threshold: -0.08\n    recovery_min_hold_s: 0.30\n");
    check(d_config.group == "D" && d_config.mode == SwitchMode::combined_switch &&
          d_config.entry_threshold == -0.05 && d_config.exit_threshold == -0.08 &&
          d_config.recovery_min_hold_s == 0.30 && d_config.hysteresis_enabled && d_config.hold_enabled,
          "D consumes both thresholds and sim-time hold");
    check(rejects("abs:\n  switching:\n    group: D\n    entry_threshold: -0.05\n    exit_threshold: -0.08\n"),
          "D missing hold rejected");
    check(rejects("abs:\n  switching:\n    group: D\n    entry_threshold: -0.05\n    recovery_min_hold_s: 0.30\n"),
          "D missing exit rejected");
    check(rejects("abs:\n  switching:\n    group: D\n    entry_threshold: -0.05\n    exit_threshold: -0.08\n    recovery_min_hold_s: 0.30\n    unknown: true\n"), "D unknown field rejected");
    check(rejects("abs:\n  switching:\n    group: D\n    entry_threshold: -0.05\n    exit_threshold: -0.04\n    recovery_min_hold_s: 0.30\n"), "D exit threshold must be below entry");

    const double de = d_config.entry_threshold;
    const double dx = d_config.exit_threshold;
    const double dh = d_config.recovery_min_hold_s;
    auto d_hold = abs_switching::stepSwitching(SwitchMode::combined_switch,
        SwitchState{}, de, de, dx, 0, 1.0, true, 11, dh);
    check(d_hold.state.in_recovery && d_hold.enter_edge && d_hold.state.recovery_enter_sim_time_s == 1.0,
          "D enters at RA == E and starts hold on the decision cycle");
    auto d_before = abs_switching::stepSwitching(SwitchMode::combined_switch,
        d_hold.state, dx - 0.001, de, dx, 0, 1.299, true, 11, dh);
    check(d_before.state.in_recovery && d_before.hold_status == abs_switching::kHoldActive,
          "D hold prevents exit below X before H");
    auto d_at_x = abs_switching::stepSwitching(SwitchMode::combined_switch,
        d_hold.state, dx, de, dx, 0, 1.3, true, 11, dh);
    check(d_at_x.state.in_recovery && d_at_x.hold_status == abs_switching::kHoldExpired,
          "D retains Recovery at RA == X when H expires");
    auto d_band_due = abs_switching::stepSwitching(SwitchMode::combined_switch,
        d_hold.state, middle, de, dx, 0, 1.3, true, 11, dh);
    check(d_band_due.state.in_recovery && d_band_due.state.recovery_enter_sim_time_s == 1.0 &&
          d_band_due.hold_status == abs_switching::kHoldExpired,
          "D combines hysteresis and hold; X <= RA < E after expiry stays without restarting H");
    auto d_reenter_risk = abs_switching::stepSwitching(SwitchMode::combined_switch,
        d_band_due.state, de + 0.01, de, dx, 0, 1.35, true, 11, dh);
    check(d_reenter_risk.state.in_recovery && d_reenter_risk.state.recovery_enter_sim_time_s == 1.0,
          "renewed risk in Recovery does not restart D hold");
    auto d_exit = abs_switching::stepSwitching(SwitchMode::combined_switch,
        d_reenter_risk.state, dx - 0.001, de, dx, 0, 1.36, true, 11, dh);
    check(!d_exit.state.in_recovery && d_exit.hold_status == abs_switching::kHoldExpired,
          "D exits after H when RA < X");
    auto d_invalid = abs_switching::stepSwitching(SwitchMode::combined_switch,
        d_hold.state, dx - 0.001, de, dx, 0, 1.4, false, 11, dh);
    check(d_invalid.state.in_recovery && d_invalid.hold_status == abs_switching::kHoldClockInvalid,
          "D does not convert on invalid sim clock");

    const auto mode = SwitchMode::paper_faithful_switch;
    const double e = config.entry_threshold;
    const double x = config.exit_threshold;
    auto d = abs_switching::stepSwitching(mode, SwitchState{false, 0}, e - 1e-6, e, x, 0);
    check(!d.state.in_recovery && d.state.hold_left == 0, "below E stays Agile");
    d = abs_switching::stepSwitching(mode, SwitchState{false, 0}, e, e, x, 0);
    check(d.state.in_recovery && d.enter_edge && d.state.hold_left == 0, "RA equals E enters without hold");
    d = abs_switching::stepSwitching(mode, SwitchState{false, 0}, e + 1e-6, e, x, 0);
    check(d.state.in_recovery && d.enter_edge, "above E enters");
    d = abs_switching::stepSwitching(mode, SwitchState{true, 99}, e - 1e-6, e, x, 0);
    check(!d.state.in_recovery && d.state.hold_left == 0, "below E exits despite legacy hold state");
    d = abs_switching::stepSwitching(mode, SwitchState{true, 0}, e, e, x, 0);
    check(d.state.in_recovery && !d.enter_edge, "equal E stays Recovery");
    d = abs_switching::stepSwitching(mode, SwitchState{false, 0},
                                     std::numeric_limits<double>::quiet_NaN(), e, x, 0);
    check(d.invalid && !d.state.in_recovery, "invalid RA rejected by switching helper");

    std::printf("S5-01 config/switch checks: %d checks, %d failures\n", checks, failures);
    return failures == 0 ? 0 : 1;
}
