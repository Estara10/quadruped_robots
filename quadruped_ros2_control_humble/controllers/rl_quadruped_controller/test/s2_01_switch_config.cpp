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

abs_switching::AGroupConfig parse(const std::string& yaml)
{
    return abs_switching::parseAGroupConfig(YAML::Load(yaml)["abs"]);
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

    const auto deployed = abs_switching::parseAGroupConfig(YAML::LoadFile(
        "quadruped_ros2_control_humble/descriptions/unitree/go2_description/config/abs/config.yaml")["abs"]);
    check(std::abs(deployed.entry_threshold + 0.05) < 1e-12 &&
          deployed.exit_threshold == deployed.entry_threshold &&
          !deployed.hysteresis_enabled && !deployed.hold_enabled,
          "actual deployed YAML is consumed as A with derived exit and no hold");

    check(rejects("abs: {}\n"), "missing switching rejected");
    check(rejects("abs:\n  switching:\n    group: A\n"), "missing entry rejected");
    check(rejects("abs:\n  switching:\n    entry_threshold: -0.05\n"), "missing group rejected");
    check(rejects("abs:\n  switching:\n    group: B\n    entry_threshold: -0.05\n"), "known unsupported B rejected");
    check(rejects("abs:\n  switching:\n    group: C\n    entry_threshold: -0.05\n"), "known unsupported C rejected");
    check(rejects("abs:\n  switching:\n    group: D\n    entry_threshold: -0.05\n"), "known unsupported D rejected");
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

    std::printf("S2-01 config/switch checks: %d checks, %d failures\n", checks, failures);
    return failures == 0 ? 0 : 1;
}
