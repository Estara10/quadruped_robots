#ifndef ABS_SWITCH_CONFIG_HPP
#define ABS_SWITCH_CONFIG_HPP

#include <cmath>
#include <stdexcept>
#include <string>

#include <yaml-cpp/yaml.h>

#include "rl_quadruped_controller/FSM/RASwitchingLogic.hpp"

namespace abs_switching {

struct SwitchConfig {
    std::string group;
    SwitchMode mode = SwitchMode::paper_faithful_switch;
    double entry_threshold = 0.0;
    double exit_threshold = 0.0;
    double recovery_min_hold_s = 0.0;
    bool hysteresis_enabled = false;
    bool hold_enabled = false;
};

// Production parser for the research-facing A/B/C/D contract. Legacy fields are
// rejected even if they happen to agree with the new configuration.
inline SwitchConfig parseSwitchConfig(const YAML::Node& abs_node)
{
    if (!abs_node || !abs_node.IsMap())
        throw std::invalid_argument("abs configuration must be a mapping");

    for (const char* legacy : {"switching_mode", "ra_threshold", "recovery_hold_steps"}) {
        if (abs_node[legacy])
            throw std::invalid_argument(std::string("legacy abs field is not allowed: ") + legacy);
    }
    for (const char* misplaced : {"group", "entry_threshold", "exit_threshold", "recovery_min_hold_s",
                                  "hysteresis", "hold", "enable_hysteresis", "enable_hold"}) {
        if (abs_node[misplaced])
            throw std::invalid_argument(std::string("switching field must be nested under abs.switching: ") + misplaced);
    }

    const YAML::Node switching = abs_node["switching"];
    if (!switching || !switching.IsMap())
        throw std::invalid_argument("abs.switching mapping is required");

    const YAML::Node group_node = switching["group"];
    if (!group_node || !group_node.IsScalar())
        throw std::invalid_argument("abs.switching.group is required");
    const std::string group = group_node.as<std::string>();
    if (group != "A" && group != "B" && group != "C" && group != "D")
        throw std::invalid_argument("unknown switching group: " + group);

    for (const auto& item : switching) {
        if (!item.first.IsScalar())
            throw std::invalid_argument("abs.switching keys must be scalar strings");
        const std::string key = item.first.as<std::string>();
        const bool allowed = key == "group" || key == "entry_threshold" ||
            ((group == "B" || group == "D") && key == "exit_threshold") ||
            ((group == "C" || group == "D") && key == "recovery_min_hold_s");
        if (!allowed)
            throw std::invalid_argument("field is unknown or disallowed for group " + group + ": abs.switching." + key);
    }

    const YAML::Node entry_node = switching["entry_threshold"];
    if (!entry_node || !entry_node.IsScalar())
        throw std::invalid_argument("abs.switching.entry_threshold is required and must be a scalar");

    const double entry = entry_node.as<double>();
    if (!std::isfinite(entry) || !(entry > -1.0 && entry < 1.0))
        throw std::invalid_argument("entry_threshold must be finite and within (-1, 1)");

    if (group == "A")
        return SwitchConfig{group, SwitchMode::paper_faithful_switch, entry, entry, 0.0, false, false};

    if (group == "B") {
        const YAML::Node exit_node = switching["exit_threshold"];
        if (!exit_node || !exit_node.IsScalar())
            throw std::invalid_argument("abs.switching.exit_threshold is required for group B");
        const double exit = exit_node.as<double>();
        if (!std::isfinite(exit) || !(exit > -1.0 && exit < 1.0))
            throw std::invalid_argument("B exit_threshold must be finite and within (-1, 1)");
        if (!(exit < entry))
            throw std::invalid_argument("B exit_threshold must be less than entry_threshold");
        return SwitchConfig{group, SwitchMode::hysteresis_switch, entry, exit, 0.0, true, false};
    }

    if (group == "B" || group == "D") {
        const YAML::Node exit_node = switching["exit_threshold"];
        if (!exit_node || !exit_node.IsScalar())
            throw std::invalid_argument("abs.switching.exit_threshold is required for group " + group);
        const double exit = exit_node.as<double>();
        if (!std::isfinite(exit) || !(exit > -1.0 && exit < 1.0))
            throw std::invalid_argument(group + " exit_threshold must be finite and within (-1, 1)");
        if (!(exit < entry))
            throw std::invalid_argument(group + " exit_threshold must be less than entry_threshold");
        if (group == "B")
            return SwitchConfig{group, SwitchMode::hysteresis_switch, entry, exit, 0.0, true, false};
    }

    const YAML::Node hold_node = switching["recovery_min_hold_s"];
    if (!hold_node || !hold_node.IsScalar())
        throw std::invalid_argument("abs.switching.recovery_min_hold_s is required for group " + group);
    const double hold_s = hold_node.as<double>();
    if (!std::isfinite(hold_s) || hold_s <= 0.0)
        throw std::invalid_argument(group + " recovery_min_hold_s must be finite and > 0 seconds");
    return SwitchConfig{group, group == "D" ? SwitchMode::combined_switch : SwitchMode::minimum_hold_switch,
                        entry, group == "D" ? switching["exit_threshold"].as<double>() : entry,
                        hold_s, group == "D", true};
}

}  // namespace abs_switching

#endif  // ABS_SWITCH_CONFIG_HPP
