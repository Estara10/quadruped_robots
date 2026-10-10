#ifndef ABS_SWITCH_CONFIG_HPP
#define ABS_SWITCH_CONFIG_HPP

#include <cmath>
#include <stdexcept>
#include <string>

#include <yaml-cpp/yaml.h>

namespace abs_switching {

struct AGroupConfig {
    double entry_threshold = 0.0;
    double exit_threshold = 0.0;
    bool hysteresis_enabled = false;
    bool hold_enabled = false;
};

// Production parser for the current research-facing switching contract.
// Legacy fields are rejected even if they happen to agree with the new entry.
inline AGroupConfig parseAGroupConfig(const YAML::Node& abs_node)
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

    for (const auto& item : switching) {
        if (!item.first.IsScalar())
            throw std::invalid_argument("abs.switching keys must be scalar strings");
        const std::string key = item.first.as<std::string>();
        if (key != "group" && key != "entry_threshold")
            throw std::invalid_argument("field is unknown or disallowed for group A: abs.switching." + key);
    }

    const YAML::Node group_node = switching["group"];
    if (!group_node || !group_node.IsScalar())
        throw std::invalid_argument("abs.switching.group is required");
    const std::string group = group_node.as<std::string>();
    if (group == "B" || group == "C" || group == "D")
        throw std::invalid_argument("switching group " + group + " is known but not supported yet");
    if (group != "A")
        throw std::invalid_argument("unknown switching group: " + group);

    const YAML::Node entry_node = switching["entry_threshold"];
    if (!entry_node || !entry_node.IsScalar())
        throw std::invalid_argument("abs.switching.entry_threshold is required and must be a scalar");

    const double entry = entry_node.as<double>();
    if (!std::isfinite(entry) || !(entry > -1.0 && entry < 1.0))
        throw std::invalid_argument("A entry_threshold must be finite and within (-1, 1)");

    return AGroupConfig{entry, entry, false, false};
}

}  // namespace abs_switching

#endif  // ABS_SWITCH_CONFIG_HPP
