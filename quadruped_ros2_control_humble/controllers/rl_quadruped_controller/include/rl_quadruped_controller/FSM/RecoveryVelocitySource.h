#ifndef RECOVERY_VELOCITY_SOURCE_H
#define RECOVERY_VELOCITY_SOURCE_H

#include <array>
#include <cmath>
#include <cstddef>
#include <optional>
#include <stdexcept>
#include <string>

enum class RecoveryVelocitySource { ODOM, ESTIMATOR };

inline RecoveryVelocitySource parseRecoveryVelocitySource(const std::string& source)
{
    if (source == "odom") return RecoveryVelocitySource::ODOM;
    if (source == "estimator") return RecoveryVelocitySource::ESTIMATOR;
    throw std::invalid_argument("recovery_velocity_source must be 'odom' or 'estimator'");
}

inline const char* recoveryVelocitySourceName(RecoveryVelocitySource source)
{
    return source == RecoveryVelocitySource::ODOM ? "odom" : "estimator";
}

inline std::optional<std::array<double, 3>> selectRecoveryVelocityWorld(
    RecoveryVelocitySource source,
    std::size_t odom_interface_count,
    const std::array<double, 3>& odom_velocity,
    bool estimator_available,
    const std::array<double, 3>& estimator_velocity)
{
    const auto finite = [](const std::array<double, 3>& velocity) {
        return std::isfinite(velocity[0]) && std::isfinite(velocity[1]) &&
               std::isfinite(velocity[2]);
    };

    if (source == RecoveryVelocitySource::ODOM)
    {
        if (odom_interface_count < 6 || !finite(odom_velocity)) return std::nullopt;
        return odom_velocity;
    }

    if (source == RecoveryVelocitySource::ESTIMATOR)
    {
        if (!estimator_available || !finite(estimator_velocity)) return std::nullopt;
        return estimator_velocity;
    }
    return std::nullopt;
}

#endif
