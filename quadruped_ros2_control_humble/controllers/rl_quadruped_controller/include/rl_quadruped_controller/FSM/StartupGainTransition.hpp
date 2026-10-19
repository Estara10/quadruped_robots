#ifndef RL_QUADRUPED_CONTROLLER_STARTUP_GAIN_TRANSITION_HPP
#define RL_QUADRUPED_CONTROLLER_STARTUP_GAIN_TRANSITION_HPP

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

namespace abs_startup_gain
{
constexpr std::size_t kJointCount = 12;

inline double parseTransitionSeconds(const YAML::Node& abs_node)
{
    if (abs_node["soft_start_steps"])
    {
        throw std::invalid_argument(
            "abs.soft_start_steps is retired; use abs.startup_gain_transition_s only");
    }
    if (!abs_node["startup_gain_transition_s"])
    {
        throw std::invalid_argument("abs.startup_gain_transition_s is required");
    }
    const double seconds = abs_node["startup_gain_transition_s"].as<double>();
    if (!std::isfinite(seconds) || seconds <= 0.0)
    {
        throw std::invalid_argument("abs.startup_gain_transition_s must be finite and positive");
    }
    return seconds;
}

class Transition
{
public:
    using Gains = std::array<double, kJointCount>;

    bool begin(const Gains& entry_kp, const Gains& entry_kd,
               const Gains& target_kp, const Gains& target_kd,
               double duration_s)
    {
        if (!validGains(entry_kp) || !validGains(entry_kd) ||
            !validGains(target_kp) || !validGains(target_kd) ||
            !std::isfinite(duration_s) || duration_s <= 0.0)
        {
            active_ = false;
            return false;
        }
        entry_kp_ = entry_kp;
        entry_kd_ = entry_kd;
        target_kp_ = target_kp;
        target_kd_ = target_kd;
        duration_s_ = duration_s;
        elapsed_s_ = 0.0;
        active_ = true;
        return true;
    }

    bool sample(Gains& kp, Gains& kd, double& alpha) const
    {
        if (!active_)
        {
            return false;
        }
        alpha = std::clamp(elapsed_s_ / duration_s_, 0.0, 1.0);
        for (std::size_t i = 0; i < kJointCount; ++i)
        {
            kp[i] = interpolate(entry_kp_[i], target_kp_[i], alpha);
            kd[i] = interpolate(entry_kd_[i], target_kd_[i], alpha);
        }
        // Guarantee exact configured gains at and after the endpoint.
        if (alpha >= 1.0)
        {
            kp = target_kp_;
            kd = target_kd_;
        }
        return true;
    }

    bool advance(double period_s)
    {
        if (!active_ || !std::isfinite(period_s) || period_s < 0.0)
        {
            return false;
        }
        const double remaining = duration_s_ - elapsed_s_;
        elapsed_s_ = period_s >= remaining ? duration_s_ : elapsed_s_ + period_s;
        return true;
    }

    bool active() const { return active_; }
    double elapsedSeconds() const { return elapsed_s_; }
    double durationSeconds() const { return duration_s_; }
    double alpha() const
    {
        return active_ ? std::clamp(elapsed_s_ / duration_s_, 0.0, 1.0) : 0.0;
    }
    const Gains& entryKp() const { return entry_kp_; }
    const Gains& entryKd() const { return entry_kd_; }
    const Gains& targetKp() const { return target_kp_; }
    const Gains& targetKd() const { return target_kd_; }

private:
    static bool validGains(const Gains& values)
    {
        return std::all_of(values.begin(), values.end(),
                           [](double value) { return std::isfinite(value) && value >= 0.0; });
    }
    static double interpolate(double start, double target, double alpha)
    {
        return start + alpha * (target - start);
    }

    Gains entry_kp_{};
    Gains entry_kd_{};
    Gains target_kp_{};
    Gains target_kd_{};
    double duration_s_ = 0.0;
    double elapsed_s_ = 0.0;
    bool active_ = false;
};
}  // namespace abs_startup_gain

#endif  // RL_QUADRUPED_CONTROLLER_STARTUP_GAIN_TRANSITION_HPP
