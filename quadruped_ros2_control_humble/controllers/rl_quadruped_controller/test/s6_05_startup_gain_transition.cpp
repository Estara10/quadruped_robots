#include "rl_quadruped_controller/FSM/StartupGainTransition.hpp"

#include <cassert>
#include <cmath>
#include <limits>
#include <stdexcept>

int main()
{
    using Transition = abs_startup_gain::Transition;
    Transition::Gains start_kp{}, start_kd{}, target_kp{}, target_kd{}, out_kp{}, out_kd{};
    for (std::size_t i = 0; i < abs_startup_gain::kJointCount; ++i)
    {
        start_kp[i] = 40.0 + i;
        start_kd[i] = 1.0 + i * 0.1;
        target_kp[i] = 30.0 + i;
        target_kd[i] = 0.65 + i * 0.01;
    }

    Transition transition;
    assert(transition.begin(start_kp, start_kd, target_kp, target_kd, 0.30));
    double alpha = -1.0;
    assert(transition.sample(out_kp, out_kd, alpha));
    assert(alpha == 0.0 && out_kp == start_kp && out_kd == start_kd);
    assert(transition.advance(0.15));
    assert(transition.sample(out_kp, out_kd, alpha));
    assert(std::abs(alpha - 0.5) < 1e-12);
    for (std::size_t i = 0; i < abs_startup_gain::kJointCount; ++i)
    {
        assert(std::abs(out_kp[i] - (start_kp[i] + target_kp[i]) / 2.0) < 1e-12);
        assert(std::abs(out_kd[i] - (start_kd[i] + target_kd[i]) / 2.0) < 1e-12);
    }
    assert(transition.advance(0.15));
    assert(transition.sample(out_kp, out_kd, alpha));
    assert(alpha == 1.0 && out_kp == target_kp && out_kd == target_kd);
    assert(transition.advance(0.01));
    assert(transition.sample(out_kp, out_kd, alpha));
    assert(out_kp == target_kp && out_kd == target_kd);
    assert(!transition.advance(-0.001));
    assert(!transition.advance(std::numeric_limits<double>::quiet_NaN()));

    auto invalid = start_kp;
    invalid[7] = -1.0;
    assert(!transition.begin(invalid, start_kd, target_kp, target_kd, 0.30));
    invalid = start_kp;
    invalid[3] = std::numeric_limits<double>::infinity();
    assert(!transition.begin(start_kp, start_kd, invalid, target_kd, 0.30));
    assert(!transition.begin(start_kp, start_kd, target_kp, target_kd, 0.0));
    assert(!transition.begin(start_kp, start_kd, target_kp, target_kd,
                             std::numeric_limits<double>::infinity()));

    const auto valid_config = YAML::Load("startup_gain_transition_s: 0.30");
    assert(std::abs(abs_startup_gain::parseTransitionSeconds(valid_config) - 0.30) < 1e-12);
    const auto equivalent_config = YAML::Load("startup_gain_transition_s: 0.3");
    assert(std::abs(abs_startup_gain::parseTransitionSeconds(equivalent_config) - 0.30) < 1e-12);
    for (const auto& yaml : {"soft_start_steps: 250\nstartup_gain_transition_s: 0.30",
                             "startup_gain_transition_s: .nan",
                             "startup_gain_transition_s: 0",
                             "startup_gain_transition_s: -0.1",
                             "other: true"})
    {
        bool rejected = false;
        try { (void)abs_startup_gain::parseTransitionSeconds(YAML::Load(yaml)); }
        catch (const std::exception&) { rejected = true; }
        assert(rejected);
    }

    // A new RL session starts from its newly observed interface values.
    auto next_start_kp = start_kp;
    auto next_start_kd = start_kd;
    next_start_kp.fill(80.0);
    next_start_kd.fill(3.0);
    assert(transition.begin(next_start_kp, next_start_kd, target_kp, target_kd, 0.30));
    assert(transition.sample(out_kp, out_kd, alpha));
    assert(out_kp == next_start_kp && out_kd == next_start_kd);
}
