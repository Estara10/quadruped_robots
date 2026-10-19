#pragma once

#include <cmath>
#include <cstdint>

namespace abs_physics_coverage {

struct ContinuityResult {
  bool continuous;
  const char* reason;
};

// Keep the producer's existing coverage predicate in one production helper so
// the exact step/time boundary can be checked offline without relaxing it.
inline ContinuityResult checkContinuity(bool has_previous,
                                        uint64_t previous_step,
                                        double previous_sim_time,
                                        uint64_t current_step,
                                        double current_sim_time,
                                        double timestep) noexcept {
  if (!std::isfinite(current_sim_time) || !std::isfinite(timestep)) {
    return {false, "nonfinite_current_time_or_timestep"};
  }
  if (!has_previous) {
    if (current_step != 1U) return {false, "first_physics_step_not_1"};
    if (std::fabs(current_sim_time - timestep) > 1e-9) {
      return {false, "first_sim_time_not_one_timestep"};
    }
    return {true, "continuous"};
  }
  if (current_step != previous_step + 1U) {
    return {false, "physics_step_gap_or_reset"};
  }
  if (!std::isfinite(previous_sim_time)) {
    return {false, "nonfinite_previous_sim_time"};
  }
  if (std::fabs((current_sim_time - previous_sim_time) - timestep) > 1e-9) {
    return {false, "sim_time_delta_not_one_timestep"};
  }
  return {true, "continuous"};
}

}  // namespace abs_physics_coverage
