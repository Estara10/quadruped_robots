#include "../src/physics_coverage_diagnostic.h"

#include <cassert>
#include <cmath>
#include <limits>
#include <string>

int main() {
  using abs_physics_coverage::checkContinuity;
  constexpr double dt = 0.002;
  assert(checkContinuity(false, 0, 0.0, 1, dt, dt).continuous);
  assert(checkContinuity(false, 0, 0.0, 2, 2 * dt, dt).reason ==
         std::string("first_physics_step_not_1"));
  assert(checkContinuity(false, 0, 0.0, 1, dt + 2e-9, dt).reason ==
         std::string("first_sim_time_not_one_timestep"));
  assert(checkContinuity(true, 1, dt, 2, 2 * dt, dt).continuous);
  assert(checkContinuity(true, 1, dt, 3, 3 * dt, dt).reason ==
         std::string("physics_step_gap_or_reset"));
  assert(checkContinuity(true, 10, 10 * dt, 11, 2 * dt, dt).reason ==
         std::string("sim_time_delta_not_one_timestep"));
  assert(checkContinuity(true, 10, std::numeric_limits<double>::quiet_NaN(),
                         11, 11 * dt, dt).reason == std::string("nonfinite_previous_sim_time"));
  assert(checkContinuity(true, 10, 10 * dt, 11,
                         std::numeric_limits<double>::infinity(), dt).reason ==
         std::string("nonfinite_current_time_or_timestep"));
  assert(checkContinuity(true, 10, 10 * dt, 11, 11 * dt,
                         std::numeric_limits<double>::quiet_NaN()).reason ==
         std::string("nonfinite_current_time_or_timestep"));
}
