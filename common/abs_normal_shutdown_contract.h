#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

// Diagnostic-only confirmation limits for the simulated normal-arrival
// closeout. These are not real-robot safety limits or research parameters.
namespace abs_normal_shutdown {

enum class Decision { WAIT, CONFIRM, FAIL, ABORT };

inline double decelerationScale(double elapsed_s, double ramp_s) {
  if (!std::isfinite(elapsed_s) || !std::isfinite(ramp_s) || ramp_s <= 0.0) return 0.0;
  return std::clamp(1.0 - elapsed_s / ramp_s, 0.0, 1.0);
}

inline Decision evaluate(bool hard_stop, bool abnormal, bool feedback_valid,
                         bool condition_met, uint64_t stable_elapsed_ns,
                         uint64_t stage_elapsed_ns, uint64_t dwell_ns,
                         uint64_t timeout_ns) {
  if (hard_stop) return Decision::ABORT;
  if (abnormal || !feedback_valid) return Decision::FAIL;
  if (condition_met && stable_elapsed_ns >= dwell_ns) return Decision::CONFIRM;
  if (stage_elapsed_ns >= timeout_ns) return Decision::FAIL;
  return Decision::WAIT;
}

struct Feedback {
  bool valid = false;
  double horizontal_speed_mps = 0.0;
  double angular_speed_radps = 0.0;
  double roll_rad = 0.0;
  double pitch_rad = 0.0;
  double base_height_m = 0.0;
  double max_joint_position_error_rad = 0.0;
  double max_joint_speed_radps = 0.0;
  double max_kp_command = 0.0;
  double max_kd_command = 0.0;
  int supported_feet = 0;
};

inline bool finite(const Feedback& f) {
  return std::isfinite(f.horizontal_speed_mps) &&
         std::isfinite(f.angular_speed_radps) &&
         std::isfinite(f.roll_rad) && std::isfinite(f.pitch_rad) &&
         std::isfinite(f.base_height_m) &&
         std::isfinite(f.max_joint_position_error_rad) &&
         std::isfinite(f.max_joint_speed_radps) &&
         std::isfinite(f.max_kp_command) &&
         std::isfinite(f.max_kd_command);
}

inline bool standingSupportEnabled(const Feedback& f) {
  return f.valid && finite(f) && f.max_kp_command > 0.0 && f.max_kd_command > 0.0;
}

inline bool decelerationSettled(const Feedback& f) {
  return f.valid && finite(f) && f.horizontal_speed_mps <= 0.08 &&
         f.angular_speed_radps <= 0.15 && std::abs(f.roll_rad) <= 0.35 &&
         std::abs(f.pitch_rad) <= 0.35 && f.supported_feet >= 2;
}

inline bool standingConfirmed(const Feedback& f) {
  return f.valid && finite(f) && f.horizontal_speed_mps <= 0.08 &&
         f.angular_speed_radps <= 0.15 && std::abs(f.roll_rad) <= 0.30 &&
         std::abs(f.pitch_rad) <= 0.30 && f.base_height_m >= 0.25 &&
         f.max_joint_position_error_rad <= 0.12 &&
         f.max_joint_speed_radps <= 0.10 && f.supported_feet >= 3;
}

inline bool downConfirmed(const Feedback& f) {
  return f.valid && finite(f) && f.horizontal_speed_mps <= 0.08 &&
         f.angular_speed_radps <= 0.15 && std::abs(f.roll_rad) <= 0.35 &&
         std::abs(f.pitch_rad) <= 0.35 && f.base_height_m <= 0.29 &&
         f.max_joint_position_error_rad <= 0.12 &&
         f.max_joint_speed_radps <= 0.10 && f.supported_feet >= 2;
}

}  // namespace abs_normal_shutdown
