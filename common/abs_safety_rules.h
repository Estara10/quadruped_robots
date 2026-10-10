#pragma once

#include <cmath>
#include <cstdint>

#include "abs_collision_contract.h"

namespace abs_safety {

constexpr double kFallHeightM = 0.22;
constexpr double kFallTiltRad = 1.0471975511965976;  // 60 degrees
constexpr double kFallHoldSeconds = 0.30;

struct FallTracker {
  bool candidate = false;
  bool confirmed = false;
  bool active = false;
  bool posture_continuous = true;
  double start_sim_time = 0.0;
  double end_sim_time = 0.0;
  double confirmed_sim_time = 0.0;
  uint64_t start_physics_step = 0;
  uint64_t end_physics_step = 0;
  uint64_t confirmed_physics_step = 0;
  uint64_t generation = 0;
};

inline uint32_t classifyContact(bool robot_first, bool robot_second,
                                bool obstacle_first, bool obstacle_second,
                                bool floor_first, bool floor_second) {
  if ((robot_first && obstacle_second) || (robot_second && obstacle_first))
    return abs_collision::kContactRobotObstacle;
  if ((robot_first && floor_second) || (robot_second && floor_first))
    return abs_collision::kContactGround;
  if (robot_first && robot_second) return abs_collision::kContactSelf;
  if (floor_first || floor_second) {
    if (obstacle_first || obstacle_second) return abs_collision::kContactOther;
    return abs_collision::kContactUnknown;
  }
  if ((robot_first || obstacle_first) && (robot_second || obstacle_second))
    return abs_collision::kContactOther;
  return abs_collision::kContactUnknown;
}

inline bool newCollisionEdge(bool current, bool previous_valid,
                             bool previous_current) {
  return current && (!previous_valid || !previous_current);
}

inline bool updateFall(FallTracker& state, double height_m, double roll_rad,
                       double pitch_rad, uint64_t physics_step, double sim_time,
                       bool valid, bool continuous_step) {
  const bool finite = std::isfinite(height_m) && std::isfinite(roll_rad) &&
                      std::isfinite(pitch_rad) && std::isfinite(sim_time);
  if (!valid || !finite) {
    // Invalid posture is latched separately by the producer. Do not infer
    // recovery from an invalid sample, and do not restart a confirmed episode.
    state.posture_continuous = false;
    return state.confirmed;
  }
  const bool candidate = height_m < kFallHeightM ||
                         std::fabs(roll_rad) > kFallTiltRad ||
                         std::fabs(pitch_rad) > kFallTiltRad;
  if (!candidate) {
    state.candidate = false;
    state.confirmed = false;  // normal observation rearms the next episode
    state.active = false;
    state.posture_continuous = true;
  } else if (!state.active) {
    state.active = true;
    state.candidate = true;
    state.start_sim_time = sim_time;
    state.start_physics_step = physics_step;
    state.end_sim_time = sim_time;
    state.end_physics_step = physics_step;
    state.confirmed_sim_time = 0.0;
    state.confirmed_physics_step = 0;
    ++state.generation;
  } else if ((!continuous_step || !state.posture_continuous) && !state.confirmed) {
    // A physics gap breaks the hold, but the same unresolved anomaly remains
    // one episode until a valid normal observation is seen.
    state.start_sim_time = sim_time;
    state.start_physics_step = physics_step;
    state.end_sim_time = sim_time;
    state.end_physics_step = physics_step;
    state.candidate = true;
  } else if (!state.confirmed && sim_time - state.start_sim_time >= kFallHoldSeconds) {
    state.confirmed = true;
    state.confirmed_sim_time = sim_time;
    state.confirmed_physics_step = physics_step;
  }
  if (candidate) {
    state.candidate = true;
    state.end_sim_time = sim_time;
    state.end_physics_step = physics_step;
  }
  state.posture_continuous = true;
  return state.confirmed;
}

}  // namespace abs_safety
