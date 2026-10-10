#pragma once

#include <cmath>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <sstream>
#include <string>

#include "abs_rt_frame_contract.h"

namespace abs_mujoco_live_panel {

constexpr uint64_t kStaleTimeoutNs = 500'000'000ULL;

enum class Status { kMissing, kInvalid, kStale, kLive };

struct Snapshot {
  Status status = Status::kMissing;
  abs_rt_frame::RuntimeFrame frame{};
  uint64_t age_ns = 0;
};

inline bool finiteFrame(const abs_rt_frame::RuntimeFrame& f) {
  const auto finite = [](float value) { return std::isfinite(value); };
  if (!std::isfinite(f.sim_time_s) || !finite(f.entry_threshold) ||
      !finite(f.exit_threshold) || !finite(f.ra_value)) return false;
  for (float value : f.lin_vel) if (!finite(value)) return false;
  for (float value : f.command) if (!finite(value)) return false;
  for (float value : f.world_pose) if (!finite(value)) return false;
  for (float value : f.ray2d) if (!finite(value)) return false;
  for (float value : f.action_raw) if (!finite(value)) return false;
  for (float value : f.action_clipped) if (!finite(value)) return false;
  for (float value : f.joint_target_rad) if (!finite(value)) return false;
  for (float value : f.torque_nm) if (!finite(value)) return false;
  for (float value : f.torque_saturated) if (!finite(value)) return false;
  return true;
}

// Classify a coherent frame copied by the caller's seqlock reader. Keep the
// display boundary aligned with scripts/abs_rt_frame.py: only fresh,
// authoritative, structurally valid frames can expose values.
inline Snapshot classify(const void* bytes, size_t size, uint64_t now_ns,
                        uint64_t stale_timeout_ns = kStaleTimeoutNs) {
  Snapshot result;
  if (bytes == nullptr || size == 0) return result;
  if (size != sizeof(abs_rt_frame::RuntimeFrame)) {
    result.status = Status::kInvalid;
    return result;
  }
  std::memcpy(&result.frame, bytes, sizeof(result.frame));
  const auto& f = result.frame;
  if (f.header.magic != abs_rt_frame::kMagic ||
      f.header.version != abs_rt_frame::kVersion ||
      f.header.sequence == 0 || (f.header.sequence & 1U) ||
      f.source != abs_rt_frame::kSourceAuthoritativeRuntime ||
      f.controller_active != 1 || f.rl_entered > 1 || f.rl_active > 1 ||
      f.safety_faulted > 1 || f.ray_valid > 1 ||
      f.torque_saturated_computed > 1 || f.risk_condition_met > 1 ||
      f.policy_mode_changed > 1 || f.sim_clock_valid > 1 ||
      f.policy_state > abs_rt_frame::kPolicyFaulted ||
      f.ray_origin > abs_rt_frame::kRayShmRuntime ||
      f.collision_origin != abs_rt_frame::kCollisionUnavailable ||
      f.mode_before > abs_rt_frame::kPolicyFaulted ||
      f.switching_mode > abs_rt_frame::kSwitchPaperFaithful ||
      f.action_source > abs_rt_frame::kActionNone ||
      f.risk_condition_entered > abs_rt_frame::kRiskEdgeUnknown ||
      f.sim_clock_status > abs_rt_frame::kSimClockStationary ||
      (f.rl_active && (!f.rl_entered || f.safety_faulted)) ||
      (f.sim_clock_valid && f.sim_clock_status != abs_rt_frame::kSimClockFresh &&
       f.sim_clock_status != abs_rt_frame::kSimClockStationary) ||
      (f.risk_condition_entered == abs_rt_frame::kRiskEdgeTrue &&
       (!f.risk_condition_met || f.risk_condition_entered_ns == 0)) ||
      (f.risk_condition_entered != abs_rt_frame::kRiskEdgeTrue &&
       f.risk_condition_entered_ns != 0) ||
      f.policy_mode_changed != (f.mode_change_ns != 0) ||
      (f.policy_state != abs_rt_frame::kPolicyFaulted &&
       f.policy_mode_changed != (f.mode_before != f.policy_state)) ||
      (f.sim_clock_valid &&
       (f.sim_clock_sequence == 0 || f.sim_clock_monotonic_ns == 0 ||
        f.sim_clock_monotonic_ns > f.header.monotonic_ns ||
        f.sim_clock_age_ns > f.header.monotonic_ns - f.sim_clock_monotonic_ns)) ||
      f.header.monotonic_ns == 0 || now_ns < f.header.monotonic_ns ||
      !finiteFrame(f)) {
    result.status = Status::kInvalid;
    return result;
  }
  result.age_ns = now_ns - f.header.monotonic_ns;
  result.status = result.age_ns > stale_timeout_ns ? Status::kStale
                                                   : Status::kLive;
  return result;
}

inline const char* statusName(Status status) {
  switch (status) {
    case Status::kMissing: return "MISSING";
    case Status::kInvalid: return "INVALID";
    case Status::kStale: return "STALE";
    case Status::kLive: return "LIVE";
  }
  return "INVALID";
}

inline std::string render(const Snapshot& snapshot) {
  std::ostringstream out;
  out << "ABS live state | " << statusName(snapshot.status);
  if (snapshot.status != Status::kLive) {
    out << "\nNo current policy data";
    if (snapshot.status == Status::kStale)
      out << "\nLast frame age: " << std::fixed << std::setprecision(0)
          << snapshot.age_ns / 1e6 << " ms";
    return out.str();
  }

  const auto& f = snapshot.frame;
  out << "\nPolicy: ";
  if (f.safety_faulted || f.policy_state == abs_rt_frame::kPolicyFaulted) {
    out << "FAULTED";
  } else if (!f.rl_entered) {
    out << "NOT IN RL";
  } else if (!f.rl_active) {
    out << "POLICY STOPPED";
  } else {
    switch (f.policy_state) {
      case abs_rt_frame::kPolicyAgile: out << "Agile"; break;
      case abs_rt_frame::kPolicyRecovery: out << "Recovery"; break;
      default: out << "UNKNOWN"; break;
    }
  }

  // A fresh controller frame may describe a pre-RL or stopped controller.
  // Do not display its last policy values as current in those states.
  if (f.rl_entered && f.rl_active && !f.safety_faulted &&
      f.policy_state != abs_rt_frame::kPolicyFaulted) {
    const double body_xy_speed = std::hypot(f.lin_vel[0], f.lin_vel[1]);
    out << std::fixed << std::setprecision(2)
        << "\nBody XY speed: " << body_xy_speed << " m/s"
        << "\nRA: " << std::setprecision(3) << f.ra_value
        << " | enter " << f.entry_threshold << " / exit " << f.exit_threshold
        << "\nGoal distance: N/A (goal absent from runtime frame)"
        << "\nSim time: ";
    if (f.sim_clock_valid && std::isfinite(f.sim_time_s))
      out << std::setprecision(2) << f.sim_time_s << " s";
    else
      out << "N/A";
  }
  out << "\nFrame age: " << std::fixed << std::setprecision(0)
      << snapshot.age_ns / 1e6 << " ms";
  return out.str();
}

}  // namespace abs_mujoco_live_panel
