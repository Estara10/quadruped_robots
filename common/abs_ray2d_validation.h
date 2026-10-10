#pragma once

#include "abs_ray2d_shm_contract.h"

#include <cmath>

namespace abs_ray2d_shm {

enum class Validation {
  valid,
  header,
  unarmed_timestamp,
  monotonic_clock_order,
  stale,
  non_finite,
};

inline bool coherentSequence(uint64_t before, uint64_t after) {
  return before != 0 && (before & 1U) == 0U && before == after && (after & 1U) == 0U;
}

inline Validation validate(const float* rays, int count, uint64_t magic, uint64_t version,
                           uint64_t stamp_ns, uint64_t validation_now_ns, uint64_t timeout_ns) {
  if (magic != kMagic || version != kVersion || rays == nullptr || count != kRayCount)
    return Validation::header;
  if (stamp_ns == 0) return Validation::unarmed_timestamp;
  if (validation_now_ns < stamp_ns) return Validation::monotonic_clock_order;
  if (validation_now_ns - stamp_ns > timeout_ns) return Validation::stale;
  for (int i = 0; i < count; ++i) {
    if (!std::isfinite(rays[i])) return Validation::non_finite;
  }
  return Validation::valid;
}

}  // namespace abs_ray2d_shm
