#pragma once

#include "abs_ray2d_shm_contract.h"

#include <array>
#include <chrono>
#include <cmath>
#include <cstring>

namespace abs_ray2d_shm {

enum class Validation {
  valid,
  header,
  unarmed_timestamp,
  monotonic_clock_order,
  stale,
  non_finite,
};

enum class ReadStatus {
  stable,
  unarmed_sequence,
  writer_in_progress,
  incoherent_snapshot,
  retry_limit,
  time_budget,
};

struct SnapshotReadResult {
  ReadStatus status{ReadStatus::retry_limit};
  ReadStatus last_observed_status{ReadStatus::retry_limit};
  Validation validation{Validation::header};
  std::array<float, kRayCount> rays{};
  uint64_t sequence_first{0};
  uint64_t sequence_last{0};
  uint64_t magic{0};
  uint64_t version{0};
  uint64_t stamp_ns{0};
  uint64_t validation_now_ns{0};
  uint64_t read_duration_ns{0};
  uint32_t attempts{0};
  bool first_sequence_observed{false};
  bool header_observed{false};
  bool stamp_observed{false};
  bool snapshot_consistent{false};
  bool validation_clock_observed{false};
};

inline bool coherentSequence(uint64_t before, uint64_t after);
inline Validation validate(const float* rays, int count, uint64_t magic, uint64_t version,
                           uint64_t stamp_ns, uint64_t validation_now_ns, uint64_t timeout_ns);

// Production and deterministic tests use the same bounded seqlock reader.
// Source must provide sequence(), magic(), version(), stamp(), and copy(float*).
// Retries apply only to an unarmed/in-progress or incoherent read. Once a
// stable snapshot exists, semantic validation is performed exactly once so a
// genuinely invalid frame cannot be hidden by reading a later frame.
template <typename Source, typename ClockNow, typename Yield>
SnapshotReadResult readConsistentSnapshot(Source& source, ClockNow clock_now_ns, Yield yield,
                                          uint32_t max_attempts = 16,
                                          uint64_t max_duration_ns = 250000ULL,
                                          uint64_t timeout_ns = 200000000ULL,
                                          int configured_count = kRayCount) {
  using Steady = std::chrono::steady_clock;
  SnapshotReadResult result{};
  const auto started = Steady::now();
  const auto deadline = started + std::chrono::nanoseconds(max_duration_ns);
  for (uint32_t attempt = 0; attempt < max_attempts; ++attempt) {
    if (Steady::now() >= deadline) {
      result.status = ReadStatus::time_budget;
      break;
    }
    ++result.attempts;
    const uint64_t before = source.sequence();
    if (!result.first_sequence_observed) {
      result.sequence_first = before;
      result.first_sequence_observed = true;
    }
    result.sequence_last = before;
    if (before == 0) {
      result.status = ReadStatus::unarmed_sequence;
      result.last_observed_status = result.status;
    } else if ((before & 1U) != 0U) {
      result.status = ReadStatus::writer_in_progress;
      result.last_observed_status = result.status;
    } else {
      result.magic = source.magic();
      result.version = source.version();
      result.stamp_ns = source.stamp();
      result.header_observed = true;
      result.stamp_observed = true;
      source.copy(result.rays.data());
      const uint64_t after = source.sequence();
      result.sequence_last = after;
      if (!coherentSequence(before, after)) {
        result.status = ReadStatus::incoherent_snapshot;
        result.last_observed_status = result.status;
      } else {
        result.snapshot_consistent = true;
        result.validation_now_ns = clock_now_ns();
        result.validation_clock_observed = true;
        result.validation = validate(result.rays.data(), configured_count, result.magic,
                                     result.version, result.stamp_ns,
                                     result.validation_now_ns, timeout_ns);
        result.status = ReadStatus::stable;
        result.last_observed_status = result.status;
        const auto ended = Steady::now();
        result.read_duration_ns = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(ended - started).count());
        if (ended > deadline) result.status = ReadStatus::time_budget;
        return result;
      }
    }
    if (attempt + 1 < max_attempts && Steady::now() < deadline) yield();
  }
  const auto ended = Steady::now();
  result.read_duration_ns = static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(ended - started).count());
  if (ended >= deadline) result.status = ReadStatus::time_budget;
  return result;
}

inline const char* readStatusName(ReadStatus status) {
  switch (status) {
    case ReadStatus::stable: return "stable";
    case ReadStatus::unarmed_sequence: return "sequence_zero_unarmed";
    case ReadStatus::writer_in_progress: return "odd_sequence_writer_in_progress";
    case ReadStatus::incoherent_snapshot: return "sequence_changed_during_copy";
    case ReadStatus::retry_limit: return "retry_limit";
    case ReadStatus::time_budget: return "read_time_budget";
  }
  return "unknown";
}

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
