#pragma once

#include <cstddef>
#include <cstdint>

// Versioned cross-process contract for the 11-beam MuJoCo ray frame.
// All timestamps use std::chrono::steady_clock::time_since_epoch() in ns.
namespace abs_ray2d_shm {

constexpr const char* kRayShmName = "/mujoco_ray2d";
constexpr const char* kHeaderShmName = "/mujoco_ray2d_stamp";
constexpr int kRayCount = 11;
constexpr uint64_t kMagic = 0x415253594143544FULL;  // "ARSYACTO"
constexpr uint64_t kVersion = 2;

struct FrameHeader {
  uint64_t magic;
  uint64_t version;
  uint64_t sequence;      // even: stable; odd: writer owns the frame
  uint64_t monotonic_ns;  // completion time of the corresponding ray payload
};

static_assert(sizeof(FrameHeader) == 4 * sizeof(uint64_t), "unexpected shared-memory header layout");

inline uint64_t loadAcquire(const uint64_t* value) {
  return __atomic_load_n(value, __ATOMIC_ACQUIRE);
}

inline void storeRelease(uint64_t* value, uint64_t next) {
  __atomic_store_n(value, next, __ATOMIC_RELEASE);
}

}  // namespace abs_ray2d_shm
