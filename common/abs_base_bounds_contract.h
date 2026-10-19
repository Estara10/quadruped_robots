#pragma once

#include <cstddef>
#include <cstdint>

// PhysicsLoop-only base_link XY boundary accumulator. The cumulative outside
// step count makes a short event observable even when readers poll slowly.
namespace abs_base_bounds {

constexpr const char* kShmName = "/mujoco_base_bounds_v1";
constexpr uint64_t kMagic = 0x414253424E445331ULL;  // "ABSBNDS1"
constexpr uint64_t kVersion = 1;

struct Snapshot {
  uint64_t magic;
  uint64_t version;
  uint64_t sequence;
  uint64_t monotonic_ns;
  uint64_t physics_step;
  uint64_t producer_session_id;
  uint64_t physics_coverage_complete;
  uint64_t bounds_valid;
  uint64_t currently_outside;
  uint64_t outside_physics_steps;
  uint64_t first_outside_step;
  uint64_t last_outside_step;
  double sim_time;
  double base_x;
  double base_y;
  double xmin;
  double xmax;
  double ymin;
  double ymax;
  double first_outside_sim_time;
  double last_outside_sim_time;
  double min_observed_x;
  double max_observed_x;
  double min_observed_y;
  double max_observed_y;
  char scenario_id[32];
  char capture_id[64];
  char runtime_model_fingerprint[64];
};

static_assert(sizeof(Snapshot) == 360, "unexpected base-bounds snapshot layout");

inline uint64_t loadAcquire(const uint64_t* value) {
  return __atomic_load_n(value, __ATOMIC_ACQUIRE);
}
inline void storeRelease(uint64_t* value, uint64_t next) {
  __atomic_store_n(value, next, __ATOMIC_RELEASE);
}

}  // namespace abs_base_bounds
