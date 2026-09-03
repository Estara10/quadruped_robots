#pragma once

#include <cstddef>
#include <cstdint>

// Versioned, fail-closed collision snapshot written by the simulator physics
// step.  This is the formal source; the legacy /mujoco_collision int32 buffer
// remains only for old diagnostics and is never read by the formal recorder.
namespace abs_collision {

constexpr const char* kShmName = "/mujoco_collision_v2";
constexpr uint64_t kMagic = 0x414253434F4E5432ULL;  // "ABSCONT2"
constexpr uint64_t kVersion = 2;
constexpr std::size_t kScenarioIdBytes = 32;
constexpr std::size_t kSha256HexBytes = 64;
constexpr std::size_t kCaptureIdBytes = 64;
constexpr std::size_t kFingerprintBytes = 64;

enum ContactClass : uint32_t {
  kContactNone = 0,
  kContactRobotObstacle = 1,
  kContactGround = 2,
  kContactSelf = 3,
  kContactOther = 4,
  kContactUnknown = 5,
};

enum InvalidReason : uint32_t {
  kInvalidNone = 0,
  kInvalidModelIdentity = 1,
  kInvalidNonFiniteState = 2,
  kInvalidPhysicsStep = 3,
  kInvalidCaptureIdentity = 4,
  kInvalidModelFingerprint = 5,
};

struct Snapshot {
  uint64_t magic;
  uint64_t version;
  uint64_t sequence;          // even stable, odd writer-in-progress
  uint64_t monotonic_ns;      // steady_clock domain
  uint64_t physics_step;      // one-based PhysicsLoop mj_step identity
  double sim_time;             // mjData::time after this mj_step
  uint32_t authoritative;      // 1 only after bound model identity matches
  uint32_t current_collision;  // robot ↔ bound obstacle contact this step
  uint32_t collision_edge;     // current=1 and previous valid current=0
  uint32_t classified_contacts;
  uint32_t unknown_contacts;
  uint32_t robot_obstacle_contacts;
  uint32_t ground_contacts;
  uint32_t self_contacts;
  uint32_t other_contacts;
  uint32_t last_contact_class;
  int32_t last_robot_geom_id;
  int32_t last_obstacle_geom_id;
  uint32_t invalid_reason;
  char scenario_id[kScenarioIdBytes];
  char scene_root_sha256[kSha256HexBytes];
  char model_closure_sha256[kSha256HexBytes];
  char capture_id[kCaptureIdBytes];
  char runtime_model_fingerprint[kFingerprintBytes];
};

static_assert(sizeof(Snapshot) == 392, "unexpected collision snapshot layout");

inline uint64_t loadAcquire(const uint64_t* value) {
  return __atomic_load_n(value, __ATOMIC_ACQUIRE);
}

inline void storeRelease(uint64_t* value, uint64_t next) {
  __atomic_store_n(value, next, __ATOMIC_RELEASE);
}

}  // namespace abs_collision
