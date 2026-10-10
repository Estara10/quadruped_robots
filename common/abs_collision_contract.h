#pragma once

#include <cstddef>
#include <cstdint>

// Versioned, fail-closed collision snapshot written by the simulator physics
// step.  This is the formal source; the legacy /mujoco_collision int32 buffer
// remains only for old diagnostics and is never read by the formal recorder.
namespace abs_collision {

constexpr const char* kShmName = "/mujoco_collision_v2";
constexpr uint64_t kMagic = 0x414253434F4E5432ULL;  // "ABSCONT2"
// v3 is the fixed 3864-byte layout. v4 is 5760 bytes. v5 appends explicit
// foot-contact/impact events and changes current_collision to mean non-foot
// robot-obstacle collision. Readers must pair version with exact size.
constexpr uint64_t kVersion = 5;
constexpr std::size_t kScenarioIdBytes = 32;
constexpr std::size_t kSha256HexBytes = 64;
constexpr std::size_t kCaptureIdBytes = 64;
constexpr std::size_t kFingerprintBytes = 64;
constexpr std::size_t kCollisionEventCapacity = 32;
constexpr std::size_t kGeomNameBytes = 32;
constexpr std::size_t kFallEventCapacity = 32;
constexpr std::size_t kFootEventCapacity = 32;

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
  kInvalidFootGeometryIdentity = 6,
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
  uint32_t foot_ground_contacts;
  uint32_t nonfoot_ground_contacts;
  uint64_t collision_contact_steps;
  uint64_t collision_episode_count;
  uint64_t last_collision_start_step;
  uint64_t last_collision_end_step;
  double collision_duration_s;
  double last_collision_start_sim_time;
  double last_collision_end_sim_time;
  double base_height_m;
  double base_roll_rad;
  double base_pitch_rad;
  double fall_start_sim_time;
  double fall_confirmed_sim_time;
  uint64_t fall_start_physics_step;
  uint64_t fall_confirmed_physics_step;
  uint32_t fall_candidate;
  uint32_t fall_confirmed;
  uint32_t collision_history_count;
  uint32_t collision_history_overflow;
  uint32_t physics_coverage_complete;
  uint32_t reserved_extension;
  uint64_t unknown_contact_steps;
  uint64_t unknown_contact_episodes;
  uint64_t first_unknown_contact_step;
  uint64_t last_unknown_contact_step;
  uint64_t invalid_posture_steps;
  uint64_t invalid_posture_episodes;
  uint64_t first_invalid_posture_step;
  uint64_t last_invalid_posture_step;
  double first_unknown_contact_sim_time;
  double last_unknown_contact_sim_time;
  double first_invalid_posture_sim_time;
  double last_invalid_posture_sim_time;
  uint32_t fall_history_count;
  uint32_t fall_history_overflow;
  struct FallEpisode {
    uint64_t start_physics_step;
    uint64_t end_physics_step;
    uint64_t confirmed_physics_step;
    double start_sim_time;
    double end_sim_time;
    double confirmed_sim_time;
    uint32_t confirmed;
    uint32_t closed;
  } fall_history[kFallEventCapacity];
  struct CollisionEpisode {
    uint64_t start_physics_step;
    uint64_t end_physics_step;
    double start_sim_time;
    double end_sim_time;
    int32_t robot_geom_id;
    int32_t obstacle_geom_id;
    char robot_geom_name[kGeomNameBytes];
    char obstacle_geom_name[kGeomNameBytes];
  } collision_history[kCollisionEventCapacity];

  // v5 extension. The v3/v4 prefix above is byte-for-byte unchanged. In v5,
  // current_collision/collision_history refer only to non-foot termination.
  // robot_obstacle_contacts remains the total raw robot-obstacle contact count.
  struct V5Extension {
    uint32_t foot_identity_valid;
    uint32_t nonfoot_obstacle_contacts;
    uint32_t foot_obstacle_contacts;
    uint32_t foot_contact_mask;          // bit order FL, FR, RL, RR
    uint32_t foot_impact_mask;
    uint32_t foot_force_valid_mask;
    uint32_t foot_impact_unknown_mask;
    uint32_t foot_contact_history_count;
    uint32_t foot_contact_history_overflow;
    uint32_t foot_impact_history_count;
    uint32_t foot_impact_history_overflow;
    uint32_t reserved_flags;
    uint64_t foot_contact_physics_steps;
    uint64_t foot_contact_episode_count;
    uint64_t foot_contact_first_step;
    uint64_t foot_contact_last_step;
    uint64_t foot_impact_physics_steps;
    uint64_t foot_impact_episode_count;
    uint64_t foot_impact_first_step;
    uint64_t foot_impact_last_step;
    uint64_t foot_impact_unknown_steps;
    uint64_t foot_impact_unknown_first_step;
    uint64_t foot_impact_unknown_last_step;
    double foot_contact_duration_s;
    double foot_contact_first_sim_time;
    double foot_contact_last_sim_time;
    double foot_impact_duration_s;
    double foot_impact_first_sim_time;
    double foot_impact_last_sim_time;
    double foot_impact_unknown_first_sim_time;
    double foot_impact_unknown_last_sim_time;
    double foot_world_force[4][3];        // N; net force from known external contacts
    double foot_fxy[4];                   // N; hypot(world Fx, world Fy)
    double foot_abs_fz[4];                // N
    double foot_impact_threshold[4];      // N; 2*abs(Fz)+10
    struct FootEvent {
      uint64_t start_physics_step;
      uint64_t end_physics_step;
      double start_sim_time;
      double end_sim_time;
      int32_t foot_geom_id;
      int32_t obstacle_geom_id;
      uint32_t closed;                    // false means right-censored/coverage gap
      uint32_t force_known;
      double trigger_fxy;
      double trigger_abs_fz;
      double trigger_threshold;
      char foot_geom_name[8];
      char obstacle_geom_name[kGeomNameBytes];
    } foot_contact_history[kFootEventCapacity];
    FootEvent foot_impact_history[kFootEventCapacity];
    int32_t nonfoot_collision_body_ids[kCollisionEventCapacity];
    char nonfoot_collision_body_names[kCollisionEventCapacity][kGeomNameBytes];
  } v5;
};

static_assert(offsetof(Snapshot, v5) == 5'760, "v4 prefix layout changed");
static_assert(sizeof(Snapshot) == 14'472, "unexpected collision snapshot v5 layout");

inline uint64_t loadAcquire(const uint64_t* value) {
  return __atomic_load_n(value, __ATOMIC_ACQUIRE);
}

inline void storeRelease(uint64_t* value, uint64_t next) {
  __atomic_store_n(value, next, __ATOMIC_RELEASE);
}

}  // namespace abs_collision
