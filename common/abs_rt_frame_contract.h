#pragma once

#include <cstddef>
#include <cstdint>

// Versioned cross-process contract for the ABS-Go2 real-time observation frame.
//
// This frame is the SINGLE real-time data link from the authoritative runtime
// producer (StateRL controller) to (1) a real-time HUD and (2) a future formal
// recorder. It carries only values the controller authoritatively computes each
// RL step; anything the controller does not compute is signalled by an explicit
// availability flag (collision_origin, torque_saturated_computed) so consumers
// fail closed instead of showing a fabricated zero as a real result.
//
// All timestamps use std::chrono::steady_clock::time_since_epoch() in ns, the
// same clock domain as /mujoco_ray2d_stamp (abs_ray2d_shm::FrameHeader).
//
// The publish protocol is a single-writer seqlock: the writer sets `sequence`
// odd while the payload changes, writes magic/version/monotonic_ns, then sets
// `sequence` even. A reader accepts a snapshot only when `sequence` is even and
// unchanged across the read (see StateRL::updateRay2d for the same pattern).
//
// Joint-order note (documented, never remapped here):
//   action_raw[]         policy order (ROS1 FL,FR,RL,RR)
//   action_clipped[]     controller order (FR,FL,RR,RL) via policyToCtrlDofOrder
//   joint_target_rad[]   controller order (output_dof_pos_)
//   torque_nm[]          controller order (output_torques)
namespace abs_rt_frame {

constexpr const char* kFrameShmName = "/mujoco_rt_frame";
constexpr uint64_t kMagic = 0x414253525446524DULL;  // mnemonic "ABSRTFRM"
constexpr uint64_t kVersion = 1;
constexpr int kJointCount = 12;
constexpr int kRayCount = 11;

// Frame source classification (mirrors the P1-09B adapter input-origin boundary).
// Only AUTHORITATIVE_RUNTIME may be shown as "live simulation data".
enum Source : uint32_t {
  kSourceUnset = 0,
  kSourceAuthoritativeRuntime = 1,
  kSourceSyntheticTest = 2,
  kSourceLegacyOnly = 3,
};

enum PolicyState : uint32_t {
  kPolicyAgile = 0,
  kPolicyRecovery = 1,
  kPolicyFaulted = 2,
};

enum RayOrigin : uint32_t {
  kRayUnavailable = 0,
  kRayShmRuntime = 1,
};

enum CollisionOrigin : uint32_t {
  kCollisionUnavailable = 0,  // collision shm is bridge-side; not in controller frame
};

struct FrameHeader {
  uint64_t magic;
  uint64_t version;
  uint64_t sequence;      // even: stable; odd: writer owns the frame
  uint64_t monotonic_ns;  // completion time of the frame
};

static_assert(sizeof(FrameHeader) == 32, "unexpected shared-memory header layout");

// Fixed-layout frame. Field order is chosen so the struct has no implicit
// padding and matches the Python struct format "<7Q11I81f" exactly (424 bytes).
struct RuntimeFrame {
  FrameHeader header;             // offset 0..32
  uint64_t session_id;            // monotonicNowNs() assigned at StateRL::enter()
  uint64_t rl_step;               // rl_step_count_
  uint64_t ray_age_ns;            // last_ray_age_ns_ (0 when rays never valid)
  uint32_t source;                // Source
  uint32_t controller_active;     // 1 while the controller state machine owns RL
  uint32_t rl_entered;            // running_
  uint32_t rl_active;             // running_ && !safety_faulted_
  uint32_t safety_faulted;        // 0/1
  uint32_t policy_state;          // PolicyState (AGILE/RECOVERY/FAULTED)
  uint32_t ray_origin;            // RayOrigin (SHM_RUNTIME only when ray2d shm mapped)
  uint32_t ray_valid;             // 0/1 (last updateRay2d accepted a fresh frame)
  uint32_t collision_origin;      // CollisionOrigin (always UNAVAILABLE today)
  uint32_t torque_saturated_computed;  // 0 = not computed anywhere yet
  uint32_t reserved_pad;          // explicit padding to keep an 8-byte total size
  float ra_value;                 // RA value (-inf..inf)
  float lin_vel[3];               // body-frame actual velocity (obs_.lin_vel)
  float command[3];               // body_x, body_y, heading_cmd (obs_.commands)
  float world_pose[3];            // robot_wx, robot_wy, robot_yaw
  float ray2d[11];                // log2 ray distances (obs_.ray2d)
  float action_raw[12];           // policy order (policy_actions)
  float action_clipped[12];       // controller order (clamped_actions)
  float joint_target_rad[12];     // controller order (output_dof_pos_)
  float torque_nm[12];            // controller order (output_torques)
  float torque_saturated[12];     // NOT computed; always 0.0 with flag=0
};

static_assert(sizeof(RuntimeFrame) == 424, "unexpected runtime frame layout");

inline uint64_t loadAcquire(const uint64_t* value) {
  return __atomic_load_n(value, __ATOMIC_ACQUIRE);
}

inline void storeRelease(uint64_t* value, uint64_t next) {
  __atomic_store_n(value, next, __ATOMIC_RELEASE);
}

}  // namespace abs_rt_frame
