#pragma once

#include <mujoco/mujoco.h>

#include <array>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <limits>
#include <iostream>
#include <map>
#include <string>
#include <sys/mman.h>
#include <unistd.h>
#include <vector>

#include "abs_collision_contract.h"
#include "abs_collision_model_fingerprint.h"
#include "abs_scene_catalog.h"
#include "abs_safety_rules.h"
#include "foot_contact_policy.h"

// Versioned collision authority for the explicitly supported S2 scene set.
class ObstacleCollisionAuthority {
 public:
  explicit ObstacleCollisionAuthority(const char* shm_name = abs_collision::kShmName) {
    const char* scenario = std::getenv("ABS_P1_10_SCENARIO_ID");
    const char* root = std::getenv("ABS_P1_10_ROOT_XML_SHA256");
    const char* closure = std::getenv("ABS_P1_10_MODEL_CLOSURE_SHA256");
    const char* capture = std::getenv("ABS_P1_10_CAPTURE_ID");
    const char* expected = std::getenv("ABS_P1_10_EXPECTED_MODEL_FINGERPRINT");
    scenario_id_ = scenario == nullptr ? "" : scenario;
    root_sha256_ = root == nullptr ? "" : root;
    closure_sha256_ = closure == nullptr ? "" : closure;
    capture_id_ = capture == nullptr ? "" : capture;
    expected_fingerprint_ = expected == nullptr ? "" : expected;
    scene_spec_ = abs_scene::byId(scenario_id_);
    capture_binding_valid_ = scene_spec_ != nullptr &&
                             root_sha256_ == scene_spec_->root_sha256 &&
                             closure_sha256_ == scene_spec_->closure_sha256 &&
                             expected_fingerprint_ == scene_spec_->model_fingerprint &&
                             validHex(capture_id_, 32, "p1-10-capture-") &&
                             validHex(expected_fingerprint_, 64, "");
    shm_name_ = shm_name == nullptr ? abs_collision::kShmName : shm_name;
    const mode_t shm_mode = shm_name_ == abs_collision::kShmName ? 0666 : 0600;
    fd_ = shm_open(shm_name_.c_str(), O_CREAT | O_RDWR, shm_mode);
    if (fd_ < 0) return;
    if (ftruncate(fd_, static_cast<off_t>(sizeof(abs_collision::Snapshot))) != 0) {
      close(fd_);
      fd_ = -1;
      return;
    }
    ptr_ = static_cast<abs_collision::Snapshot*>(mmap(
        nullptr, sizeof(abs_collision::Snapshot), PROT_READ | PROT_WRITE,
        MAP_SHARED, fd_, 0));
    if (ptr_ == MAP_FAILED) {
      ptr_ = nullptr;
      close(fd_);
      fd_ = -1;
      return;
    }
    abs_collision::storeRelease(&ptr_->sequence, 1);
    std::memset(reinterpret_cast<char*>(ptr_) + sizeof(uint64_t), 0,
                sizeof(abs_collision::Snapshot) - sizeof(uint64_t));
    abs_collision::storeRelease(&ptr_->magic, abs_collision::kMagic);
    abs_collision::storeRelease(&ptr_->version, abs_collision::kVersion);
    abs_collision::storeRelease(&ptr_->sequence, 2);
  }

  ~ObstacleCollisionAuthority() {
    if (ptr_ != nullptr && ptr_ != MAP_FAILED) munmap(ptr_, sizeof(abs_collision::Snapshot));
    if (fd_ >= 0) close(fd_);
  }

  ObstacleCollisionAuthority(const ObstacleCollisionAuthority&) = delete;
  ObstacleCollisionAuthority& operator=(const ObstacleCollisionAuthority&) = delete;

  bool available() const { return ptr_ != nullptr && ptr_ != MAP_FAILED; }

  // Called after each harness-controlled PhysicsLoop mj_step. UI
  // step-forward in simulate.cc is interactive debugging, not formal P1-10
  // capture authority and intentionally does not publish this snapshot.
  // No policy/controller inputs or physical state are modified.
  void publish(const mjModel* model, const mjData* data, uint64_t physics_step) {
    if (!available()) return;
    current_model_ = model;
    abs_collision::Snapshot local{};
    local.magic = abs_collision::kMagic;
    local.version = abs_collision::kVersion;
    local.monotonic_ns = monotonicNowNs();
    local.physics_step = physics_step;
    local.sim_time = data == nullptr ? std::numeric_limits<double>::quiet_NaN() : data->time;
    local.last_robot_geom_id = -1;
    local.last_obstacle_geom_id = -1;
    local.v5 = foot_state_;
    local.v5.nonfoot_obstacle_contacts = 0;
    local.v5.foot_obstacle_contacts = 0;
    local.v5.foot_contact_mask = 0;
    local.v5.foot_impact_mask = 0;
    local.v5.foot_impact_unknown_mask = 0;
    local.v5.foot_force_valid_mask = 0;
    std::memset(local.v5.foot_world_force, 0, sizeof(local.v5.foot_world_force));
    std::memset(local.v5.foot_fxy, 0, sizeof(local.v5.foot_fxy));
    std::memset(local.v5.foot_abs_fz, 0, sizeof(local.v5.foot_abs_fz));
    std::memset(local.v5.foot_impact_threshold, 0, sizeof(local.v5.foot_impact_threshold));
    local.collision_contact_steps = collision_contact_steps_;
    local.collision_episode_count = collision_episode_count_;
    local.last_collision_start_step = last_collision_start_step_;
    local.last_collision_end_step = last_collision_end_step_;
    local.collision_duration_s = collision_duration_s_;
    local.last_collision_start_sim_time = last_collision_start_sim_time_;
    local.last_collision_end_sim_time = last_collision_end_sim_time_;
    local.fall_start_sim_time = fall_tracker_.start_sim_time;
    local.fall_confirmed_sim_time = fall_tracker_.confirmed_sim_time;
    local.fall_start_physics_step = fall_tracker_.start_physics_step;
    local.fall_confirmed_physics_step = fall_tracker_.confirmed_physics_step;
    local.fall_candidate = fall_tracker_.candidate ? 1U : 0U;
    local.fall_confirmed = fall_tracker_.confirmed ? 1U : 0U;
    local.collision_history_count = static_cast<uint32_t>(
        std::min<uint64_t>(collision_episode_count_, abs_collision::kCollisionEventCapacity));
    local.collision_history_overflow = collision_episode_count_ > abs_collision::kCollisionEventCapacity ? 1U : 0U;
    local.physics_coverage_complete = physics_coverage_complete_ ? 1U : 0U;
    local.unknown_contact_steps = unknown_contact_steps_;
    local.unknown_contact_episodes = unknown_contact_episodes_;
    local.first_unknown_contact_step = first_unknown_contact_step_;
    local.last_unknown_contact_step = last_unknown_contact_step_;
    local.first_unknown_contact_sim_time = first_unknown_contact_sim_time_;
    local.last_unknown_contact_sim_time = last_unknown_contact_sim_time_;
    local.invalid_posture_steps = invalid_posture_steps_;
    local.invalid_posture_episodes = invalid_posture_episodes_;
    local.first_invalid_posture_step = first_invalid_posture_step_;
    local.last_invalid_posture_step = last_invalid_posture_step_;
    local.first_invalid_posture_sim_time = first_invalid_posture_sim_time_;
    local.last_invalid_posture_sim_time = last_invalid_posture_sim_time_;
    local.fall_history_count = static_cast<uint32_t>(
        std::min<uint64_t>(fall_tracker_.generation, abs_collision::kFallEventCapacity));
    local.fall_history_overflow = fall_tracker_.generation > abs_collision::kFallEventCapacity ? 1U : 0U;
    std::memcpy(local.fall_history, fall_history_.data(), sizeof(local.fall_history));
    std::memcpy(local.collision_history, collision_history_.data(), sizeof(local.collision_history));

    std::vector<int> obstacle_ids;
    if (model != nullptr) {
      std::string fingerprint_error;
      if (!abs_collision_model::compute(model, &runtime_fingerprint_, &fingerprint_error)) {
        runtime_fingerprint_.clear();
      }
      copyField(local.runtime_model_fingerprint, sizeof(local.runtime_model_fingerprint),
                runtime_fingerprint_);
    }
    copyField(local.capture_id, sizeof(local.capture_id), capture_id_);
    const bool fingerprint_bound = capture_binding_valid_ &&
                                   !runtime_fingerprint_.empty() &&
                                   runtime_fingerprint_ == expected_fingerprint_;
    const bool scene_geometries_bound = model != nullptr &&
        abs_scene::selectObstacleGeoms(model, scene_spec_, obstacle_ids, &scene_error_);
    if (model != nullptr && !foot_geoms_resolved_) {
      foot_geom_set_valid_ = abs_foot::resolveFootGeoms(model, &foot_geoms_, &foot_error_);
      foot_geoms_resolved_ = true;
    }
    const bool bound = model != nullptr && data != nullptr && physics_step > 0 &&
                       capture_binding_valid_ && fingerprint_bound && foot_geom_set_valid_ &&
                       scene_geometries_bound;
    if (!bound) {
      if (previous_physics_step_ != 0) physics_coverage_complete_ = false;
      fall_tracker_.candidate = false;
      fall_tracker_.start_sim_time = 0.0;
      fall_tracker_.start_physics_step = 0;
      local.invalid_reason = physics_step == 0 ? abs_collision::kInvalidPhysicsStep
                           : !capture_binding_valid_ ? abs_collision::kInvalidCaptureIdentity
                           : !fingerprint_bound ? abs_collision::kInvalidModelFingerprint
                           : !foot_geom_set_valid_ ? abs_collision::kInvalidFootGeometryIdentity
                           : abs_collision::kInvalidModelIdentity;
      previous_valid_ = false;
      commit(local);
      return;
    }
    if (!std::isfinite(local.sim_time) || local.monotonic_ns == 0) {
      if (previous_physics_step_ != 0) physics_coverage_complete_ = false;
      fall_tracker_.candidate = false;
      fall_tracker_.start_sim_time = 0.0;
      fall_tracker_.start_physics_step = 0;
      local.invalid_reason = abs_collision::kInvalidNonFiniteState;
      previous_valid_ = false;
      commit(local);
      return;
    }

    local.authoritative = 1;
    copyField(local.scenario_id, sizeof(local.scenario_id), scenario_id_);
    copyField(local.scene_root_sha256, sizeof(local.scene_root_sha256), root_sha256_);
    copyField(local.model_closure_sha256, sizeof(local.model_closure_sha256), closure_sha256_);
    if (!scene_identity_logged_) {
      std::cout << "[ABS-SCENE-AUTHORITY] scene_id=" << scenario_id_
                << " root_sha256=" << root_sha256_
                << " model_closure_sha256=" << closure_sha256_
                << " runtime_model_fingerprint=" << runtime_fingerprint_
                << " obstacle_count=" << obstacle_ids.size()
                << " obstacle_ids=";
      for (std::size_t i = 0; i < obstacle_ids.size(); ++i) {
        if (i) std::cout << ",";
        std::cout << obstacle_ids[i];
      }
      std::cout << std::endl;
      scene_identity_logged_ = true;
    }

    const bool continuous_step = previous_physics_step_ == 0
        ? (physics_step == 1U && std::fabs(local.sim_time - model->opt.timestep) <= 1e-9)
        : (physics_step == previous_physics_step_ + 1U &&
           std::fabs((local.sim_time - previous_sim_time_) - model->opt.timestep) <= 1e-9);
    std::map<uint64_t, FootPairObservation> foot_pairs;
    std::array<std::array<double, 3>, 4> net_foot_force{};
    uint32_t force_valid_mask = 0xFU;
    for (int i = 0; i < data->ncon; ++i) {
      const mjContact& contact = data->contact[i];
      const auto kind = classifyContact(model, contact.geom1, contact.geom2, obstacle_ids);
      const int foot1 = abs_foot::footIndex(foot_geoms_, contact.geom1);
      const int foot2 = abs_foot::footIndex(foot_geoms_, contact.geom2);
      const int foot_index = foot1 >= 0 ? foot1 : foot2;
      const int foot_geom_id = foot1 >= 0 ? contact.geom1 : (foot2 >= 0 ? contact.geom2 : -1);
      const int other_geom_id = foot1 >= 0 ? contact.geom2 : (foot2 >= 0 ? contact.geom1 : -1);
      if (kind == abs_collision::kContactUnknown) {
        ++local.unknown_contacts;
      } else {
        ++local.classified_contacts;
        switch (kind) {
          case abs_collision::kContactRobotObstacle:
            ++local.robot_obstacle_contacts;
            if (!abs_foot::isTerminalRobotObstacleContact(foot_index >= 0)) {
              ++local.v5.foot_obstacle_contacts;
              local.v5.foot_contact_mask |= (1U << foot_index);
              const int obstacle_id = obstacleGeom(contact.geom1, contact.geom2, obstacle_ids);
              foot_pairs[pairKey(foot_geom_id, obstacle_id)].foot_geom_id = foot_geom_id;
              foot_pairs[pairKey(foot_geom_id, obstacle_id)].obstacle_geom_id = obstacle_id;
            } else {
              ++local.v5.nonfoot_obstacle_contacts;
              local.last_robot_geom_id = robotGeom(model, contact.geom1, contact.geom2);
              local.last_obstacle_geom_id = obstacleGeom(contact.geom1, contact.geom2, obstacle_ids);
            }
            break;
          case abs_collision::kContactGround:
            ++local.ground_contacts;
            if (abs_foot::footIndex(foot_geoms_, contact.geom1) >= 0 ||
                abs_foot::footIndex(foot_geoms_, contact.geom2) >= 0) {
              ++local.foot_ground_contacts;
            } else {
              ++local.nonfoot_ground_contacts;
            }
            break;
          case abs_collision::kContactSelf: ++local.self_contacts; break;
          case abs_collision::kContactOther: ++local.other_contacts; break;
          default: break;
        }
        local.last_contact_class = kind;
      }
      // Match the reference net body-force semantics: sum every known external
      // foot contact (including floor and obstacle), excluding robot self-contact.
      if (foot_index >= 0 && other_geom_id >= 0 && !robotGeom(model, other_geom_id)) {
        if (kind == abs_collision::kContactUnknown || contact.efc_address < 0) {
          force_valid_mask &= ~(1U << foot_index);
        } else {
          mjtNum force_torque[6]{};
          std::array<double, 3> world{};
          mj_contactForce(model, data, i, force_torque);
          if (!abs_foot::contactForceOnFootWorld(contact, force_torque,
                                                foot1 >= 0, &world)) {
            force_valid_mask &= ~(1U << foot_index);
          } else {
            if (!abs_foot::accumulateWorldForce(&net_foot_force[foot_index], world)) {
              force_valid_mask &= ~(1U << foot_index);
            }
          }
        }
      }
    }
    local.v5.foot_identity_valid = foot_geom_set_valid_ ? 1U : 0U;
    local.v5.foot_force_valid_mask = force_valid_mask;
    local.v5.foot_impact_mask = 0;
    local.v5.foot_impact_unknown_mask = 0;
    std::map<uint64_t, FootPairObservation> impact_pairs;
    for (int foot = 0; foot < 4; ++foot) {
      for (int axis = 0; axis < 3; ++axis) local.v5.foot_world_force[foot][axis] = net_foot_force[foot][axis];
      const double fxy = std::hypot(net_foot_force[foot][0], net_foot_force[foot][1]);
      const double abs_fz = std::fabs(net_foot_force[foot][2]);
      const double threshold = abs_foot::kImpactVerticalMultiplier * abs_fz + abs_foot::kImpactOffsetN;
      local.v5.foot_fxy[foot] = fxy;
      local.v5.foot_abs_fz[foot] = abs_fz;
      local.v5.foot_impact_threshold[foot] = threshold;
      if ((local.v5.foot_contact_mask & (1U << foot)) == 0) continue;
      for (auto& pair : foot_pairs) {
        if (abs_foot::footIndex(foot_geoms_, pair.second.foot_geom_id) == foot) {
          pair.second.force_known = (force_valid_mask & (1U << foot)) != 0;
          pair.second.fxy = fxy;
          pair.second.abs_fz = abs_fz;
          pair.second.threshold = threshold;
        }
      }
      if ((force_valid_mask & (1U << foot)) == 0 || !std::isfinite(fxy) ||
          !std::isfinite(abs_fz) || !std::isfinite(threshold)) {
        local.v5.foot_impact_unknown_mask |= (1U << foot);
        continue;
      }
      if (!(fxy > threshold)) continue;
      local.v5.foot_impact_mask |= (1U << foot);
      for (auto& pair : foot_pairs) {
        if (abs_foot::footIndex(foot_geoms_, pair.second.foot_geom_id) != foot) continue;
        pair.second.force_known = true;
        pair.second.fxy = fxy;
        pair.second.abs_fz = abs_fz;
        pair.second.threshold = threshold;
        impact_pairs.emplace(pair.first, pair.second);
      }
    }
    updateFootEvents(physics_step, local.sim_time, continuous_step, foot_pairs, impact_pairs,
                     local.v5.foot_impact_unknown_mask, local.v5);
    if (local.unknown_contacts > 0) {
      ++unknown_contact_steps_;
      if (!previous_unknown_contacts_) {
        ++unknown_contact_episodes_;
        if (first_unknown_contact_step_ == 0) {
          first_unknown_contact_step_ = physics_step;
          first_unknown_contact_sim_time_ = local.sim_time;
        }
      }
      last_unknown_contact_step_ = physics_step;
      last_unknown_contact_sim_time_ = local.sim_time;
    }
    // In v5 only a non-foot robot-obstacle contact is a navigation terminal.
    local.current_collision = local.v5.nonfoot_obstacle_contacts > 0 ? 1 : 0;
    local.collision_edge = abs_safety::newCollisionEdge(
        local.current_collision != 0, previous_valid_, previous_current_collision_ != 0) ? 1 : 0;
    if (local.collision_edge) {
      ++collision_episode_count_;
      last_collision_start_step_ = physics_step;
      last_collision_start_sim_time_ = local.sim_time;
      auto& episode = collision_history_[(collision_episode_count_ - 1U) %
                                         abs_collision::kCollisionEventCapacity];
      episode = {};
      episode.start_physics_step = physics_step;
      episode.end_physics_step = physics_step;
      episode.start_sim_time = local.sim_time;
      episode.end_sim_time = local.sim_time;
      episode.robot_geom_id = local.last_robot_geom_id;
      episode.obstacle_geom_id = local.last_obstacle_geom_id;
      copyName(episode.robot_geom_name, sizeof(episode.robot_geom_name),
               mj_id2name(model, mjOBJ_GEOM, episode.robot_geom_id));
      copyName(episode.obstacle_geom_name, sizeof(episode.obstacle_geom_name),
               mj_id2name(model, mjOBJ_GEOM, episode.obstacle_geom_id));
      const std::size_t slot = (collision_episode_count_ - 1U) % abs_collision::kCollisionEventCapacity;
      const int body_id = episode.robot_geom_id >= 0 ? model->geom_bodyid[episode.robot_geom_id] : -1;
      local.v5.nonfoot_collision_body_ids[slot] = body_id;
      copyName(local.v5.nonfoot_collision_body_names[slot],
               sizeof(local.v5.nonfoot_collision_body_names[slot]),
               body_id >= 0 ? mj_id2name(model, mjOBJ_BODY, body_id) : nullptr);
      foot_state_.nonfoot_collision_body_ids[slot] = body_id;
      copyName(foot_state_.nonfoot_collision_body_names[slot],
               sizeof(foot_state_.nonfoot_collision_body_names[slot]),
               body_id >= 0 ? mj_id2name(model, mjOBJ_BODY, body_id) : nullptr);
    }
    if (local.current_collision) {
      ++collision_contact_steps_;
      last_collision_end_step_ = physics_step;
      last_collision_end_sim_time_ = local.sim_time;
      if (previous_valid_ && previous_current_collision_ && physics_step == previous_physics_step_ + 1U &&
          local.sim_time >= previous_sim_time_) {
        collision_duration_s_ += local.sim_time - previous_sim_time_;
      }
      if (collision_episode_count_ > 0) {
        auto& episode = collision_history_[(collision_episode_count_ - 1U) %
                                           abs_collision::kCollisionEventCapacity];
        episode.end_physics_step = physics_step;
        episode.end_sim_time = local.sim_time;
      }
    }
    if (!continuous_step) {
      physics_coverage_complete_ = false;
    }
    updateFallState(model, data, physics_step, local.sim_time, continuous_step, local);
    if (fall_tracker_.generation > 0) {
      auto& episode = fall_history_[(fall_tracker_.generation - 1U) % abs_collision::kFallEventCapacity];
      if (episode.start_physics_step != fall_tracker_.start_physics_step) {
        episode = {};
        episode.start_physics_step = fall_tracker_.start_physics_step;
        episode.start_sim_time = fall_tracker_.start_sim_time;
      }
      episode.start_physics_step = fall_tracker_.start_physics_step;
      episode.end_physics_step = fall_tracker_.end_physics_step;
      episode.start_sim_time = fall_tracker_.start_sim_time;
      episode.end_sim_time = fall_tracker_.end_sim_time;
      if (fall_tracker_.confirmed) {
        episode.confirmed_physics_step = fall_tracker_.confirmed_physics_step;
        episode.confirmed_sim_time = fall_tracker_.confirmed_sim_time;
        episode.confirmed = 1U;
      }
      episode.closed = fall_tracker_.active ? 0U : 1U;
    }
    local.unknown_contact_steps = unknown_contact_steps_;
    local.unknown_contact_episodes = unknown_contact_episodes_;
    local.first_unknown_contact_step = first_unknown_contact_step_;
    local.last_unknown_contact_step = last_unknown_contact_step_;
    local.first_unknown_contact_sim_time = first_unknown_contact_sim_time_;
    local.last_unknown_contact_sim_time = last_unknown_contact_sim_time_;
    local.invalid_posture_steps = invalid_posture_steps_;
    local.invalid_posture_episodes = invalid_posture_episodes_;
    local.first_invalid_posture_step = first_invalid_posture_step_;
    local.last_invalid_posture_step = last_invalid_posture_step_;
    local.first_invalid_posture_sim_time = first_invalid_posture_sim_time_;
    local.last_invalid_posture_sim_time = last_invalid_posture_sim_time_;
    local.fall_history_count = static_cast<uint32_t>(
        std::min<uint64_t>(fall_tracker_.generation, abs_collision::kFallEventCapacity));
    local.fall_history_overflow = fall_tracker_.generation > abs_collision::kFallEventCapacity ? 1U : 0U;
    std::memcpy(local.fall_history, fall_history_.data(), sizeof(local.fall_history));
    previous_unknown_contacts_ = local.unknown_contacts > 0;
    local.collision_contact_steps = collision_contact_steps_;
    local.collision_episode_count = collision_episode_count_;
    local.last_collision_start_step = last_collision_start_step_;
    local.last_collision_end_step = last_collision_end_step_;
    local.collision_duration_s = collision_duration_s_;
    local.last_collision_start_sim_time = last_collision_start_sim_time_;
    local.last_collision_end_sim_time = last_collision_end_sim_time_;
    local.fall_start_sim_time = fall_tracker_.start_sim_time;
    local.fall_confirmed_sim_time = fall_tracker_.confirmed_sim_time;
    local.fall_start_physics_step = fall_tracker_.start_physics_step;
    local.fall_confirmed_physics_step = fall_tracker_.confirmed_physics_step;
    local.fall_candidate = fall_tracker_.candidate ? 1U : 0U;
    local.fall_confirmed = fall_tracker_.confirmed ? 1U : 0U;
    local.collision_history_count = static_cast<uint32_t>(
        std::min<uint64_t>(collision_episode_count_, abs_collision::kCollisionEventCapacity));
    local.collision_history_overflow = collision_episode_count_ > abs_collision::kCollisionEventCapacity ? 1U : 0U;
    local.physics_coverage_complete = physics_coverage_complete_ ? 1U : 0U;
    std::memcpy(local.collision_history, collision_history_.data(), sizeof(local.collision_history));
    foot_state_ = local.v5;
    previous_current_collision_ = local.current_collision;
    previous_valid_ = true;
    previous_physics_step_ = physics_step;
    previous_sim_time_ = local.sim_time;
    commit(local);
  }

 private:
  static uint64_t monotonicNowNs() {
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
  }

  static bool validHex(const std::string& value, std::size_t hex_count,
                       const char* prefix) {
    if (value.size() != std::strlen(prefix) + hex_count) return false;
    if (value.compare(0, std::strlen(prefix), prefix) != 0) return false;
    for (std::size_t i = std::strlen(prefix); i < value.size(); ++i) {
      const char c = value[i];
      if (!((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f'))) return false;
    }
    return true;
  }

  static void copyField(char* target, std::size_t capacity, const std::string& value) {
    if (capacity == 0 || value.size() > capacity) return;
    std::memcpy(target, value.data(), value.size());
  }

  static bool contains(const std::vector<int>& ids, int geom_id) {
    for (int id : ids) if (id == geom_id) return true;
    return false;
  }

  static bool robotGeom(const mjModel* model, int geom_id) {
    return geom_id >= 0 && geom_id < model->ngeom && model->geom_group[geom_id] == 3;
  }

  static bool floorGeom(const mjModel* model, int geom_id) {
    if (geom_id < 0 || geom_id >= model->ngeom) return false;
    const char* name = mj_id2name(model, mjOBJ_GEOM, geom_id);
    return name != nullptr && std::strcmp(name, "floor") == 0;
  }

  static void copyName(char* target, std::size_t capacity, const char* value) {
    if (capacity == 0) return;
    target[0] = '\0';
    if (value == nullptr) return;
    const std::size_t length = std::min(capacity - 1, std::strlen(value));
    std::memcpy(target, value, length);
    target[length] = '\0';
  }

  struct FootPairObservation {
    int foot_geom_id = -1;
    int obstacle_geom_id = -1;
    bool force_known = false;
    double fxy = 0.0;
    double abs_fz = 0.0;
    double threshold = 0.0;
  };

  static uint64_t pairKey(int foot_geom_id, int obstacle_geom_id) {
    return (static_cast<uint64_t>(static_cast<uint32_t>(foot_geom_id)) << 32) |
           static_cast<uint32_t>(obstacle_geom_id);
  }

  static unsigned bitCount4(uint32_t mask) {
    unsigned count = 0;
    for (unsigned bit = 0; bit < 4; ++bit) count += (mask >> bit) & 1U;
    return count;
  }

  template <std::size_t Capacity>
  void updateFootEventGroup(
      uint64_t physics_step, double sim_time, bool continuous_step,
      const std::map<uint64_t, FootPairObservation>& current,
      std::map<uint64_t, uint32_t>& active,
      abs_collision::Snapshot::V5Extension::FootEvent (&history)[Capacity],
      uint64_t& episode_count, uint64_t& step_count, uint64_t& first_step,
      uint64_t& last_step, double& duration_s, double& first_time, double& last_time) {
    using Event = abs_collision::Snapshot::V5Extension::FootEvent;
    if (!continuous_step) {
      for (const auto& item : active) {
        const std::size_t index = item.second;
        if (index < Capacity) {
          history[index].end_physics_step = previous_physics_step_;
          history[index].end_sim_time = previous_sim_time_;
          history[index].closed = 0;  // interrupted coverage: right-censored
        }
      }
      active.clear();
    } else {
      for (auto it = active.begin(); it != active.end();) {
        if (current.find(it->first) != current.end()) { ++it; continue; }
        const std::size_t index = it->second;
        if (index < Capacity) {
          history[index].end_physics_step = previous_physics_step_;
          history[index].end_sim_time = previous_sim_time_;
          history[index].closed = 1;
        }
        it = active.erase(it);
      }
    }

    if (!current.empty()) {
      ++step_count;
      const double dt = previous_physics_step_ == 0
          ? sim_time : sim_time - previous_sim_time_;
      if (continuous_step && dt >= 0.0 && std::isfinite(dt)) duration_s += dt;
    }
    for (const auto& item : current) {
      const uint64_t key = item.first;
      const FootPairObservation& observation = item.second;
      auto found = active.find(key);
      if (found == active.end()) {
        ++episode_count;
        if (first_step == 0) {
          first_step = physics_step;
          first_time = sim_time;
        }
        const std::size_t index = (episode_count - 1U) % Capacity;
        Event event{};
        event.start_physics_step = physics_step;
        event.end_physics_step = physics_step;
        event.start_sim_time = sim_time;
        event.end_sim_time = sim_time;
        event.foot_geom_id = observation.foot_geom_id;
        event.obstacle_geom_id = observation.obstacle_geom_id;
        event.closed = 0;
        event.force_known = observation.force_known ? 1U : 0U;
        event.trigger_fxy = observation.fxy;
        event.trigger_abs_fz = observation.abs_fz;
        event.trigger_threshold = observation.threshold;
        copyName(event.foot_geom_name, sizeof(event.foot_geom_name),
                 mj_id2name(current_model_, mjOBJ_GEOM, observation.foot_geom_id));
        copyName(event.obstacle_geom_name, sizeof(event.obstacle_geom_name),
                 mj_id2name(current_model_, mjOBJ_GEOM, observation.obstacle_geom_id));
        history[index] = event;
        active[key] = static_cast<uint32_t>(index);
      } else {
        Event& event = history[found->second];
        event.end_physics_step = physics_step;
        event.end_sim_time = sim_time;
        event.closed = 0;
      }
      last_step = physics_step;
      last_time = sim_time;
    }
  }

  void updateFootEvents(uint64_t physics_step, double sim_time, bool continuous_step,
                        const std::map<uint64_t, FootPairObservation>& contact_pairs,
                        const std::map<uint64_t, FootPairObservation>& impact_pairs,
                        uint32_t unknown_impact_mask,
                        abs_collision::Snapshot::V5Extension& state) {
    updateFootEventGroup(physics_step, sim_time, continuous_step, contact_pairs,
        active_foot_contacts_, state.foot_contact_history, state.foot_contact_episode_count,
        state.foot_contact_physics_steps, state.foot_contact_first_step,
        state.foot_contact_last_step, state.foot_contact_duration_s,
        state.foot_contact_first_sim_time, state.foot_contact_last_sim_time);
    updateFootEventGroup(physics_step, sim_time, continuous_step, impact_pairs,
        active_foot_impacts_, state.foot_impact_history, state.foot_impact_episode_count,
        state.foot_impact_physics_steps, state.foot_impact_first_step,
        state.foot_impact_last_step, state.foot_impact_duration_s,
        state.foot_impact_first_sim_time, state.foot_impact_last_sim_time);
    const unsigned unknown_legs = bitCount4(unknown_impact_mask);
    state.foot_impact_unknown_steps += unknown_legs;
    if (unknown_legs > 0) {
      if (state.foot_impact_unknown_first_step == 0) {
        state.foot_impact_unknown_first_step = physics_step;
        state.foot_impact_unknown_first_sim_time = sim_time;
      }
      state.foot_impact_unknown_last_step = physics_step;
      state.foot_impact_unknown_last_sim_time = sim_time;
    }
    state.foot_contact_history_count = static_cast<uint32_t>(
        std::min<uint64_t>(state.foot_contact_episode_count, abs_collision::kFootEventCapacity));
    state.foot_contact_history_overflow =
        state.foot_contact_episode_count > abs_collision::kFootEventCapacity ? 1U : 0U;
    state.foot_impact_history_count = static_cast<uint32_t>(
        std::min<uint64_t>(state.foot_impact_episode_count, abs_collision::kFootEventCapacity));
    state.foot_impact_history_overflow =
        state.foot_impact_episode_count > abs_collision::kFootEventCapacity ? 1U : 0U;
    foot_state_ = state;
  }

  void updateFallState(const mjModel* model, const mjData* data, uint64_t physics_step,
                       double sim_time, bool continuous_step,
                       abs_collision::Snapshot& local) {
    const int base_id = mj_name2id(model, mjOBJ_BODY, "base_link");
    const bool body_valid = base_id >= 0 && std::isfinite(sim_time);
    if (body_valid) {
      local.base_height_m = data->xpos[3 * base_id + 2];
      const mjtNum* q = data->xquat + 4 * base_id;  // MuJoCo world quaternion: w,x,y,z.
      const double w = q[0], x = q[1], y = q[2], z = q[3];
      local.base_roll_rad = std::atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
      local.base_pitch_rad = std::asin(std::clamp(2.0 * (w * y - z * x), -1.0, 1.0));
    }
    const bool posture_valid = body_valid && std::isfinite(local.base_height_m) &&
        std::isfinite(local.base_roll_rad) && std::isfinite(local.base_pitch_rad);
    if (!posture_valid) {
      ++invalid_posture_steps_;
      if (!previous_invalid_posture_) {
        ++invalid_posture_episodes_;
        if (first_invalid_posture_step_ == 0) {
          first_invalid_posture_step_ = physics_step;
          first_invalid_posture_sim_time_ = sim_time;
        }
      }
      last_invalid_posture_step_ = physics_step;
      last_invalid_posture_sim_time_ = sim_time;
    }
    previous_invalid_posture_ = !posture_valid;
    // The invalid-pose occurrence is preserved by the v4 latch above. Keep
    // numeric payload fields finite so the ordinary reader can accept this
    // otherwise authoritative snapshot and classify the motion window UNKNOWN.
    if (!posture_valid) {
      local.base_height_m = 0.0;
      local.base_roll_rad = 0.0;
      local.base_pitch_rad = 0.0;
    }
    abs_safety::updateFall(fall_tracker_, local.base_height_m, local.base_roll_rad,
                           local.base_pitch_rad, physics_step, sim_time, posture_valid,
                           continuous_step);
    local.fall_candidate = fall_tracker_.candidate ? 1U : 0U;
    local.fall_confirmed = fall_tracker_.confirmed ? 1U : 0U;
    local.fall_start_sim_time = fall_tracker_.start_sim_time;
    local.fall_start_physics_step = fall_tracker_.start_physics_step;
    local.fall_confirmed_sim_time = fall_tracker_.confirmed_sim_time;
    local.fall_confirmed_physics_step = fall_tracker_.confirmed_physics_step;
  }

  static int robotGeom(const mjModel* model, int first, int second) {
    return robotGeom(model, first) ? first : (robotGeom(model, second) ? second : -1);
  }

  static int obstacleGeom(int first, int second, const std::vector<int>& ids) {
    return contains(ids, first) ? first : (contains(ids, second) ? second : -1);
  }

  static uint32_t classifyContact(const mjModel* model, int first, int second,
                                  const std::vector<int>& obstacle_ids) {
    if (first < 0 || second < 0 || first >= model->ngeom || second >= model->ngeom) {
      return abs_collision::kContactUnknown;
    }
    const bool robot_first = robotGeom(model, first);
    const bool robot_second = robotGeom(model, second);
    const bool obstacle_first = contains(obstacle_ids, first);
    const bool obstacle_second = contains(obstacle_ids, second);
    const bool floor_first = floorGeom(model, first);
    const bool floor_second = floorGeom(model, second);
    return abs_safety::classifyContact(robot_first, robot_second, obstacle_first,
                                      obstacle_second, floor_first, floor_second);
  }

  void commit(abs_collision::Snapshot& local) {
    uint64_t sequence = abs_collision::loadAcquire(&ptr_->sequence);
    if (sequence & 1U) ++sequence;
    abs_collision::storeRelease(&ptr_->sequence, sequence + 1U);
    std::memcpy(reinterpret_cast<char*>(ptr_) + sizeof(uint64_t),
                reinterpret_cast<const char*>(&local) + sizeof(uint64_t),
                sizeof(abs_collision::Snapshot) - sizeof(uint64_t));
    abs_collision::storeRelease(&ptr_->sequence, sequence + 2U);
  }

  int fd_ = -1;
  abs_collision::Snapshot* ptr_ = nullptr;
  std::string scenario_id_;
  std::string shm_name_;
  std::string root_sha256_;
  std::string closure_sha256_;
  const abs_scene::SceneSpec* scene_spec_ = nullptr;
  std::string scene_error_;
  const mjModel* current_model_ = nullptr;
  abs_foot::FootGeomSet foot_geoms_{};
  bool foot_geoms_resolved_ = false;
  bool foot_geom_set_valid_ = false;
  std::string foot_error_;
  abs_collision::Snapshot::V5Extension foot_state_{};
  std::map<uint64_t, uint32_t> active_foot_contacts_;
  std::map<uint64_t, uint32_t> active_foot_impacts_;
  std::string capture_id_;
  std::string expected_fingerprint_;
  std::string runtime_fingerprint_;
  bool capture_binding_valid_ = false;
  bool scene_identity_logged_ = false;
  bool previous_valid_ = false;
  uint32_t previous_current_collision_ = 0;
  uint64_t collision_contact_steps_ = 0;
  uint64_t collision_episode_count_ = 0;
  uint64_t last_collision_start_step_ = 0;
  uint64_t last_collision_end_step_ = 0;
  double collision_duration_s_ = 0.0;
  double last_collision_start_sim_time_ = 0.0;
  double last_collision_end_sim_time_ = 0.0;
  uint64_t previous_physics_step_ = 0;
  double previous_sim_time_ = 0.0;
  abs_safety::FallTracker fall_tracker_{};
  bool physics_coverage_complete_ = true;
  bool previous_unknown_contacts_ = false;
  bool previous_invalid_posture_ = false;
  uint64_t unknown_contact_steps_ = 0;
  uint64_t unknown_contact_episodes_ = 0;
  uint64_t first_unknown_contact_step_ = 0;
  uint64_t last_unknown_contact_step_ = 0;
  double first_unknown_contact_sim_time_ = 0.0;
  double last_unknown_contact_sim_time_ = 0.0;
  uint64_t invalid_posture_steps_ = 0;
  uint64_t invalid_posture_episodes_ = 0;
  uint64_t first_invalid_posture_step_ = 0;
  uint64_t last_invalid_posture_step_ = 0;
  double first_invalid_posture_sim_time_ = 0.0;
  double last_invalid_posture_sim_time_ = 0.0;
  std::array<abs_collision::Snapshot::FallEpisode,
             abs_collision::kFallEventCapacity> fall_history_{};
  std::array<abs_collision::Snapshot::CollisionEpisode,
             abs_collision::kCollisionEventCapacity> collision_history_{};
};
