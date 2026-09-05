#pragma once

#include <mujoco/mujoco.h>

#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <limits>
#include <string>
#include <sys/mman.h>
#include <unistd.h>
#include <vector>

#include "abs_collision_contract.h"
#include "abs_collision_model_fingerprint.h"

// Minimal Stage-B authority for the canonical obstacle_test1 model.  The
// bridge's legacy integer telemetry is deliberately not consulted here.
class ObstacleCollisionAuthority {
 public:
  ObstacleCollisionAuthority() {
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
    capture_binding_valid_ = scenario_id_ == kScenarioId &&
                             root_sha256_ == kSceneRootSha256 &&
                             closure_sha256_ == kModelClosureSha256 &&
                             validHex(capture_id_, 32, "p1-10-capture-") &&
                             validHex(expected_fingerprint_, 64, "");
    fd_ = shm_open(abs_collision::kShmName, O_CREAT | O_RDWR, 0666);
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
    abs_collision::Snapshot local{};
    local.magic = abs_collision::kMagic;
    local.version = abs_collision::kVersion;
    local.monotonic_ns = monotonicNowNs();
    local.physics_step = physics_step;
    local.sim_time = data == nullptr ? std::numeric_limits<double>::quiet_NaN() : data->time;
    local.last_robot_geom_id = -1;
    local.last_obstacle_geom_id = -1;

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
    const bool bound = model != nullptr && data != nullptr && physics_step > 0 &&
                       capture_binding_valid_ && fingerprint_bound &&
                       modelIdentityMatches(model, obstacle_ids);
    if (!bound) {
      local.invalid_reason = physics_step == 0 ? abs_collision::kInvalidPhysicsStep
                           : !capture_binding_valid_ ? abs_collision::kInvalidCaptureIdentity
                           : !fingerprint_bound ? abs_collision::kInvalidModelFingerprint
                           : abs_collision::kInvalidModelIdentity;
      previous_valid_ = false;
      commit(local);
      return;
    }
    if (!std::isfinite(local.sim_time) || local.monotonic_ns == 0) {
      local.invalid_reason = abs_collision::kInvalidNonFiniteState;
      previous_valid_ = false;
      commit(local);
      return;
    }

    local.authoritative = 1;
    copyField(local.scenario_id, sizeof(local.scenario_id), scenario_id_);
    copyField(local.scene_root_sha256, sizeof(local.scene_root_sha256), root_sha256_);
    copyField(local.model_closure_sha256, sizeof(local.model_closure_sha256), closure_sha256_);

    for (int i = 0; i < data->ncon; ++i) {
      const mjContact& contact = data->contact[i];
      const auto kind = classifyContact(model, contact.geom1, contact.geom2, obstacle_ids);
      if (kind == abs_collision::kContactUnknown) {
        ++local.unknown_contacts;
      } else {
        ++local.classified_contacts;
        switch (kind) {
          case abs_collision::kContactRobotObstacle:
            ++local.robot_obstacle_contacts;
            local.last_robot_geom_id = robotGeom(model, contact.geom1, contact.geom2);
            local.last_obstacle_geom_id = obstacleGeom(contact.geom1, contact.geom2, obstacle_ids);
            break;
          case abs_collision::kContactGround: ++local.ground_contacts; break;
          case abs_collision::kContactSelf: ++local.self_contacts; break;
          case abs_collision::kContactOther: ++local.other_contacts; break;
          default: break;
        }
        local.last_contact_class = kind;
      }
    }
    local.current_collision = local.robot_obstacle_contacts > 0 ? 1 : 0;
    local.collision_edge = (local.current_collision != 0 &&
                            (!previous_valid_ || previous_current_collision_ == 0)) ? 1 : 0;
    previous_current_collision_ = local.current_collision;
    previous_valid_ = true;
    commit(local);
  }

 private:
  struct Signature { int type; double pos[3]; double size[3]; };

  static constexpr const char* kScenarioId = "obstacle_test1";
  static constexpr const char* kSceneRootSha256 =
      "e12a69fa5463e723d115696b8872c27c71b03a9d029a9ef933343ae93ba6dd5e";
  static constexpr const char* kModelClosureSha256 =
      "6ca5da14be6909815ac9c41bf6db0f8108e07082aea5aba22c91e833e6181746";
  static constexpr std::array<Signature, 7> kObstacleSignatures = {{
      {mjGEOM_BOX, {4.77821, 1.39196, 0.319396}, {0.178009, 0.177997, 0.0656475}},
      {mjGEOM_BOX, {4.39779, 0.60669, 0.368633}, {0.110292, 0.584955, 0.315243}},
      {mjGEOM_BOX, {1.45553, -1.90905, 0.132532}, {0.252121, 0.362378, 0.0856493}},
      {mjGEOM_BOX, {1.81053, 0.671117, 0.112772}, {0.246072, 0.283181, 0.0786285}},
      {mjGEOM_BOX, {4.03329, -1.80196, 0.281405}, {0.396207, 0.123225, 0.190589}},
      {mjGEOM_BOX, {1.26736, -2.60969, 0.476998}, {0.582816, 0.504199, 0.18007}},
      {mjGEOM_CYLINDER, {1.69268, -1.29487, 0.609958}, {0.252499, 0.609958, 0.609958}},
  }};

  static uint64_t monotonicNowNs() {
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
  }

  static bool nearlyEqual(double a, double b) {
    return std::isfinite(a) && std::fabs(a - b) <= 1e-12;
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

  bool modelIdentityMatches(const mjModel* model, std::vector<int>& ids) const {
    if (model->ngeom <= 0) return false;
    std::array<bool, kObstacleSignatures.size()> found{};
    for (int geom_id = 0; geom_id < model->ngeom; ++geom_id) {
      for (std::size_t i = 0; i < kObstacleSignatures.size(); ++i) {
        const Signature& sig = kObstacleSignatures[i];
        if (found[i] || model->geom_type[geom_id] != sig.type) continue;
        bool match = true;
        for (int axis = 0; axis < 3; ++axis) {
          match = match && nearlyEqual(model->geom_pos[geom_id * 3 + axis], sig.pos[axis]);
          match = match && nearlyEqual(model->geom_size[geom_id * 3 + axis], sig.size[axis]);
        }
        if (match) {
          found[i] = true;
          ids.push_back(geom_id);
          break;
        }
      }
    }
    for (bool item : found) if (!item) return false;
    return ids.size() == kObstacleSignatures.size();
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
    if ((robot_first && obstacle_second) || (robot_second && obstacle_first)) {
      return abs_collision::kContactRobotObstacle;
    }
    if (floor_first || floor_second) return abs_collision::kContactGround;
    if (robot_first && robot_second) return abs_collision::kContactSelf;
    const bool known_first = robot_first || obstacle_first;
    const bool known_second = robot_second || obstacle_second;
    if (known_first && known_second) return abs_collision::kContactOther;
    return abs_collision::kContactUnknown;
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
  std::string root_sha256_;
  std::string closure_sha256_;
  std::string capture_id_;
  std::string expected_fingerprint_;
  std::string runtime_fingerprint_;
  bool capture_binding_valid_ = false;
  bool previous_valid_ = false;
  uint32_t previous_current_collision_ = 0;
};
