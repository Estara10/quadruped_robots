#pragma once

#include <mujoco/mujoco.h>

#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/dds_wrapper/robots/go2/go2.h>
#include <unitree/dds_wrapper/robots/g1/g1.h>

#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

#include "joinable_thread.h"

#include <abs_ray2d_shm_contract.h>
#include "param.h"
#include "physics_joystick.h"

#define MOTOR_SENSOR_NUM 3

// ===== Ray2d shared memory constants =====
#define RAY2D_SHM_NAME "/mujoco_ray2d"
#define RAY2D_STAMP_SHM_NAME "/mujoco_ray2d_stamp"
#define QPOS_SHM_NAME "/mujoco_qpos"
#define COLLISION_SHM_NAME "/mujoco_collision"
#define QPOS_COUNT 19
#define RAY2D_COUNT 11
#define COLLISION_COUNT 5
#define RAY2D_MAX_DIST 6.0f
#define RAY2D_MIN_DIST 0.1f
#define RAY2D_THETA_START (-M_PI / 4.0)   // -45 degrees
#define RAY2D_THETA_END (M_PI / 4.0)       // +45 degrees
#define RAY2D_THETA_STEP (M_PI / 20.0)     // 9 degrees -> 11 beams
#define RAY2D_X0 (-0.05f)                   // body-frame x offset
#define RAY2D_Y0 0.0f                       // body-frame y offset
#define RAY2D_MIN_DIST_SQ (0.01f)            // min distance squared (0.1^2), avoid self-hit

class UnitreeSDK2BridgeBase {
public:
  UnitreeSDK2BridgeBase(mjModel* model, mjData* data,
                        std::recursive_mutex* md_mutex)
    : mj_model_(model), mj_data_(data), md_mutex_(md_mutex) {
    {
      std::lock_guard<std::recursive_mutex> md_lock(*md_mutex_);
      _check_sensor();
    }
    if (param::config.print_scene_information == 1) {
      printSceneInformation();
    }
    if (param::config.use_joystick == 1) {
      if (param::config.joystick_type == "xbox") {
        joystick = std::make_shared<XBoxJoystick>(
            param::config.joystick_device, param::config.joystick_bits);
      } else if (param::config.joystick_type == "switch") {
        joystick = std::make_shared<SwitchJoystick>(
            param::config.joystick_device, param::config.joystick_bits);
      } else {
        std::cerr << "Unsupported joystick type: " << param::config.
            joystick_type << std::endl;
        exit(EXIT_FAILURE);
      }
    }

    const char* ray_source_env = std::getenv("MUJOCO_RAY_SOURCE");
    if (ray_source_env != nullptr &&
        (std::strcmp(ray_source_env, "ray_pred") == 0 ||
         std::strcmp(ray_source_env, "external") == 0)) {
      geometric_ray_write_enabled_ = false;
    }
    ray_telemetry_enabled_ = std::getenv("MUJOCO_RAY_TELEMETRY") != nullptr &&
                             std::strcmp(std::getenv("MUJOCO_RAY_TELEMETRY"), "1") == 0;

    // Setup ray2d shared memory
    _setupRay2dShm();
    if (const char* requested_fault = std::getenv("MUJOCO_RAY_TEST_FAULT"); requested_fault != nullptr && requested_fault[0] != '\0') {
      if (std::getenv("MUJOCO_SIMULATION_TEST") == nullptr || std::strcmp(std::getenv("MUJOCO_SIMULATION_TEST"), "1") != 0) {
        std::cerr << "[ABS-LIVE-FAULT] event=blocked id=" << requested_fault
                  << " reason=MUJOCO_SIMULATION_TEST_not_enabled" << std::endl;
        std::exit(EXIT_FAILURE);
      }
      ray_test_fault_ = requested_fault;
      if (const char* delay = std::getenv("MUJOCO_RAY_TEST_DELAY_MS"); delay != nullptr) {
        ray_test_delay_ms_ = std::max(0, std::atoi(delay));
      }
      ray_test_start_ns_ = monotonicNowNs();
      std::cout << "[ABS-LIVE-FAULT] event=armed id=" << ray_test_fault_
                << " clock=steady_clock_ns delay_ms=" << ray_test_delay_ms_ << std::endl;
    }
    _setupQposShm();
    _setupCollisionShm();
    std::cout << "[Ray2D] Source: "
              << (geometric_ray_write_enabled_ ? "geometric" : "external/ray_pred")
              << std::endl;
  }

  virtual ~UnitreeSDK2BridgeBase() {
    if (ray2d_shm_ptr_ != MAP_FAILED && ray2d_shm_ptr_ != nullptr) {
      munmap(ray2d_shm_ptr_, RAY2D_COUNT * sizeof(float));
    }
    if (ray2d_shm_fd_ >= 0) {
      close(ray2d_shm_fd_);
    }
    if (ray2d_stamp_shm_ptr_ != MAP_FAILED && ray2d_stamp_shm_ptr_ != nullptr) munmap(ray2d_stamp_shm_ptr_, sizeof(abs_ray2d_shm::FrameHeader));
    if (ray2d_stamp_shm_fd_ >= 0) close(ray2d_stamp_shm_fd_);
    if (qpos_shm_ptr_ != MAP_FAILED && qpos_shm_ptr_ != nullptr) {
      munmap(qpos_shm_ptr_, QPOS_COUNT * sizeof(double));
    }
    if (qpos_shm_fd_ >= 0) {
      close(qpos_shm_fd_);
    }
    if (collision_shm_ptr_ != MAP_FAILED && collision_shm_ptr_ != nullptr) {
      munmap(collision_shm_ptr_, COLLISION_COUNT * sizeof(int32_t));
    }
    if (collision_shm_fd_ >= 0) {
      close(collision_shm_fd_);
    }
  }

  virtual void requestStop() {}

  void computeRay2d() {
    updateCollisionTelemetry();

    // Write full qpos to shared memory (for Python depth renderer sync)
    if (qpos_shm_ptr_ != nullptr && qpos_shm_ptr_ != MAP_FAILED) {
      int nq = mj_model_->nq;
      for (int i = 0; i < QPOS_COUNT && i < nq; i++) {
        qpos_shm_ptr_[i] = mj_data_->qpos[i];
      }
    }

    if (!geometric_ray_write_enabled_) return;
    if (ray2d_shm_ptr_ == nullptr || ray2d_shm_ptr_ == MAP_FAILED || ray2d_stamp_shm_ptr_ == nullptr || ray2d_stamp_shm_ptr_ == MAP_FAILED) return;

    if (!ray_test_fault_.empty() && !ray_test_active_ &&
        monotonicNowNs() - ray_test_start_ns_ >= static_cast<uint64_t>(ray_test_delay_ms_) * 1000000ULL) {
      ray_test_active_ = true;
      if (ray_test_fault_ == "exit") {
        std::_Exit(EXIT_SUCCESS);  // preserve the original immediate fault path
      }
    }
    if (ray_test_active_ && ray_test_fault_ == "freeze") return;

    int body_id = mj_name2id(mj_model_, mjOBJ_BODY, "base_link");
    if (body_id < 0) {
      body_id = mj_name2id(mj_model_, mjOBJ_BODY, "torso_link");
    }
    if (body_id < 0) return;

    // Get body world position (2D only: x,y) and yaw from rotation matrix
    double* xpos = &mj_data_->xpos[body_id * 3];
    double* xmat = &mj_data_->xmat[body_id * 9];
    // Extract yaw from rotation matrix (xmat[0]=cos(yaw), xmat[3]=sin(yaw))
    double body_yaw = atan2(xmat[3], xmat[0]);

    // 2D ray origin in world frame (matches training: body pos + local offset rotated)
    double ray_x0 = xpos[0] + RAY2D_X0 * cos(body_yaw) - RAY2D_Y0 * sin(body_yaw);
    double ray_y0 = xpos[1] + RAY2D_X0 * sin(body_yaw) + RAY2D_Y0 * cos(body_yaw);

    // Compute into private memory first.  The cross-process writer critical section
    // below is then only a bounded 11-float copy, not the whole geometry query.
    std::array<float, RAY2D_COUNT> ray_frame{};

    // 2D circle_ray_query for each ray and each static obstacle geom
    for (int i = 0; i < RAY2D_COUNT; i++) {
      double theta = RAY2D_THETA_START + i * RAY2D_THETA_STEP;
      // Ray direction in world frame (rotate body-frame angle by body yaw)
      double world_theta = theta + body_yaw;
      double ctheta = cos(world_theta);
      double stheta = sin(world_theta);

      float best_dist = RAY2D_MAX_DIST;

      // Iterate all geoms, find closest 2D intersection with static obstacle geoms
      for (int gid = 0; gid < mj_model_->ngeom; gid++) {
        // Skip robot geoms (group 2=visual, group 3=collision)
        int grp = mj_model_->geom_group[gid];
        if (grp == 2 || grp == 3) continue;
        // Skip floor plane
        const char* gname = mj_id2name(mj_model_, mjOBJ_GEOM, gid);
        if (gname && strcmp(gname, "floor") == 0) continue;
        // Skip non-static geoms (bodyid > 0 with joints = dynamic)
        int g_bodyid = mj_model_->geom_bodyid[gid];
        if (g_bodyid > 0 && mj_model_->body_mass[g_bodyid] > 0) continue;
        // Skip non-obstacle geom types: plane(0)=ground, hfield(1)=terrain, mesh(7)=visual
        int gtype = mj_model_->geom_type[gid];
        if (gtype == 0 || gtype == 1 || gtype == 7) continue;

        // Geom world center
        double* gpos = &mj_data_->geom_xpos[gid * 3];
        double gx = gpos[0];
        double gy = gpos[1];
        double* gsize = &mj_model_->geom_size[gid * 3];

        double raydist = std::numeric_limits<double>::infinity();

        if (gtype == mjGEOM_BOX) {
          // Exact 2D ray-box intersection in the geom local frame.
          // This keeps visually passable gaps open instead of replacing boxes with large circles.
          double* gmat = &mj_data_->geom_xmat[gid * 9];
          double dx = ray_x0 - gx;
          double dy = ray_y0 - gy;

          // world -> local: R^T * vector (z ignored for 2D ray footprint)
          double local_x = gmat[0] * dx + gmat[3] * dy;
          double local_y = gmat[1] * dx + gmat[4] * dy;
          double dir_x = gmat[0] * ctheta + gmat[3] * stheta;
          double dir_y = gmat[1] * ctheta + gmat[4] * stheta;

          double tmin = -std::numeric_limits<double>::infinity();
          double tmax = std::numeric_limits<double>::infinity();
          bool hit = true;

          auto updateSlab = [&](double origin, double dir, double half_size) {
            const double eps = 1e-9;
            if (std::abs(dir) < eps) {
              if (origin < -half_size || origin > half_size) hit = false;
              return;
            }
            double t1 = (-half_size - origin) / dir;
            double t2 = ( half_size - origin) / dir;
            if (t1 > t2) std::swap(t1, t2);
            tmin = std::max(tmin, t1);
            tmax = std::min(tmax, t2);
          };

          updateSlab(local_x, dir_x, gsize[0]);
          updateSlab(local_y, dir_y, gsize[1]);

          if (hit && tmax >= std::max(0.0, tmin)) {
            raydist = (tmin >= 0.0) ? tmin : tmax;
          }
        } else {
          // ABS training ray query is circle-based. For round geoms, use the 2D footprint
          // radius only. Do not include height (geom_size[1]/[2]) in the horizontal radius.
          double gradius = gsize[0];
          if (gtype == mjGEOM_ELLIPSOID) {
            gradius = std::max(gsize[0], gsize[1]);
          } else if (gtype != mjGEOM_CYLINDER && gtype != mjGEOM_SPHERE && gtype != mjGEOM_CAPSULE) {
            gradius = std::max(gsize[0], gsize[1]);
          }

          // 2D ray-circle intersection (matches training circle_ray_query)
          double d_c2line = std::abs(stheta * gx - ctheta * gy - stheta * ray_x0 + ctheta * ray_y0);
          if (d_c2line >= gradius) continue;  // ray misses circle

          double d_c0_sq = (gx - ray_x0)*(gx - ray_x0) + (gy - ray_y0)*(gy - ray_y0);
          double d_0p = std::sqrt(std::max(0.0, d_c0_sq - d_c2line * d_c2line));
          double semi_arc = std::sqrt(std::max(0.0, gradius * gradius - d_c2line * d_c2line));
          raydist = d_0p - semi_arc;

          // Check direction: geom must be in front of ray
          double check_dir = ctheta * (gx - ray_x0) + stheta * (gy - ray_y0);
          if (check_dir <= 0) continue;
        }

        if (!std::isfinite(raydist)) continue;

        // Clip and take minimum
        if (raydist < RAY2D_MIN_DIST) raydist = RAY2D_MIN_DIST;
        if (raydist < best_dist) {
          best_dist = static_cast<float>(raydist);
        }
      }

      ray_frame[i] = std::log2(best_dist);
    }
    if (ray_test_active_ && (ray_test_fault_ == "nan" || ray_test_fault_ == "inf")) {
      ray_frame[0] = ray_test_fault_ == "nan" ? std::numeric_limits<float>::quiet_NaN()
                                                : std::numeric_limits<float>::infinity();
    }

    // Publish protocol: sequence odd while the shared payload is changed; even only
    // after the timestamp corresponding to the completed payload is visible.
    uint64_t sequence = abs_ray2d_shm::loadAcquire(&ray2d_stamp_shm_ptr_->sequence);
    if (sequence & 1U) ++sequence;
    abs_ray2d_shm::storeRelease(&ray2d_stamp_shm_ptr_->sequence, sequence + 1U);
    std::memcpy(ray2d_shm_ptr_, ray_frame.data(), RAY2D_COUNT * sizeof(float));
    abs_ray2d_shm::storeRelease(&ray2d_stamp_shm_ptr_->magic, abs_ray2d_shm::kMagic);
    abs_ray2d_shm::storeRelease(&ray2d_stamp_shm_ptr_->version, abs_ray2d_shm::kVersion);
    const uint64_t stamp_ns = monotonicNowNs();
    abs_ray2d_shm::storeRelease(&ray2d_stamp_shm_ptr_->monotonic_ns, stamp_ns);
    abs_ray2d_shm::storeRelease(&ray2d_stamp_shm_ptr_->sequence, sequence + 2U);
    if (ray_telemetry_enabled_ && ++ray_write_count_ % 1000U == 0U) {
      const double average_period_ms = ray_last_telemetry_ns_ == 0 ? 0.0
          : static_cast<double>(stamp_ns - ray_last_telemetry_ns_) / 1000.0 / 1e6;
      ray_telemetry_log_pending_ = true;
      ray_telemetry_stamp_ns_ = stamp_ns;
      ray_telemetry_period_ms_ = average_period_ms;
      ray_last_telemetry_ns_ = stamp_ns;
    }
  }

  void emitRayDiagnostics() {
    if (ray_telemetry_log_pending_) {
      std::cout << "[ABS-LIVE-RAY] clock=steady_clock_ns stamp_ns="
                << ray_telemetry_stamp_ns_ << " frames=1000 average_period_ms="
                << ray_telemetry_period_ms_ << std::endl;
      ray_telemetry_log_pending_ = false;
    }

  }

private:
  static uint64_t monotonicNowNs() {
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
  }
  bool geometric_ray_write_enabled_ = true;
  bool ray_telemetry_enabled_ = false;
  uint64_t ray_write_count_ = 0;
  uint64_t ray_last_telemetry_ns_ = 0;
  std::string ray_test_fault_;
  int ray_test_delay_ms_ = 1000;
  uint64_t ray_test_start_ns_ = 0;
  bool ray_test_active_ = false;
  bool ray_telemetry_log_pending_ = false;
  uint64_t ray_telemetry_stamp_ns_ = 0;
  double ray_telemetry_period_ms_ = 0.0;

  int ray2d_shm_fd_ = -1;
  float* ray2d_shm_ptr_ = nullptr;
  int ray2d_stamp_shm_fd_ = -1;
  abs_ray2d_shm::FrameHeader* ray2d_stamp_shm_ptr_ = nullptr;

  int qpos_shm_fd_ = -1;
  double* qpos_shm_ptr_ = nullptr;

  int collision_shm_fd_ = -1;
  int32_t* collision_shm_ptr_ = nullptr;
  int32_t collision_event_total_ = 0;
  int32_t collision_last_robot_geom_ = -1;
  int32_t collision_last_obstacle_geom_ = -1;

  void _setupCollisionShm() {
    collision_shm_fd_ = shm_open(COLLISION_SHM_NAME, O_CREAT | O_RDWR, 0666);
    if (collision_shm_fd_ < 0) {
      std::cerr << "[CollisionShm] shm_open failed: " << strerror(errno) << std::endl;
      return;
    }
    if (ftruncate(collision_shm_fd_, COLLISION_COUNT * sizeof(int32_t)) < 0) {
      std::cerr << "[CollisionShm] ftruncate failed: " << strerror(errno) << std::endl;
      return;
    }
    collision_shm_ptr_ = static_cast<int32_t*>(
        mmap(NULL, COLLISION_COUNT * sizeof(int32_t), PROT_READ | PROT_WRITE,
             MAP_SHARED, collision_shm_fd_, 0));
    if (collision_shm_ptr_ == MAP_FAILED) {
      std::cerr << "[CollisionShm] mmap failed: " << strerror(errno) << std::endl;
      collision_shm_ptr_ = nullptr;
      return;
    }
    for (int i = 0; i < COLLISION_COUNT; i++) {
      collision_shm_ptr_[i] = 0;
    }
    std::cout << "[CollisionShm] Shared memory initialized: " << COLLISION_SHM_NAME
              << " (" << COLLISION_COUNT << " int32)" << std::endl;
  }

  bool isRobotCollisionGeom(int geom_id) const {
    if (geom_id < 0 || geom_id >= mj_model_->ngeom) return false;
    return mj_model_->geom_group[geom_id] == 3;
  }

  bool isObstacleCollisionGeom(int geom_id) const {
    if (geom_id < 0 || geom_id >= mj_model_->ngeom) return false;
    if (mj_model_->geom_group[geom_id] == 2 || mj_model_->geom_group[geom_id] == 3) return false;

    const char* name = mj_id2name(mj_model_, mjOBJ_GEOM, geom_id);
    if (name && strcmp(name, "floor") == 0) return false;

    int type = mj_model_->geom_type[geom_id];
    if (type == mjGEOM_PLANE || type == mjGEOM_HFIELD || type == mjGEOM_MESH) return false;
    if (type != mjGEOM_BOX && type != mjGEOM_CYLINDER && type != mjGEOM_SPHERE &&
        type != mjGEOM_CAPSULE && type != mjGEOM_ELLIPSOID) {
      return false;
    }

    int body_id = mj_model_->geom_bodyid[geom_id];
    if (body_id > 0 && mj_model_->body_mass[body_id] > 0) return false;
    return true;
  }

  void updateCollisionTelemetry() {
    if (collision_shm_ptr_ == nullptr || collision_shm_ptr_ == MAP_FAILED) return;

    int collision_count = 0;
    int last_robot_geom = -1;
    int last_obstacle_geom = -1;
    for (int i = 0; i < mj_data_->ncon; i++) {
      const mjContact& contact = mj_data_->contact[i];
      int geom1 = contact.geom1;
      int geom2 = contact.geom2;
      bool robot1 = isRobotCollisionGeom(geom1);
      bool robot2 = isRobotCollisionGeom(geom2);
      bool obstacle1 = isObstacleCollisionGeom(geom1);
      bool obstacle2 = isObstacleCollisionGeom(geom2);
      if ((robot1 && obstacle2) || (robot2 && obstacle1)) {
        collision_count++;
        last_robot_geom = robot1 ? geom1 : geom2;
        last_obstacle_geom = obstacle1 ? geom1 : geom2;
      }
    }

    if (collision_count > 0) {
      collision_event_total_++;
      collision_last_robot_geom_ = last_robot_geom;
      collision_last_obstacle_geom_ = last_obstacle_geom;
    }
    collision_shm_ptr_[0] = collision_count > 0 ? 1 : 0;
    collision_shm_ptr_[1] = collision_count;
    collision_shm_ptr_[2] = collision_event_total_;
    collision_shm_ptr_[3] = collision_last_robot_geom_;
    collision_shm_ptr_[4] = collision_last_obstacle_geom_;
  }

  void _setupQposShm() {
    qpos_shm_fd_ = shm_open(QPOS_SHM_NAME, O_CREAT | O_RDWR, 0666);
    if (qpos_shm_fd_ < 0) {
      std::cerr << "[QposShm] shm_open failed: " << strerror(errno) << std::endl;
      return;
    }
    if (ftruncate(qpos_shm_fd_, QPOS_COUNT * sizeof(double)) < 0) {
      std::cerr << "[QposShm] ftruncate failed: " << strerror(errno) << std::endl;
      return;
    }
    qpos_shm_ptr_ = static_cast<double*>(
        mmap(NULL, QPOS_COUNT * sizeof(double), PROT_READ | PROT_WRITE,
             MAP_SHARED, qpos_shm_fd_, 0));
    if (qpos_shm_ptr_ == MAP_FAILED) {
      std::cerr << "[QposShm] mmap failed: " << strerror(errno) << std::endl;
      qpos_shm_ptr_ = nullptr;
      return;
    }
    std::cout << "[QposShm] Shared memory initialized: " << QPOS_SHM_NAME
              << " (" << QPOS_COUNT << " doubles)" << std::endl;
  }

  void _setupRay2dShm() {
    ray2d_shm_fd_ = shm_open(RAY2D_SHM_NAME, O_CREAT | O_RDWR, 0666);
    if (ray2d_shm_fd_ < 0) {
      std::cerr << "[Ray2D] shm_open failed: " << strerror(errno) << std::endl;
      return;
    }
    if (ftruncate(ray2d_shm_fd_, RAY2D_COUNT * sizeof(float)) < 0) {
      std::cerr << "[Ray2D] ftruncate failed: " << strerror(errno) << std::endl;
      return;
    }
    ray2d_shm_ptr_ = static_cast<float*>(
        mmap(NULL, RAY2D_COUNT * sizeof(float), PROT_READ | PROT_WRITE,
             MAP_SHARED, ray2d_shm_fd_, 0));
    if (ray2d_shm_ptr_ == MAP_FAILED) {
      std::cerr << "[Ray2D] mmap failed: " << strerror(errno) << std::endl;
      ray2d_shm_ptr_ = nullptr;
      return;
    }
    // Initialize with max range (log2(6.0) ~ 2.585)
    for (int i = 0; i < RAY2D_COUNT; i++) {
      ray2d_shm_ptr_[i] = std::log2(RAY2D_MAX_DIST);
    }
    std::cout << "[Ray2D] Shared memory initialized: " << RAY2D_SHM_NAME
              << " (" << RAY2D_COUNT << " floats)" << std::endl;
    ray2d_stamp_shm_fd_ = shm_open(RAY2D_STAMP_SHM_NAME, O_CREAT | O_RDWR, 0666);
    if (ray2d_stamp_shm_fd_ < 0 || ftruncate(ray2d_stamp_shm_fd_, sizeof(abs_ray2d_shm::FrameHeader)) < 0) {
      std::cerr << "[Ray2D] stamp shm setup failed: " << strerror(errno) << std::endl;
      return;
    }
    ray2d_stamp_shm_ptr_ = static_cast<abs_ray2d_shm::FrameHeader*>(mmap(NULL, sizeof(abs_ray2d_shm::FrameHeader), PROT_READ | PROT_WRITE, MAP_SHARED, ray2d_stamp_shm_fd_, 0));
    if (ray2d_stamp_shm_ptr_ == MAP_FAILED) { ray2d_stamp_shm_ptr_ = nullptr; return; }
    abs_ray2d_shm::storeRelease(&ray2d_stamp_shm_ptr_->magic, 0);  // invalid until first complete frame
    abs_ray2d_shm::storeRelease(&ray2d_stamp_shm_ptr_->version, abs_ray2d_shm::kVersion);
    abs_ray2d_shm::storeRelease(&ray2d_stamp_shm_ptr_->sequence, 0);
    abs_ray2d_shm::storeRelease(&ray2d_stamp_shm_ptr_->monotonic_ns, 0);
  }

  void printSceneInformation() {
    struct SceneEntry {
      int index;
      std::string name;
      int dimension = -1;
    };
    struct SceneInfoSnapshot {
      std::vector<SceneEntry> links;
      std::vector<SceneEntry> joints;
      std::vector<SceneEntry> actuators;
      std::vector<SceneEntry> sensors;
    } snapshot;

    {
      std::lock_guard<std::recursive_mutex> md_lock(*md_mutex_);
      auto collectObjects = [this](std::vector<SceneEntry>& entries, int count,
                                   int type, bool with_dimensions) {
        int sensor_index = 0;
        for (int i = 0; i < count; ++i) {
          const char* name = mj_id2name(mj_model_, type, i);
          if (name == nullptr) continue;
          SceneEntry entry{sensor_index, name};
          if (with_dimensions) {
            entry.dimension = mj_model_->sensor_dim[i];
            sensor_index += entry.dimension;
          } else {
            entry.index = i;
          }
          entries.push_back(std::move(entry));
        }
      };
      collectObjects(snapshot.links, mj_model_->nbody, mjOBJ_BODY, false);
      collectObjects(snapshot.joints, mj_model_->njnt, mjOBJ_JOINT, false);
      collectObjects(snapshot.actuators, mj_model_->nu, mjOBJ_ACTUATOR, false);
      collectObjects(snapshot.sensors, mj_model_->nsensor, mjOBJ_SENSOR, true);
    }

    auto printObjects = [](const char* title,
                           const std::vector<SceneEntry>& entries) {
      std::cout << "<<------------- " << title << " ------------->> "
                << std::endl;
      for (const auto& entry : entries) {
        std::cout << title << "_index: " << entry.index << ", name: "
                  << entry.name;
        if (entry.dimension >= 0) {
          std::cout << ", dim: " << entry.dimension;
        }
        std::cout << std::endl;
      }
      std::cout << std::endl;
    };

    printObjects("Link", snapshot.links);
    printObjects("Joint", snapshot.joints);
    printObjects("Actuator", snapshot.actuators);
    printObjects("Sensor", snapshot.sensors);
  }

protected:
  int num_motor_ = 0;
  int dim_motor_sensor_ = 0;

  mjData* mj_data_;
  mjModel* mj_model_;
  std::recursive_mutex* md_mutex_;

  int have_imu_ = false;
  int have_frame_sensor_ = false;
  bool have_touch_sensor_ = false;

  std::shared_ptr<unitree::common::UnitreeJoystick> joystick = nullptr;

  void _check_sensor() {
    num_motor_ = mj_model_->nu;
    dim_motor_sensor_ = MOTOR_SENSOR_NUM * num_motor_;

    for (int i = dim_motor_sensor_; i < mj_model_->nsensor; i++) {
      const char* name = mj_id2name(mj_model_, mjOBJ_SENSOR, i);
      if (strcmp(name, "imu_quat") == 0) {
        have_imu_ = true;
      }
      if (strcmp(name, "frame_pos") == 0) {
        have_frame_sensor_ = true;
      }
      if (strstr(name, "touch") != nullptr) {
        have_touch_sensor_ = true;
      }
    }
  }
};

template <typename LowCmd_t, typename LowState_t>
class RobotBridge : public UnitreeSDK2BridgeBase {
  using HighState_t = unitree::robot::go2::publisher::SportModeState;
  using WirelessController_t =
  unitree::robot::go2::publisher::WirelessController;

protected:
  // 触摸传感器处理的虚函数，子类可以重写
  virtual void readTouchSensors(std::array<double, 4>&) {
    // 默认实现为空，适用于没有触摸传感器的机器人
  }

  virtual void writeTouchSensors(const std::array<double, 4>&) {}

public:
  RobotBridge(mjModel* model, mjData* data, std::recursive_mutex* md_mutex)
      : UnitreeSDK2BridgeBase(model, data, md_mutex) {
    lowcmd = std::make_shared<LowCmd_t>("rt/lowcmd");
    lowstate = std::make_unique<LowState_t>();
    lowstate->joystick = joystick;
    highstate = std::make_unique<HighState_t>();
    wireless_controller = std::make_unique<WirelessController_t>();
    wireless_controller->joystick = joystick;
    thread_ = std::make_unique<JoinableThread>([this] {
      while (!thread_->stopRequested()) {
        run();
        thread_->waitForStop(std::chrono::milliseconds(1));
      }
    });
    thread_->start();
  }

  void requestStop() override {
    if (thread_) thread_->stopAndJoin();
  }

  void run() {
    if (lowstate->joystick) { lowstate->joystick->update(); }

    std::vector<double> command_q(num_motor_);
    std::vector<double> command_dq(num_motor_);
    std::vector<double> command_kp(num_motor_);
    std::vector<double> command_kd(num_motor_);
    std::vector<double> command_tau(num_motor_);
    {
      std::lock_guard<std::mutex> lock(lowcmd->mutex_);
      for (int i = 0; i < num_motor_; ++i) {
        auto& command = lowcmd->msg_.motor_cmd()[i];
        command_q[i] = command.q();
        command_dq[i] = command.dq();
        command_kp[i] = command.kp();
        command_kd[i] = command.kd();
        command_tau[i] = command.tau();
      }
    }

    std::vector<double> state_q(num_motor_);
    std::vector<double> state_dq(num_motor_);
    std::vector<double> state_tau(num_motor_);
    std::array<double, 4> quaternion{};
    std::array<double, 3> gyroscope{};
    std::array<double, 3> accelerometer{};
    std::array<double, 3> position{};
    std::array<double, 3> velocity{};
    std::array<double, 4> foot_force{};
    {
      std::lock_guard<std::recursive_mutex> md_lock(*md_mutex_);
      if (!mj_data_) return;
      for (int i = 0; i < num_motor_; ++i) {
        mj_data_->ctrl[i] = command_tau[i] +
            command_kp[i] * (command_q[i] - mj_data_->sensordata[i]) +
            command_kd[i] * (command_dq[i] -
                             mj_data_->sensordata[i + num_motor_]);
        state_q[i] = mj_data_->sensordata[i];
        state_dq[i] = mj_data_->sensordata[i + num_motor_];
        state_tau[i] = mj_data_->sensordata[i + 2 * num_motor_];
      }
      if (have_frame_sensor_) {
        for (int i = 0; i < 4; ++i) {
          quaternion[i] = mj_data_->sensordata[dim_motor_sensor_ + i];
        }
        for (int i = 0; i < 3; ++i) {
          gyroscope[i] = mj_data_->sensordata[dim_motor_sensor_ + 4 + i];
          accelerometer[i] = mj_data_->sensordata[dim_motor_sensor_ + 7 + i];
          position[i] = mj_data_->sensordata[dim_motor_sensor_ + 10 + i];
          velocity[i] = mj_data_->sensordata[dim_motor_sensor_ + 13 + i];
        }
      }
      if (have_touch_sensor_) readTouchSensors(foot_force);
      computeRay2d();
    }

    emitRayDiagnostics();

    if (lowstate->trylock()) {
      for (int i = 0; i < num_motor_; ++i) {
        lowstate->msg_.motor_state()[i].q() = state_q[i];
        lowstate->msg_.motor_state()[i].dq() = state_dq[i];
        lowstate->msg_.motor_state()[i].tau_est() = state_tau[i];
      }
      if (have_frame_sensor_) {
        for (int i = 0; i < 4; ++i) {
          lowstate->msg_.imu_state().quaternion()[i] = quaternion[i];
        }
        double w = quaternion[0], x = quaternion[1];
        double y = quaternion[2], z = quaternion[3];
        lowstate->msg_.imu_state().rpy()[0] = atan2(
            2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
        lowstate->msg_.imu_state().rpy()[1] = asin(2 * (w * y - z * x));
        lowstate->msg_.imu_state().rpy()[2] = atan2(
            2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
        for (int i = 0; i < 3; ++i) {
          lowstate->msg_.imu_state().gyroscope()[i] = gyroscope[i];
          lowstate->msg_.imu_state().accelerometer()[i] = accelerometer[i];
        }
      }
      if (have_touch_sensor_) {
        writeTouchSensors(foot_force);
      }
      lowstate->unlockAndPublish();
    }
    if (have_frame_sensor_ && highstate->trylock()) {
      for (int i = 0; i < 3; ++i) {
        highstate->msg_.position()[i] = position[i];
        highstate->msg_.velocity()[i] = velocity[i];
      }
      highstate->unlockAndPublish();
    }
    if (wireless_controller->joystick) {
      wireless_controller->unlockAndPublish();
    }
  }

  std::unique_ptr<HighState_t> highstate;
  std::unique_ptr<WirelessController_t> wireless_controller;
  std::shared_ptr<LowCmd_t> lowcmd;
  std::unique_ptr<LowState_t> lowstate;

private:
  std::unique_ptr<JoinableThread> thread_;
};

class Go2Bridge : public RobotBridge<
      unitree::robot::go2::subscription::LowCmd,
      unitree::robot::go2::publisher::LowState> {
public:
  Go2Bridge(mjModel* model, mjData* data, std::recursive_mutex* md_mutex)
    : RobotBridge(model, data, md_mutex) {}

protected:
  void readTouchSensors(std::array<double, 4>& foot_force) override {
    if (have_touch_sensor_) {
      for (int i = 0; i < 4; ++i) {
        foot_force[i] = mj_data_->sensordata[dim_motor_sensor_ + 16 + i];
      }
    }
  }

  void writeTouchSensors(const std::array<double, 4>& foot_force) override {
    if (have_touch_sensor_) {
      for (int i = 0; i < 4; ++i) lowstate->msg_.foot_force()[i] = foot_force[i];
    }
  }
};

using G1Bridge = RobotBridge<unitree::robot::g1::subscription::LowCmd,
                             unitree::robot::g1::publisher::LowState>;
