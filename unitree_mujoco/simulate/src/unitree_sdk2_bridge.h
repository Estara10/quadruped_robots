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
#include <cmath>
#include <cstring>

#include "param.h"
#include "physics_joystick.h"

#define MOTOR_SENSOR_NUM 3

// ===== Ray2d shared memory constants =====
#define RAY2D_SHM_NAME "/mujoco_ray2d"
#define QPOS_SHM_NAME "/mujoco_qpos"
#define QPOS_COUNT 19
#define RAY2D_COUNT 11
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
  UnitreeSDK2BridgeBase(mjModel* model, mjData* data)
    : mj_model_(model), mj_data_(data) {
    _check_sensor();
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

    // Setup ray2d shared memory
    _setupRay2dShm();
    _setupQposShm();
  }

  ~UnitreeSDK2BridgeBase() {
    if (ray2d_shm_ptr_ != MAP_FAILED && ray2d_shm_ptr_ != nullptr) {
      munmap(ray2d_shm_ptr_, RAY2D_COUNT * sizeof(float));
    }
    if (ray2d_shm_fd_ >= 0) {
      close(ray2d_shm_fd_);
    }
    if (qpos_shm_ptr_ != MAP_FAILED && qpos_shm_ptr_ != nullptr) {
      munmap(qpos_shm_ptr_, QPOS_COUNT * sizeof(double));
    }
    if (qpos_shm_fd_ >= 0) {
      close(qpos_shm_fd_);
    }
  }

  void computeRay2d() {
    // Write full qpos to shared memory (for Python depth renderer sync)
    if (qpos_shm_ptr_ != nullptr && qpos_shm_ptr_ != MAP_FAILED) {
      int nq = mj_model_->nq;
      for (int i = 0; i < QPOS_COUNT && i < nq; i++) {
        qpos_shm_ptr_[i] = mj_data_->qpos[i];
      }
    }

    if (ray2d_shm_ptr_ == nullptr || ray2d_shm_ptr_ == MAP_FAILED) return;

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

        // Approximate radius from geom size (max of half-widths)
        double* gsize = &mj_model_->geom_size[gid * 3];
        double gradius = std::max(std::max(gsize[0], gsize[1]), gsize[2]);

        // 2D ray-circle intersection (matches training circle_ray_query)
        double d_c2line = std::abs(stheta * gx - ctheta * gy - stheta * ray_x0 + ctheta * ray_y0);
        if (d_c2line >= gradius) continue;  // ray misses circle

        double d_c0_sq = (gx - ray_x0)*(gx - ray_x0) + (gy - ray_y0)*(gy - ray_y0);
        double d_0p = std::sqrt(std::max(0.0, d_c0_sq - d_c2line * d_c2line));
        double semi_arc = std::sqrt(std::max(0.0, gradius * gradius - d_c2line * d_c2line));
        double raydist = d_0p - semi_arc;

        // Check direction: geom must be in front of ray
        double check_dir = ctheta * (gx - ray_x0) + stheta * (gy - ray_y0);
        if (check_dir <= 0) continue;

        // Clip and take minimum
        if (raydist < RAY2D_MIN_DIST) raydist = RAY2D_MIN_DIST;
        if (raydist < best_dist) {
          best_dist = static_cast<float>(raydist);
        }
      }

      ray2d_shm_ptr_[i] = std::log2(best_dist);
    }
  }

private:
  int ray2d_shm_fd_ = -1;
  float* ray2d_shm_ptr_ = nullptr;

  int qpos_shm_fd_ = -1;
  double* qpos_shm_ptr_ = nullptr;

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
  }

  void printSceneInformation() {
    auto printObjects = [this](const char* title, int count, int type,
                               auto getIndex) {
      std::cout << "<<------------- " << title << " ------------->> " <<
          std::endl;
      for (int i = 0; i < count; i++) {
        const char* name = mj_id2name(mj_model_, type, i);
        if (name) {
          std::cout << title << "_index: " << getIndex(i) << ", " << "name: " <<
              name;
          if (type == mjOBJ_SENSOR) {
            std::cout << ", dim: " << mj_model_->sensor_dim[i];
          }
          std::cout << std::endl;
        }
      }
      std::cout << std::endl;
    };

    printObjects("Link", mj_model_->nbody, mjOBJ_BODY, [](int i) { return i; });
    printObjects("Joint", mj_model_->njnt, mjOBJ_JOINT,
                 [](int i) { return i; });
    printObjects("Actuator", mj_model_->nu, mjOBJ_ACTUATOR,
                 [](int i) { return i; });

    int sensorIndex = 0;
    printObjects("Sensor", mj_model_->nsensor, mjOBJ_SENSOR, [&](int i) {
      int currentIndex = sensorIndex;
      sensorIndex += mj_model_->sensor_dim[i];
      return currentIndex;
    });
  }

protected:
  int num_motor_ = 0;
  int dim_motor_sensor_ = 0;

  mjData* mj_data_;
  mjModel* mj_model_;

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
  virtual void processTouchSensors() {
    // 默认实现为空，适用于没有触摸传感器的机器人
  }

public:
  RobotBridge(mjModel* model, mjData* data) : UnitreeSDK2BridgeBase(
      model, data) {
    lowcmd = std::make_shared<LowCmd_t>("rt/lowcmd");
    lowstate = std::make_unique<LowState_t>();
    lowstate->joystick = joystick;
    highstate = std::make_unique<HighState_t>();
    wireless_controller = std::make_unique<WirelessController_t>();
    wireless_controller->joystick = joystick;
    thread_ = std::make_shared<unitree::common::RecurrentThread>(
        "unitree_bridge", UT_CPU_ID_NONE, 1000,
        std::bind(&RobotBridge::run, this));
  }

  void run() {
    if (!mj_data_)
      return;
    if (lowstate->joystick) { lowstate->joystick->update(); }
    // lowcmd
    {
      std::lock_guard<std::mutex> lock(lowcmd->mutex_);
      for (int i(0); i < num_motor_; i++) {
        auto& m = lowcmd->msg_.motor_cmd()[i];
        mj_data_->ctrl[i] = m.tau() +
                            m.kp() * (m.q() - mj_data_->sensordata[i]) +
                            m.kd() * (
                              m.dq() - mj_data_->sensordata[i + num_motor_]);
      }
    }

    // lowstate
    if (lowstate->trylock()) {
      for (int i(0); i < num_motor_; i++) {
        lowstate->msg_.motor_state()[i].q() = mj_data_->sensordata[i];
        lowstate->msg_.motor_state()[i].dq() = mj_data_->sensordata[
          i + num_motor_];
        lowstate->msg_.motor_state()[i].tau_est() = mj_data_->sensordata[
          i + 2 * num_motor_];
      }
      if (have_frame_sensor_) {
        lowstate->msg_.imu_state().quaternion()[0] = mj_data_->sensordata[
          dim_motor_sensor_ + 0];
        lowstate->msg_.imu_state().quaternion()[1] = mj_data_->sensordata[
          dim_motor_sensor_ + 1];
        lowstate->msg_.imu_state().quaternion()[2] = mj_data_->sensordata[
          dim_motor_sensor_ + 2];
        lowstate->msg_.imu_state().quaternion()[3] = mj_data_->sensordata[
          dim_motor_sensor_ + 3];

        double w = lowstate->msg_.imu_state().quaternion()[0];
        double x = lowstate->msg_.imu_state().quaternion()[1];
        double y = lowstate->msg_.imu_state().quaternion()[2];
        double z = lowstate->msg_.imu_state().quaternion()[3];

        lowstate->msg_.imu_state().rpy()[0] = atan2(
            2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
        lowstate->msg_.imu_state().rpy()[1] = asin(2 * (w * y - z * x));
        lowstate->msg_.imu_state().rpy()[2] = atan2(
            2 * (w * z + x * y), 1 - 2 * (y * y + z * z));

        lowstate->msg_.imu_state().gyroscope()[0] = mj_data_->sensordata[
          dim_motor_sensor_ + 4];
        lowstate->msg_.imu_state().gyroscope()[1] = mj_data_->sensordata[
          dim_motor_sensor_ + 5];
        lowstate->msg_.imu_state().gyroscope()[2] = mj_data_->sensordata[
          dim_motor_sensor_ + 6];

        lowstate->msg_.imu_state().accelerometer()[0] = mj_data_->sensordata[
          dim_motor_sensor_ + 7];
        lowstate->msg_.imu_state().accelerometer()[1] = mj_data_->sensordata[
          dim_motor_sensor_ + 8];
        lowstate->msg_.imu_state().accelerometer()[2] = mj_data_->sensordata[
          dim_motor_sensor_ + 9];
      }

      if (have_touch_sensor_) {
        processTouchSensors();
      }

      lowstate->unlockAndPublish();
    }
    // highstate
    if (have_frame_sensor_ && highstate->trylock()) {
      highstate->msg_.position()[0] = mj_data_->sensordata[
        dim_motor_sensor_ + 10];
      highstate->msg_.position()[1] = mj_data_->sensordata[
        dim_motor_sensor_ + 11];
      highstate->msg_.position()[2] = mj_data_->sensordata[
        dim_motor_sensor_ + 12];
      highstate->msg_.velocity()[0] = mj_data_->sensordata[
        dim_motor_sensor_ + 13];
      highstate->msg_.velocity()[1] = mj_data_->sensordata[
        dim_motor_sensor_ + 14];
      highstate->msg_.velocity()[2] = mj_data_->sensordata[
        dim_motor_sensor_ + 15];
      highstate->unlockAndPublish();
    }
    // wireless_controller
    if (wireless_controller->joystick) {
      wireless_controller->unlockAndPublish();
    }

    // compute ray2d and write to shared memory
    computeRay2d();
  }

  std::unique_ptr<HighState_t> highstate;
  std::unique_ptr<WirelessController_t> wireless_controller;
  std::shared_ptr<LowCmd_t> lowcmd;
  std::unique_ptr<LowState_t> lowstate;

private:
  unitree::common::RecurrentThreadPtr thread_;
};

class Go2Bridge : public RobotBridge<
      unitree::robot::go2::subscription::LowCmd,
      unitree::robot::go2::publisher::LowState> {
public:
  Go2Bridge(mjModel* model, mjData* data)
    : RobotBridge(model, data) {}

protected:
  void processTouchSensors() override {
    if (have_touch_sensor_) {
      lowstate->msg_.foot_force()[0] = mj_data_->sensordata[
        dim_motor_sensor_ + 16];
      lowstate->msg_.foot_force()[1] = mj_data_->sensordata[
        dim_motor_sensor_ + 17];
      lowstate->msg_.foot_force()[2] = mj_data_->sensordata[
        dim_motor_sensor_ + 18];
      lowstate->msg_.foot_force()[3] = mj_data_->sensordata[
        dim_motor_sensor_ + 19];
    }
  }
};

using G1Bridge = RobotBridge<unitree::robot::g1::subscription::LowCmd,
                             unitree::robot::g1::publisher::LowState>;