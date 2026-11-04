#include "rl_quadruped_controller/control/Estimator.h"
#include "rl_quadruped_controller/control/CtrlComponent.h"
#include "rl_quadruped_controller/robot/QuadrupedRobot.h"
#include <controller_common/CtrlInterfaces.h>
#include <controller_common/common/mathTools.h>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/loaned_state_interface.hpp>

#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <deque>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace {

using Clock = std::chrono::steady_clock;
using LowState = unitree_go::msg::dds_::LowState_;
constexpr int kEstimatorRateHz = 200;  // rl_quadruped_controller.update_rate in robot_control.yaml
constexpr double kStaleLimitSeconds = 0.100;
constexpr double kFeetForceThreshold = 1.0;  // Estimator's current default; do not tune from prone samples.
constexpr std::array<const char*, 12> kJoints{
    "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint"};
constexpr std::array<const char*, 4> kLegs{"FR", "FL", "RR", "RL"};
constexpr std::array<const char*, 10> kImuNames{
    "orientation.w", "orientation.x", "orientation.y", "orientation.z",
    "angular_velocity.x", "angular_velocity.y", "angular_velocity.z",
    "linear_acceleration.x", "linear_acceleration.y", "linear_acceleration.z"};

struct Sample {
  bool received = false;
  uint64_t sequence = 0;
  Clock::time_point first{};
  Clock::time_point last{};
  double last_gap_s = 0.0;
  uint32_t tick = 0;
  std::array<double, 12> q{};
  std::array<double, 12> dq{};
  std::array<double, 10> imu{};
  std::array<double, 4> force{};
};

std::mutex g_sample_mutex;
Sample g_sample;
volatile std::sig_atomic_t g_stop = 0;

void onSignal(int) { g_stop = 1; }

void onLowState(const void* message) {
  const auto& state = *static_cast<const LowState*>(message);
  const auto now = Clock::now();
  std::lock_guard<std::mutex> lock(g_sample_mutex);
  if (!g_sample.received) g_sample.first = now;
  else g_sample.last_gap_s = std::chrono::duration<double>(now - g_sample.last).count();
  g_sample.received = true;
  ++g_sample.sequence;
  g_sample.last = now;
  g_sample.tick = state.tick();
  // HardwareUnitree currently uses identity motor_index_map_: DDS 0..11 -> FR, FL, RR, RL.
  for (size_t i = 0; i < g_sample.q.size(); ++i) {
    g_sample.q[i] = state.motor_state()[i].q();
    g_sample.dq[i] = state.motor_state()[i].dq();
  }
  for (size_t i = 0; i < 4; ++i) g_sample.imu[i] = state.imu_state().quaternion()[i];
  for (size_t i = 0; i < 3; ++i) {
    g_sample.imu[4 + i] = state.imu_state().gyroscope()[i];
    g_sample.imu[7 + i] = state.imu_state().accelerometer()[i];
    g_sample.force[i] = state.foot_force()[i];
  }
  g_sample.force[3] = state.foot_force()[3];
}

struct InterfaceStorage {
  std::array<double, 12> q{};
  std::array<double, 12> dq{};
  std::array<double, 10> imu{};
  std::array<double, 4> force{};
  std::deque<hardware_interface::StateInterface> state_handles;
  std::deque<hardware_interface::LoanedStateInterface> loaned_handles;
  CtrlInterfaces interfaces;

  void add(const std::string& prefix, const std::string& name, double* value,
           std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>& target) {
    state_handles.emplace_back(prefix, name, value);
    loaned_handles.emplace_back(state_handles.back());
    target.emplace_back(loaned_handles.back());
  }

  InterfaceStorage() {
    interfaces.frequency_ = kEstimatorRateHz;
    for (size_t i = 0; i < kJoints.size(); ++i) {
      add(kJoints[i], "position", &q[i], interfaces.joint_position_state_interface_);
      add(kJoints[i], "velocity", &dq[i], interfaces.joint_velocity_state_interface_);
    }
    for (size_t i = 0; i < kImuNames.size(); ++i)
      add("imu_sensor", kImuNames[i], &imu[i], interfaces.imu_state_interface_);
    for (size_t i = 0; i < kLegs.size(); ++i)
      add(std::string("foot_force/") + kLegs[i], "force", &force[i],
          interfaces.foot_force_state_interface_);
  }

  void load(const Sample& sample) {
    q = sample.q;
    dq = sample.dq;
    imu = sample.imu;
    force = sample.force;
  }
};

std::string readFile(const std::string& path) {
  std::ifstream input(path);
  if (!input) throw std::runtime_error("cannot open URDF: " + path);
  std::ostringstream content;
  content << input.rdbuf();
  return content.str();
}

std::shared_ptr<QuadrupedRobot> makeRobot(InterfaceStorage& storage, CtrlComponent& component,
                                         const std::string& urdf_path) {
  const auto description = readFile(urdf_path);
  auto robot = std::make_shared<QuadrupedRobot>(storage.interfaces, description,
      std::vector<std::string>{"FR_foot", "FL_foot", "RR_foot", "RL_foot"}, "base");
  if (!(robot->mass_ > 0.0)) throw std::runtime_error("URDF KDL model has zero total mass");
  component.robot_model_ = robot;
  return robot;
}

bool finite(const Sample& sample) {
  for (double x : sample.q) if (!std::isfinite(x)) return false;
  for (double x : sample.dq) if (!std::isfinite(x)) return false;
  for (double x : sample.imu) if (!std::isfinite(x)) return false;
  for (double x : sample.force) if (!std::isfinite(x)) return false;
  double quaternion_norm_squared = 0.0;
  for (size_t i = 0; i < 4; ++i) quaternion_norm_squared += sample.imu[i] * sample.imu[i];
  return quaternion_norm_squared > std::numeric_limits<double>::epsilon();
}

bool finite(const Vec3& value) {
  return value.allFinite();
}

void verifyRecoveryFrameTransform() {
  const double half_angle = std::sqrt(0.5);
  Quat q_wxyz;
  q_wxyz << half_angle, 0.0, 0.0, half_angle;
  Vec3 world_x;
  world_x << 1.0, 0.0, 0.0;
  const Vec3 body_from_estimator = quatToRotMat(q_wxyz).transpose() * world_x;

  // StateRL::quatRotateInverse uses IsaacGym [x,y,z,w] and a -q cross-product term.
  Vec3 q_vector;
  q_vector << 0.0, 0.0, half_angle;
  const double q_w = half_angle;
  const Vec3 b = 2.0 * q_w * q_vector.cross(world_x);
  const Vec3 c = 2.0 * q_vector * q_vector.dot(world_x);
  const Vec3 body_from_recovery = (2.0 * q_w * q_w - 1.0) * world_x - b + c;
  Vec3 expected;
  expected << 0.0, -1.0, 0.0;
  if ((body_from_estimator - expected).norm() > 1e-12 ||
      (body_from_estimator - body_from_recovery).norm() > 1e-12)
    throw std::runtime_error("Estimator-to-Recovery yaw/body transform self-check failed");
}

void writeHeader(std::ostream& out) {
  out << "update_time_s,lowstate_receive_time_s,update_period_ms,lowstate_age_ms,lowstate_receive_gap_ms,lowstate_sequence,"
         "lowstate_tick,repeated_sample,inputs_finite,lowstate_fresh,estimate_finite,quaternion_norm";
  for (const char* joint : kJoints) out << ',' << joint << "_q_rad," << joint << "_dq_rad_s";
  for (const char* item : kImuNames) out << ',' << item;
  for (const char* leg : kLegs) out << ',' << leg << "_foot_force_raw," << leg << "_contact_force_ge_1";
  out << ",world_vx_m_s,world_vy_m_s,world_vz_m_s,body_vx_m_s,body_vy_m_s,body_vz_m_s\n";
}

void writeRow(std::ostream& out, const Sample& sample, double steady_s, double period_ms,
              double age_ms, bool repeated, bool inputs_finite, bool lowstate_fresh,
              bool estimate_finite, Estimator* estimator) {
  out << std::setprecision(12) << steady_s << ',' << (steady_s - age_ms / 1000.0) << ','
      << period_ms << ',' << age_ms << ','
      << sample.last_gap_s * 1000.0 << ',' << sample.sequence << ',' << sample.tick << ','
      << (repeated ? 1 : 0) << ',' << (inputs_finite ? 1 : 0) << ','
      << (lowstate_fresh ? 1 : 0) << ',' << (estimate_finite ? 1 : 0) << ',';
  double qnorm = 0.0;
  for (size_t i = 0; i < 4; ++i) qnorm += sample.imu[i] * sample.imu[i];
  out << std::sqrt(qnorm);
  for (size_t i = 0; i < kJoints.size(); ++i) out << ',' << sample.q[i] << ',' << sample.dq[i];
  for (double x : sample.imu) out << ',' << x;
  for (double x : sample.force) out << ',' << x << ',' << (x >= kFeetForceThreshold ? 1 : 0);
  if (estimate_finite && estimator != nullptr) {
    const Vec3 world = estimator->getVelocity();
    const Vec3 body = estimator->getRotation().transpose() * world;
    out << ',' << world.x() << ',' << world.y() << ',' << world.z()
        << ',' << body.x() << ',' << body.y() << ',' << body.z();
  } else {
    out << ",,,,,";
  }
  out << '\n';
}

bool parseOptions(int argc, char** argv, int& domain, std::string& interface_name,
                  std::string& urdf, std::string& csv, double& duration_s, bool& self_check) {
  bool have_domain = false, have_interface = false, have_urdf = false, have_csv = false;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--help") return false;
    if (arg == "--self-check") { self_check = true; continue; }
    if (i + 1 >= argc) throw std::invalid_argument("missing value for " + arg);
    const std::string value = argv[++i];
    if (arg == "--domain") { domain = std::stoi(value); have_domain = true; }
    else if (arg == "--interface") { interface_name = value; have_interface = true; }
    else if (arg == "--urdf") { urdf = value; have_urdf = true; }
    else if (arg == "--csv") { csv = value; have_csv = true; }
    else if (arg == "--duration-s") { duration_s = std::stod(value); }
    else throw std::invalid_argument("unknown option: " + arg);
  }
  if (!have_urdf) throw std::invalid_argument("--urdf is required");
  if (self_check) return true;
  if (!have_domain || !have_interface || !have_csv || !(duration_s > 0.0))
    throw std::invalid_argument("live mode requires --domain, --interface, --urdf, --csv and positive --duration-s");
  return true;
}

int selfCheck(const std::string& urdf_path) {
  verifyRecoveryFrameTransform();
  InterfaceStorage storage;
  CtrlComponent component;
  makeRobot(storage, component, urdf_path);
  Estimator estimator(storage.interfaces, component);  // preserves default feet-force threshold = 1
  Sample static_sample;
  static_sample.received = true;
  static_sample.q.fill(0.0);
  for (size_t leg = 0; leg < 4; ++leg) {
    static_sample.q[3 * leg + 1] = 0.8;
    static_sample.q[3 * leg + 2] = -1.5;
  }
  static_sample.imu = {1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 9.81};
  static_sample.force.fill(1.0);
  if (!finite(static_sample)) throw std::runtime_error("synthetic static input unexpectedly non-finite");
  storage.load(static_sample);
  for (int i = 0; i < 20; ++i) {
    component.robot_model_->update();
    estimator.update();
  }
  const Vec3 world = estimator.getVelocity();
  const Vec3 body = estimator.getRotation().transpose() * world;
  if (!finite(world) || !finite(body) || world.norm() > 5e-6 || body.norm() > 5e-6) {
    std::ostringstream message;
    message << "static plumbing self-check failed: world=[" << world.transpose()
            << "] body=[" << body.transpose() << "]";
    throw std::runtime_error(message.str());
  }
  std::cout << "SELF_CHECK_OK mass_kg=" << component.robot_model_->mass_
            << " dt_s=" << 1.0 / storage.interfaces.frequency_
            << " world_velocity_m_s=" << world.transpose()
            << " body_velocity_m_s=" << body.transpose()
            << " contacts=[1,1,1,1] threshold=1\n"
            << "Synthetic static-input plumbing only; not a Go2 accuracy or dynamic validation.\n";
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  int domain = 0;
  std::string interface_name, urdf_path, csv_path;
  double duration_s = 10.0;
  bool self_check = false;
  if (argc == 2 && std::string(argv[1]) == "--help") {
    std::cout << "Usage (live): " << argv[0]
              << " --domain N --interface IFACE --urdf FILE --csv FILE [--duration-s 10]\n"
              << "Usage (offline, no DDS): " << argv[0] << " --self-check --urdf FILE\n";
    return 0;
  }
  try {
    if (!parseOptions(argc, argv, domain, interface_name, urdf_path, csv_path, duration_s, self_check)) return 2;
    if (self_check) return selfCheck(urdf_path);

    std::ofstream csv(csv_path);
    if (!csv) throw std::runtime_error("cannot open CSV output: " + csv_path);
    writeHeader(csv);

    InterfaceStorage storage;
    CtrlComponent component;
    makeRobot(storage, component, urdf_path);
    Estimator estimator(storage.interfaces, component);  // existing estimator, default threshold 1
    auto subscriber = std::make_shared<unitree::robot::ChannelSubscriber<LowState>>("rt/lowstate");
    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);
    bool sdk_initialized = false;
    int exit_code = 0;
    try {
      unitree::robot::ChannelFactory::Instance()->Init(domain, interface_name);
      sdk_initialized = true;
      subscriber->InitChannel(onLowState, 1);
      std::cerr << "只订阅 rt/lowstate；不创建 LowCmd 发布器、MotionSwitcher 客户端、ROS controller 或策略模型。"
                << " estimator_rate=" << kEstimatorRateHz << "Hz dt="
                << (1.0 / kEstimatorRateHz) << "s threshold=" << kFeetForceThreshold << '\n';

      const auto start = Clock::now();
      auto next_tick = start;
      auto previous_tick = start;
      auto startup_deadline = start + std::chrono::seconds(3);
      uint64_t previous_sequence = 0;
      while (!g_stop && std::chrono::duration<double>(Clock::now() - start).count() < duration_s) {
        next_tick += std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(1.0 / kEstimatorRateHz));
        std::this_thread::sleep_until(next_tick);
        if (g_stop) break;
        Sample sample;
        {
          std::lock_guard<std::mutex> lock(g_sample_mutex);
          sample = g_sample;
        }
        const auto now = Clock::now();
        if (!sample.received) {
          if (now >= startup_deadline) throw std::runtime_error("no LowState received within 3s; no estimator update performed");
          continue;
        }
        const double age_s = std::chrono::duration<double>(now - sample.last).count();
        const double period_ms = std::chrono::duration<double, std::milli>(now - previous_tick).count();
        previous_tick = now;
        const bool inputs_finite = finite(sample);
        const bool lowstate_fresh = age_s <= kStaleLimitSeconds;
        const bool valid = inputs_finite && lowstate_fresh;
        const bool repeated = sample.sequence == previous_sequence;
        const double elapsed_s = std::chrono::duration<double>(now - start).count();
        bool estimate_finite = false;
        if (valid) {
          storage.load(sample);
          component.robot_model_->update();
          estimator.update();
          const Vec3 world = estimator.getVelocity();
          const Vec3 body = estimator.getRotation().transpose() * world;
          estimate_finite = finite(world) && finite(body);
        }
        writeRow(csv, sample, elapsed_s, period_ms, age_s * 1000.0, repeated,
                 inputs_finite, lowstate_fresh, estimate_finite,
                 estimate_finite ? &estimator : nullptr);
        previous_sequence = sample.sequence;
        if (!valid || !estimate_finite) {
          std::cerr << "LowState过期/非有限，或Estimator输出非有限；记录最后样本后停止；age_ms="
                    << age_s * 1000.0 << '\n';
          exit_code = 2;
          break;
        }
      }
    } catch (const std::exception& e) {
      std::cerr << "只读观察失败: " << e.what() << '\n';
      exit_code = 1;
    }
    subscriber->CloseChannel();
    if (sdk_initialized) unitree::robot::ChannelFactory::Instance()->Release();
    csv.flush();
    std::cerr << "Estimator观察已退出；CSV=" << csv_path << '\n';
    return exit_code;
  } catch (const std::exception& e) {
    std::cerr << "参数/初始化错误: " << e.what() << '\n';
    return 2;
  }
}
