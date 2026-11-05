#include "rl_quadruped_controller/control/Estimator.h"
#include "rl_quadruped_controller/control/CtrlComponent.h"
#include "rl_quadruped_controller/robot/QuadrupedRobot.h"
#include <controller_common/CtrlInterfaces.h>
#include <controller_common/common/mathTools.h>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/loaned_state_interface.hpp>

#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>
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
using SportModeState = unitree_go::msg::dds_::SportModeState_;
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
  std::array<double, 12> tau_est{};
  std::array<double, 10> imu{};
  std::array<double, 4> force{};
};

struct SportSample {
  bool received = false;
  uint64_t sequence = 0;
  Clock::time_point first{};
  Clock::time_point last{};
  double last_gap_s = 0.0;
  int32_t stamp_sec = 0;
  uint32_t stamp_nanosec = 0;
  uint8_t mode = 0;
  uint32_t error_code = 0;
  bool values_finite = true;
  std::array<double, 3> position{};
  std::array<double, 3> velocity{};
};

std::mutex g_sample_mutex;
Sample g_sample;
std::mutex g_sport_mutex;
SportSample g_sport;
std::deque<SportSample> g_sport_pending;
Clock::time_point g_session_start;
volatile std::sig_atomic_t g_stop = 0;

void onSignal(int) { g_stop = 1; }

void writeSportRecord(std::ostream& out, const SportSample& sample, double receive_time_s) {
  out << std::setprecision(12) << receive_time_s << ',';
  if (sample.sequence > 1) out << sample.last_gap_s * 1000.0;
  out << ',' << sample.sequence << ',' << sample.stamp_sec << ',' << sample.stamp_nanosec
      << ',' << static_cast<unsigned>(sample.mode) << ',' << sample.error_code
      << ',' << (sample.values_finite ? 1 : 0);
  for (double x : sample.position) out << ',' << x;
  for (double x : sample.velocity) out << ',' << x;
  out << '\n';
}

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
    g_sample.tau_est[i] = state.motor_state()[i].tau_est();
  }
  for (size_t i = 0; i < 4; ++i) g_sample.imu[i] = state.imu_state().quaternion()[i];
  for (size_t i = 0; i < 3; ++i) {
    g_sample.imu[4 + i] = state.imu_state().gyroscope()[i];
    g_sample.imu[7 + i] = state.imu_state().accelerometer()[i];
    g_sample.force[i] = state.foot_force()[i];
  }
  g_sample.force[3] = state.foot_force()[3];
}

void onSportModeState(const void* message) {
  const auto& state = *static_cast<const SportModeState*>(message);
  const auto now = Clock::now();
  std::lock_guard<std::mutex> lock(g_sport_mutex);
  if (!g_sport.received) g_sport.first = now;
  else g_sport.last_gap_s = std::chrono::duration<double>(now - g_sport.last).count();
  g_sport.received = true;
  ++g_sport.sequence;
  g_sport.last = now;
  g_sport.stamp_sec = state.stamp().sec();
  g_sport.stamp_nanosec = state.stamp().nanosec();
  g_sport.mode = state.mode();
  g_sport.error_code = state.error_code();
  g_sport.values_finite = true;
  for (size_t i = 0; i < 3; ++i) {
    g_sport.position[i] = state.position()[i];
    g_sport.velocity[i] = state.velocity()[i];
    g_sport.values_finite = g_sport.values_finite && std::isfinite(g_sport.position[i]) &&
                            std::isfinite(g_sport.velocity[i]);
  }
  g_sport_pending.push_back(g_sport);
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
  out << "update_time_s,update_period_ms,lowstate_received,lowstate_receive_time_s,lowstate_age_ms,lowstate_receive_gap_ms,lowstate_sequence,"
         "sport_received,sport_receive_time_s,sport_age_ms,sport_receive_gap_ms,sport_sequence,sport_stamp_sec,sport_stamp_nanosec,sport_mode,sport_error_code,"
         "sport_values_finite,"
         "lowstate_tick,repeated_sample,inputs_finite,lowstate_fresh,estimate_finite,quaternion_norm";
  for (const char* joint : kJoints) out << ',' << joint << "_q_rad," << joint << "_dq_rad_s," << joint << "_tau_est_raw";
  for (const char* item : kImuNames) out << ',' << item;
  for (const char* leg : kLegs) out << ',' << leg << "_foot_force_raw," << leg << "_contact_force_ge_1";
  out << ",sport_position_x_raw,sport_position_y_raw,sport_position_z_raw,sport_velocity_x_raw,sport_velocity_y_raw,sport_velocity_z_raw"
         ",estimator_world_vx_m_s,estimator_world_vy_m_s,estimator_world_vz_m_s,estimator_body_vx_m_s,estimator_body_vy_m_s,estimator_body_vz_m_s\n";
}

void writeSportHeader(std::ostream& out) {
  out << "sport_receive_time_s,sport_receive_gap_ms,sport_sequence,sport_stamp_sec,sport_stamp_nanosec,sport_mode,sport_error_code,sport_values_finite,"
         "position_x_raw,position_y_raw,position_z_raw,velocity_x_raw,velocity_y_raw,velocity_z_raw\n";
}

void writeRow(std::ostream& out, const Sample& sample, const SportSample& sport,
              Clock::time_point start, Clock::time_point update_time, double period_ms,
              double age_ms, bool repeated, bool inputs_finite, bool lowstate_fresh,
              bool estimate_finite, Estimator* estimator) {
  out << std::setprecision(12) << std::chrono::duration<double>(update_time - start).count()
      << ',' << period_ms << ',' << (sample.received ? 1 : 0) << ',';
  if (sample.received) out << std::chrono::duration<double>(sample.last - start).count();
  out << ',' << age_ms << ',';
  if (sample.received && sample.sequence > 1) out << sample.last_gap_s * 1000.0;
  out << ',';
  if (sample.received) out << sample.sequence;
  out << ',' << (sport.received ? 1 : 0) << ',';
  if (sport.received) out << std::chrono::duration<double>(sport.last - start).count();
  out << ',';
  if (sport.received) out << std::chrono::duration<double, std::milli>(update_time - sport.last).count();
  out << ',';
  if (sport.received && sport.sequence > 1) out << sport.last_gap_s * 1000.0;
  out << ',';
  if (sport.received) out << sport.sequence << ',' << sport.stamp_sec << ',' << sport.stamp_nanosec
                          << ',' << static_cast<unsigned>(sport.mode) << ',' << sport.error_code
                          << ',' << (sport.values_finite ? 1 : 0);
  else out << ",,,,,";
  out << ',';
  if (sample.received) out << sample.tick;
  out << ','
      << (repeated ? 1 : 0) << ',' << (inputs_finite ? 1 : 0) << ','
      << (lowstate_fresh ? 1 : 0) << ',' << (estimate_finite ? 1 : 0) << ',';
  double qnorm = 0.0;
  for (size_t i = 0; i < 4; ++i) qnorm += sample.imu[i] * sample.imu[i];
  out << std::sqrt(qnorm);
  for (size_t i = 0; i < kJoints.size(); ++i)
    out << ',' << sample.q[i] << ',' << sample.dq[i] << ',' << sample.tau_est[i];
  for (double x : sample.imu) out << ',' << x;
  for (double x : sample.force) out << ',' << x << ',' << (x >= kFeetForceThreshold ? 1 : 0);
  if (sport.received) {
    for (double x : sport.position) out << ',' << x;
    for (double x : sport.velocity) out << ',' << x;
  } else {
    out << ",,,,,,";
  }
  if (estimate_finite && estimator != nullptr) {
    const Vec3 world = estimator->getVelocity();
    const Vec3 body = estimator->getRotation().transpose() * world;
    out << ',' << world.x() << ',' << world.y() << ',' << world.z()
        << ',' << body.x() << ',' << body.y() << ',' << body.z();
  } else {
    out << ",,,,,,";
  }
  out << '\n';
}

bool parseOptions(int argc, char** argv, int& domain, std::string& interface_name,
                  std::string& urdf, std::string& csv, double& duration_s, bool& self_check,
                  Estimator::FootMeasurementMode& foot_measurement_mode) {
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
    else if (arg == "--foot-measurement-mode") {
      if (value == "legacy") foot_measurement_mode = Estimator::FootMeasurementMode::LegacyBodyAsWorld;
      else if (value == "world") foot_measurement_mode = Estimator::FootMeasurementMode::RotateToWorld;
      else if (value == "world_gyro") foot_measurement_mode = Estimator::FootMeasurementMode::RotateToWorldWithGyro;
      else throw std::invalid_argument("--foot-measurement-mode must be legacy, world, or world_gyro");
    }
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
  Estimator legacy(storage.interfaces, component);
  Estimator world(storage.interfaces, component, kFeetForceThreshold, Estimator::FootMeasurementMode::RotateToWorld);
  Estimator world_gyro(storage.interfaces, component, kFeetForceThreshold,
                       Estimator::FootMeasurementMode::RotateToWorldWithGyro);
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
    legacy.update();
    world.update();
    world_gyro.update();
  }
  const Vec3 legacy_velocity = legacy.getVelocity();
  const Vec3 world_velocity = world.getVelocity();
  const Vec3 world_gyro_velocity = world_gyro.getVelocity();
  const Vec3 body = legacy.getRotation().transpose() * legacy_velocity;
  if (!finite(legacy_velocity) || !finite(world_velocity) || !finite(world_gyro_velocity) ||
      !finite(body) || legacy_velocity.norm() > 5e-6 || world_velocity.norm() > 5e-6 ||
      world_gyro_velocity.norm() > 5e-6) {
    std::ostringstream message;
    message << "static plumbing self-check failed: legacy=[" << legacy_velocity.transpose()
            << "] world=[" << world_velocity.transpose() << "] world_gyro=["
            << world_gyro_velocity.transpose()
            << "] body=[" << body.transpose() << "]";
    throw std::runtime_error(message.str());
  }

  // Synthetic asynchronous samples exercise the added comparison CSV path without DDS.
  Sample low;
  low.received = true;
  low.sequence = 2;
  low.tick = 1234;
  low.last_gap_s = 0.005;
  low.q.fill(0.1);
  low.dq.fill(0.2);
  low.tau_est.fill(0.3);
  low.imu = {1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 9.81};
  low.force = {15.0, 15.0, 14.0, 14.0};
  SportSample sport;
  sport.received = true;
  sport.sequence = 3;
  sport.stamp_sec = 77;
  sport.stamp_nanosec = 88;
  sport.mode = 9;
  sport.error_code = 4;
  sport.values_finite = true;
  sport.last_gap_s = 0.01;
  sport.position = {1.0, 2.0, 3.0};
  sport.velocity = {4.0, 5.0, 6.0};
  const auto base = Clock::time_point{} + std::chrono::seconds(10);
  low.last = base + std::chrono::milliseconds(10);
  sport.last = base + std::chrono::milliseconds(20);
  const auto update_time = base + std::chrono::milliseconds(25);
  std::ostringstream header, row;
  writeHeader(header);
  writeRow(row, low, sport, base, update_time, 5.0, 15.0, false,
           true, true, true, &legacy);
  auto splitCsv = [](const std::string& line) {
    std::vector<std::string> fields;
    std::string content = line;
    while (!content.empty() && (content.back() == '\n' || content.back() == '\r')) content.pop_back();
    std::stringstream input(content);
    std::string field;
    while (std::getline(input, field, ',')) fields.push_back(field);
    if (!content.empty() && content.back() == ',') fields.emplace_back();
    return fields;
  };
  const auto columns = splitCsv(header.str());
  const auto values = splitCsv(row.str());
  if (columns.size() != values.size())
    throw std::runtime_error("synthetic comparison CSV column count mismatch");
  auto columnIndex = [&](const std::string& name) -> size_t {
    for (size_t i = 0; i < columns.size(); ++i)
      if (columns[i] == name) return i;
    throw std::runtime_error("missing comparison CSV column: " + name);
  };
  SportSample no_sport;
  std::ostringstream no_sport_row;
  writeRow(no_sport_row, low, no_sport, base, update_time, 5.0, 15.0, false,
           true, true, true, &legacy);
  const auto no_sport_values = splitCsv(no_sport_row.str());
  if (no_sport_values.size() != columns.size() ||
      no_sport_values[columnIndex("sport_received")] != "0" ||
      !no_sport_values[columnIndex("sport_receive_time_s")].empty() ||
      !no_sport_values[columnIndex("sport_values_finite")].empty() ||
      !no_sport_values[columnIndex("sport_velocity_x_raw")].empty())
    throw std::runtime_error("synthetic no-Sport CSV field count/blank check failed");
  std::ostringstream invalid_estimate_row;
  writeRow(invalid_estimate_row, low, no_sport, base, update_time, 5.0, 15.0, false,
           true, true, false, nullptr);
  if (splitCsv(invalid_estimate_row.str()).size() != columns.size())
    throw std::runtime_error("synthetic invalid-estimate CSV field count mismatch");
  auto field = [&](const std::string& name) -> double {
    return std::stod(values[columnIndex(name)]);
  };
  if (std::abs(field("lowstate_receive_time_s") - 0.01) > 1e-12 ||
      std::abs(field("sport_receive_time_s") - 0.02) > 1e-12 ||
      std::abs(field("sport_age_ms") - 5.0) > 1e-12 ||
      std::abs(field("sport_velocity_x_raw") - 4.0) > 1e-12 ||
      std::abs(field("FR_hip_joint_tau_est_raw") - 0.3) > 1e-12 ||
      field("lowstate_sequence") != 2 || field("sport_sequence") != 3)
    throw std::runtime_error("synthetic comparison CSV time/sequence/value check failed");
  std::ostringstream sport_header;
  std::ostringstream sport_row;
  writeSportHeader(sport_header);
  writeSportRecord(sport_row, sport, 0.02);
  const auto sport_columns = splitCsv(sport_header.str());
  const auto sport_values = splitCsv(sport_row.str());
  if (sport_header.str().find("sport_receive_time_s,sport_receive_gap_ms,sport_sequence") != 0 ||
      sport_header.str().find("position_x_raw") == std::string::npos ||
      sport_header.str().find("velocity_z_raw") == std::string::npos ||
      sport_columns.size() != sport_values.size() ||
      std::abs(std::stod(sport_values[0]) - 0.02) > 1e-12 ||
      std::abs(std::stod(sport_values[13]) - 6.0) > 1e-12)
    throw std::runtime_error("synthetic SportModeState CSV schema check failed");
  std::cout << "SELF_CHECK_OK mass_kg=" << component.robot_model_->mass_
            << " dt_s=" << 1.0 / storage.interfaces.frequency_
            << " legacy_static_velocity_m_s=" << legacy_velocity.transpose()
            << " corrected_static_velocity_m_s=" << world_gyro_velocity.transpose()
            << " body_velocity_m_s=" << body.transpose()
            << " contacts=[1,1,1,1] threshold=1\n"
            << "synthetic_async_csv=ok sport_present_absent=ok invalid_estimate_columns=ok\n"
            << "Synthetic input plumbing only; not a Go2 accuracy or dynamic validation.\n";
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  int domain = 0;
  std::string interface_name, urdf_path, csv_path;
  double duration_s = 10.0;
  bool self_check = false;
  auto foot_measurement_mode = Estimator::FootMeasurementMode::LegacyBodyAsWorld;
  if (argc == 2 && std::string(argv[1]) == "--help") {
    std::cout << "Usage (live): " << argv[0]
              << " --domain N --interface IFACE --urdf FILE --csv FILE [--duration-s 10]"
                 " [--foot-measurement-mode legacy|world|world_gyro]\n"
              << "Usage (offline, no DDS): " << argv[0] << " --self-check --urdf FILE\n";
    return 0;
  }
  try {
    if (!parseOptions(argc, argv, domain, interface_name, urdf_path, csv_path, duration_s, self_check,
                      foot_measurement_mode)) return 2;
    if (self_check) return selfCheck(urdf_path);

    std::ofstream csv(csv_path);
    if (!csv) throw std::runtime_error("cannot open CSV output: " + csv_path);
    writeHeader(csv);
    std::ofstream sport_csv(csv_path + ".sport.csv");
    if (!sport_csv) throw std::runtime_error("cannot open SportModeState CSV output: " + csv_path + ".sport.csv");
    writeSportHeader(sport_csv);

    InterfaceStorage storage;
    CtrlComponent component;
    makeRobot(storage, component, urdf_path);
    Estimator estimator(storage.interfaces, component, kFeetForceThreshold, foot_measurement_mode);
    auto subscriber = std::make_shared<unitree::robot::ChannelSubscriber<LowState>>("rt/lowstate");
    auto sport_subscriber = std::make_shared<unitree::robot::ChannelSubscriber<SportModeState>>("rt/sportmodestate");
    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);
    bool sdk_initialized = false;
    int exit_code = 0;
    try {
      unitree::robot::ChannelFactory::Instance()->Init(domain, interface_name);
      sdk_initialized = true;
      g_session_start = Clock::now();
      subscriber->InitChannel(onLowState, 1);
      sport_subscriber->InitChannel(onSportModeState, 1);
      std::cerr << "只订阅 rt/lowstate 与 rt/sportmodestate；不创建 LowCmd 发布器、MotionSwitcher 客户端、ROS controller 或策略模型。"
                << " estimator_rate=" << kEstimatorRateHz << "Hz dt="
                << (1.0 / kEstimatorRateHz) << "s threshold=" << kFeetForceThreshold
                << " foot_measurement_mode="
                << (foot_measurement_mode == Estimator::FootMeasurementMode::LegacyBodyAsWorld ? "legacy" :
                    (foot_measurement_mode == Estimator::FootMeasurementMode::RotateToWorld ? "world" : "world_gyro"))
                << '\n';

      const auto start = Clock::now();
      auto next_tick = start;
      auto previous_tick = start;
      auto last_status = start;
      auto startup_deadline = start + std::chrono::seconds(3);
      uint64_t previous_sequence = 0;
      uint64_t last_status_low_sequence = 0;
      uint64_t last_status_sport_sequence = 0;
      while (!g_stop && std::chrono::duration<double>(Clock::now() - start).count() < duration_s) {
        next_tick += std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(1.0 / kEstimatorRateHz));
        std::this_thread::sleep_until(next_tick);
        if (g_stop) break;
        Sample sample;
        SportSample sport;
        std::deque<SportSample> sport_updates;
        {
          std::lock_guard<std::mutex> lock(g_sample_mutex);
          sample = g_sample;
        }
        {
          std::lock_guard<std::mutex> lock(g_sport_mutex);
          sport = g_sport;
          sport_updates.swap(g_sport_pending);
        }
        for (const auto& update : sport_updates)
          writeSportRecord(sport_csv, update,
                           std::chrono::duration<double>(update.last - g_session_start).count());
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
        bool estimate_finite = false;
        if (valid) {
          storage.load(sample);
          component.robot_model_->update();
          estimator.update();
          const Vec3 world = estimator.getVelocity();
          const Vec3 body = estimator.getRotation().transpose() * world;
          estimate_finite = finite(world) && finite(body);
        }
        writeRow(csv, sample, sport, g_session_start, now, period_ms, age_s * 1000.0, repeated,
                 inputs_finite, lowstate_fresh, estimate_finite,
                 estimate_finite ? &estimator : nullptr);
        previous_sequence = sample.sequence;
        if (std::chrono::duration<double>(now - last_status).count() >= 1.0) {
          std::cerr << "只读观察 t=" << std::chrono::duration<double>(now - g_session_start).count()
                    << "s LowState_seq=" << sample.sequence << " age=" << age_s * 1000.0
                    << "ms new_LowState_this_interval="
                    << (sample.sequence > last_status_low_sequence ? "yes" : "no")
                    << " Sport_seq=" << sport.sequence << " new_Sport_this_interval="
                    << (sport.sequence > last_status_sport_sequence ? "yes" : "no")
                    << " age=";
          if (sport.received)
            std::cerr << std::chrono::duration<double, std::milli>(now - sport.last).count()
                      << "ms values_finite=" << (sport.values_finite ? "yes" : "no");
          else std::cerr << "none values_finite=unknown";
          std::cerr << '\n';
          last_status = now;
          last_status_low_sequence = sample.sequence;
          last_status_sport_sequence = sport.sequence;
        }
        if (!valid || !estimate_finite) {
          std::cerr << "LowState过期/非有限，或Estimator输出非有限；记录最后故障行后结束采集；age_ms="
                    << age_s * 1000.0 << '\n';
          exit_code = 2;
          break;
        }
      }
    } catch (const std::exception& e) {
      std::cerr << "只读观察失败: " << e.what() << '\n';
      exit_code = 1;
    }
    sport_subscriber->CloseChannel();
    subscriber->CloseChannel();
    {
      std::deque<SportSample> sport_updates;
      std::lock_guard<std::mutex> lock(g_sport_mutex);
      sport_updates.swap(g_sport_pending);
      for (const auto& update : sport_updates)
        writeSportRecord(sport_csv, update,
                         std::chrono::duration<double>(update.last - g_session_start).count());
    }
    sport_csv.flush();
    csv.flush();
    if (sdk_initialized) unitree::robot::ChannelFactory::Instance()->Release();
    std::cerr << "Estimator观察已退出；CSV=" << csv_path << '\n';
    return exit_code;
  } catch (const std::exception& e) {
    std::cerr << "参数/初始化错误: " << e.what() << '\n';
    return 2;
  }
}
