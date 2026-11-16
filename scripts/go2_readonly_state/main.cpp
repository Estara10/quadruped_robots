#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include <array>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unistd.h>

namespace {

using Clock = std::chrono::steady_clock;
using LowState = unitree_go::msg::dds_::LowState_;
using SportModeState = unitree_go::msg::dds_::SportModeState_;

constexpr char kLowStateTopic[] = "rt/lowstate";
constexpr char kSportModeStateTopic[] = "rt/sportmodestate";
constexpr std::array<const char*, 12> kJointNames{
    "FR_hip", "FR_thigh", "FR_calf", "FL_hip", "FL_thigh", "FL_calf",
    "RR_hip", "RR_thigh", "RR_calf", "RL_hip", "RL_thigh", "RL_calf"};

struct LowSnapshot {
  bool received = false;
  uint64_t count = 0;
  Clock::time_point first{};
  Clock::time_point last{};
  double max_gap_s = 0.0;
  std::array<double, 12> q{};
  std::array<double, 12> dq{};
  std::array<double, 12> tau_est{};
  std::array<double, 12> temperature{};
  std::array<double, 4> quaternion_wxyz{};
  std::array<double, 3> gyro_xyz{};
  std::array<double, 3> accel_xyz{};
  uint32_t tick = 0;
  std::array<uint32_t, 2> version{};
  std::array<int16_t, 4> foot_force{};
};

struct SportSnapshot {
  bool received = false;
  uint64_t count = 0;
  Clock::time_point first{};
  Clock::time_point last{};
  double max_gap_s = 0.0;
  int32_t stamp_sec = 0;
  uint32_t stamp_nanosec = 0;
  Clock::time_point stamp_last_changed{};
  uint32_t error_code = 0;
  uint8_t mode = 0;
  std::array<double, 3> position{};
  std::array<double, 3> velocity{};
};

std::mutex g_low_mutex;
std::mutex g_sport_mutex;
LowSnapshot g_low;
SportSnapshot g_sport;
volatile std::sig_atomic_t g_stop = 0;

void onSignal(int) { g_stop = 1; }

void onLowState(const void* message) {
  const auto& state = *static_cast<const LowState*>(message);
  const auto now = Clock::now();
  std::lock_guard<std::mutex> lock(g_low_mutex);
  if (!g_low.received) g_low.first = now;
  else g_low.max_gap_s = std::max(g_low.max_gap_s, std::chrono::duration<double>(now - g_low.last).count());
  g_low.received = true;
  ++g_low.count;
  g_low.last = now;

  for (size_t i = 0; i < kJointNames.size(); ++i) {
    g_low.q[i] = state.motor_state()[i].q();
    g_low.dq[i] = state.motor_state()[i].dq();
    g_low.tau_est[i] = state.motor_state()[i].tau_est();
    g_low.temperature[i] = state.motor_state()[i].temperature();
  }
  for (size_t i = 0; i < 4; ++i)
    g_low.quaternion_wxyz[i] = state.imu_state().quaternion()[i];
  for (size_t i = 0; i < 3; ++i) {
    g_low.gyro_xyz[i] = state.imu_state().gyroscope()[i];
    g_low.accel_xyz[i] = state.imu_state().accelerometer()[i];
  }
  g_low.tick = state.tick();
  g_low.version = state.version();
  for (size_t i = 0; i < 4; ++i)
    g_low.foot_force[i] = state.foot_force()[i];
}

void onSportModeState(const void* message) {
  const auto& state = *static_cast<const SportModeState*>(message);
  const auto now = Clock::now();
  std::lock_guard<std::mutex> lock(g_sport_mutex);
  if (!g_sport.received) g_sport.first = now;
  else g_sport.max_gap_s = std::max(g_sport.max_gap_s, std::chrono::duration<double>(now - g_sport.last).count());
  if (!g_sport.received || g_sport.stamp_sec != state.stamp().sec() ||
      g_sport.stamp_nanosec != state.stamp().nanosec())
    g_sport.stamp_last_changed = now;
  g_sport.stamp_sec = state.stamp().sec();
  g_sport.stamp_nanosec = state.stamp().nanosec();
  g_sport.error_code = state.error_code();
  g_sport.mode = state.mode();
  g_sport.received = true;
  ++g_sport.count;
  g_sport.last = now;
  for (size_t i = 0; i < 3; ++i) {
    g_sport.position[i] = state.position()[i];
    g_sport.velocity[i] = state.velocity()[i];
  }
}

double receiveHz(uint64_t count, Clock::time_point first, Clock::time_point last) {
  if (count < 2) return std::numeric_limits<double>::quiet_NaN();
  const double seconds = std::chrono::duration<double>(last - first).count();
  return seconds > 0.0 ? static_cast<double>(count - 1) / seconds
                       : std::numeric_limits<double>::quiet_NaN();
}

std::string value(double x) {
  if (!std::isfinite(x)) return "异常(NaN/Inf)";
  std::ostringstream out;
  out << std::fixed << std::setprecision(4) << x;
  return out.str();
}

std::string vectorText(const double* values, size_t count) {
  std::ostringstream out;
  out << '[';
  for (size_t i = 0; i < count; ++i) {
    if (i) out << ", ";
    out << value(values[i]);
  }
  out << ']';
  return out.str();
}

std::string receiveStatus(bool received, uint64_t count, Clock::time_point first,
                          Clock::time_point last, Clock::time_point now) {
  if (!received) return "尚未收到数据 | 接收数=0 | 频率=待测 | 最近接收=无";
  const double age = std::chrono::duration<double>(now - last).count();
  const double hz = receiveHz(count, first, last);
  std::ostringstream out;
  out << "已收到 | 接收数=" << count << " | 本机平均接收频率=";
  if (std::isfinite(hz)) out << std::fixed << std::setprecision(2) << hz << " Hz";
  else out << "样本不足";
  out << " | 最近接收距今=" << std::fixed << std::setprecision(2) << age << " s";
  return out.str();
}

void printFrame(const LowSnapshot& low, const SportSnapshot& sport) {
  const auto now = Clock::now();
  std::ostringstream out;
  out << "Go2只读状态观察（本机DDS接收统计；Ctrl+C退出）\n"
      << "============================================================\n"
      << "LowState [rt/lowstate]: "
      << receiveStatus(low.received, low.count, low.first, low.last, now) << '\n';
  if (!low.received) {
    out << "  状态：尚未收到数据（不显示占位数值）\n";
  } else {
    out << "  最大本机接收间隔=" << low.max_gap_s * 1000.0 << " ms\n"
        << "  LowState原始tick=" << low.tick << " | 原始version=["
        << low.version[0] << ", " << low.version[1] << "]（不直接解释为固件版本）\n"
        << "  关节顺序 FR, FL, RR, RL；每腿 hip, thigh, calf\n"
        << "  q (接口预期 rad；待实机核实): ";
    for (size_t i = 0; i < kJointNames.size(); ++i) {
      if (i) out << ' ';
      out << kJointNames[i] << '=' << value(low.q[i]);
    }
    out << "\n  dq (接口预期 rad/s；待实机核实): ";
    for (size_t i = 0; i < kJointNames.size(); ++i) {
      if (i) out << ' ';
      out << kJointNames[i] << '=' << value(low.dq[i]);
    }
    out << "\n  tau_est (接口预期 Nm，估计值非独立力矩测量): ";
    for (size_t i = 0; i < kJointNames.size(); ++i) {
      if (i) out << ' ';
      out << kJointNames[i] << '=' << value(low.tau_est[i]);
    }
    out << "\n  电机温度 (SDK原始值，接口预期摄氏度): ";
    for (size_t i = 0; i < kJointNames.size(); ++i) {
      if (i) out << ' ';
      out << kJointNames[i] << '=' << value(low.temperature[i]);
    }
    out << "\n  IMU四元数 [w,x,y,z] (无量纲): "
        << vectorText(low.quaternion_wxyz.data(), low.quaternion_wxyz.size())
        << "\n  IMU角速度 [x,y,z] (SDK原值；单位待实机核实): "
        << vectorText(low.gyro_xyz.data(), low.gyro_xyz.size())
        << "\n  IMU加速度 [x,y,z] (SDK原始值，接口预期 m/s^2): "
        << vectorText(low.accel_xyz.data(), low.accel_xyz.size())
        << "\n  足端力SDK原始值 [FR,FL,RR,RL] (比例/物理单位待实机核实): ["
        << low.foot_force[0] << ", " << low.foot_force[1] << ", "
        << low.foot_force[2] << ", " << low.foot_force[3] << "]\n";
  }

  out << "SportModeState [rt/sportmodestate]: "
      << receiveStatus(sport.received, sport.count, sport.first, sport.last, now) << '\n';
  if (!sport.received) {
    out << "  状态：尚未收到数据（不显示占位数值）\n";
  } else {
    out << "  最大本机接收间隔=" << sport.max_gap_s * 1000.0 << " ms"
        << " | 机器人stamp=" << sport.stamp_sec << "s+" << sport.stamp_nanosec << "ns"
        << " | 距最近stamp变化=" << std::chrono::duration<double>(now - sport.stamp_last_changed).count() << " s"
        << " | mode=" << static_cast<unsigned>(sport.mode) << " | error_code=" << sport.error_code << '\n'
        << "  stamp是否代表有效采样时间仍待实机核实；持续收包不等于里程有效。\n"
        << "  原始position [x,y,z] (坐标系/单位待实机核实): "
        << vectorText(sport.position.data(), sport.position.size())
        << "\n  原始velocity [x,y,z] (坐标系/单位待实机核实): "
        << vectorText(sport.velocity.data(), sport.velocity.size()) << '\n';
  }
  out << "频率是本机收到消息的平均频率，不代表传感器采样频率。显示的是各topic最近一条样本，时龄单独计时。\n";

  if (::isatty(STDOUT_FILENO)) std::cout << "\033[2J\033[H";
  std::cout << out.str() << std::flush;
}

void printJson(const LowSnapshot& low, const SportSnapshot& sport) {
  const auto now = Clock::now();
  std::ostringstream out;
  out << std::setprecision(10);
  auto number = [&out](double x) {
    if (std::isfinite(x)) out << x;
    else out << "null";
  };
  auto array = [&out, &number](const auto& values) {
    out << '[';
    for (size_t i = 0; i < values.size(); ++i) {
      if (i) out << ',';
      number(values[i]);
    }
    out << ']';
  };
  out << "{\"time\":" << std::chrono::duration<double>(now.time_since_epoch()).count()
      << ",\"low\":";
  if (!low.received) out << "null";
  else {
    out << "{\"age\":" << std::chrono::duration<double>(now - low.last).count()
        << ",\"hz\":";
    number(receiveHz(low.count, low.first, low.last));
    out << ",\"gap_ms\":" << low.max_gap_s * 1000.0 << ",\"q\":"; array(low.q);
    out << ",\"dq\":"; array(low.dq);
    out << ",\"tau\":"; array(low.tau_est);
    out << ",\"temperature\":"; array(low.temperature);
    out << ",\"quaternion\":"; array(low.quaternion_wxyz);
    out << ",\"gyro\":"; array(low.gyro_xyz);
    out << ",\"accel\":"; array(low.accel_xyz);
    out << ",\"foot_force\":"; array(low.foot_force);
    out << '}';
  }
  out << ",\"sport\":";
  if (!sport.received) out << "null";
  else {
    out << "{\"age\":" << std::chrono::duration<double>(now - sport.last).count()
        << ",\"hz\":";
    number(receiveHz(sport.count, sport.first, sport.last));
    out << ",\"gap_ms\":" << sport.max_gap_s * 1000.0
        << ",\"mode\":" << static_cast<unsigned>(sport.mode)
        << ",\"error_code\":" << sport.error_code << ",\"position\":";
    array(sport.position);
    out << ",\"velocity\":"; array(sport.velocity);
    out << '}';
  }
  out << "}\n";
  std::cout << out.str() << std::flush;
}

bool parseOptions(int argc, char** argv, int& domain, std::string& interface_name, bool& json) {
  bool have_domain = false;
  bool have_interface = false;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--help") {
      std::cout << "Usage: " << argv[0]
                << " --domain <DDS_DOMAIN> --interface <NETWORK_INTERFACE> [--json]\n";
      return false;
    }
    if ((arg == "--domain" || arg == "--interface") && i + 1 >= argc) {
      std::cerr << "Missing value for " << arg << '\n';
      return false;
    }
    if (arg == "--json") {
      json = true;
    } else if (arg == "--domain") {
      try {
        size_t used = 0;
        domain = std::stoi(argv[++i], &used);
        if (argv[i][used] != '\0') throw std::invalid_argument("trailing characters");
      } catch (const std::exception&) {
        std::cerr << "Invalid DDS domain\n";
        return false;
      }
      have_domain = true;
    } else if (arg == "--interface") {
      interface_name = argv[++i];
      have_interface = !interface_name.empty();
    } else {
      std::cerr << "Unknown option: " << arg << '\n';
      return false;
    }
  }
  if (!have_domain || !have_interface) {
    std::cerr << "Both --domain and --interface are required.\n";
    return false;
  }
  return true;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc == 2 && std::string(argv[1]) == "--help") {
    std::cout << "Usage: " << argv[0]
              << " --domain <DDS_DOMAIN> --interface <NETWORK_INTERFACE> [--json]\n";
    return 0;
  }
  int domain = 0;
  std::string interface_name;
  bool json = false;
  if (!parseOptions(argc, argv, domain, interface_name, json)) return 2;

  std::signal(SIGINT, onSignal);
  std::signal(SIGTERM, onSignal);

  auto low_subscriber = std::make_shared<unitree::robot::ChannelSubscriber<LowState>>(kLowStateTopic);
  auto sport_subscriber = std::make_shared<unitree::robot::ChannelSubscriber<SportModeState>>(kSportModeStateTopic);
  bool sdk_initialized = false;
  int exit_code = 0;
  try {
    unitree::robot::ChannelFactory::Instance()->Init(domain, interface_name);
    sdk_initialized = true;
    low_subscriber->InitChannel(onLowState, 1);
    sport_subscriber->InitChannel(onSportModeState, 1);
    std::cerr << "只订阅 rt/lowstate 与 rt/sportmodestate；网卡=" << interface_name
              << " DDS domain=" << domain << "。未创建命令发布器。\n";

    while (!g_stop) {
      LowSnapshot low;
      SportSnapshot sport;
      {
        std::lock_guard<std::mutex> lock(g_low_mutex);
        low = g_low;
      }
      {
        std::lock_guard<std::mutex> lock(g_sport_mutex);
        sport = g_sport;
      }
      if (json) printJson(low, sport);
      else printFrame(low, sport);
      for (int i = 0; i < (json ? 1 : 10) && !g_stop; ++i)
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  } catch (const std::exception& e) {
    std::cerr << "SDK订阅初始化/运行失败: " << e.what() << '\n';
    exit_code = 1;
  }

  low_subscriber->CloseChannel();
  sport_subscriber->CloseChannel();
  if (sdk_initialized) unitree::robot::ChannelFactory::Instance()->Release();
  std::cerr << "只读状态观察程序已退出。\n";
  return exit_code;
}
