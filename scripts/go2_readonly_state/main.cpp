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
  std::array<double, 12> q{};
  std::array<double, 12> dq{};
  std::array<double, 4> quaternion_wxyz{};
  std::array<double, 3> gyro_xyz{};
  std::array<int16_t, 4> foot_force{};
};

struct SportSnapshot {
  bool received = false;
  uint64_t count = 0;
  Clock::time_point first{};
  Clock::time_point last{};
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
  g_low.received = true;
  ++g_low.count;
  g_low.last = now;

  for (size_t i = 0; i < kJointNames.size(); ++i) {
    g_low.q[i] = state.motor_state()[i].q();
    g_low.dq[i] = state.motor_state()[i].dq();
  }
  for (size_t i = 0; i < 4; ++i)
    g_low.quaternion_wxyz[i] = state.imu_state().quaternion()[i];
  for (size_t i = 0; i < 3; ++i)
    g_low.gyro_xyz[i] = state.imu_state().gyroscope()[i];
  for (size_t i = 0; i < 4; ++i)
    g_low.foot_force[i] = state.foot_force()[i];
}

void onSportModeState(const void* message) {
  const auto& state = *static_cast<const SportModeState*>(message);
  const auto now = Clock::now();
  std::lock_guard<std::mutex> lock(g_sport_mutex);
  if (!g_sport.received) g_sport.first = now;
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
    out << "  关节顺序 FR, FL, RR, RL；每腿 hip, thigh, calf\n"
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
    out << "\n  IMU四元数 [w,x,y,z] (无量纲): "
        << vectorText(low.quaternion_wxyz.data(), low.quaternion_wxyz.size())
        << "\n  IMU角速度 [x,y,z] (SDK原值；单位待实机核实): "
        << vectorText(low.gyro_xyz.data(), low.gyro_xyz.size())
        << "\n  足端力SDK原始值 [FR,FL,RR,RL] (比例/物理单位待实机核实): ["
        << low.foot_force[0] << ", " << low.foot_force[1] << ", "
        << low.foot_force[2] << ", " << low.foot_force[3] << "]\n";
  }

  out << "SportModeState [rt/sportmodestate]: "
      << receiveStatus(sport.received, sport.count, sport.first, sport.last, now) << '\n';
  if (!sport.received) {
    out << "  状态：尚未收到数据（不显示占位数值）\n";
  } else {
    out << "  原始position [x,y,z] (坐标系/单位待实机核实): "
        << vectorText(sport.position.data(), sport.position.size())
        << "\n  原始velocity [x,y,z] (坐标系/单位待实机核实): "
        << vectorText(sport.velocity.data(), sport.velocity.size()) << '\n';
  }
  out << "频率是本机收到消息的平均频率，不代表传感器采样频率。显示的是各topic最近一条样本，时龄单独计时。\n";

  if (::isatty(STDOUT_FILENO)) std::cout << "\033[2J\033[H";
  std::cout << out.str() << std::flush;
}

bool parseOptions(int argc, char** argv, int& domain, std::string& interface_name) {
  bool have_domain = false;
  bool have_interface = false;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--help") {
      std::cout << "Usage: " << argv[0]
                << " --domain <DDS_DOMAIN> --interface <NETWORK_INTERFACE>\n";
      return false;
    }
    if ((arg == "--domain" || arg == "--interface") && i + 1 >= argc) {
      std::cerr << "Missing value for " << arg << '\n';
      return false;
    }
    if (arg == "--domain") {
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
              << " --domain <DDS_DOMAIN> --interface <NETWORK_INTERFACE>\n";
    return 0;
  }
  int domain = 0;
  std::string interface_name;
  if (!parseOptions(argc, argv, domain, interface_name)) return 2;

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
      printFrame(low, sport);
      for (int i = 0; i < 10 && !g_stop; ++i)
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
