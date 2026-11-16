#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/robot/go2/robot_state/robot_state_client.hpp>
#include <unitree/robot/go2/sport/sport_client.hpp>

#include <array>
#include <chrono>
#include <cmath>
#include <csignal>
#include <ctime>
#include <deque>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <signal.h>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>

namespace {

using Clock = std::chrono::steady_clock;
using SportModeState = unitree_go::msg::dds_::SportModeState_;
using ServiceState = unitree::robot::go2::ServiceState;

constexpr char kSportModeStateTopic[] = "rt/sportmodestate";
constexpr double kStateWaitS = 3.0;
constexpr double kQueryTimeoutS = 1.0;
constexpr double kPollPeriodS = 0.05;

volatile std::sig_atomic_t g_interrupted = 0;

void handleSignal(int) { g_interrupted = 1; }

void installSignalHandlers() {
  struct sigaction action {};
  action.sa_handler = handleSignal;
  sigemptyset(&action.sa_mask);
  action.sa_flags = 0;
  sigaction(SIGINT, &action, nullptr);
  sigaction(SIGTERM, &action, nullptr);
  sigaction(SIGHUP, &action, nullptr);
}

struct StateSnapshot {
  bool received = false;
  uint64_t seq = 0;
  uint8_t mode = 0;
  uint32_t error_code = 0;
  std::array<double, 3> position{};
  std::array<double, 3> velocity{};
  double yaw_rad = 0.0;
  Clock::time_point last_receive{};
};

std::mutex g_state_mutex;
StateSnapshot g_state;

void onSportModeState(const void* message) {
  const auto& msg = *static_cast<const SportModeState*>(message);
  const auto now = Clock::now();
  std::lock_guard<std::mutex> lock(g_state_mutex);
  g_state.received = true;
  ++g_state.seq;
  g_state.mode = msg.mode();
  g_state.error_code = msg.error_code();
  g_state.position = {static_cast<double>(msg.position()[0]),
                      static_cast<double>(msg.position()[1]),
                      static_cast<double>(msg.position()[2])};
  g_state.velocity = {static_cast<double>(msg.velocity()[0]),
                      static_cast<double>(msg.velocity()[1]),
                      static_cast<double>(msg.velocity()[2])};
  g_state.yaw_rad = static_cast<double>(msg.imu_state().rpy()[2]);
  g_state.last_receive = now;
}

bool parseInt(const std::string& text, int& value) {
  try {
    std::size_t position = 0;
    const long parsed = std::stol(text, &position);
    if (position != text.size() || parsed < 0 || parsed > 232) {
      return false;
    }
    value = static_cast<int>(parsed);
    return true;
  } catch (...) {
    return false;
  }
}

void printUsage(const char* program) {
  std::cout
      << "用法: " << program
      << " --domain <DDS domain> --interface <NIC>\n"
      << "只读查询入口：订阅 rt/sportmodestate，并发送 CheckMode、"
         "ServiceList、GetServerApiVersion 查询请求。\n"
      << "不调用运动、停止、站立、趴伏、阻尼、步态、模式切换、"
         "ServiceSwitch、SwitchJoystick、SetSilent 或 SetReportFreq。\n"
      << "查询有有限等待；不支持或失败的项打印 NOT OBTAINED，不补默认值。\n"
      << "退出时不发送停止或恢复指令。\n";
}

int parseArgs(int argc, char** argv, int& domain, std::string& interface_name,
              bool& help) {
  for (int i = 1; i < argc; ++i) {
    const std::string key = argv[i];
    auto takeValue = [&](std::string& value) {
      if (i + 1 >= argc) {
        std::cerr << "缺少参数值: " << key << "\n";
        return false;
      }
      value = argv[++i];
      return true;
    };
    if (key == "--help" || key == "-h") {
      help = true;
      return 0;
    }
    if (key == "--domain") {
      std::string value;
      if (!takeValue(value)) return 2;
      if (!parseInt(value, domain)) {
        std::cerr << "--domain 必须是 0..232 的整数\n";
        return 2;
      }
    } else if (key == "--interface") {
      if (!takeValue(interface_name)) return 2;
    } else {
      std::cerr << "未知参数: " << key << "\n";
      return 2;
    }
  }
  if (domain < 0) {
    std::cerr << "必须显式给出 --domain\n";
    return 2;
  }
  if (interface_name.empty()) {
    std::cerr << "必须显式给出 --interface\n";
    return 2;
  }
  if (interface_name == "lo") {
    std::cerr << "--interface lo 被拒绝：本入口面向实机查询；"
                 "离线构建请使用 --help。\n";
    return 2;
  }
  return 0;
}

void printState(const char* label) {
  StateSnapshot snapshot;
  {
    std::lock_guard<std::mutex> lock(g_state_mutex);
    snapshot = g_state;
  }
  const double age_s =
      snapshot.received
          ? std::chrono::duration<double>(Clock::now() -
                                          snapshot.last_receive)
                .count()
          : 0.0;
  std::cout << label << ": ";
  if (!snapshot.received) {
    std::cout << "NOT OBTAINED\n";
    return;
  }
  std::cout << "seq=" << snapshot.seq
            << " mode=" << static_cast<unsigned>(snapshot.mode)
            << " error_code=" << snapshot.error_code << " age_s=" << std::fixed
            << std::setprecision(3) << age_s << " pos=(" << snapshot.position[0]
            << "," << snapshot.position[1] << "," << snapshot.position[2]
            << ") vel=(" << snapshot.velocity[0] << "," << snapshot.velocity[1]
            << "," << snapshot.velocity[2] << ") yaw_raw="
            << snapshot.yaw_rad << "\n";
}

void queryCheckMode() {
  try {
    unitree::robot::b2::MotionSwitcherClient client;
    client.SetTimeout(static_cast<float>(kQueryTimeoutS));
    client.Init();
    std::string form;
    std::string name;
    const int32_t code = client.CheckMode(form, name);
    std::cout << "query check_mode: rc=" << code << " form=\"" << form
              << "\" name=\"" << name << "\"\n";
  } catch (const std::exception& error) {
    std::cout << "query check_mode: NOT OBTAINED (exception: " << error.what()
              << ")\n";
  } catch (...) {
    std::cout << "query check_mode: NOT OBTAINED (unknown exception)\n";
  }
}

void queryServiceList() {
  try {
    unitree::robot::go2::RobotStateClient client;
    client.SetTimeout(static_cast<float>(kQueryTimeoutS));
    client.Init();
    std::vector<ServiceState> services;
    const int32_t code = client.ServiceList(services);
    std::cout << "query service_list: rc=" << code
              << " count=" << services.size() << "\n";
    if (code != 0) {
      std::cout << "query service_list: NOT OBTAINED (nonzero rc)\n";
      return;
    }
    for (std::size_t i = 0; i < services.size(); ++i) {
      std::cout << "  service[" << i << "] name=\"" << services[i].name
                << "\" status=" << services[i].status
                << " protect=" << services[i].protect << "\n";
    }
  } catch (const std::exception& error) {
    std::cout << "query service_list: NOT OBTAINED (exception: "
              << error.what() << ")\n";
  } catch (...) {
    std::cout << "query service_list: NOT OBTAINED (unknown exception)\n";
  }
}

void querySportApiVersion() {
  try {
    unitree::robot::go2::SportClient client;
    client.SetTimeout(static_cast<float>(kQueryTimeoutS));
    client.Init();
    const std::string local_version = client.GetApiVersion();
    const std::string server_version = client.GetServerApiVersion();
    std::cout << "query sport_api: local=\"" << local_version << "\" server=\"";
    if (server_version.empty()) {
      std::cout << "NOT OBTAINED";
    } else {
      std::cout << server_version;
    }
    std::cout << "\"\n";
  } catch (const std::exception& error) {
    std::cout << "query sport_api: NOT OBTAINED (exception: " << error.what()
              << ")\n";
  } catch (...) {
    std::cout << "query sport_api: NOT OBTAINED (unknown exception)\n";
  }
}

}  // namespace

int main(int argc, char** argv) {
  int domain = -1;
  std::string interface_name;
  bool help = false;
  const int parse_result = parseArgs(argc, argv, domain, interface_name, help);
  if (parse_result != 0) {
    return parse_result;
  }
  if (help) {
    printUsage(argv[0]);
    return 0;
  }

  installSignalHandlers();

  std::cout << "Go2 native read-only query\n"
            << "DDS domain=" << domain << ", interface=" << interface_name
            << "\n"
            << "说明：本程序会发送 CheckMode、ServiceList、"
               "GetServerApiVersion 查询请求；不是“只订阅、完全不发包”。\n"
            << "不会发送运动、停止、站立、趴伏、阻尼、步态、模式切换、"
               "ServiceSwitch、SwitchJoystick、SetSilent 或 SetReportFreq。\n";

  try {
    unitree::robot::ChannelFactory::Instance()->Init(domain, interface_name);
  } catch (const std::exception& error) {
    std::cerr << "DDS 初始化失败: " << error.what() << "\n";
    return 1;
  }

  unitree::robot::ChannelSubscriberPtr<SportModeState> subscriber;
  try {
    subscriber.reset(
        new unitree::robot::ChannelSubscriber<SportModeState>(
            kSportModeStateTopic));
    subscriber->InitChannel(onSportModeState, 10);
  } catch (const std::exception& error) {
    std::cout << "state subscription: NOT OBTAINED (exception: " << error.what()
              << ")\n";
  } catch (...) {
    std::cout << "state subscription: NOT OBTAINED (unknown exception)\n";
  }

  const auto deadline = Clock::now() +
                        std::chrono::duration_cast<Clock::duration>(
                            std::chrono::duration<double>(kStateWaitS));
  while (!g_interrupted) {
    {
      std::lock_guard<std::mutex> lock(g_state_mutex);
      if (g_state.received) {
        break;
      }
    }
    if (Clock::now() >= deadline) {
      break;
    }
    std::this_thread::sleep_for(
        std::chrono::duration<double>(kPollPeriodS));
  }

  printState("state before queries");
  if (g_interrupted) {
    std::cout << "interrupted before queries; closing resources\n";
  } else {
    queryCheckMode();
    if (!g_interrupted) {
      queryServiceList();
    }
    if (!g_interrupted) {
      querySportApiVersion();
    }
    printState("state after queries");
  }

  if (subscriber) {
    subscriber->CloseChannel();
  }
  unitree::robot::ChannelFactory::Instance()->Release();
  std::cout << (g_interrupted ? "queries interrupted" : "queries complete")
            << "; no motion or mode-changing request sent; "
               "no stop/restore on exit\n";
  return g_interrupted ? 1 : 0;
}
