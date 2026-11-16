#include "native_motion_core.hpp"

#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/robot/go2/sport/sport_client.hpp>

#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <csignal>
#include <ctime>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <poll.h>
#include <signal.h>
#include <string>
#include <system_error>
#include <thread>
#include <unistd.h>
#include <utility>

namespace {

using Clock = std::chrono::steady_clock;
using SportModeState = unitree_go::msg::dds_::SportModeState_;

constexpr char kSportModeStateTopic[] = "rt/sportmodestate";

volatile std::sig_atomic_t g_interrupted = 0;
std::atomic<bool> g_reader_stop{false};

void handleSignal(int) { g_interrupted = 1; }

void installSignalHandlers() {
  struct sigaction action {};
  action.sa_handler = handleSignal;
  sigemptyset(&action.sa_mask);
  action.sa_flags = 0;  // 不设置 SA_RESTART，使阻塞的 poll/read 能返回 EINTR。
  sigaction(SIGINT, &action, nullptr);
  sigaction(SIGTERM, &action, nullptr);
  sigaction(SIGHUP, &action, nullptr);
}

struct Options {
  int domain = -1;
  std::string interface_name;
  go2_native_motion::Config config;
  bool help = false;
};

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

bool parseDouble(const std::string& text, double& value) {
  try {
    std::size_t position = 0;
    const double parsed = std::stod(text, &position);
    if (position != text.size() || !std::isfinite(parsed)) {
      return false;
    }
    value = parsed;
    return true;
  } catch (...) {
    return false;
  }
}

void printUsage(const char* program) {
  std::cout
      << "用法: " << program
      << " --domain <DDS domain> --interface <NIC>"
         " [--vx 0.10] [--duration-s 2.0]"
         " [--goto-max-speed 0.30] [--goto-approach-speed 0.30]"
         " [--csv <path>]\n"
      << "仅使用 Go2 原生 SportClient Move/StopMove 和 rt/sportmodestate；\n"
      << "不创建 LowCmd、不释放/切换控制权、不自动站立或切换步态。\n"
      << "运行后等待有效原生状态，先输入 origin 确认起点，再输入运动命令。\n"
      << "命令: status | origin | straight | goto x y | yaw deg | stop | "
         "quit | help\n"
      << "普通命令按 Enter 执行；Ctrl+C 立即中断，不需要 Enter。\n"
      << "--vx/--duration-s 只配置 straight 短时直走（默认 0.10 m/s, 2.0 s）；\n"
      << "允许 vx [0.05, 0.50] m/s, duration [1.0, 30.0] s。\n"
      << "--goto-max-speed/--goto-approach-speed 只配置 goto（默认 0.30/0.30 m/s）；\n"
      << "允许 [0.05, 0.50] m/s，approach 不得大于 max；approach 是首轮防失速下限，"
         "不是已验证的硬件最低速度。\n"
      << "默认 CSV: logs/go2_native_motion/<UTC时间戳>/native_motion.csv\n"
      << "--interface lo 被拒绝；离线自检请运行 go2_native_motion_selftest。\n";
}

int parseArgs(int argc, char** argv, Options& options, std::string& error) {
  for (int i = 1; i < argc; ++i) {
    const std::string key = argv[i];
    auto takeValue = [&](std::string& value) {
      if (i + 1 >= argc) {
        error = "缺少参数值: " + key;
        return false;
      }
      value = argv[++i];
      return true;
    };

    if (key == "--help" || key == "-h") {
      options.help = true;
      return 0;
    }
    if (key == "--domain") {
      std::string value;
      if (!takeValue(value)) return 2;
      if (!parseInt(value, options.domain)) {
        error = "--domain 必须是 0..232 的整数";
        return 2;
      }
    } else if (key == "--interface") {
      if (!takeValue(options.interface_name)) return 2;
    } else if (key == "--vx") {
      std::string value;
      if (!takeValue(value)) return 2;
      if (!parseDouble(value, options.config.vx_mps)) {
        error = "--vx 必须是有限数值";
        return 2;
      }
    } else if (key == "--duration-s") {
      std::string value;
      if (!takeValue(value)) return 2;
      if (!parseDouble(value, options.config.duration_s)) {
        error = "--duration-s 必须是有限数值";
        return 2;
      }
    } else if (key == "--goto-max-speed") {
      std::string value;
      if (!takeValue(value)) return 2;
      if (!parseDouble(value, options.config.goto_max_speed_mps)) {
        error = "--goto-max-speed 必须是有限数值";
        return 2;
      }
    } else if (key == "--goto-approach-speed") {
      std::string value;
      if (!takeValue(value)) return 2;
      if (!parseDouble(value, options.config.goto_approach_speed_mps)) {
        error = "--goto-approach-speed 必须是有限数值";
        return 2;
      }
    } else if (key == "--csv") {
      if (!takeValue(options.config.csv_path)) return 2;
    } else {
      error = "未知参数: " + key;
      return 2;
    }
  }

  if (options.domain < 0) {
    error = "必须显式给出 --domain";
    return 2;
  }
  if (options.interface_name.empty()) {
    error = "必须显式给出 --interface";
    return 2;
  }
  if (options.interface_name == "lo") {
    error = "--interface lo 被拒绝：本工具面向实机原生运动服务，不接受 loopback；"
            "离线路径请运行 go2_native_motion_selftest";
    return 2;
  }
  return 0;
}

std::string timestampUtc() {
  const auto now = std::chrono::system_clock::now();
  const std::time_t seconds = std::chrono::system_clock::to_time_t(now);
  std::tm tm{};
  gmtime_r(&seconds, &tm);
  char text[32];
  std::strftime(text, sizeof(text), "%Y%m%d_%H%M%S", &tm);
  return text;
}

class CommandQueue {
 public:
  void push(go2_native_motion::Command command) {
    std::lock_guard<std::mutex> lock(mutex_);
    commands_.push_back(std::move(command));
  }

  bool pop(go2_native_motion::Command& command) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (commands_.empty()) {
      return false;
    }
    command = std::move(commands_.front());
    commands_.pop_front();
    return true;
  }

 private:
  std::mutex mutex_;
  std::deque<go2_native_motion::Command> commands_;
};

void readerLoop(CommandQueue& queue) {
  std::string pending;
  while (!g_reader_stop.load() && g_interrupted == 0) {
    struct pollfd descriptor {};
    descriptor.fd = STDIN_FILENO;
    descriptor.events = POLLIN;
    const int ready = poll(&descriptor, 1, 100);
    if (ready < 0) {
      if (errno == EINTR) {
        continue;
      }
      if (!g_reader_stop.load() && g_interrupted == 0) {
        go2_native_motion::Command quit;
        quit.type = go2_native_motion::CommandType::Quit;
        quit.raw = "READ_ERROR";
        queue.push(std::move(quit));
      }
      break;
    }
    if (ready == 0) {
      continue;
    }
    char buffer[512];
    const ssize_t count = ::read(STDIN_FILENO, buffer, sizeof(buffer));
    if (count <= 0) {
      if (!pending.empty() && !g_reader_stop.load() && g_interrupted == 0) {
        queue.push(go2_native_motion::parseCommand(pending));
        pending.clear();
      }
      if (!g_reader_stop.load() && g_interrupted == 0) {
        go2_native_motion::Command quit;
        quit.type = go2_native_motion::CommandType::Quit;
        quit.raw = "EOF";
        queue.push(std::move(quit));
      }
      break;
    }
    pending.append(buffer, static_cast<std::size_t>(count));
    std::size_t newline = 0;
    while ((newline = pending.find('\n')) != std::string::npos) {
      const std::string line = pending.substr(0, newline);
      pending.erase(0, newline + 1);
      queue.push(go2_native_motion::parseCommand(line));
    }
  }
}

class SportBackend final : public go2_native_motion::Backend {
 public:
  SportBackend() {
    // SDK 示例使用 float 秒重载（例如 SetTimeout(10.0f)）。
    client_.SetTimeout(
        static_cast<float>(go2_native_motion::kSdkCallTimeoutS));
    client_.Init();
    subscriber_.reset(
        new unitree::robot::ChannelSubscriber<SportModeState>(
            kSportModeStateTopic));
    subscriber_->InitChannel(
        [this](const void* message) { onState(message); }, 10);
  }

  ~SportBackend() override {
    if (subscriber_) {
      subscriber_->CloseChannel();
    }
  }

  go2_native_motion::StateSnapshot snapshot() override {
    std::lock_guard<std::mutex> lock(state_mutex_);
    go2_native_motion::StateSnapshot result = state_;
    if (result.received) {
      result.age_s =
          std::chrono::duration<double>(Clock::now() - last_receive_).count();
    } else {
      result.age_s = std::numeric_limits<double>::infinity();
    }
    return result;
  }

  int32_t move(float vx, float vy, float vyaw) override {
    try {
      const int32_t code = client_.Move(vx, vy, vyaw);
      setLastError(code == 0 ? std::string()
                             : "Move returned " + std::to_string(code));
      return code;
    } catch (const std::exception& error) {
      setLastError(error.what());
      return -1;
    } catch (...) {
      setLastError("unknown exception in Move");
      return -1;
    }
  }

  int32_t stopMove() override {
    try {
      const int32_t code = client_.StopMove();
      setLastError(code == 0 ? std::string()
                             : "StopMove returned " + std::to_string(code));
      return code;
    } catch (const std::exception& error) {
      setLastError(error.what());
      return -1;
    } catch (...) {
      setLastError("unknown exception in StopMove");
      return -1;
    }
  }

  std::string lastError() override {
    std::lock_guard<std::mutex> lock(error_mutex_);
    return last_error_;
  }

 private:
  void onState(const void* message) {
    const auto& msg = *static_cast<const SportModeState*>(message);
    const auto receive_time = Clock::now();

    std::lock_guard<std::mutex> lock(state_mutex_);
    state_.received = true;
    ++state_.seq;
    state_.position = {static_cast<double>(msg.position()[0]),
                       static_cast<double>(msg.position()[1]),
                       static_cast<double>(msg.position()[2])};
    state_.velocity = {static_cast<double>(msg.velocity()[0]),
                       static_cast<double>(msg.velocity()[1]),
                       static_cast<double>(msg.velocity()[2])};
    state_.yaw_rad = static_cast<double>(msg.imu_state().rpy()[2]);
    state_.finite = true;
    for (const double value : state_.position) {
      state_.finite = state_.finite && std::isfinite(value);
    }
    for (const double value : state_.velocity) {
      state_.finite = state_.finite && std::isfinite(value);
    }
    state_.finite = state_.finite && std::isfinite(state_.yaw_rad);
    last_receive_ = receive_time;
    state_.age_s = 0.0;
  }

  void setLastError(const std::string& error) {
    std::lock_guard<std::mutex> lock(error_mutex_);
    last_error_ = error;
  }

  unitree::robot::go2::SportClient client_;
  unitree::robot::ChannelSubscriberPtr<SportModeState> subscriber_;
  std::mutex state_mutex_;
  go2_native_motion::StateSnapshot state_;
  Clock::time_point last_receive_{};
  std::mutex error_mutex_;
  std::string last_error_;
};

}  // namespace

int main(int argc, char** argv) {
  Options options;
  std::string error;
  const int parse_result = parseArgs(argc, argv, options, error);
  if (parse_result != 0) {
    std::cerr << error << "\n";
    printUsage(argv[0]);
    return 2;
  }
  if (options.help) {
    printUsage(argv[0]);
    return 0;
  }

  if (options.config.csv_path.empty()) {
    options.config.csv_path =
        (std::filesystem::path("logs") / "go2_native_motion" /
         timestampUtc() / "native_motion.csv")
            .string();
  }
  if (!go2_native_motion::validateConfig(options.config, &error)) {
    std::cerr << "参数错误: " << error << "\n";
    return 2;
  }

  const std::filesystem::path csv_path(options.config.csv_path);
  std::error_code filesystem_error;
  if (csv_path.has_parent_path()) {
    std::filesystem::create_directories(csv_path.parent_path(),
                                        filesystem_error);
  }
  if (filesystem_error) {
    std::cerr << "无法创建 CSV 目录: " << filesystem_error.message() << "\n";
    return 2;
  }
  std::ofstream csv(options.config.csv_path,
                    std::ios::out | std::ios::trunc);
  if (!csv.is_open()) {
    std::cerr << "无法打开 CSV: " << options.config.csv_path << "\n";
    return 2;
  }
  go2_native_motion::writeCsvHeader(csv);

  installSignalHandlers();

  std::cout << "Go2 原生交互运动工具\n"
            << "只使用 SportClient Move/StopMove 和 rt/sportmodestate；"
               "不创建 LowCmd、不释放/切换控制权、"
               "不启动控制器/策略/相机/定位。\n"
            << "DDS domain=" << options.domain
            << ", interface=" << options.interface_name << "\n"
            << "CSV: " << options.config.csv_path << "\n"
            << "普通命令按 Enter 执行；Ctrl+C 立即中断任务并尝试 StopMove。\n";

  try {
    unitree::robot::ChannelFactory::Instance()->Init(options.domain,
                                                     options.interface_name);
  } catch (const std::exception& exception) {
    std::cerr << "DDS 初始化失败: " << exception.what()
              << "\n未发送运动指令。\n";
    return 2;
  }

  int exit_code = 2;
  CommandQueue command_queue;
  go2_native_motion::SessionState session_state;
  std::thread reader;
  try {
    {
      SportBackend backend;
      g_reader_stop.store(false);
      reader = std::thread(readerLoop, std::ref(command_queue));

      go2_native_motion::Runtime runtime;
      runtime.now_s = [] {
        return std::chrono::duration<double>(
                   Clock::now().time_since_epoch())
            .count();
      };
      runtime.sleep_s = [](double seconds) {
        if (seconds > 0.0) {
          std::this_thread::sleep_for(
              std::chrono::duration<double>(seconds));
        }
      };
      runtime.interrupted = [] { return g_interrupted != 0; };
      runtime.poll_command = [&command_queue](
                                 go2_native_motion::Command& command) {
        return command_queue.pop(command);
      };

      exit_code = go2_native_motion::runInteractiveSession(
          options.config, backend, runtime, session_state, std::cout, csv);
    }
    g_reader_stop.store(true);
    if (reader.joinable()) {
      reader.join();
    }
    unitree::robot::ChannelFactory::Instance()->Release();
  } catch (const std::exception& exception) {
    std::cerr << "运行异常: " << exception.what() << "\n";
    g_reader_stop.store(true);
    if (reader.joinable()) {
      reader.join();
    }
    unitree::robot::ChannelFactory::Instance()->Release();
    if (session_state.stop_failed) {
      std::cerr << "!! 运行异常；停止失败状态未解除，退出码 3。"
                   "请现场确认机器人已停止。\n";
      exit_code = 3;
    } else {
      exit_code = 2;
    }
  }

  csv.flush();
  std::cout << "CSV: " << options.config.csv_path << "\n";
  return exit_code;
}
