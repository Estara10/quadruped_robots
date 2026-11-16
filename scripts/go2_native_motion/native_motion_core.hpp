#ifndef GO2_NATIVE_MOTION_CORE_HPP
#define GO2_NATIVE_MOTION_CORE_HPP

#include <array>
#include <cstdint>
#include <functional>
#include <iosfwd>
#include <string>

namespace go2_native_motion {

inline constexpr double kPi = 3.14159265358979323846;

// 首次试验暂定值，不是已验证的机器人安全保证。
inline constexpr double kMinVxMps = 0.05;
inline constexpr double kMaxVxMps = 0.50;
inline constexpr double kMinDurationS = 1.0;
inline constexpr double kMaxDurationS = 30.0;
inline constexpr double kMovePeriodS = 0.10;
inline constexpr double kMaxStateAgeS = 0.30;
inline constexpr double kStartupStateWaitS = 10.0;
inline constexpr double kSdkCallTimeoutS = 1.0;
inline constexpr double kCommandPollPeriodS = 0.05;

// goto/yaw 首次试验初值，需现场验证。
inline constexpr double kMaxGotoDistanceM = 3.0;
inline constexpr double kPositionToleranceM = 0.05;
inline constexpr double kYawToleranceRad = 5.0 * kPi / 180.0;
inline constexpr double kPositionGain = 0.8;
inline constexpr double kYawGain = 1.5;
inline constexpr double kMinGotoSpeedMps = 0.05;
inline constexpr double kMaxGotoSpeedMps = 0.50;
inline constexpr double kDefaultGotoMaxSpeedMps = 0.30;
inline constexpr double kDefaultGotoApproachSpeedMps = 0.30;
inline constexpr double kMaxYawRateRadps = 0.50;
inline constexpr double kGotoTimeoutS = 30.0;
inline constexpr double kYawTimeoutS = 15.0;

struct Config {
  double vx_mps = 0.10;
  double vy_mps = 0.0;
  double vyaw_radps = 0.0;
  double duration_s = 2.0;
  double goto_max_speed_mps = kDefaultGotoMaxSpeedMps;
  double goto_approach_speed_mps = kDefaultGotoApproachSpeedMps;
  std::string csv_path;
};

struct StateSnapshot {
  bool received = false;
  bool finite = false;
  uint64_t seq = 0;
  double age_s = 0.0;
  std::array<double, 3> position{};
  std::array<double, 3> velocity{};
  double yaw_rad = 0.0;
};

enum class CommandType {
  Invalid,
  Help,
  Status,
  Origin,
  Straight,
  Goto,
  Yaw,
  Stop,
  Quit,
};

struct Command {
  CommandType type = CommandType::Invalid;
  std::string raw;
  double x = 0.0;
  double y = 0.0;
  double angle_deg = 0.0;
};

struct Origin {
  bool set = false;
  double native_x = 0.0;
  double native_y = 0.0;
  double native_yaw = 0.0;
};

enum class TaskType {
  None,
  Straight,
  Goto,
  Yaw,
};

struct Task {
  TaskType type = TaskType::None;
  double start_s = 0.0;
  double timeout_s = 0.0;
  double straight_duration_s = 0.0;
  double straight_vx_mps = 0.0;
  double target_local_x = 0.0;
  double target_local_y = 0.0;
  double target_local_yaw_deg = 0.0;
  double target_native_x = 0.0;
  double target_native_y = 0.0;
  double target_native_yaw = 0.0;
};

struct LocalPose {
  double x = 0.0;
  double y = 0.0;
  double heading = 0.0;
};

struct BodyError {
  double forward = 0.0;
  double left = 0.0;
  double distance = 0.0;
};

enum class TaskEndReason {
  Arrived,
  Completed,
  Timeout,
  Stopped,
  Interrupted,
  StateStale,
  MoveApiError,
  Quit,
};

// 替代接口：实机实现调用 SDK，离线自检实现不初始化 DDS。
class Backend {
 public:
  virtual ~Backend() = default;
  virtual StateSnapshot snapshot() = 0;
  virtual int32_t move(float vx, float vy, float vyaw) = 0;
  virtual int32_t stopMove() = 0;
  virtual std::string lastError() = 0;
};

struct Runtime {
  std::function<double()> now_s;
  std::function<void(double)> sleep_s;
  std::function<bool()> interrupted;
  std::function<bool(Command&)> poll_command;
};

struct SessionState {
  bool stop_failed = false;
};

bool validateConfig(const Config& config, std::string* error);
bool isFresh(const StateSnapshot& state);

Command parseCommand(const std::string& line);
const char* commandName(CommandType type);
const char* taskName(TaskType type);
const char* taskEndReasonName(TaskEndReason reason);

double wrapPi(double angle);
LocalPose toLocalPose(const StateSnapshot& state, const Origin& origin);
void localToNative(const Origin& origin, double local_x, double local_y,
                   double* native_x, double* native_y);
BodyError computeBodyError(const StateSnapshot& state, double target_native_x,
                           double target_native_y);

void writeCsvHeader(std::ostream& csv);
int runInteractiveSession(const Config& config, Backend& backend,
                          const Runtime& runtime, SessionState& session_state,
                          std::ostream& log, std::ostream& csv);

}  // namespace go2_native_motion

#endif
