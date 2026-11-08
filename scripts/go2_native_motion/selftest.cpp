#include "native_motion_core.hpp"

#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace g = go2_native_motion;

namespace {

int failures = 0;

void check(bool condition, const std::string& name) {
  if (!condition) {
    std::cerr << "FAIL " << name << "\n";
    ++failures;
  }
}

void checkNear(double actual, double expected, const std::string& name,
               double tolerance = 1e-6) {
  check(std::abs(actual - expected) <= tolerance, name);
}

std::size_t countOccurrences(const std::string& text,
                             const std::string& needle) {
  std::size_t count = 0;
  std::size_t position = 0;
  while ((position = text.find(needle, position)) != std::string::npos) {
    ++count;
    position += needle.size();
  }
  return count;
}

bool noCmdAfter(const std::string& csv, const std::string& marker) {
  const std::size_t position = csv.find(marker);
  if (position == std::string::npos) {
    return false;
  }
  return csv.find(",cmd,", position) == std::string::npos;
}

class FakeBackend final : public g::Backend {
 public:
  FakeBackend() {
    state.received = true;
    state.finite = true;
    state.seq = 1;
    state.age_s = 0.01;
    state.position = {0.0, 0.0, 0.30};
    state.yaw_rad = 0.0;
  }

  g::StateSnapshot snapshot() override { return state; }

  int32_t move(float vx, float vy, float vyaw) override {
    ++move_calls;
    last_vx = vx;
    last_vy = vy;
    last_vyaw = vyaw;
    if (stale_after_moves >= 0 && move_calls >= stale_after_moves) {
      state.age_s = g::kMaxStateAgeS + 0.2;
    }
    if (move_throws) {
      throw std::runtime_error("fake Move exception");
    }
    if (move_fail_at >= 0 && move_calls >= move_fail_at) {
      error = "fake Move return " + std::to_string(move_return);
      return move_return;
    }
    return 0;
  }

  int32_t stopMove() override {
    ++stop_calls;
    if (stop_throws) {
      throw std::runtime_error("fake StopMove exception");
    }
    return stop_return;
  }

  std::string lastError() override { return error; }

  g::StateSnapshot state;
  int32_t move_return = 0;
  int32_t stop_return = 0;
  int move_fail_at = -1;
  int stale_after_moves = -1;
  bool move_throws = false;
  bool stop_throws = false;
  int move_calls = 0;
  int stop_calls = 0;
  double last_vx = 0.0;
  double last_vy = 0.0;
  double last_vyaw = 0.0;
  std::string error;
};

struct FakeClock {
  double now = 0.0;
};

class ScriptedCommands {
 public:
  explicit ScriptedCommands(FakeClock& clock) : clock_(clock) {}

  void add(double at_s, const std::string& line,
           std::function<void()> on_deliver = {}) {
    commands_.push_back({at_s, line, std::move(on_deliver)});
  }

  bool poll(g::Command& command) {
    if (index_ >= commands_.size()) {
      return false;
    }
    const Entry& entry = commands_[index_];
    if (clock_.now + 1e-9 < entry.at_s) {
      return false;
    }
    if (entry.on_deliver) {
      entry.on_deliver();
    }
    command = g::parseCommand(entry.line);
    ++index_;
    return true;
  }

 private:
  struct Entry {
    double at_s;
    std::string line;
    std::function<void()> on_deliver;
  };

  FakeClock& clock_;
  std::vector<Entry> commands_;
  std::size_t index_ = 0;
};

struct Case {
  FakeBackend backend;
  FakeClock clock;
  ScriptedCommands script{clock};
  g::Config config;
  g::SessionState session_state;
  std::ostringstream log;
  std::ostringstream csv;
  int exit_code = 0;
  bool interrupted = false;
  double interrupt_at = std::numeric_limits<double>::infinity();
};

void prepare(Case& test_case) {
  test_case.config.csv_path = "selftest.csv";
  test_case.config.vx_mps = 0.10;
  test_case.config.vy_mps = 0.0;
  test_case.config.vyaw_radps = 0.0;
  test_case.config.duration_s = 2.0;
  g::writeCsvHeader(test_case.csv);
}

int run(Case& test_case) {
  g::Runtime runtime;
  runtime.now_s = [&test_case] { return test_case.clock.now; };
  runtime.sleep_s = [&test_case](double seconds) {
    test_case.clock.now += seconds;
  };
  runtime.interrupted = [&test_case] {
    return test_case.interrupted ||
           test_case.clock.now + 1e-9 >= test_case.interrupt_at;
  };
  runtime.poll_command = [&test_case](g::Command& command) {
    return test_case.script.poll(command);
  };
  test_case.exit_code = g::runInteractiveSession(
      test_case.config, test_case.backend, runtime, test_case.session_state,
      test_case.log, test_case.csv);
  return test_case.exit_code;
}

void testTransforms() {
  const int before = failures;
  g::Origin origin;
  origin.set = true;
  origin.native_x = 1.0;
  origin.native_y = 2.0;
  origin.native_yaw = g::kPi / 2.0;

  double native_x = 0.0;
  double native_y = 0.0;
  g::localToNative(origin, 1.0, 0.0, &native_x, &native_y);
  checkNear(native_x, 1.0, "local forward x at yaw 90");
  checkNear(native_y, 3.0, "local forward y at yaw 90");
  g::localToNative(origin, 0.0, 1.0, &native_x, &native_y);
  checkNear(native_x, 0.0, "local left x at yaw 90");
  checkNear(native_y, 2.0, "local left y at yaw 90");

  g::StateSnapshot state;
  state.received = true;
  state.finite = true;
  state.position = {1.0, 2.0, 0.30};
  state.yaw_rad = g::kPi / 2.0;
  g::LocalPose local = g::toLocalPose(state, origin);
  checkNear(local.x, 0.0, "local pose origin x");
  checkNear(local.y, 0.0, "local pose origin y");
  checkNear(local.heading, 0.0, "local pose origin heading");

  state.position = {1.0, 3.0, 0.30};
  local = g::toLocalPose(state, origin);
  checkNear(local.x, 1.0, "local pose forward x");
  checkNear(local.y, 0.0, "local pose forward y");

  state.position = {0.0, 2.0, 0.30};
  local = g::toLocalPose(state, origin);
  checkNear(local.x, 0.0, "local pose left x");
  checkNear(local.y, 1.0, "local pose left y");

  state.position = {1.0, 2.0, 0.30};
  g::BodyError error = g::computeBodyError(state, 1.0, 3.0);
  checkNear(error.forward, 1.0, "body error forward forward");
  checkNear(error.left, 0.0, "body error forward left");
  checkNear(error.distance, 1.0, "body error forward distance");

  error = g::computeBodyError(state, 0.0, 2.0);
  checkNear(error.forward, 0.0, "body error left forward");
  checkNear(error.left, 1.0, "body error left left");
  checkNear(error.distance, 1.0, "body error left distance");

  std::cout << (failures == before ? "PASS" : "FAIL") << " transforms\n";
}

void testWrapPi() {
  const int before = failures;
  const double deg = g::kPi / 180.0;
  checkNear(g::wrapPi(179.0 * deg - (-179.0 * deg)), -2.0 * deg,
            "wrap 179 to -179 is -2 deg");
  checkNear(g::wrapPi(-179.0 * deg - 179.0 * deg), 2.0 * deg,
            "wrap -179 to 179 is +2 deg");
  checkNear(g::wrapPi(181.0 * deg), -179.0 * deg, "wrap 181");
  checkNear(g::wrapPi(-181.0 * deg), 179.0 * deg, "wrap -181");
  checkNear(g::wrapPi(g::kPi), g::kPi, "wrap +pi");
  checkNear(g::wrapPi(-g::kPi), g::kPi, "wrap -pi deterministic +pi");
  checkNear(g::wrapPi(0.1), 0.1, "wrap small angle");
  std::cout << (failures == before ? "PASS" : "FAIL") << " wrap_pi\n";
}

void testParseCommand() {
  const int before = failures;
  check(g::parseCommand("status").type == g::CommandType::Status,
        "parse status");
  check(g::parseCommand("origin").type == g::CommandType::Origin,
        "parse origin");
  check(g::parseCommand("straight").type == g::CommandType::Straight,
        "parse straight");
  check(g::parseCommand("stop").type == g::CommandType::Stop, "parse stop");
  check(g::parseCommand("quit").type == g::CommandType::Quit, "parse quit");
  check(g::parseCommand("help").type == g::CommandType::Help, "parse help");

  g::Command command = g::parseCommand("  GOTO   1.5   -2.5 ");
  check(command.type == g::CommandType::Goto, "parse goto type");
  checkNear(command.x, 1.5, "parse goto x");
  checkNear(command.y, -2.5, "parse goto y");
  command = g::parseCommand("yaw 90");
  check(command.type == g::CommandType::Yaw, "parse yaw type");
  checkNear(command.angle_deg, 90.0, "parse yaw angle");

  check(g::parseCommand("goto 1").type == g::CommandType::Invalid,
        "parse goto missing arg");
  check(g::parseCommand("goto a b").type == g::CommandType::Invalid,
        "parse goto bad number");
  check(g::parseCommand("yaw a").type == g::CommandType::Invalid,
        "parse yaw bad number");
  check(g::parseCommand("unknown").type == g::CommandType::Invalid,
        "parse unknown");
  check(g::parseCommand("").type == g::CommandType::Invalid, "parse empty");
  std::cout << (failures == before ? "PASS" : "FAIL") << " parse_command\n";
}

void testStraightPreserved() {
  const int before = failures;
  Case test_case;
  prepare(test_case);
  test_case.config.vx_mps = 0.10;
  test_case.config.duration_s = 1.0;
  test_case.script.add(0.0, "origin");
  test_case.script.add(0.1, "straight");
  test_case.script.add(2.0, "quit");
  run(test_case);
  const std::string csv = test_case.csv.str();
  check(test_case.backend.move_calls == 10, "straight 10 Move calls");
  check(test_case.backend.stop_calls == 2, "straight stop + quit stop");
  checkNear(test_case.backend.last_vx, 0.10, "straight last vx");
  checkNear(test_case.backend.last_vy, 0.0, "straight last vy");
  checkNear(test_case.backend.last_vyaw, 0.0, "straight last vyaw");
  check(test_case.exit_code == 0, "straight quit exit 0");
  check(csv.find(",task_start,") != std::string::npos,
        "straight CSV task_start");
  check(csv.find("COMPLETED") != std::string::npos,
        "straight CSV completed");
  check(csv.find("QUIT") != std::string::npos, "straight CSV quit");
  check(noCmdAfter(csv, "stop_request"), "straight no cmd after stop");
  std::cout << (failures == before ? "PASS" : "FAIL") << " straight_preserved\n";
}

void testGotoForwardAndStop() {
  const int before = failures;
  Case test_case;
  prepare(test_case);
  test_case.backend.state.position = {1.0, 2.0, 0.30};
  test_case.backend.state.yaw_rad = g::kPi / 2.0;
  test_case.script.add(0.0, "origin");
  test_case.script.add(0.1, "goto 1 0");
  test_case.script.add(0.6, "stop");
  test_case.script.add(1.0, "quit");
  run(test_case);
  const std::string csv = test_case.csv.str();
  check(test_case.backend.move_calls == 5, "goto stop after 5 Move calls");
  checkNear(test_case.backend.last_vx, 0.30, "goto forward vx");
  checkNear(test_case.backend.last_vy, 0.0, "goto forward vy");
  checkNear(test_case.backend.last_vyaw, 0.0, "goto forward vyaw");
  check(csv.find("STOPPED") != std::string::npos, "goto CSV stopped");
  check(noCmdAfter(csv, "stop_request"), "goto no cmd after stop");
  std::cout << (failures == before ? "PASS" : "FAIL") << " goto_forward_stop\n";
}

void testGotoLeft() {
  const int before = failures;
  Case test_case;
  prepare(test_case);
  test_case.script.add(0.0, "origin");
  test_case.script.add(0.1, "goto 0 1");
  test_case.script.add(0.6, "stop");
  test_case.script.add(1.0, "quit");
  run(test_case);
  checkNear(test_case.backend.last_vx, 0.0, "goto left vx");
  checkNear(test_case.backend.last_vy, 0.30, "goto left vy");
  checkNear(test_case.backend.last_vyaw, 0.0, "goto left vyaw");
  std::cout << (failures == before ? "PASS" : "FAIL") << " goto_left\n";
}

void testGotoApproachFloor() {
  const int before = failures;
  Case test_case;
  prepare(test_case);
  test_case.script.add(0.0, "origin");
  test_case.script.add(0.1, "goto 0.2 0");
  test_case.script.add(0.6, "stop");
  test_case.script.add(1.0, "quit");
  run(test_case);
  checkNear(test_case.backend.last_vx, 0.30,
            "goto approach floor keeps vx at 0.30");
  checkNear(test_case.backend.last_vy, 0.0, "goto approach floor vy");
  std::cout << (failures == before ? "PASS" : "FAIL")
            << " goto_approach_floor\n";
}

void testYawWrap() {
  const int before = failures;
  Case test_case;
  prepare(test_case);
  test_case.script.add(0.0, "origin");
  test_case.script.add(0.1, "yaw 190", [&test_case] {
    test_case.backend.state.yaw_rad = 170.0 * g::kPi / 180.0;
  });
  test_case.script.add(0.6, "stop");
  test_case.script.add(1.0, "quit");
  run(test_case);
  check(test_case.backend.last_vyaw > 0.0, "yaw wrap turns left");
  checkNear(test_case.backend.last_vx, 0.0, "yaw wrap vx zero");
  checkNear(test_case.backend.last_vy, 0.0, "yaw wrap vy zero");
  std::cout << (failures == before ? "PASS" : "FAIL") << " yaw_wrap\n";
}

void testArrivalStops() {
  const int before = failures;
  Case test_case;
  prepare(test_case);
  test_case.script.add(0.0, "origin");
  test_case.script.add(0.1, "goto 1 0", [&test_case] {
    test_case.backend.state.position = {0.94, 0.0, 0.30};
  });
  test_case.script.add(0.2, "status", [&test_case] {
    test_case.backend.state.position = {0.96, 0.0, 0.30};
  });
  test_case.script.add(0.5, "quit");
  run(test_case);
  const std::string csv = test_case.csv.str();
  check(test_case.backend.move_calls == 1,
        "6 cm remaining continues; 4 cm remaining stops");
  check(test_case.backend.stop_calls == 2, "arrival stop + quit stop");
  check(csv.find("ARRIVED") != std::string::npos, "arrival CSV ARRIVED");
  check(countOccurrences(csv, ",cmd,") == 1, "arrival one cmd row");
  check(noCmdAfter(csv, "stop_request"), "arrival no cmd after stop");
  std::cout << (failures == before ? "PASS" : "FAIL") << " arrival_stop\n";
}

void testTimeoutStops() {
  const int before = failures;
  Case test_case;
  prepare(test_case);
  test_case.script.add(0.0, "origin");
  test_case.script.add(0.1, "goto 1 0");
  test_case.script.add(31.0, "quit");
  run(test_case);
  const std::string csv = test_case.csv.str();
  check(test_case.backend.move_calls > 250, "timeout many Move calls");
  check(test_case.backend.stop_calls == 2, "timeout stop + quit stop");
  check(csv.find("TIMEOUT") != std::string::npos, "timeout CSV TIMEOUT");
  std::cout << (failures == before ? "PASS" : "FAIL") << " timeout_stop\n";
}

void testNewMotionRejected() {
  const int before = failures;
  Case test_case;
  prepare(test_case);
  test_case.script.add(0.0, "origin");
  test_case.script.add(0.1, "goto 1 0");
  test_case.script.add(0.5, "yaw 90");
  test_case.script.add(0.6, "stop");
  test_case.script.add(1.0, "quit");
  run(test_case);
  const std::string csv = test_case.csv.str();
  check(csv.find("task_active") != std::string::npos,
        "new motion rejected while task active");
  check(countOccurrences(csv, ",task_start,") == 1,
        "only one task_start");
  checkNear(test_case.backend.last_vyaw, 0.0,
            "rejected yaw did not change command");
  std::cout << (failures == before ? "PASS" : "FAIL") << " new_motion_rejected\n";
}

void testOriginRules() {
  const int before = failures;
  Case test_case;
  prepare(test_case);
  test_case.script.add(0.0, "goto 1 0");
  test_case.script.add(0.1, "origin");
  test_case.script.add(0.2, "origin");
  test_case.script.add(0.5, "quit");
  run(test_case);
  const std::string csv = test_case.csv.str();
  check(test_case.backend.move_calls == 0, "origin rules no Move");
  check(csv.find("origin_unset") != std::string::npos,
        "origin rules unset rejected");
  check(csv.find("origin_already_set") != std::string::npos,
        "origin rules re-zero rejected");
  check(countOccurrences(csv, "origin_set") == 1,
        "origin rules one origin_set");
  std::cout << (failures == before ? "PASS" : "FAIL") << " origin_rules\n";
}

void testMoveErrorStops() {
  const int before = failures;
  Case test_case;
  prepare(test_case);
  test_case.backend.move_fail_at = 1;
  test_case.backend.move_return = 2311;
  test_case.script.add(0.0, "origin");
  test_case.script.add(0.1, "straight");
  test_case.script.add(1.0, "quit");
  run(test_case);
  const std::string csv = test_case.csv.str();
  check(test_case.backend.move_calls == 1, "move error one Move");
  check(test_case.backend.stop_calls == 2, "move error stop + quit stop");
  check(csv.find("MOVE_API_ERROR") != std::string::npos,
        "move error CSV reason");
  check(csv.find("2311") != std::string::npos, "move error CSV return");
  std::cout << (failures == before ? "PASS" : "FAIL") << " move_error_stop\n";
}

void testStateStaleStops() {
  const int before = failures;
  Case test_case;
  prepare(test_case);
  test_case.backend.stale_after_moves = 1;
  test_case.script.add(0.0, "origin");
  test_case.script.add(0.1, "goto 1 0");
  test_case.script.add(1.0, "quit");
  run(test_case);
  const std::string csv = test_case.csv.str();
  check(csv.find("STATE_STALE") != std::string::npos,
        "stale CSV reason");
  check(test_case.backend.stop_calls == 2, "stale stop + quit stop");
  std::cout << (failures == before ? "PASS" : "FAIL") << " state_stale_stop\n";
}

void testStopFailureExit() {
  const int before = failures;
  Case test_case;
  prepare(test_case);
  test_case.config.duration_s = 1.0;
  test_case.backend.stop_return = 3104;
  test_case.script.add(0.0, "origin");
  test_case.script.add(0.1, "straight");
  test_case.script.add(3.0, "quit");
  run(test_case);
  const std::string csv = test_case.csv.str();
  check(csv.find("COMPLETED") != std::string::npos,
        "stop failure completed task");
  check(csv.find("3104") != std::string::npos, "stop failure CSV return");
  check(test_case.exit_code == 3, "stop failure exit 3");
  std::cout << (failures == before ? "PASS" : "FAIL") << " stop_failure\n";
}

void testStopFailureLatch() {
  const int before = failures;
  Case test_case;
  prepare(test_case);
  test_case.config.duration_s = 1.0;
  test_case.backend.stop_return = 3104;
  test_case.script.add(0.0, "origin");
  test_case.script.add(0.1, "straight");
  test_case.script.add(1.5, "status");
  test_case.script.add(1.6, "goto 1 0");
  test_case.script.add(1.7, "yaw 90");
  test_case.script.add(1.8, "stop", [&test_case] {
    test_case.backend.stop_return = 0;
  });
  test_case.script.add(1.9, "straight");
  test_case.script.add(2.2, "goto 1 0");
  test_case.script.add(2.5, "quit");
  run(test_case);
  const std::string csv = test_case.csv.str();
  const std::string log = test_case.log.str();
  check(test_case.backend.move_calls == 10,
        "stop failure latch no extra Move");
  check(test_case.session_state.stop_failed,
        "stop failure latch state remains");
  check(test_case.exit_code == 3, "stop failure latch exit 3");
  check(test_case.backend.stop_calls == 3,
        "stop failure latch stop attempts");
  check(csv.find("terminal_rejected,stop_failed") != std::string::npos,
        "stop failure latch CSV rejection");
  check(csv.find(",status,") != std::string::npos,
        "stop failure latch status allowed");
  check(countOccurrences(csv, ",task_start,") == 1,
        "stop failure latch only straight task");
  check(csv.find("QUIT_STOP_FAILED") != std::string::npos,
        "stop failure latch end event");
  check(log.find("只允许 status、stop、quit") != std::string::npos,
        "stop failure latch guidance");
  std::cout << (failures == before ? "PASS" : "FAIL")
            << " stop_failure_latch\n";
}

void testStopExceptionLatch() {
  const int before = failures;
  Case test_case;
  prepare(test_case);
  test_case.config.duration_s = 1.0;
  test_case.backend.stop_throws = true;
  test_case.script.add(0.0, "origin");
  test_case.script.add(0.1, "straight");
  test_case.script.add(1.5, "goto 1 0");
  test_case.script.add(2.0, "quit");
  run(test_case);
  const std::string csv = test_case.csv.str();
  check(test_case.backend.move_calls == 10,
        "stop exception latch no extra Move");
  check(test_case.session_state.stop_failed,
        "stop exception latch state remains");
  check(test_case.exit_code == 3, "stop exception latch exit 3");
  check(test_case.backend.stop_calls == 2,
        "stop exception latch stop attempts");
  check(csv.find("fake StopMove exception") != std::string::npos,
        "stop exception latch CSV exception");
  check(csv.find("terminal_rejected,stop_failed") != std::string::npos,
        "stop exception latch CSV rejection");
  check(csv.find("QUIT_STOP_FAILED") != std::string::npos,
        "stop exception latch end event");
  std::cout << (failures == before ? "PASS" : "FAIL")
            << " stop_exception_latch\n";
}

void testCtrlCStops() {
  const int before = failures;
  Case test_case;
  prepare(test_case);
  test_case.interrupt_at = 0.6;
  test_case.script.add(0.0, "origin");
  test_case.script.add(0.1, "goto 1 0");
  run(test_case);
  const std::string csv = test_case.csv.str();
  check(test_case.backend.stop_calls == 1, "ctrl-c one StopMove");
  check(csv.find("INTERRUPTED") != std::string::npos,
        "ctrl-c CSV INTERRUPTED");
  check(test_case.exit_code == 0, "ctrl-c exit 0");
  check(noCmdAfter(csv, "stop_request"), "ctrl-c no cmd after stop");
  std::cout << (failures == before ? "PASS" : "FAIL") << " ctrl_c_stop\n";
}

void testQuitDuringMotion() {
  const int before = failures;
  Case test_case;
  prepare(test_case);
  test_case.script.add(0.0, "origin");
  test_case.script.add(0.1, "goto 1 0");
  test_case.script.add(0.6, "quit");
  run(test_case);
  const std::string csv = test_case.csv.str();
  check(test_case.backend.move_calls == 5, "quit during motion 5 Move calls");
  check(test_case.backend.stop_calls == 1, "quit during motion one StopMove");
  check(csv.find("QUIT") != std::string::npos, "quit during motion CSV QUIT");
  check(test_case.exit_code == 0, "quit during motion exit 0");
  check(noCmdAfter(csv, "stop_request"),
        "quit during motion no cmd after stop");
  std::cout << (failures == before ? "PASS" : "FAIL") << " quit_during_motion\n";
}

void testStartupTimeout() {
  const int before = failures;
  Case test_case;
  prepare(test_case);
  test_case.backend.state.received = false;
  test_case.backend.state.finite = false;
  run(test_case);
  const std::string csv = test_case.csv.str();
  check(test_case.exit_code == 1, "startup timeout exit 1");
  check(test_case.backend.move_calls == 0, "startup timeout no Move");
  check(test_case.backend.stop_calls == 0, "startup timeout no StopMove");
  check(csv.find("STARTUP_TIMEOUT") != std::string::npos,
        "startup timeout CSV reason");
  std::cout << (failures == before ? "PASS" : "FAIL") << " startup_timeout\n";
}

void testConfigValidation() {
  const int before = failures;
  std::string error;
  g::Config config;
  config.csv_path = "x.csv";

  config.vx_mps = 0.50;
  check(g::validateConfig(config, &error), "accept vx max 0.50");
  config.vx_mps = 0.5001;
  check(!g::validateConfig(config, &error), "reject vx above 0.50");
  config.vx_mps = 0.0;
  check(!g::validateConfig(config, &error), "reject vx below cap");
  config.vx_mps = std::numeric_limits<double>::quiet_NaN();
  check(!g::validateConfig(config, &error), "reject NaN vx");
  config.vx_mps = 0.10;

  config.duration_s = 30.0;
  check(g::validateConfig(config, &error), "accept duration max 30");
  config.duration_s = 30.01;
  check(!g::validateConfig(config, &error), "reject duration above 30");
  config.duration_s = 0.5;
  check(!g::validateConfig(config, &error), "reject short duration");
  config.duration_s = 2.0;

  config.goto_max_speed_mps = 0.30;
  config.goto_approach_speed_mps = 0.30;
  check(g::validateConfig(config, &error), "accept goto 0.30/0.30");
  config.goto_max_speed_mps = 0.50;
  check(g::validateConfig(config, &error), "accept goto max 0.50");
  config.goto_max_speed_mps = 0.5001;
  check(!g::validateConfig(config, &error), "reject goto max above 0.50");
  config.goto_max_speed_mps = 0.30;
  config.goto_approach_speed_mps = 0.31;
  check(!g::validateConfig(config, &error),
        "reject goto approach above max");
  config.goto_approach_speed_mps = 0.0;
  check(!g::validateConfig(config, &error),
        "reject goto approach below min");
  config.goto_approach_speed_mps = 0.30;

  config.vy_mps = 0.1;
  check(!g::validateConfig(config, &error), "reject lateral velocity");
  config.vy_mps = 0.0;
  config.vyaw_radps = 0.1;
  check(!g::validateConfig(config, &error), "reject yaw velocity");
  config.vyaw_radps = 0.0;

  config.csv_path.clear();
  check(!g::validateConfig(config, &error), "reject empty CSV path");
  config.csv_path = "x.csv";

  check(g::validateConfig(config, &error), "accept default-range config");

  config.vx_mps = 0.20;
  config.duration_s = 6.0;
  check(g::validateConfig(config, &error), "accept field 0.20/6 config");

  config.vx_mps = 0.05;
  config.duration_s = 1.0;
  check(g::validateConfig(config, &error), "accept minimum trial config");
  config.vx_mps = 0.50;
  config.duration_s = 30.0;
  check(g::validateConfig(config, &error), "accept maximum trial config");
  std::cout << (failures == before ? "PASS" : "FAIL") << " config\n";
}

}  // namespace

int main() {
  testTransforms();
  testWrapPi();
  testParseCommand();
  testStraightPreserved();
  testGotoForwardAndStop();
  testGotoLeft();
  testGotoApproachFloor();
  testYawWrap();
  testArrivalStops();
  testTimeoutStops();
  testNewMotionRejected();
  testOriginRules();
  testMoveErrorStops();
  testStateStaleStops();
  testStopFailureExit();
  testStopFailureLatch();
  testStopExceptionLatch();
  testCtrlCStops();
  testQuitDuringMotion();
  testStartupTimeout();
  testConfigValidation();

  if (failures != 0) {
    std::cerr << "selftest failures: " << failures << "\n";
    return 1;
  }
  std::cout << "all selftests passed; no DDS initialized\n";
  return 0;
}
