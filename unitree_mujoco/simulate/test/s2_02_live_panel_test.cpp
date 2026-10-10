#include <cassert>
#include <cstring>
#include <fstream>
#include <string>
#include <vector>

#include "abs_mujoco_live_panel.h"

namespace panel = abs_mujoco_live_panel;
namespace frame = abs_rt_frame;

int main(int argc, char** argv) {
  const uint64_t now = 10'000'000'000ULL;
  frame::RuntimeFrame live{};
  live.header.magic = frame::kMagic;
  live.header.version = frame::kVersion;
  live.header.sequence = 2;
  live.header.monotonic_ns = now - 20'000'000ULL;
  live.session_id = 123;
  live.rl_step = 8;
  live.source = frame::kSourceAuthoritativeRuntime;
  live.controller_active = 1;
  live.rl_entered = 1;
  live.rl_active = 1;
  live.policy_state = frame::kPolicyAgile;
  live.sim_clock_status = frame::kSimClockFresh;
  live.sim_clock_valid = 1;
  live.sim_clock_sequence = 44;
  live.sim_clock_monotonic_ns = now - 25'000'000ULL;
  live.sim_time_s = 12.5;
  live.entry_threshold = -0.05f;
  live.exit_threshold = -0.05f;
  live.ra_value = -0.2f;
  live.lin_vel[0] = 0.3f;
  live.lin_vel[1] = 0.4f;
  live.lin_vel[2] = 99.0f;
  live.world_pose[0] = 1.0f;
  live.world_pose[1] = 2.0f;
  live.world_pose[2] = 0.0f;

  auto current = panel::classify(&live, sizeof(live), now);
  assert(current.status == panel::Status::kLive);
  const std::string live_text = panel::render(current);
  assert(live_text.find("Body XY speed: 0.50 m/s") != std::string::npos);
  assert(live_text.find("Policy: Agile") != std::string::npos);
  assert(live_text.find("RA: -0.200 | enter -0.050 / exit -0.050") != std::string::npos);
  assert(live_text.find("Goal distance: N/A") != std::string::npos);
  assert(live_text.find("Sim time: 12.50 s") != std::string::npos);

  auto no_sim_clock_frame = live;
  no_sim_clock_frame.sim_clock_valid = 0;
  no_sim_clock_frame.sim_clock_status = frame::kSimClockUnavailable;
  const std::string no_sim_clock_text = panel::render(
      panel::classify(&no_sim_clock_frame, sizeof(no_sim_clock_frame), now));
  assert(no_sim_clock_text.find("Sim time: N/A") != std::string::npos);

  auto missing = panel::classify(nullptr, 0, now);
  assert(missing.status == panel::Status::kMissing);
  assert(panel::render(missing).find("RA:") == std::string::npos);

  auto stale = panel::classify(&live, sizeof(live), now + 600'000'000ULL);
  assert(stale.status == panel::Status::kStale);
  assert(panel::render(stale).find("RA:") == std::string::npos);
  assert(panel::render(stale).find("Last frame age") != std::string::npos);

  auto wrong_version_frame = live;
  wrong_version_frame.header.version += 1;
  assert(panel::classify(&wrong_version_frame, sizeof(wrong_version_frame), now).status ==
         panel::Status::kInvalid);
  auto wrong_source_frame = live;
  wrong_source_frame.source = frame::kSourceSyntheticTest;
  assert(panel::classify(&wrong_source_frame, sizeof(wrong_source_frame), now).status ==
         panel::Status::kInvalid);
  assert(panel::classify(&live, sizeof(live) - 1, now).status == panel::Status::kInvalid);

  auto recovery_frame = live;
  recovery_frame.session_id = 456;  // a new session is read as its own snapshot
  recovery_frame.policy_state = frame::kPolicyRecovery;
  recovery_frame.mode_before = frame::kPolicyRecovery;
  assert(panel::render(panel::classify(&recovery_frame, sizeof(recovery_frame), now))
             .find("Policy: Recovery") != std::string::npos);
  auto faulted_frame = live;
  faulted_frame.policy_state = frame::kPolicyFaulted;
  auto faulted_text = panel::render(panel::classify(&faulted_frame, sizeof(faulted_frame), now));
  assert(faulted_text.find("Policy: FAULTED") != std::string::npos);
  assert(faulted_text.find("RA:") == std::string::npos);

  auto pre_rl_frame = live;
  pre_rl_frame.rl_entered = 0;
  pre_rl_frame.rl_active = 0;
  auto pre_rl_text = panel::render(panel::classify(&pre_rl_frame, sizeof(pre_rl_frame), now));
  assert(pre_rl_text.find("Policy: NOT IN RL") != std::string::npos);
  assert(pre_rl_text.find("Body XY speed:") == std::string::npos);
  assert(pre_rl_text.find("RA:") == std::string::npos);

  auto stopped_frame = live;
  stopped_frame.rl_active = 0;
  auto stopped_text = panel::render(panel::classify(&stopped_frame, sizeof(stopped_frame), now));
  assert(stopped_text.find("Policy: POLICY STOPPED") != std::string::npos);
  assert(stopped_text.find("RA:") == std::string::npos);

  auto invalid_frame = live;
  invalid_frame.ra_value = 0.0f / 0.0f;
  assert(panel::classify(&invalid_frame, sizeof(invalid_frame), now).status ==
         panel::Status::kInvalid);

  if (argc == 2) {
    std::ifstream input(argv[1], std::ios::binary);
    std::vector<char> bytes((std::istreambuf_iterator<char>(input)),
                            std::istreambuf_iterator<char>());
    assert(input.good() || input.eof());
    frame::RuntimeFrame recorded_header{};
    assert(bytes.size() == sizeof(recorded_header));
    std::memcpy(&recorded_header, bytes.data(), sizeof(recorded_header));
    auto recorded = panel::classify(bytes.data(), bytes.size(),
                                    recorded_header.header.monotonic_ns + 20'000'000ULL);
    assert(recorded.status == panel::Status::kLive);
    const auto recorded_text = panel::render(recorded);
    assert(recorded_text.find("Body XY speed:") != std::string::npos);
    assert(recorded_text.find("Policy:") != std::string::npos);
    assert(recorded_text.find("RA:") != std::string::npos);
  }
  return 0;
}
