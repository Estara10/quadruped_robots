#pragma once

// P1-10 Stage-A common-start contract.
//
// This is a deliberately small, opt-in shared-memory control/observation
// link.  It is not used by the accepted P1-08 path.  The simulator publishes
// readiness and first-physics facts; the harness is the only release writer;
// StateRL publishes the producer-side RL-enter/first-frame facts.

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <string>
#include <sys/mman.h>
#include <sys/stat.h>
#include <thread>
#include <unistd.h>
#include <utility>

namespace abs_stage_a_common_start {

constexpr const char* kSchema = "abs-go2-p1-10-stage-a-common-start/v1";
constexpr uint64_t kMagic = 0x4142535354474154ULL;  // "ABSSTGAT"
constexpr uint64_t kVersion = 1;
constexpr const char* kDefaultShmName = "/mujoco_p1_10_stage_a_common_start";
constexpr std::size_t kCaptureIdBytes = 64;
constexpr std::size_t kSha256Bytes = 64;

enum State : uint32_t {
  kInvalid = 0,
  kWaitingForRelease = 1,
  kReleased = 2,
  kFailed = 3,
};

enum Flags : uint32_t {
  kInitialReady = 1U << 0,
  kReleaseRecorded = 1U << 1,
  kRlEnterRecorded = 1U << 2,
  kFirstPhysicsRecorded = 1U << 3,
  kFirstRuntimeFrameRecorded = 1U << 4,
};

// Do not change field order or widths without incrementing kVersion.  The
// Python harness uses the same fixed little-endian layout.
struct SharedRecord {
  uint64_t magic;
  uint64_t version;
  uint64_t sequence;
  uint32_t state;
  uint32_t flags;
  char capture_id[kCaptureIdBytes];
  char initial_qpos_sha256[kSha256Bytes];
  uint64_t gate_ready_monotonic_ns;
  uint64_t release_monotonic_ns;
  uint64_t rl_session_id;
  uint64_t rl_step;
  uint64_t rl_frame_sequence;
  uint64_t rl_monotonic_ns;
  uint64_t first_frame_sequence;
  uint64_t first_frame_rl_step;
  uint64_t first_frame_monotonic_ns;
  uint64_t first_physics_step;
  uint64_t first_physics_monotonic_ns;
  double first_sim_time;
  uint32_t failure_code;
  uint32_t reserved;
};

static_assert(sizeof(SharedRecord) == 264, "common-start shared layout drift");

inline uint64_t monotonicNowNs() {
  return static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch())
          .count());
}

inline uint64_t loadAcquire(const uint64_t* address) {
  return __atomic_load_n(address, __ATOMIC_ACQUIRE);
}

inline void storeRelease(uint64_t* address, uint64_t value) {
  __atomic_store_n(address, value, __ATOMIC_RELEASE);
}

inline uint32_t loadStateAcquire(const uint32_t* address) {
  return __atomic_load_n(address, __ATOMIC_ACQUIRE);
}

inline void storeStateRelease(uint32_t* address, uint32_t value) {
  __atomic_store_n(address, value, __ATOMIC_RELEASE);
}

inline bool validHex(const std::string& value, std::size_t length) {
  if (value.size() != length) return false;
  for (char c : value) {
    const bool digit = c >= '0' && c <= '9';
    const bool lower = c >= 'a' && c <= 'f';
    if (!digit && !lower) return false;
  }
  return true;
}

inline bool validCaptureId(const std::string& value) {
  constexpr const char* prefix = "p1-10-capture-";
  return value.rfind(prefix, 0) == 0 && validHex(value.substr(std::strlen(prefix)), 32);
}

inline void copyFixed(char* destination, std::size_t capacity, const std::string& value) {
  std::memset(destination, 0, capacity);
  const std::size_t count = value.size() < capacity ? value.size() : capacity;
  std::memcpy(destination, value.data(), count);
}

inline std::string readFixed(const char* source, std::size_t capacity) {
  std::size_t length = 0;
  while (length < capacity && source[length] != '\0') ++length;
  return std::string(source, length);
}

// Pure state machine used by the offline contract test.  Runtime writers use
// the same transitions around the shared-memory record.
class StateMachine {
 public:
  explicit StateMachine(std::string capture_id)
      : capture_id_(std::move(capture_id)), state_(kWaitingForRelease) {}

  bool stepAllowed() const { return state_ == kReleased; }
  State state() const { return state_; }
  uint64_t physics_steps() const { return physics_steps_; }
  bool release(const std::string& capture_id) {
    if (state_ != kWaitingForRelease || capture_id != capture_id_) return false;
    state_ = kReleased;
    return true;
  }
  bool noteRlEnter(uint64_t session, uint64_t rl_step, uint64_t frame_sequence) {
    if ((state_ != kWaitingForRelease && state_ != kReleased) ||
        session == 0 || rl_session_id_ != 0) return false;
    rl_session_id_ = session;
    rl_step_ = rl_step;
    rl_frame_sequence_ = frame_sequence;
    return true;
  }
  bool notePhysicsStep(uint64_t step) {
    if (!stepAllowed() || step == 0 || first_physics_step_ != 0) return false;
    first_physics_step_ = step;
    physics_steps_ = 1;
    return true;
  }
  bool noteFirstFrame(uint64_t sequence, uint64_t rl_step) {
    if (!stepAllowed() || sequence == 0 || first_frame_sequence_ != 0 ||
        rl_session_id_ == 0) return false;
    first_frame_sequence_ = sequence;
    first_frame_rl_step_ = rl_step;
    return true;
  }
  uint64_t first_physics_step() const { return first_physics_step_; }
  uint64_t first_frame_sequence() const { return first_frame_sequence_; }
  uint64_t rl_session_id() const { return rl_session_id_; }

 private:
  std::string capture_id_;
  State state_;
  uint64_t rl_session_id_ = 0;
  uint64_t rl_step_ = 0;
  uint64_t rl_frame_sequence_ = 0;
  uint64_t first_physics_step_ = 0;
  uint64_t first_frame_sequence_ = 0;
  uint64_t first_frame_rl_step_ = 0;
  uint64_t physics_steps_ = 0;
};

class SharedClient {
 public:
  explicit SharedClient(const std::string& shm_name = kDefaultShmName,
                        const std::string& expected_capture_id = {})
      : expected_capture_id_(expected_capture_id) {
    fd_ = shm_open(shm_name.c_str(), O_RDWR, 0666);
    if (fd_ < 0) return;
    struct stat info{};
    if (fstat(fd_, &info) != 0 || info.st_size < static_cast<off_t>(sizeof(SharedRecord))) {
      close(fd_);
      fd_ = -1;
      return;
    }
    record_ = static_cast<SharedRecord*>(mmap(nullptr, sizeof(SharedRecord),
                                               PROT_READ | PROT_WRITE, MAP_SHARED,
                                               fd_, 0));
    if (record_ == MAP_FAILED) {
      record_ = nullptr;
      close(fd_);
      fd_ = -1;
    }
  }

  ~SharedClient() {
    if (record_ != nullptr && record_ != MAP_FAILED) munmap(record_, sizeof(SharedRecord));
    if (fd_ >= 0) close(fd_);
  }

  SharedClient(const SharedClient&) = delete;
  SharedClient& operator=(const SharedClient&) = delete;

  bool valid() const { return record_ != nullptr && record_ != MAP_FAILED; }

  bool recordRlEnter(uint64_t session, uint64_t rl_step, uint64_t frame_sequence,
                     uint64_t monotonic_ns) {
    if (session == 0 || monotonic_ns == 0) return false;
    SharedRecord current{};
    if (!readStable(&current) || !bindingMatches(current) ||
        (current.state != kWaitingForRelease && current.state != kReleased) ||
        current.rl_session_id != 0) return false;
    return update([&](SharedRecord& value) {
      value.rl_session_id = session;
      value.rl_step = rl_step;
      value.rl_frame_sequence = frame_sequence;
      value.rl_monotonic_ns = monotonic_ns;
      value.flags |= kRlEnterRecorded;
    });
  }

  bool recordFirstRuntimeFrame(uint64_t session, uint64_t rl_step,
                               uint64_t frame_sequence, uint64_t monotonic_ns) {
    if (session == 0 || frame_sequence == 0 || monotonic_ns == 0) return false;
    SharedRecord current{};
    if (!readStable(&current) || !bindingMatches(current) ||
        current.rl_session_id != session) return false;
    if (current.state == kWaitingForRelease) return true;
    if (current.state != kReleased) return false;
    // A formal first frame is anchored only after the first released physics
    // step.  Pre-step controller frames may exist while the gate is waiting,
    // but they are never selected as the formal capture start.
    if (current.first_physics_step == 0) return true;
    // The first post-release frame is recorded once. Later frames are valid
    // and need no second mutation of the one-shot anchor.
    if (current.first_frame_sequence != 0) return true;
    return update([&](SharedRecord& value) {
      value.first_frame_sequence = frame_sequence;
      value.first_frame_rl_step = rl_step;
      value.first_frame_monotonic_ns = monotonic_ns;
      value.flags |= kFirstRuntimeFrameRecorded;
    });
  }

 protected:
  bool bindingMatches(const SharedRecord& value) const {
    const std::string capture = readFixed(value.capture_id, kCaptureIdBytes);
    return value.magic == kMagic && value.version == kVersion &&
           value.state != kInvalid && validCaptureId(capture) &&
           (expected_capture_id_.empty() || capture == expected_capture_id_);
  }

  bool readStable(SharedRecord* output) const {
    if (!valid() || output == nullptr) return false;
    for (int attempt = 0; attempt < 100; ++attempt) {
      const uint64_t before = loadAcquire(&record_->sequence);
      if (before == 0 || (before & 1U)) continue;
      std::memcpy(output, record_, sizeof(SharedRecord));
      const uint64_t after = loadAcquire(&record_->sequence);
      if (before == after && !(after & 1U)) return true;
    }
    return false;
  }

  template <typename Mutation>
  bool update(Mutation mutation) {
    if (!valid()) return false;
    // There are two producer writers (simulator and controller).  A plain
    // even->odd increment is not sufficient: concurrent writers could both
    // read the same even sequence and lose one another's fields.  Claim the
    // seqlock with CAS, keep the odd marker in the copied record, and fail
    // closed if the claim cannot be made promptly.
    for (int attempt = 0; attempt < 1000; ++attempt) {
      uint64_t sequence = loadAcquire(&record_->sequence);
      if (sequence == 0 || (sequence & 1U)) {
        std::this_thread::yield();
        continue;
      }
      uint64_t expected = sequence;
      if (!__atomic_compare_exchange_n(&record_->sequence, &expected,
                                       sequence + 1U, false,
                                       __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE)) {
        continue;
      }
      SharedRecord copy{};
      std::memcpy(&copy, record_, sizeof(copy));
      copy.sequence = sequence + 1U;
      mutation(copy);
      std::memcpy(record_, &copy, sizeof(copy));
      storeRelease(&record_->sequence, sequence + 2U);
      return true;
    }
    return false;
  }

  int fd_ = -1;
  SharedRecord* record_ = nullptr;
  std::string expected_capture_id_;
};

class Writer : public SharedClient {
 public:
  Writer(const std::string& capture_id, const std::string& initial_qpos_sha256,
         const std::string& shm_name = kDefaultShmName)
      : SharedClient(shm_name, capture_id), capture_id_(capture_id),
        initial_qpos_sha256_(initial_qpos_sha256) {
    if (!validCaptureId(capture_id_) || !validHex(initial_qpos_sha256_, 64)) {
      return;
    }
    // Re-open with create semantics: the simulator owns creation of the gate.
    if (fd_ >= 0) {
      if (record_ != nullptr && record_ != MAP_FAILED) munmap(record_, sizeof(SharedRecord));
      close(fd_);
      record_ = nullptr;
      fd_ = -1;
    }
    fd_ = shm_open(shm_name.c_str(), O_CREAT | O_RDWR, 0666);
    if (fd_ < 0 || ftruncate(fd_, static_cast<off_t>(sizeof(SharedRecord))) != 0) {
      if (fd_ >= 0) close(fd_);
      fd_ = -1;
      return;
    }
    record_ = static_cast<SharedRecord*>(mmap(nullptr, sizeof(SharedRecord),
                                               PROT_READ | PROT_WRITE, MAP_SHARED,
                                               fd_, 0));
    if (record_ == MAP_FAILED) {
      record_ = nullptr;
      close(fd_);
      fd_ = -1;
      return;
    }
    const uint64_t sequence = loadAcquire(&record_->sequence);
    storeRelease(&record_->sequence, (sequence & 1U) ? sequence + 1U : sequence + 1U);
    std::memset(record_, 0, sizeof(SharedRecord));
    record_->magic = kMagic;
    record_->version = kVersion;
    record_->state = kWaitingForRelease;
    copyFixed(record_->capture_id, kCaptureIdBytes, capture_id_);
    copyFixed(record_->initial_qpos_sha256, kSha256Bytes, initial_qpos_sha256_);
    storeRelease(&record_->sequence, 2U);
    initialized_ = true;
  }

  bool initialized() const { return initialized_ && valid(); }
  bool stepAllowed() const {
    SharedRecord current{};
    return readStable(&current) && bindingMatches(current) &&
           current.state == kReleased;
  }

  bool markInitialReady() {
    if (!initialized()) return false;
    return update([&](SharedRecord& value) {
      value.gate_ready_monotonic_ns = monotonicNowNs();
      value.flags |= kInitialReady;
    });
  }

  bool recordPhysicsStep(uint64_t physics_step, double sim_time, uint64_t monotonic_ns) {
    if (!initialized() || physics_step == 0 || monotonic_ns == 0 ||
        !std::isfinite(sim_time)) return false;
    SharedRecord current{};
    if (!readStable(&current) || !bindingMatches(current) ||
        current.state != kReleased || current.first_physics_step != 0) return false;
    return update([&](SharedRecord& value) {
      value.first_physics_step = physics_step;
      value.first_physics_monotonic_ns = monotonic_ns;
      value.first_sim_time = sim_time;
      value.flags |= kFirstPhysicsRecorded;
    });
  }

 private:
  std::string capture_id_;
  std::string initial_qpos_sha256_;
  bool initialized_ = false;
};

}  // namespace abs_stage_a_common_start
