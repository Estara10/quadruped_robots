#pragma once

#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <mutex>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

// P1-08 — observability-only physics clock (contract v2).
//
// The MuJoCo physics loop publishes, immediately after every mj_step(), a
// single latest sample {sequence, monotonic_ns, sim_time} to this shared
// memory.  This is the ONLY authoritative runtime source for the observed
// physics timestep / wall-clock physics period; no existing stream carries
// simulation time.
//
// v2 protocol (strict odd/even seqlock + reader validation):
//   - writer marks `sequence` ODD before writing any payload field, then writes
//     every payload field, then publishes an EVEN `sequence` only after all
//     payload fields are written (acquire/release ordering);
//   - a reader rejects/retries an ODD, changed (before != after), or unequal
//     sequence, and rejects unexpected magic/version and non-finite payload;
//   - no reader may accept a cross-field torn snapshot.
//
// Atomicity / ABI:
//   - EVERY access to the shared `SimClock` fields (writer init, writer publish,
//     reader snapshot) uses `__atomic_load_n` / `__atomic_store_n` with
//     acquire/release. There are no plain non-atomic reads or writes of the
//     shared fields, so a concurrent reader can never form a data race with the
//     writer even during construction.
//   - The struct layout/ABI is unchanged (40 bytes: 4 x uint64 + 1 x double, all
//     8-byte aligned; lock-free atomic on x86-64) and matches the Python reader
//     struct format "<4Qd".
//
// Writer boundary:
//   - Construction is fail-closed and strictly ordered: after mapping, the
//     FIRST shared-memory store is an ATOMIC release-store of `sequence = 1`
//     (odd / in-progress), and only THEN are magic/version/sim_time/monotonic
//     written atomically. There is no `memset` before the odd marker. The odd
//     marker is kept until the first publish(), so a reader racing construction
//     (or a stale valid v2 frame left by a previous process) is always rejected.
//   - `SimClockWriter::publish()` is internally serialized with a std::mutex, so
//     concurrent callers (PhysicsLoop thread + UI step-forward thread) can never
//     enter the odd->payload->even critical section simultaneously. It does NOT
//     rely on external `sim.mtx` serialization.
//
// Global publish hook (library step paths, e.g. the UI step-forward in
// simulate.cc) publish through the same contract. Hook storage uses
// acquire/release atomics; lifetime rule: install once before any consumer
// thread starts, and the installed function must outlive the last publishStep()
// call (main.cc installs a non-capturing lambda over a namespace-scope static
// writer). Until installed, publishStep() is a no-op — it is NOT a library-global
// every-step guarantee.
//
// Behavior contract (unchanged from v1):
//   - observability instrumentation only: reads d->time and a monotonic clock
//     and writes one shared-memory struct. It never modifies scheduling,
//     thresholds, policy inputs/outputs, switching, the optimizer, the MuJoCo
//     model, or controller behavior.
//   - monotonic_ns uses std::chrono::steady_clock (same domain as
//     /mujoco_rt_frame and /mujoco_ray2d_stamp).
//   - sim_time is mjData.time after the step (seconds).

namespace abs_sim_clock {

constexpr const char* kShmName = "/mujoco_sim_clock";
constexpr uint64_t kMagic = 0x414253434C4F434BULL;  // mnemonic "ABSCLOCK"
constexpr uint64_t kVersion = 2;                    // v2: strict seqlock
constexpr size_t kSize = 40;

// Fixed-layout, 40-byte frame. Field order/offsets are part of the ABI and MUST
// stay 8-byte aligned so every field is a valid __atomic operand and the Python
// reader struct "<4Qd" keeps matching.
struct SimClock {
  uint64_t magic;         // offset 0  (kMagic)
  uint64_t version;       // offset 8  (kVersion)
  uint64_t sequence;      // offset 16 (odd = in-progress; even = stable)
  uint64_t monotonic_ns;  // offset 24 (steady_clock ns after mj_step)
  double sim_time;        // offset 32 (mjData.time after mj_step, s)
};
static_assert(sizeof(SimClock) == 40, "unexpected sim-clock layout");

// bit_cast via memcpy (C++17 has no std::bit_cast); strict-aliasing safe.
template <typename To, typename From>
inline To bit_cast(const From& from)
{
    static_assert(sizeof(To) == sizeof(From), "bit_cast size mismatch");
    To to;
    std::memcpy(&to, &from, sizeof(To));
    return to;
}

// sim_time is a double; it is accessed atomically through its IEEE-754 bit
// pattern (8 bytes, little-endian on x86-64) via a uint64 view, because GCC's
// __atomic builtins do not accept floating-point operands. Byte layout, finite
// checks and the Python reader are consistent: Python unpacks "<4Qd" which
// reads the same 8 bytes as a double and applies math.isfinite to the decoded
// double.
inline void storeSimTime(SimClock* ptr, double value)
{
    __atomic_store_n(reinterpret_cast<uint64_t*>(&ptr->sim_time),
                     bit_cast<uint64_t>(value), __ATOMIC_RELEASE);
}
inline double loadSimTime(const SimClock* ptr)
{
    const uint64_t bits = __atomic_load_n(
        reinterpret_cast<const uint64_t*>(&ptr->sim_time), __ATOMIC_ACQUIRE);
    return bit_cast<double>(bits);
}

inline uint64_t monotonicNowNs()
{
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
}

// ---------------------------------------------------------------------------
// Global publish hook. Installed once by main.cc before any consumer thread
// starts; acquire/release so a concurrent publishStep() sees a fully installed
// hook. Lifetime: the installed function must outlive the last publishStep().
// ---------------------------------------------------------------------------
using PublishFn = void (*)(uint64_t monotonic_ns, double sim_time);

inline PublishFn& publishHook()
{
    static PublishFn fn = nullptr;
    return fn;
}

inline void installPublishHook(PublishFn fn)
{
    __atomic_store_n(&publishHook(), fn, __ATOMIC_RELEASE);
}

inline void publishStep(uint64_t monotonic_ns, double sim_time)
{
    PublishFn fn = __atomic_load_n(&publishHook(), __ATOMIC_ACQUIRE);
    if (fn)
    {
        fn(monotonic_ns, sim_time);
    }
}

// Single-writer RAII publisher, internally serialized. Created once by the
// MuJoCo physics thread; concurrent callers are serialized inside publish().
// `shm_name` defaults to the production name; tests may pass a unique temporary
// name. The object is not unlinked on destruction (POSIX semantics).
class SimClockWriter
{
public:
    explicit SimClockWriter(const char* shm_name = kShmName)
    {
        fd_ = shm_open(shm_name, O_CREAT | O_RDWR, 0666);
        if (fd_ < 0) return;
        if (ftruncate(fd_, static_cast<off_t>(kSize)) != 0)
        {
            close(fd_);
            fd_ = -1;
            return;
        }
        ptr_ = static_cast<SimClock*>(mmap(nullptr, kSize, PROT_READ | PROT_WRITE,
                                           MAP_SHARED, fd_, 0));
        if (ptr_ == MAP_FAILED)
        {
            ptr_ = nullptr;
            close(fd_);
            fd_ = -1;
            return;
        }
        // Fail-closed construction, strictly ordered:
        //   1) FIRST: atomically publish the ODD (in-progress) sequence so any
        //      concurrent reader fails closed before we touch payload/header.
        //   2) THEN: write magic/version and an invalid (NaN / zero) payload,
        //      each with an atomic release store.
        // The odd marker is kept until the first publish(). There is NO memset
        // before the odd marker, and a stale valid v2 frame from a previous
        // process can never be misread as this writer's data.
        __atomic_store_n(&ptr_->sequence, 1ULL, __ATOMIC_RELEASE);
        __atomic_store_n(&ptr_->magic, kMagic, __ATOMIC_RELEASE);
        __atomic_store_n(&ptr_->version, kVersion, __ATOMIC_RELEASE);
        storeSimTime(ptr_, std::numeric_limits<double>::quiet_NaN());
        __atomic_store_n(&ptr_->monotonic_ns, 0ULL, __ATOMIC_RELEASE);
    }
    ~SimClockWriter()
    {
        if (ptr_) munmap(ptr_, kSize);
        if (fd_ >= 0) close(fd_);
    }
    SimClockWriter(const SimClockWriter&) = delete;
    SimClockWriter& operator=(const SimClockWriter&) = delete;

    bool ok() const { return ptr_ != nullptr; }

    // Strict odd/even seqlock publish, internally serialized so concurrent
    // callers can never interleave inside the critical section. Called after
    // each mj_step. monotonic_ns must be a steady_clock sample taken at the
    // same instant as the step; sim_time must be mjData.time after it.
    void publish(uint64_t monotonic_ns, double sim_time)
    {
        if (!ptr_) return;
        std::lock_guard<std::mutex> lock(lock_);
        // Read the last even sequence; realign if we are still at the odd
        // construction marker (sequence == 1).
        uint64_t even = __atomic_load_n(&ptr_->sequence, __ATOMIC_ACQUIRE);
        if (even & 1) even += 1;
        // 1) mark in-progress (odd), 2) write payload, 3) publish stable (even).
        __atomic_store_n(&ptr_->sequence, even + 1, __ATOMIC_RELEASE);
        storeSimTime(ptr_, sim_time);
        __atomic_store_n(&ptr_->monotonic_ns, monotonic_ns, __ATOMIC_RELEASE);
        __atomic_store_n(&ptr_->sequence, even + 2, __ATOMIC_RELEASE);
    }

private:
    int fd_ = -1;
    SimClock* ptr_ = nullptr;
    std::mutex lock_;
};

// Pure seqlock + validity decision shared by every reader. A snapshot is
// accepted only when: magic matches, version matches exactly (no silent
// old-version fallback), the before/after sequences are equal, even, and
// non-zero, the monotonic timestamp is non-zero, and sim_time is finite.
inline bool validSnapshot(uint64_t seq_before, uint64_t seq_after,
                          uint64_t magic, uint64_t version,
                          uint64_t monotonic_ns, double sim_time)
{
    if (magic != kMagic) return false;
    if (version != kVersion) return false;
    if (seq_before == 0 || (seq_before & 1)) return false;
    if (seq_after != seq_before || (seq_after & 1)) return false;
    if (monotonic_ns == 0) return false;
    if (!std::isfinite(sim_time)) return false;
    return true;
}

// Reader: standard seqlock over the single writer. All shared-field accesses
// are atomic (acquire) — no plain loads of the shared struct. Returns true and
// fills `out` only on a stable, valid snapshot. A torn/odd/changed/
// version-mismatched/non-finite snapshot is never returned.
inline bool readSnapshot(SimClock* ptr, SimClock* out, int max_attempts = 5)
{
    if (!ptr || !out) return false;
    for (int i = 0; i < max_attempts; ++i)
    {
        const uint64_t seq_before = __atomic_load_n(&ptr->sequence, __ATOMIC_ACQUIRE);
        if (seq_before == 0 || (seq_before & 1)) continue;
        SimClock snap;
        snap.magic = __atomic_load_n(&ptr->magic, __ATOMIC_ACQUIRE);
        snap.version = __atomic_load_n(&ptr->version, __ATOMIC_ACQUIRE);
        snap.sequence = seq_before;  // the stable even sequence just validated
        snap.monotonic_ns = __atomic_load_n(&ptr->monotonic_ns, __ATOMIC_ACQUIRE);
        snap.sim_time = loadSimTime(ptr);
        const uint64_t seq_after = __atomic_load_n(&ptr->sequence, __ATOMIC_ACQUIRE);
        if (!validSnapshot(seq_before, seq_after, snap.magic, snap.version,
                           snap.monotonic_ns, snap.sim_time))
        {
            continue;
        }
        *out = snap;
        return true;
    }
    return false;
}

}  // namespace abs_sim_clock
