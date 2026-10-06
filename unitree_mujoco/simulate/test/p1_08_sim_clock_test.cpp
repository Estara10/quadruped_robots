//
// P1-08 — v2 sim-clock contract mechanical tests (registered, Release-safe).
//
// Uses a custom CHECK macro (NOT assert), so every failure produces a non-zero
// exit even under Release/-DNDEBUG. CTest-registered as `p1_08_sim_clock_test`.
//
// Coverage:
//   1. construction / init state is fail-closed (odd in-progress + NaN/zero);
//   2. a pre-seeded stale valid v2 snapshot is INVALIDATED by a new writer's
//      construction (rejected before first publish), then accepted after it;
//   3. stable EVEN snapshot accepted (sequence, monotonic_ns, sim_time);
//   4. ODD (in-progress) sequence rejected;
//   5. sequence changed during copy (before != after) rejected;
//   6. wrong magic / wrong (incl. old) version rejected;
//   7. NaN / +Inf / -Inf sim_time rejected;
//   8. zero monotonic timestamp rejected;
//   9. hook dispatch round-trip: install a NON-CAPTURING static callback that
//      routes to the same writer, publishStep() -> writer -> reader round-trip;
//  10. publishStep with no hook installed is a no-op (does not alter the shm);
//  11. multi-threaded multi-caller publish stress: N writer threads + reader;
//      every accepted snapshot must be internally consistent (sim == mono*1e-9),
//      proving no torn cross-field snapshot is ever accepted.
//
// All direct writes to the shared struct use __atomic_store_n to stay
// consistent with the header's atomic access rule (no plain shared-field
// writes in this test).
//

#include "../../../common/abs_sim_clock_contract.h"

#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <limits>
#include <thread>
#include <vector>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

using abs_sim_clock::kMagic;
using abs_sim_clock::kVersion;
using abs_sim_clock::SimClock;
using abs_sim_clock::SimClockWriter;
using abs_sim_clock::installPublishHook;
using abs_sim_clock::publishStep;
using abs_sim_clock::readSnapshot;
using abs_sim_clock::validSnapshot;

static int g_checks = 0;
static bool g_fail = false;

#define CHECK(cond)                                             \
    do {                                                        \
        ++g_checks;                                             \
        if (!(cond)) {                                          \
            g_fail = true;                                      \
            std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); \
        }                                                       \
    } while (0)

// --- non-capturing hook callback used by the round-trip test ----------------
static SimClockWriter* g_route_target = nullptr;
static void route_to_writer(uint64_t ns, double t)
{
    if (g_route_target) g_route_target->publish(ns, t);
}

// --- multi-threaded stress ---------------------------------------------------
static const int kPubThreads = 4;
static const int kPubPerThread = 20000;
static std::atomic<uint64_t> g_next_pub{1000000};
static std::atomic<int> g_active_writers{0};

static void writer_thread(SimClockWriter* w)
{
    for (int i = 0; i < kPubPerThread; ++i)
    {
        const uint64_t m = g_next_pub.fetch_add(1);
        w->publish(m, static_cast<double>(m) * 1e-9);  // sim == mono*1e-9 bijection
    }
    g_active_writers.fetch_sub(1);
}

static void run_stress(SimClockWriter* writer, SimClock* mem)
{
    // Stress-phase baseline: the shared memory still holds the hook round-trip
    // test's final snapshot ({9000, 0.018}) when this function starts. That is a
    // perfectly VALID seqlock snapshot (consistent, even sequence) but belongs to
    // the PREVIOUS test phase, so it must not be judged against the stress
    // invariant (sim == mono*1e-9). Only validate snapshots whose monotonic_ns is
    // >= the stress baseline (the first value g_next_pub will hand out).
    const uint64_t baseline = g_next_pub.load();
    std::vector<std::thread> pubs;
    for (int t = 0; t < kPubThreads; ++t)
    {
        pubs.emplace_back(writer_thread, writer);
    }
    long accepted = 0;
    long inconsistent = 0;
    while (g_active_writers.load() > 0)
    {
        SimClock out;
        if (readSnapshot(mem, &out))
        {
            // Skip stale (pre-stress) snapshots; they are consistent but not
            // stress-phase data and would otherwise false-trip the invariant.
            if (out.monotonic_ns < baseline) { continue; }
            ++accepted;
            // Exact equality: for a consistent pair, sim_time is bit-identical to
            // mono*1e-9 (same double computation, same 8 bytes). Any torn pair —
            // including adjacent-publish tears — yields a non-zero difference.
            const double expect = static_cast<double>(out.monotonic_ns) * 1e-9;
            if (out.sim_time != expect)
            {
                ++inconsistent;  // cross-field tear reached the reader
                std::printf("TORN: mono=%llu sim=%.17g expect=%.17g seq=%llu\n",
                            (unsigned long long)out.monotonic_ns, out.sim_time,
                            expect, (unsigned long long)out.sequence);
                break;
            }
        }
        std::this_thread::yield();
    }
    for (auto& th : pubs) th.join();
    CHECK(accepted > 0);
    CHECK(inconsistent == 0);
}

static SimClock* map_shm(int& fd_out)
{
    int fd = shm_open(abs_sim_clock::kShmName, O_RDWR, 0666);
    if (fd < 0) return nullptr;
    SimClock* p = static_cast<SimClock*>(mmap(nullptr, sizeof(SimClock),
                                              PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0));
    if (p == MAP_FAILED)
    {
        close(fd);
        return nullptr;
    }
    fd_out = fd;
    return p;
}

int main()
{
    shm_unlink(abs_sim_clock::kShmName);

    SimClockWriter writer;
    CHECK(writer.ok());

    // Writable mapping so the test can corrupt fields and call the reader.
    int fd = -1;
    SimClock* mem = map_shm(fd);
    CHECK(mem != nullptr);
    SimClock out{};

    // --- 1. construction / init state is fail-closed -------------------------
    CHECK(!readSnapshot(mem, &out));  // seq==1 odd + NaN payload + mono 0

    // --- 2. stable even snapshots accepted -----------------------------------
    writer.publish(1000ULL, 0.002);
    CHECK(readSnapshot(mem, &out));
    CHECK(out.sequence == 4);              // init odd(1) -> realign even 2 -> odd 3 -> even 4
    CHECK(out.monotonic_ns == 1000ULL);
    CHECK(out.sim_time == 0.002);

    writer.publish(2000ULL, 0.004);
    CHECK(readSnapshot(mem, &out));
    CHECK(out.sequence == 6);
    CHECK(out.sim_time == 0.004);

    // --- 3. odd (in-progress) sequence rejected ------------------------------
    __atomic_store_n(&mem->sequence, 7ULL, __ATOMIC_RELEASE);  // force odd
    CHECK(!readSnapshot(mem, &out));
    writer.publish(3000ULL, 0.006);        // writer realigns odd -> even+2
    CHECK(readSnapshot(mem, &out));
    CHECK(out.sim_time == 0.006);

    // --- 4. sequence changed during copy rejected (pure decision) ------------
    CHECK(!validSnapshot(2, 4, kMagic, kVersion, 1, 0.0));   // before != after
    CHECK(!validSnapshot(3, 4, kMagic, kVersion, 1, 0.0));   // odd before
    CHECK(!validSnapshot(2, 3, kMagic, kVersion, 1, 0.0));   // odd after
    CHECK(!validSnapshot(0, 0, kMagic, kVersion, 1, 0.0));   // seq 0

    // --- 5. wrong magic / version rejected -----------------------------------
    __atomic_store_n(&mem->magic, 0ULL, __ATOMIC_RELEASE);
    CHECK(!readSnapshot(mem, &out));
    __atomic_store_n(&mem->magic, kMagic, __ATOMIC_RELEASE);
    __atomic_store_n(&mem->version, 99ULL, __ATOMIC_RELEASE);
    CHECK(!readSnapshot(mem, &out));
    __atomic_store_n(&mem->version, kVersion - 1, __ATOMIC_RELEASE);  // old version
    CHECK(!readSnapshot(mem, &out));
    __atomic_store_n(&mem->version, kVersion, __ATOMIC_RELEASE);
    CHECK(readSnapshot(mem, &out));
    CHECK(!validSnapshot(2, 2, 0, kVersion, 1, 0.0));        // wrong magic
    CHECK(!validSnapshot(2, 2, kMagic, 1, 1, 0.0));          // old version

    // --- 6. NaN / +/-Inf sim_time rejected -----------------------------------
    abs_sim_clock::storeSimTime(mem, std::numeric_limits<double>::quiet_NaN());
    CHECK(!readSnapshot(mem, &out));
    abs_sim_clock::storeSimTime(mem, std::numeric_limits<double>::infinity());
    CHECK(!readSnapshot(mem, &out));
    abs_sim_clock::storeSimTime(mem, -std::numeric_limits<double>::infinity());
    CHECK(!readSnapshot(mem, &out));
    writer.publish(4000ULL, 0.008);
    CHECK(readSnapshot(mem, &out));

    // --- 7. zero monotonic timestamp rejected --------------------------------
    __atomic_store_n(&mem->monotonic_ns, 0ULL, __ATOMIC_RELEASE);
    CHECK(!readSnapshot(mem, &out));
    writer.publish(5000ULL, 0.010);
    CHECK(readSnapshot(mem, &out));

    // --- 8. hook dispatch: install -> publishStep -> writer -> reader --------
    g_route_target = &writer;
    installPublishHook(&route_to_writer);   // NON-CAPTURING static callback
    publishStep(9000ULL, 0.018);
    SimClock routed;
    CHECK(readSnapshot(mem, &routed));
    CHECK(routed.monotonic_ns == 9000ULL);
    CHECK(routed.sim_time == 0.018);

    // --- 9. no hook installed -> publishStep is a no-op ----------------------
    installPublishHook(nullptr);
    g_route_target = nullptr;
    SimClock before;
    CHECK(readSnapshot(mem, &before));      // last valid = 9000/0.018
    publishStep(7777ULL, 0.777);            // no hook: must not publish
    SimClock after;
    CHECK(readSnapshot(mem, &after));
    CHECK(after.monotonic_ns == before.monotonic_ns);
    CHECK(after.sim_time == before.sim_time);

    // --- 10. multi-threaded multi-caller publish stress ----------------------
    g_next_pub.store(1000000);
    g_active_writers.store(kPubThreads);
    run_stress(&writer, mem);

    // --- 11. stale valid v2 snapshot invalidated by a new writer's ctor -----
    shm_unlink(abs_sim_clock::kShmName);     // start fresh for this sub-case
    {
        SimClockWriter w1;                    // creates the shm object
        CHECK(w1.ok());
        w1.publish(100ULL, 0.001);            // a valid stale v2 frame
    }                                         // w1 destructs; shm object persists
    {
        SimClockWriter w2;                    // reopens the SAME shm
        CHECK(w2.ok());
        int fd2 = -1;
        SimClock* mem2 = map_shm(fd2);
        CHECK(mem2 != nullptr);
        CHECK(!readSnapshot(mem2, &out));     // construction invalidates the stale frame
        w2.publish(200ULL, 0.002);            // first publish -> reader may accept
        CHECK(readSnapshot(mem2, &out));
        CHECK(out.monotonic_ns == 200ULL);
        CHECK(out.sim_time == 0.002);
        munmap(mem2, sizeof(SimClock));
        close(fd2);
    }
    shm_unlink(abs_sim_clock::kShmName);

    munmap(mem, sizeof(SimClock));
    close(fd);

    if (g_fail)
    {
        std::printf("RESULT: FAIL (%d checks)\n", g_checks);
        return 1;
    }
    std::printf("RESULT: PASS (%d checks)\n", g_checks);
    return 0;
}
