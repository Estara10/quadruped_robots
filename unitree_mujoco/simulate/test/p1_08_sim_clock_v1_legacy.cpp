//
// LEGACY — v1 sim-clock contract test. NOT REGISTERED, NOT RUN.
//
// Retained for audit only. This file:
//   - tested the v1 contract (single final-sequence publication, no strict
//     odd/even seqlock), which is superseded by contract v2;
//   - uses assert() as its only failure mechanism, so under Release/-DNDEBUG
//     failures would NOT produce a non-zero exit — it is not a valid CTest;
//   - asserts a v1 sequence layout (seq == 3 after three publishes) that is
//     wrong under the v2 writer (odd/even, fail-closed init marker).
//
// The registered v2 test is `p1_08_sim_clock_test.cpp` (built by CMake as
// `p1_08_sim_clock_test` and run via `ctest`). Do not register this file.
//

#include "../../../common/abs_sim_clock_contract.h"

#include <cassert>
#include <cstdint>
#include <cstdio>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

int main()
{
    // Clean any stale object from a previous crashed run.
    shm_unlink(abs_sim_clock::kShmName);

    abs_sim_clock::SimClockWriter writer;
    assert(writer.ok());

    // Publish three physics steps with increasing sim time.
    writer.publish(1000ULL, 0.000);
    writer.publish(2000ULL, 0.002);
    writer.publish(3000ULL, 0.004);

    // Read back through a fresh reader mapping.
    int fd = shm_open(abs_sim_clock::kShmName, O_RDONLY, 0666);
    assert(fd >= 0);
    abs_sim_clock::SimClock* ptr = static_cast<abs_sim_clock::SimClock*>(
        mmap(nullptr, sizeof(abs_sim_clock::SimClock), PROT_READ, MAP_SHARED, fd, 0));
    assert(ptr != MAP_FAILED && ptr != nullptr);

    // Seqlock-style read: re-read sequence to detect a torn snapshot.
    uint64_t seq_a = ptr->sequence;
    uint64_t mono = ptr->monotonic_ns;
    double st = ptr->sim_time;
    uint64_t seq_b = ptr->sequence;
    assert(seq_a == seq_b);  // no writer raced in this test

    assert(ptr->magic == abs_sim_clock::kMagic);
    assert(ptr->version == abs_sim_clock::kVersion);
    assert(seq_a == 3);
    assert(mono == 3000ULL);
    assert(st == 0.004);

    // A fresh writer restarts at sequence 1 (single-writer per run).
    abs_sim_clock::SimClockWriter writer2;
    assert(writer2.ok());
    writer2.publish(4000ULL, 0.006);
    assert(ptr->sequence == 1);
    assert(ptr->monotonic_ns == 4000ULL);
    assert(ptr->sim_time == 0.006);

    munmap(ptr, sizeof(abs_sim_clock::SimClock));
    close(fd);
    shm_unlink(abs_sim_clock::kShmName);

    std::printf("RESULT: PASS\n");
    return 0;
}
