//
// P1-08 — C++ -> Python integration bridge (NO MuJoCo, NO ROS).
//
// Minimal helper that exercises the REAL `SimClockWriter` on a caller-supplied
// (unique, temporary) shared-memory name and leaves the last valid v2 snapshot
// readable by the Python `read_sim_clock` reader. This is the forward
// C++-writer -> Python-reader integration path; it does NOT use any synthetic
// bytes on the Python side.
//
// Usage:
//   p1_08_sim_clock_bridge --shm <unique-name> [--n <count>]
//
// Prints:
//   SHM <name>
//   LAST <monotonic_ns> <sim_time>
//
// The shm object persists after exit (POSIX shm_open O_CREAT); the caller must
// unlink it after reading.
//

#include "../../../common/abs_sim_clock_contract.h"

#include <cstdio>
#include <cstring>
#include <cstdint>

int main(int argc, char** argv)
{
    const char* shm_name = nullptr;
    int n = 3;
    for (int i = 1; i < argc; ++i)
    {
        if (std::strcmp(argv[i], "--shm") == 0 && i + 1 < argc) shm_name = argv[++i];
        else if (std::strcmp(argv[i], "--n") == 0 && i + 1 < argc) n = std::atoi(argv[++i]);
    }
    if (!shm_name)
    {
        std::fprintf(stderr, "usage: %s --shm <name> [--n <count>]\n", argv[0]);
        return 2;
    }
    if (n < 1) n = 1;

    abs_sim_clock::SimClockWriter writer(shm_name);
    if (!writer.ok())
    {
        std::fprintf(stderr, "bridge: SimClockWriter failed on %s\n", shm_name);
        return 1;
    }

    uint64_t last_mono = 0;
    double last_sim = 0.0;
    for (int i = 0; i < n; ++i)
    {
        last_mono = 1000ULL + static_cast<uint64_t>(i) * 1000ULL;  // 1000, 2000, ...
        last_sim = 0.002 * static_cast<double>(i + 1);             // 0.002, 0.004, ...
        writer.publish(last_mono, last_sim);
    }

    std::printf("SHM %s\n", shm_name);
    std::printf("LAST %llu %.9f\n",
                static_cast<unsigned long long>(last_mono), last_sim);
    return 0;
}
