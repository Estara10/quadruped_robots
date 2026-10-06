#include <rl_quadruped_controller/common/StoppableThread.h>

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <thread>

namespace
{
using namespace std::chrono_literals;

bool waitUntil(const std::atomic<bool>& value)
{
    const auto deadline = std::chrono::steady_clock::now() + 1s;
    while (!value.load(std::memory_order_acquire) && std::chrono::steady_clock::now() < deadline)
    {
        std::this_thread::sleep_for(1ms);
    }
    return value.load(std::memory_order_acquire);
}

bool require(bool condition, const char* message)
{
    if (!condition)
    {
        std::cerr << "FAIL: " << message << std::endl;
        return false;
    }
    return true;
}
}  // namespace

int main()
{
    using rl_quadruped_controller::StoppableThread;

    // Unstarted thread destruction and repeated stop are no-ops.
    {
        StoppableThread unstarted;
        unstarted.stopAndJoin();
        unstarted.stopAndJoin();
        if (!require(!unstarted.joinable(), "unstarted worker must not be joinable")) return EXIT_FAILURE;
    }

    std::atomic<bool> first_entered{false};
    std::atomic<bool> first_exited{false};
    StoppableThread worker;
    if (!require(worker.start([&](StoppableThread& lifecycle) {
            first_entered.store(true, std::memory_order_release);
            while (!lifecycle.stopRequested()) lifecycle.waitForStop(10s);
            first_exited.store(true, std::memory_order_release);
        }), "first start must create a worker")) return EXIT_FAILURE;
    if (!require(waitUntil(first_entered), "first worker did not start")) return EXIT_FAILURE;

    // The condition-variable wait must be interrupted by stop, not held for
    // the 10-second periodic wait above.
    const auto stop_start = std::chrono::steady_clock::now();
    worker.stopAndJoin();
    const auto stop_elapsed = std::chrono::steady_clock::now() - stop_start;
    if (!require(first_exited.load(std::memory_order_acquire), "started worker did not exit")) return EXIT_FAILURE;
    if (!require(stop_elapsed < 500ms, "stop/join was delayed by periodic wait")) return EXIT_FAILURE;
    if (!require(!worker.joinable(), "stopped worker must not remain joinable")) return EXIT_FAILURE;

    // Repeated exit is safe, and a subsequent enter can create one fresh
    // worker only after the old worker has been joined.
    worker.stopAndJoin();
    std::atomic<bool> second_entered{false};
    std::atomic<bool> second_exited{false};
    if (!require(worker.start([&](StoppableThread& lifecycle) {
            second_entered.store(true, std::memory_order_release);
            while (!lifecycle.stopRequested()) lifecycle.waitForStop(10s);
            second_exited.store(true, std::memory_order_release);
        }), "second start after exit must create a fresh worker")) return EXIT_FAILURE;
    if (!require(waitUntil(second_entered), "second worker did not start")) return EXIT_FAILURE;
    worker.stopAndJoin();
    if (!require(second_exited.load(std::memory_order_acquire), "second worker did not exit")) return EXIT_FAILURE;

    // Destructor runs after this stopped-thread scope and must be a no-op.
    std::cout << "PASS: unstarted, started, repeated-stop, restart, and stopped-destruction lifecycle cases" << std::endl;
    return EXIT_SUCCESS;
}
