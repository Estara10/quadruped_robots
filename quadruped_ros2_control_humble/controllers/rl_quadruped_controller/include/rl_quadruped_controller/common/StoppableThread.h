// Minimal join-only lifecycle owner for controller worker threads.
//
// A worker must check stopRequested() and use waitForStop() for periodic waits.
// stopAndJoin() never detaches: it wakes the wait immediately and joins the
// worker before its owning controller/FSM state can be destroyed.

#ifndef RL_QUADRUPED_CONTROLLER_COMMON_STOPPABLETHREAD_H
#define RL_QUADRUPED_CONTROLLER_COMMON_STOPPABLETHREAD_H

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <utility>

namespace rl_quadruped_controller
{
class StoppableThread final
{
public:
    StoppableThread() = default;
    ~StoppableThread() { stopAndJoin(); }

    StoppableThread(const StoppableThread&) = delete;
    StoppableThread& operator=(const StoppableThread&) = delete;

    template <typename Worker>
    bool start(Worker&& worker)
    {
        std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex_);
        if (worker_.joinable())
        {
            return false;
        }
        stop_requested_.store(false, std::memory_order_release);
        worker_ = std::thread(
            [this, callback = std::forward<Worker>(worker)]() mutable { callback(*this); });
        return true;
    }

    void requestStop()
    {
        stop_requested_.store(true, std::memory_order_release);
        wait_cv_.notify_all();
    }

    void stopAndJoin()
    {
        // Keep start()/stopAndJoin() mutually exclusive through the join, so a
        // new worker cannot observe a cleared stop request before the old one
        // has exited.
        std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex_);
        requestStop();
        if (worker_.joinable())
        {
            worker_.join();
        }
    }

    [[nodiscard]] bool stopRequested() const
    {
        return stop_requested_.load(std::memory_order_acquire);
    }

    template <typename Rep, typename Period>
    bool waitForStop(const std::chrono::duration<Rep, Period>& timeout)
    {
        std::unique_lock<std::mutex> wait_lock(wait_mutex_);
        return wait_cv_.wait_for(wait_lock, timeout,
                                 [this] { return stopRequested(); });
    }

    [[nodiscard]] bool joinable() const
    {
        std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex_);
        return worker_.joinable();
    }

    std::thread& nativeThread()
    {
        return worker_;
    }

private:
    mutable std::mutex lifecycle_mutex_;
    std::mutex wait_mutex_;
    std::condition_variable wait_cv_;
    std::atomic<bool> stop_requested_{false};
    std::thread worker_;
};
}  // namespace rl_quadruped_controller

#endif  // RL_QUADRUPED_CONTROLLER_COMMON_STOPPABLETHREAD_H
