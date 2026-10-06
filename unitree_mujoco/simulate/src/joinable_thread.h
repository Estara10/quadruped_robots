#ifndef UNITREE_MUJOCO_JOINABLE_THREAD_H_
#define UNITREE_MUJOCO_JOINABLE_THREAD_H_

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <thread>
#include <utility>

class JoinableThread {
 public:
  using Callback = std::function<void()>;

  explicit JoinableThread(Callback callback) : callback_(std::move(callback)) {}
  JoinableThread(const JoinableThread&) = delete;
  JoinableThread& operator=(const JoinableThread&) = delete;

  ~JoinableThread() { stopAndJoin(); }

  bool start() {
    std::lock_guard<std::mutex> lock(lifecycle_mutex_);
    if (worker_.joinable() || join_in_progress_ || terminal_) return false;
    stop_requested_.store(false, std::memory_order_release);
    worker_ = std::thread([this] { callback_(); });
    return true;
  }

  void requestStop() {
    stop_requested_.store(true, std::memory_order_release);
    wake_condition_.notify_all();
  }

  void stopAndJoin() {
    std::thread worker;
    {
      std::lock_guard<std::mutex> lock(lifecycle_mutex_);
      requestStop();
      if (!worker_.joinable()) {
        terminal_ = true;
        return;
      }
      join_in_progress_ = true;
      worker = std::move(worker_);
    }
    worker.join();
    {
      std::lock_guard<std::mutex> lock(lifecycle_mutex_);
      join_in_progress_ = false;
      terminal_ = true;
    }
  }

  bool stopRequested() const {
    return stop_requested_.load(std::memory_order_acquire);
  }

  template <class Rep, class Period>
  bool waitForStop(const std::chrono::duration<Rep, Period>& timeout) {
    std::unique_lock<std::mutex> lock(wake_mutex_);
    return wake_condition_.wait_for(lock, timeout, [this] {
      return stopRequested();
    });
  }

  bool joinable() const {
    std::lock_guard<std::mutex> lock(lifecycle_mutex_);
    return worker_.joinable();
  }

 private:
  Callback callback_;
  mutable std::mutex lifecycle_mutex_;
  std::thread worker_;
  std::atomic<bool> stop_requested_{false};
  bool join_in_progress_ = false;
  bool terminal_ = false;
  std::mutex wake_mutex_;
  std::condition_variable wake_condition_;
};

#endif
