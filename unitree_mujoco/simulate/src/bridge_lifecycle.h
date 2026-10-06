#ifndef UNITREE_MUJOCO_BRIDGE_LIFECYCLE_H_
#define UNITREE_MUJOCO_BRIDGE_LIFECYCLE_H_

#include <chrono>
#include <condition_variable>
#include <mutex>

class BridgeLifecycle {
 public:
  enum class State { INITIAL, RESERVED, ACTIVE, STOPPING, TERMINAL };

  bool reserveBridge() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (state_ != State::INITIAL) return false;
    state_ = State::RESERVED;
    return true;
  }

  bool markBridgeActive() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (state_ != State::RESERVED) return false;
    state_ = State::ACTIVE;
    return true;
  }

  bool beginStop() {
    bool changed = false;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (state_ == State::RESERVED || state_ == State::ACTIVE) {
        state_ = State::STOPPING;
        changed = true;
      }
    }
    condition_.notify_all();
    return changed;
  }

  bool completeTerminal() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (state_ != State::STOPPING) return false;
    state_ = State::TERMINAL;
    condition_.notify_all();
    return true;
  }

  State state() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return state_;
  }

  bool bridgeActive() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return state_ == State::RESERVED || state_ == State::ACTIVE ||
           state_ == State::STOPPING;
  }

  bool reloadAllowed() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return state_ == State::INITIAL || state_ == State::TERMINAL;
  }

  bool terminal() const { return state() == State::TERMINAL; }

  void setMdMutex(std::recursive_mutex* mutex) { md_mutex_ = mutex; }
  std::recursive_mutex* mdMutex() const { return md_mutex_; }

  bool stopRequested() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return stopRequestedLocked();
  }

  void markInitialReady() {
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      initial_ready_ = true;
    }
    condition_.notify_all();
  }

  bool initialReady() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return initial_ready_;
  }

  void waitForStop() {
    std::unique_lock<std::mutex> lock(state_mutex_);
    condition_.wait(lock, [this] { return stopRequestedLocked(); });
  }

  template <class Rep, class Period>
  bool waitForStopFor(const std::chrono::duration<Rep, Period>& timeout) {
    std::unique_lock<std::mutex> lock(state_mutex_);
    return condition_.wait_for(lock, timeout,
                               [this] { return stopRequestedLocked(); });
  }

 private:
  bool stopRequestedLocked() const {
    return state_ == State::STOPPING || state_ == State::TERMINAL;
  }

  mutable std::mutex state_mutex_;
  std::condition_variable condition_;
  State state_ = State::INITIAL;
  bool initial_ready_ = false;
  std::recursive_mutex* md_mutex_ = nullptr;
};

#endif
