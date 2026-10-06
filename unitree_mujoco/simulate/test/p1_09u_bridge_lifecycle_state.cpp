#include <atomic>
#include <cassert>
#include <future>
#include <thread>

#include "../src/bridge_lifecycle.h"

int main() {
  using State = BridgeLifecycle::State;

  BridgeLifecycle lifecycle;
  assert(lifecycle.state() == State::INITIAL);
  assert(lifecycle.reloadAllowed());
  assert(!lifecycle.beginStop());
  assert(lifecycle.state() == State::INITIAL);
  assert(lifecycle.reserveBridge());
  assert(lifecycle.state() == State::RESERVED);
  assert(!lifecycle.reloadAllowed());
  assert(lifecycle.markBridgeActive());
  assert(lifecycle.state() == State::ACTIVE);
  assert(!lifecycle.reloadAllowed());
  assert(lifecycle.beginStop());
  assert(!lifecycle.beginStop());
  assert(lifecycle.state() == State::STOPPING);
  assert(!lifecycle.reloadAllowed());
  assert(!lifecycle.completeTerminal());
  assert(lifecycle.beginStop());
  assert(lifecycle.completeTerminal());
  assert(!lifecycle.completeTerminal());
  assert(lifecycle.state() == State::TERMINAL);
  assert(lifecycle.reloadAllowed());
  assert(!lifecycle.reserveBridge());
  assert(!lifecycle.markBridgeActive());

  // Real concurrent race: stop/terminal and activation are released together.
  for (int iteration = 0; iteration < 64; ++iteration) {
    BridgeLifecycle contested;
    assert(contested.reserveBridge());
    std::promise<void> release;
    const std::shared_future<void> start = release.get_future().share();
    std::atomic<bool> activate_succeeded{false};
    std::atomic<bool> stop_succeeded{false};

    std::thread stopper([&] {
      start.wait();
      stop_succeeded.store(contested.beginStop(), std::memory_order_release);
    });
    std::thread activator([&] {
      start.wait();
      activate_succeeded.store(contested.markBridgeActive(),
                               std::memory_order_release);
    });
    release.set_value();
    stopper.join();
    activator.join();
    assert(contested.completeTerminal());

    assert(contested.state() == State::TERMINAL);
    assert(stop_succeeded.load(std::memory_order_acquire));
    assert(contested.reloadAllowed());
    assert(!contested.markBridgeActive());
    assert(!contested.reserveBridge());
    (void)activate_succeeded;
  }

  // Deterministic linearization A: activation commits before beginStop.
  {
    BridgeLifecycle ordered;
    assert(ordered.reserveBridge());
    std::promise<void> activate_go;
    std::promise<void> activated;
    std::promise<void> stop_go;
    std::atomic<bool> activate_succeeded{false};
    std::atomic<bool> stop_succeeded{false};
    std::thread activator([&] {
      activate_go.get_future().wait();
      activate_succeeded.store(ordered.markBridgeActive(),
                               std::memory_order_release);
      activated.set_value();
    });
    std::thread stopper([&] {
      stop_go.get_future().wait();
      stop_succeeded.store(ordered.beginStop(), std::memory_order_release);
    });
    activate_go.set_value();
    activated.get_future().wait();
    stop_go.set_value();
    activator.join();
    stopper.join();
    assert(activate_succeeded.load(std::memory_order_acquire));
    assert(stop_succeeded.load(std::memory_order_acquire));
    assert(ordered.state() == State::STOPPING);
    assert(ordered.completeTerminal());
  }

  // Deterministic linearization B: beginStop commits before activation.
  {
    BridgeLifecycle ordered;
    assert(ordered.reserveBridge());
    std::promise<void> stop_go;
    std::promise<void> stopped;
    std::atomic<bool> activate_succeeded{true};
    std::atomic<bool> stop_succeeded{false};
    std::thread stopper([&] {
      stop_go.get_future().wait();
      stop_succeeded.store(ordered.beginStop(), std::memory_order_release);
      stopped.set_value();
    });
    std::thread activator([&] {
      stopped.get_future().wait();
      activate_succeeded.store(ordered.markBridgeActive(),
                               std::memory_order_release);
    });
    stop_go.set_value();
    stopper.join();
    activator.join();
    assert(stop_succeeded.load(std::memory_order_acquire));
    assert(!activate_succeeded.load(std::memory_order_acquire));
    assert(ordered.state() == State::STOPPING);
    assert(ordered.completeTerminal());
  }
}
