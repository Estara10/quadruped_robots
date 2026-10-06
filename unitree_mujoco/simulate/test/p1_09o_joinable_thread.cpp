#include <atomic>
#include <cassert>
#include <chrono>
#include <thread>

#include "../src/joinable_thread.h"

int main() {
  { JoinableThread never_started([] {}); }

  std::atomic<int> runs{0};
  JoinableThread* thread_ptr = nullptr;
  JoinableThread worker([&] {
    while (!thread_ptr->stopRequested()) {
      ++runs;
      thread_ptr->waitForStop(std::chrono::hours(1));
    }
  });
  thread_ptr = &worker;
  assert(worker.start());
  assert(!worker.start());
  std::this_thread::sleep_for(std::chrono::milliseconds(1));
  worker.requestStop();
  worker.requestStop();
  worker.stopAndJoin();
  worker.stopAndJoin();
  assert(!worker.joinable());
  assert(runs.load() > 0);
  assert(!worker.start());
  worker.stopAndJoin();
  assert(!worker.joinable());
}
