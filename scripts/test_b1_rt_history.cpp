#include <abs_rt_frame_contract.h>

#include <cstdio>
#include <cstring>
#include <limits>

using namespace abs_rt_frame;

static bool write_run(const char* path, uint64_t session, uint64_t count) {
  static HistoryBuffer buffer{};
  std::memset(&buffer, 0, sizeof(buffer));
  buffer.header.version = kHistoryVersion;
  buffer.header.capacity = kHistoryCapacity;
  buffer.header.session_id = session;
  __atomic_store_n(&buffer.header.magic, kHistoryMagic, __ATOMIC_RELEASE);
  for (uint64_t i = 0; i < count; ++i) {
    RuntimeFrame frame{};
    frame.header.magic = kMagic;
    frame.header.version = kVersion;
    frame.header.sequence = 2U * (i + 1U);
    frame.header.monotonic_ns = 1'000'000U + i;
    frame.session_id = session;
    frame.rl_step = i;
    publishHistory(&buffer, frame);
  }
  FILE* file = std::fopen(path, "wb");
  if (!file) return false;
  const bool ok = std::fwrite(&buffer, sizeof(buffer), 1, file) == 1;
  std::fclose(file);
  return ok;
}

static bool write_uncommitted(const char* path) {
  static HistoryBuffer buffer{};
  std::memset(&buffer, 0, sizeof(buffer));
  buffer.header.version = kHistoryVersion;
  buffer.header.capacity = kHistoryCapacity;
  buffer.header.session_id = 101;
  buffer.header.produced_count = 1;
  buffer.slots[0].sequence = 1;  // Simulate a writer holding an odd seqlock.
  buffer.slots[0].index = 0;
  __atomic_store_n(&buffer.header.magic, kHistoryMagic, __ATOMIC_RELEASE);
  FILE* file = std::fopen(path, "wb");
  if (!file) return false;
  const bool ok = std::fwrite(&buffer, sizeof(buffer), 1, file) == 1;
  std::fclose(file);
  return ok;
}

int main(int argc, char** argv) {
  if (argc != 5) return 2;
  return write_run(argv[1], 101, kHistoryCapacity + 2U) &&
         write_run(argv[2], 202, 2U) &&
         write_run(argv[3], 101, 2U) &&
         write_uncommitted(argv[4]) ? 0 : 1;
}
