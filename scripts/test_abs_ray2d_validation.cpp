#include "abs_ray2d_validation.h"

#include <algorithm>
#include <array>
#include <cassert>
#include <cstdint>
#include <limits>
#include <vector>

struct FakeSource {
  std::vector<uint64_t> sequences;
  size_t sequence_index{0};
  uint64_t frame_magic{abs_ray2d_shm::kMagic};
  uint64_t frame_version{abs_ray2d_shm::kVersion};
  uint64_t frame_stamp{101};
  std::array<float, abs_ray2d_shm::kRayCount> stable_payload{};
  std::array<float, abs_ray2d_shm::kRayCount> torn_payload{};
  uint32_t copies{0};
  bool use_torn_payload{false};

  uint64_t sequence() {
    if (sequences.empty()) return 1;
    const size_t index = sequence_index < sequences.size() ? sequence_index++ : sequences.size() - 1;
    return sequences[index];
  }
  uint64_t magic() const { return frame_magic; }
  uint64_t version() const { return frame_version; }
  uint64_t stamp() const { return frame_stamp; }
  void copy(float* out) {
    const auto& payload = copies++ == 0 && use_torn_payload ? torn_payload : stable_payload;
    std::copy(payload.begin(), payload.end(), out);
  }
};

int main() {
  using namespace abs_ray2d_shm;
  std::array<float, kRayCount> values{};
  for (int i = 0; i < kRayCount; ++i) values[i] = static_cast<float>(i) / 10.0F;

  // A writer initially owns the frame; after yielding, one stable complete
  // frame is accepted. Validation time is acquired only after its payload copy.
  FakeSource late_writer;
  late_writer.sequences = {1, 2, 2};
  late_writer.stable_payload = values;
  uint32_t yields = 0;
  const auto after_snapshot = readConsistentSnapshot(
      late_writer, [&] { assert(late_writer.copies == 1); return 101ULL; },
      [&] { ++yields; }, 16, 250000, 200);
  assert(after_snapshot.status == ReadStatus::stable);
  assert(after_snapshot.validation == Validation::valid);
  assert(after_snapshot.attempts == 2 && yields == 1);
  assert(after_snapshot.sequence_first == 1 && after_snapshot.sequence_last == 2);
  assert(after_snapshot.rays == values);

  // A sequence change during payload copy discards the torn values and rereads.
  FakeSource torn;
  torn.sequences = {2, 4, 4, 4};
  torn.stable_payload = values;
  torn.torn_payload.fill(99.0F);
  torn.use_torn_payload = true;
  const auto reread = readConsistentSnapshot(torn, [] { return 101ULL; }, [] {}, 16, 250000, 200);
  assert(reread.status == ReadStatus::stable && reread.validation == Validation::valid);
  assert(reread.attempts == 2 && torn.copies == 2 && reread.rays == values);
  assert(reread.sequence_first == 2 && reread.sequence_last == 4);

  // Persistent unarmed, writing, and changing sources fail within fixed bounds.
  FakeSource zero;
  zero.sequences = {0};
  const auto zero_result = readConsistentSnapshot(zero, [] { return 101ULL; }, [] {}, 16, 250000, 200);
  assert(zero_result.status == ReadStatus::unarmed_sequence && zero_result.attempts == 16);
  FakeSource odd;
  odd.sequences = {3};
  const auto odd_result = readConsistentSnapshot(odd, [] { return 101ULL; }, [] {}, 16, 250000, 200);
  assert(odd_result.status == ReadStatus::writer_in_progress && odd_result.attempts == 16);
  FakeSource changing;
  changing.sequences = {2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32,
                        34, 36, 38, 40, 42, 44, 46, 48, 50, 52, 54, 56, 58, 60, 62, 64};
  const auto changing_result = readConsistentSnapshot(
      changing, [] { return 101ULL; }, [] {}, 16, 250000, 200);
  assert(changing_result.status == ReadStatus::incoherent_snapshot && changing_result.attempts == 16);

  auto validate_stable = [&](FakeSource source, uint64_t now, uint64_t timeout = 200) {
    source.sequences = {2, 2};
    return readConsistentSnapshot(source, [=] { return now; }, [] {}, 16, 250000, timeout);
  };
  FakeSource legal;
  legal.stable_payload = values;
  auto legal_result = validate_stable(legal, 101);
  assert(legal_result.validation == Validation::valid && legal_result.rays == values);
  FakeSource wrong_dimension;
  wrong_dimension.sequences = {2, 2};
  wrong_dimension.stable_payload = values;
  const auto wrong_dimension_result = readConsistentSnapshot(
      wrong_dimension, [] { return 101ULL; }, [] {}, 16, 250000, 200, 10);
  assert(wrong_dimension_result.status == ReadStatus::stable &&
         wrong_dimension_result.validation == Validation::header);

  FakeSource future;
  assert(validate_stable(future, 100).validation == Validation::monotonic_clock_order);
  FakeSource stale;
  stale.frame_stamp = 1;
  assert(validate_stable(stale, 202).validation == Validation::stale);
  FakeSource bad_magic;
  bad_magic.frame_magic = 0;
  assert(validate_stable(bad_magic, 101).validation == Validation::header);
  FakeSource bad_version;
  bad_version.frame_version = kVersion + 1;
  assert(validate_stable(bad_version, 101).validation == Validation::header);
  FakeSource zero_stamp;
  zero_stamp.frame_stamp = 0;
  assert(validate_stable(zero_stamp, 101).validation == Validation::unarmed_timestamp);
  FakeSource nan_ray;
  nan_ray.stable_payload = values;
  nan_ray.stable_payload[4] = std::numeric_limits<float>::quiet_NaN();
  assert(validate_stable(nan_ray, 101).validation == Validation::non_finite);
  FakeSource inf_ray;
  inf_ray.stable_payload = values;
  inf_ray.stable_payload[4] = std::numeric_limits<float>::infinity();
  assert(validate_stable(inf_ray, 101).validation == Validation::non_finite);

  assert(coherentSequence(2, 2));
  assert(!coherentSequence(0, 0));
  assert(!coherentSequence(3, 3));
  assert(!coherentSequence(2, 4));
  return 0;
}
