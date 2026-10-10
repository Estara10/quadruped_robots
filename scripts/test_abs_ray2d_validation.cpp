#include "abs_ray2d_validation.h"

#include <array>
#include <cassert>
#include <limits>

int main() {
  std::array<float, abs_ray2d_shm::kRayCount> rays{};
  rays.fill(1.0F);
  using abs_ray2d_shm::Validation;

  // Producer publishes after reader starts; validation clock is sampled after snapshot.
  assert(abs_ray2d_shm::coherentSequence(2, 2));
  assert(abs_ray2d_shm::validate(rays.data(), rays.size(), abs_ray2d_shm::kMagic,
      abs_ray2d_shm::kVersion, 101, 101, 200) == Validation::valid);
  // A genuinely future timestamp relative to the post-snapshot clock is rejected.
  assert(abs_ray2d_shm::validate(rays.data(), rays.size(), abs_ray2d_shm::kMagic,
      abs_ray2d_shm::kVersion, 102, 101, 200) == Validation::monotonic_clock_order);
  assert(abs_ray2d_shm::validate(rays.data(), rays.size(), abs_ray2d_shm::kMagic,
      abs_ray2d_shm::kVersion, 0, 101, 200) == Validation::unarmed_timestamp);
  assert(abs_ray2d_shm::validate(rays.data(), rays.size(), abs_ray2d_shm::kMagic,
      abs_ray2d_shm::kVersion, 1, 202, 200) == Validation::stale);
  assert(abs_ray2d_shm::validate(rays.data(), rays.size(), 0,
      abs_ray2d_shm::kVersion, 100, 101, 200) == Validation::header);
  rays[4] = std::numeric_limits<float>::quiet_NaN();
  assert(abs_ray2d_shm::validate(rays.data(), rays.size(), abs_ray2d_shm::kMagic,
      abs_ray2d_shm::kVersion, 100, 101, 200) == Validation::non_finite);
  assert(!abs_ray2d_shm::coherentSequence(2, 4));
  assert(!abs_ray2d_shm::coherentSequence(3, 3));
}
