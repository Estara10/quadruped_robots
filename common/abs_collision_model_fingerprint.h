#pragma once

#include <mujoco/mujoco.h>

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

// Canonical collision-model fingerprint shared by the simulator and the
// offline scenario probe.  The serialization is deliberately simple and
// dependency-free: little-endian fixed-width integers and IEEE-754 binary64
// values, in mjModel geom-id order, hashed with SHA-256.
namespace abs_collision_model {

constexpr const char* kFingerprintSchema = "abs-go2-collision-model-fingerprint/v1";

namespace detail {

class Sha256 {
 public:
  Sha256() : state_({0x6a09e667U, 0xbb67ae85U, 0x3c6ef372U, 0xa54ff53aU,
                    0x510e527fU, 0x9b05688cU, 0x1f83d9abU, 0x5be0cd19U}) {}

  void update(const std::vector<uint8_t>& input) {
    for (uint8_t byte : input) {
      block_[block_len_++] = byte;
      if (block_len_ == 64) {
        transform();
        bit_len_ += 512;
        block_len_ = 0;
      }
    }
  }

  std::string finalHex() {
    const uint64_t total_bits = bit_len_ + static_cast<uint64_t>(block_len_) * 8U;
    block_[block_len_++] = 0x80;
    if (block_len_ > 56) {
      while (block_len_ < 64) block_[block_len_++] = 0;
      transform();
      block_len_ = 0;
    }
    while (block_len_ < 56) block_[block_len_++] = 0;
    for (int shift = 56; shift >= 0; shift -= 8) {
      block_[block_len_++] = static_cast<uint8_t>((total_bits >> shift) & 0xffU);
    }
    transform();

    std::ostringstream output;
    output << std::hex << std::setfill('0');
    for (uint32_t word : state_) output << std::setw(8) << word;
    return output.str();
  }

 private:
  static constexpr std::array<uint32_t, 64> kRound = {
      0x428a2f98U, 0x71374491U, 0xb5c0fbcfU, 0xe9b5dba5U,
      0x3956c25bU, 0x59f111f1U, 0x923f82a4U, 0xab1c5ed5U,
      0xd807aa98U, 0x12835b01U, 0x243185beU, 0x550c7dc3U,
      0x72be5d74U, 0x80deb1feU, 0x9bdc06a7U, 0xc19bf174U,
      0xe49b69c1U, 0xefbe4786U, 0x0fc19dc6U, 0x240ca1ccU,
      0x2de92c6fU, 0x4a7484aaU, 0x5cb0a9dcU, 0x76f988daU,
      0x983e5152U, 0xa831c66dU, 0xb00327c8U, 0xbf597fc7U,
      0xc6e00bf3U, 0xd5a79147U, 0x06ca6351U, 0x14292967U,
      0x27b70a85U, 0x2e1b2138U, 0x4d2c6dfcU, 0x53380d13U,
      0x650a7354U, 0x766a0abbU, 0x81c2c92eU, 0x92722c85U,
      0xa2bfe8a1U, 0xa81a664bU, 0xc24b8b70U, 0xc76c51a3U,
      0xd192e819U, 0xd6990624U, 0xf40e3585U, 0x106aa070U,
      0x19a4c116U, 0x1e376c08U, 0x2748774cU, 0x34b0bcb5U,
      0x391c0cb3U, 0x4ed8aa4aU, 0x5b9cca4fU, 0x682e6ff3U,
      0x748f82eeU, 0x78a5636fU, 0x84c87814U, 0x8cc70208U,
      0x90befffaU, 0xa4506cebU, 0xbef9a3f7U, 0xc67178f2U};

  static uint32_t rotr(uint32_t value, uint32_t bits) {
    return (value >> bits) | (value << (32U - bits));
  }

  void transform() {
    uint32_t words[64] = {};
    for (int i = 0; i < 16; ++i) {
      words[i] = (static_cast<uint32_t>(block_[i * 4]) << 24) |
                 (static_cast<uint32_t>(block_[i * 4 + 1]) << 16) |
                 (static_cast<uint32_t>(block_[i * 4 + 2]) << 8) |
                 static_cast<uint32_t>(block_[i * 4 + 3]);
    }
    for (int i = 16; i < 64; ++i) {
      const uint32_t s0 = rotr(words[i - 15], 7) ^ rotr(words[i - 15], 18) ^ (words[i - 15] >> 3);
      const uint32_t s1 = rotr(words[i - 2], 17) ^ rotr(words[i - 2], 19) ^ (words[i - 2] >> 10);
      words[i] = words[i - 16] + s0 + words[i - 7] + s1;
    }
    uint32_t a = state_[0], b = state_[1], c = state_[2], d = state_[3];
    uint32_t e = state_[4], f = state_[5], g = state_[6], h = state_[7];
    for (int i = 0; i < 64; ++i) {
      const uint32_t s1 = rotr(e, 6) ^ rotr(e, 11) ^ rotr(e, 25);
      const uint32_t choose = (e & f) ^ ((~e) & g);
      const uint32_t temp1 = h + s1 + choose + kRound[i] + words[i];
      const uint32_t s0 = rotr(a, 2) ^ rotr(a, 13) ^ rotr(a, 22);
      const uint32_t majority = (a & b) ^ (a & c) ^ (b & c);
      const uint32_t temp2 = s0 + majority;
      h = g; g = f; f = e; e = d + temp1;
      d = c; c = b; b = a; a = temp1 + temp2;
    }
    state_[0] += a; state_[1] += b; state_[2] += c; state_[3] += d;
    state_[4] += e; state_[5] += f; state_[6] += g; state_[7] += h;
  }

  std::array<uint32_t, 8> state_;
  std::array<uint8_t, 64> block_{};
  std::size_t block_len_ = 0;
  uint64_t bit_len_ = 0;
};

inline void appendU32(std::vector<uint8_t>& bytes, uint32_t value) {
  for (int shift = 0; shift < 32; shift += 8) bytes.push_back(static_cast<uint8_t>((value >> shift) & 0xffU));
}

inline void appendI32(std::vector<uint8_t>& bytes, int32_t value) {
  appendU32(bytes, static_cast<uint32_t>(value));
}

inline bool appendF64(std::vector<uint8_t>& bytes, double value) {
  if (!std::isfinite(value)) return false;
  uint64_t bits = 0;
  static_assert(sizeof(bits) == sizeof(value), "binary64 required");
  std::memcpy(&bits, &value, sizeof(bits));
  for (int shift = 0; shift < 64; shift += 8) bytes.push_back(static_cast<uint8_t>((bits >> shift) & 0xffU));
  return true;
}

inline bool appendString(std::vector<uint8_t>& bytes, const char* value) {
  const std::string text = value == nullptr ? std::string() : std::string(value);
  if (text.size() > UINT32_MAX) return false;
  appendU32(bytes, static_cast<uint32_t>(text.size()));
  bytes.insert(bytes.end(), text.begin(), text.end());
  return true;
}

}  // namespace detail

inline bool compute(const mjModel* model, std::string* fingerprint, std::string* error = nullptr) {
  if (model == nullptr || fingerprint == nullptr || model->ngeom < 0 || model->nbody < 0) {
    if (error) *error = "null_or_invalid_model";
    return false;
  }
  std::vector<uint8_t> bytes;
  bytes.insert(bytes.end(), kFingerprintSchema, kFingerprintSchema + std::strlen(kFingerprintSchema));
  bytes.push_back(0);
  detail::appendU32(bytes, static_cast<uint32_t>(model->nbody));
  detail::appendU32(bytes, static_cast<uint32_t>(model->ngeom));
  for (int geom_id = 0; geom_id < model->ngeom; ++geom_id) {
    if (model->geom_bodyid == nullptr || model->geom_type == nullptr ||
        model->geom_group == nullptr || model->geom_contype == nullptr ||
        model->geom_conaffinity == nullptr || model->geom_pos == nullptr ||
        model->geom_quat == nullptr || model->geom_size == nullptr) {
      if (error) *error = "missing_geom_field";
      return false;
    }
    const int body_id = model->geom_bodyid[geom_id];
    if (body_id < 0 || body_id >= model->nbody) {
      if (error) *error = "unknown_geom_body";
      return false;
    }
    detail::appendU32(bytes, static_cast<uint32_t>(geom_id));
    detail::appendI32(bytes, model->geom_type[geom_id]);
    detail::appendI32(bytes, body_id);
    detail::appendI32(bytes, model->geom_group[geom_id]);
    detail::appendI32(bytes, model->geom_contype[geom_id]);
    detail::appendI32(bytes, model->geom_conaffinity[geom_id]);
    for (int axis = 0; axis < 3; ++axis) if (!detail::appendF64(bytes, model->geom_pos[geom_id * 3 + axis])) return false;
    for (int axis = 0; axis < 4; ++axis) if (!detail::appendF64(bytes, model->geom_quat[geom_id * 4 + axis])) return false;
    for (int axis = 0; axis < 3; ++axis) if (!detail::appendF64(bytes, model->geom_size[geom_id * 3 + axis])) return false;
    if (!detail::appendString(bytes, mj_id2name(model, mjOBJ_GEOM, geom_id))) return false;
    if (!detail::appendString(bytes, mj_id2name(model, mjOBJ_BODY, body_id))) return false;
  }
  detail::Sha256 hash;
  hash.update(bytes);
  *fingerprint = hash.finalHex();
  return true;
}

}  // namespace abs_collision_model
