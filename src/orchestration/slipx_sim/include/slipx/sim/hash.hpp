// Copyright 2026 The SlipX Authors
// SPDX-License-Identifier: Apache-2.0
//
// Trajectory hashing: the mechanism the determinism claim is checked with
// (NFR-02, SIM-07).
//
// A trajectory hash reduces an entire run to sixteen hex characters, which is
// what makes "the same run produces the same result" something CI can assert
// and a competition appeal can be settled with. Comparing trajectories
// point-by-point within a tolerance would not do: a tolerance is a place for
// nondeterminism to hide.
//
// The hash is FNV-1a over the raw bit patterns of the state doubles. FNV-1a is
// chosen for being trivial to reimplement correctly in another language,
// because the Python bindings and any future C ABI consumer must agree with it
// exactly. It is not a cryptographic hash and is not used as one: nothing here
// resists a determined forger, it detects divergence.
//
// Two deliberate details:
//
//   Negative zero is normalised to positive zero. IEEE-754 says -0.0 == 0.0
//   but their bit patterns differ, so a run that reached zero by decelerating
//   would otherwise hash differently from one that reached it by accelerating,
//   and the two states are the same state.
//
//   NaN is hashed as it lies. A NaN in a hashed trajectory means the run is
//   already broken, and silently canonicalising it would make two differently
//   broken runs agree.

#ifndef SLIPX_SIM_HASH_HPP
#define SLIPX_SIM_HASH_HPP

#include <cstdint>
#include <cstring>
#include <string>

#include "slipx/state.hpp"

namespace slipx {
namespace sim {

class TrajectoryHash {
 public:
  // FNV-1a 64-bit parameters, from the reference specification. Pinned here
  // because changing them invalidates every published reference hash.
  static constexpr std::uint64_t kOffsetBasis = 14695981039346656037ULL;
  static constexpr std::uint64_t kPrime = 1099511628211ULL;

  void update(double v) {
    if (v == 0.0) v = 0.0;  // normalises -0.0; see the header comment
    std::uint64_t bits = 0;
    std::memcpy(&bits, &v, sizeof(bits));
    update_u64(bits);
  }

  void update_u64(std::uint64_t bits) {
    for (int byte = 0; byte < 8; ++byte) {
      state_ ^= static_cast<std::uint64_t>((bits >> (byte * 8)) & 0xFFu);
      state_ *= kPrime;
    }
  }

  void update(const char* text) {
    for (const char* p = text; *p != '\0'; ++p) {
      state_ ^= static_cast<std::uint64_t>(static_cast<unsigned char>(*p));
      state_ *= kPrime;
    }
  }

  void update(const std::string& text) { update(text.c_str()); }

  // Hashes the full VehicleState in a FIXED field order. The order is part of
  // the hash: reordering these lines changes every reference value, so a
  // change here is a versioned, deliberate act.
  void update(const VehicleState& s) {
    update(s.pos.x);
    update(s.pos.y);
    update(s.pos.z);
    update(s.yaw);
    update(s.pitch);
    update(s.roll);
    update(s.vel_body.x);
    update(s.vel_body.y);
    update(s.vel_body.z);
    update(s.rates.x);
    update(s.rates.y);
    update(s.rates.z);
    for (unsigned i = 0; i < kWheelCount; ++i) update(s.omega_w[i]);
    update(s.steer);
    update(s.steer_rate);
    update(s.soc);
    update(s.pack_v);
    for (unsigned i = 0; i < kWheelCount; ++i) update(s.Fz[i]);
  }

  std::uint64_t value() const { return state_; }

  // Lower-case, zero-padded, sixteen characters. The form that appears in CI
  // logs and in manifests, so it is produced in exactly one place.
  std::string hex() const {
    static const char* digits = "0123456789abcdef";
    std::string out(16, '0');
    for (int i = 0; i < 16; ++i) {
      out[static_cast<std::size_t>(15 - i)] =
          digits[(state_ >> (i * 4)) & 0xFu];
    }
    return out;
  }

 private:
  std::uint64_t state_ = kOffsetBasis;
};

// Convenience for hashing a short piece of text on its own, used by the
// manifest for file digests and version strings.
inline std::string hash_text(const std::string& text) {
  TrajectoryHash h;
  h.update(text);
  return h.hex();
}

}  // namespace sim
}  // namespace slipx

#endif  // SLIPX_SIM_HASH_HPP
