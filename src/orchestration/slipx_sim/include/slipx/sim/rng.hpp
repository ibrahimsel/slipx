// Copyright 2026 The SlipX Authors
// SPDX-License-Identifier: Apache-2.0
//
// Per-agent seeded random numbers (SIM-03).
//
// Nothing in slipx_core uses this, and nothing in slipx_core ever will
// (CORE-04): the vehicle model is a deterministic function of its arguments,
// which is what makes a trajectory hash meaningful. Randomness enters above
// the core, in sensor noise, dropouts, actuator jitter and scenario
// generation, and it enters through this type so that it is always seeded and
// always attributable.
//
// Why not <random>:
//
//   The distributions in <random> are not specified to produce identical
//   sequences across implementations. std::uniform_real_distribution and
//   std::normal_distribution give different numbers on libstdc++, libc++ and
//   MSVC for the same engine and seed. That is standard-conforming and it is
//   fatal to NFR-02 and to the cross-platform conformance suite of NFR-03, so
//   both the engine and the distributions are written out here.
//
// The engine is SplitMix64: 64 bits of state, one multiply-xorshift round,
// passes BigCrush, and short enough to reimplement in the Python bindings and
// verify against these tests. It is not cryptographic and does not need to be.
//
// Seeding: each agent's stream is derived from (master_seed, agent_index) by
// mixing, not by adding. Adding gives adjacent agents adjacent streams, and
// adjacent SplitMix64 streams are correlated in their first few outputs, which
// shows up as twenty cars whose sensor noise looks suspiciously similar in the
// first millisecond of a race.

#ifndef SLIPX_SIM_RNG_HPP
#define SLIPX_SIM_RNG_HPP

#include <cmath>
#include <cstdint>

namespace slipx {
namespace sim {

class Rng {
 public:
  explicit Rng(std::uint64_t seed = 0) : state_(seed) {}

  std::uint64_t next_u64() {
    state_ += 0x9E3779B97F4A7C15ULL;  // golden-ratio odd increment
    std::uint64_t z = state_;
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
    return z ^ (z >> 31);
  }

  // Uniform in [0, 1). Built from the top 53 bits, which is exactly the
  // mantissa width of a double, so every representable value in the range is
  // reachable and none is favoured.
  double uniform() {
    return static_cast<double>(next_u64() >> 11) * (1.0 / 9007199254740992.0);
  }

  double uniform(double lo, double hi) { return lo + (hi - lo) * uniform(); }

  // Standard normal by the polar Box-Muller method. Rejection-based, so it
  // consumes a variable number of engine outputs; that is deterministic given
  // the seed, but it means an agent's stream position depends on how many
  // normals it has drawn. Streams are per agent precisely so that this cannot
  // couple one agent's draw count to another's numbers.
  double normal() {
    if (has_spare_) {
      has_spare_ = false;
      return spare_;
    }
    double u = 0.0;
    double v = 0.0;
    double s = 0.0;
    do {
      u = uniform(-1.0, 1.0);
      v = uniform(-1.0, 1.0);
      s = u * u + v * v;
    } while (s >= 1.0 || s == 0.0);

    const double factor = std::sqrt(-2.0 * std::log(s) / s);
    spare_ = v * factor;
    has_spare_ = true;
    return u * factor;
  }

  double normal(double mean, double stddev) {
    return mean + stddev * normal();
  }

  std::uint64_t state() const { return state_; }

 private:
  std::uint64_t state_;
  double spare_ = 0.0;
  bool has_spare_ = false;
};

// Derives one agent's stream from the run seed. Mixing rather than adding: see
// the header comment.
inline std::uint64_t derive_seed(std::uint64_t master_seed,
                                 std::uint64_t stream_index) {
  std::uint64_t z = master_seed + 0x9E3779B97F4A7C15ULL * (stream_index + 1);
  z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
  z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
  return z ^ (z >> 31);
}

}  // namespace sim
}  // namespace slipx

#endif  // SLIPX_SIM_RNG_HPP
