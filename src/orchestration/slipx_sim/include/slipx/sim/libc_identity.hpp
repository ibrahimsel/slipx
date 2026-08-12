// Copyright 2026 The SlipX Authors
// SPDX-License-Identifier: Apache-2.0
//
// Which C library this process is running against.
//
// The trajectory hash tracks libm, not the compiler. sin, cos, atan and exp
// are not correctly rounded, their implementations change between library
// versions, and a run recorded under one of them is not entitled to agree with
// a run recorded under another. One wheel, byte for byte identical, produced
// two different hashes on glibc 2.28 and glibc 2.39; every other field the
// manifest records was the same across those two runs. So the C library is
// part of the build a result is keyed by (ADR-0033), and the manifest records
// it.
//
// Read at RUN time rather than baked in when CMake configures. The value has
// to describe the library the process is linked against, which for a
// redistributed wheel is chosen on the installing machine, long after the
// machine that compiled it stopped mattering.

#ifndef SLIPX_SIM_LIBC_IDENTITY_HPP
#define SLIPX_SIM_LIBC_IDENTITY_HPP

#include <string>

namespace slipx {
namespace sim {

struct LibcIdentity {
  // Short lower-case identifier: "glibc", "musl", "apple-libc", "ucrt",
  // "unknown". Never empty, because it is half of a lookup key.
  std::string id;
  // Version as the library reports it at run time, e.g. "2.39". Empty where
  // the platform offers no way to ask; an invented version would be worse
  // than an absent one, and the id alone still separates two platforms.
  std::string version;
};

LibcIdentity libc_identity();

}  // namespace sim
}  // namespace slipx

#endif  // SLIPX_SIM_LIBC_IDENTITY_HPP
