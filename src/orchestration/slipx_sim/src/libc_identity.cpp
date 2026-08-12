// Copyright 2026 The SlipX Authors
// SPDX-License-Identifier: Apache-2.0

#include "slipx/sim/libc_identity.hpp"

// <cstdlib> first: on glibc it pulls in <features.h>, which is what defines
// __GLIBC__. Testing that macro before any libc header has been included
// would test a macro that is not defined yet, and would quietly report
// "unknown" on the one platform this is best able to answer for.
#include <cstdlib>

#if defined(__GLIBC__)
#include <gnu/libc-version.h>
#endif

namespace slipx {
namespace sim {

LibcIdentity libc_identity() {
  LibcIdentity out;
#if defined(__GLIBC__)
  out.id = "glibc";
  // The runtime version, not __GLIBC_MINOR__: a binary built against 2.31
  // headers and run against 2.39 is a 2.39 run, and that is the case the
  // whole record exists for.
  out.version = gnu_get_libc_version();
#elif defined(_WIN32)
  // The Universal CRT ships with the operating system and exposes no version
  // string a program can ask for that means what this field means.
  out.id = "ucrt";
#elif defined(__APPLE__)
  out.id = "apple-libc";
#elif defined(__linux__)
  // Linux without __GLIBC__ is musl in practice. musl deliberately publishes
  // no version macro and no version function, so the id stands alone.
  out.id = "musl";
#else
  out.id = "unknown";
#endif
  return out;
}

}  // namespace sim
}  // namespace slipx
