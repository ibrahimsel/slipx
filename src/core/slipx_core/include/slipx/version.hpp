// Copyright 2026 The SlipX Authors
// SPDX-License-Identifier: Apache-2.0
//
// Version of slipx_core's public API. Versioned independently of
// slipx_schema (NFR-09): a schema addition that the core does not see must not
// force a core release, and a core ABI break must not silently invalidate
// every car file.
//
// This version string is one of the inputs to the run manifest hash (SIM-06),
// so a replay produced by a different core cannot compare equal by accident.

#ifndef SLIPX_VERSION_HPP
#define SLIPX_VERSION_HPP

namespace slipx {

inline constexpr int kVersionMajor = 0;
inline constexpr int kVersionMinor = 1;
inline constexpr int kVersionPatch = 0;
inline constexpr const char* kVersion = "0.1.0a1";

// Zero major version: the public API is not yet stable, and semantic
// versioning's guarantees begin at 1.0.0. Said here rather than only in the
// README so that a consumer reading the header knows what they are pinning.
//
// The triple and the string are deliberately not the same information. The
// triple is what a consumer compares in a preprocessor conditional, so it
// stays numeric. The string carries the pre-release suffix, because it is the
// one that reaches the run manifest (SIM-06) and the published distribution,
// and two artefacts recording the same core version there have to be the same
// core. tools/version_check.py refuses a build where they disagree.

}  // namespace slipx

#endif  // SLIPX_VERSION_HPP
