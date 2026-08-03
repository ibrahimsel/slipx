// Copyright 2026 The SlipX Authors
// SPDX-License-Identifier: Apache-2.0
//
// slipx_conformance: run the canonical step steer and print its manifest.
//
// This is the executable the determinism CI job runs (NFR-02) and the one a
// third party runs to check their build against the published reference
// (the P0 exit gate). It prints the trajectory hash on the last line and
// nothing else there, so a shell script can compare it without parsing.
//
//   slipx_conformance                    print the manifest and the hash
//   slipx_conformance --expect <hash>    exit non-zero if it does not match
//   slipx_conformance --manifest <path>  also write the manifest as JSON
//   slipx_conformance --tier L0|L1       which tier to run
//   slipx_conformance --integrator rk4|semi_implicit_euler

#include <cstdio>
#include <cstring>
#include <string>

#include "slipx/sim/manoeuvres.hpp"

namespace {

void usage() {
  std::fprintf(stderr,
               "usage: slipx_conformance [--tier L0|L1] "
               "[--integrator rk4|semi_implicit_euler] [--expect HASH] "
               "[--manifest PATH] [--quiet]\n");
}

}  // namespace

int main(int argc, char** argv) {
  slipx::sim::ConformanceSpec spec;
  std::string expected;
  std::string manifest_path;
  bool quiet = false;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    const auto next = [&](const char* what) -> std::string {
      if (i + 1 >= argc) {
        std::fprintf(stderr, "slipx_conformance: %s needs a value\n", what);
        usage();
        std::exit(2);
      }
      return argv[++i];
    };

    if (arg == "--tier") {
      const std::string v = next("--tier");
      if (v == "L0") spec.tier = slipx::Tier::L0_Kinematic;
      else if (v == "L1") spec.tier = slipx::Tier::L1_Bicycle;
      else if (v == "L2") spec.tier = slipx::Tier::L2_DoubleTrack;
      else {
        // L3 is not implemented (CORE-16, P4). Refused rather than
        // substituted, as everywhere else.
        std::fprintf(stderr, "slipx_conformance: unknown or unavailable tier "
                             "'%s'\n", v.c_str());
        return 2;
      }
    } else if (arg == "--integrator") {
      const std::string v = next("--integrator");
      if (v == "rk4") spec.integrator = slipx::Integrator::kRK4;
      else if (v == "semi_implicit_euler")
        spec.integrator = slipx::Integrator::kSemiImplicitEuler;
      else {
        std::fprintf(stderr, "slipx_conformance: unknown integrator '%s'\n",
                     v.c_str());
        return 2;
      }
    } else if (arg == "--expect") {
      expected = next("--expect");
    } else if (arg == "--manifest") {
      manifest_path = next("--manifest");
    } else if (arg == "--quiet") {
      quiet = true;
    } else if (arg == "--help" || arg == "-h") {
      usage();
      return 0;
    } else {
      std::fprintf(stderr, "slipx_conformance: unknown argument '%s'\n",
                   arg.c_str());
      usage();
      return 2;
    }
  }

  slipx::sim::Simulation sim = slipx::sim::make_conformance_run(spec);
  sim.run_for(spec.duration);

  const slipx::sim::RunManifest manifest = sim.manifest();
  const std::string hash = manifest.trajectory_hash;

  if (!manifest_path.empty() && !manifest.write(manifest_path)) {
    std::fprintf(stderr, "slipx_conformance: could not write %s\n",
                 manifest_path.c_str());
    return 3;
  }

  if (!quiet) {
    std::fputs(manifest.to_json().c_str(), stdout);
    const slipx::VehicleState& s = sim.state(0);
    std::printf(
        "\nfinal state: x=%.9f y=%.9f yaw=%.9f vx=%.9f vy=%.9f r=%.9f\n",
        s.pos.x, s.pos.y, s.yaw, s.vel_body.x, s.vel_body.y, s.rates.z);
  }

  if (!expected.empty() && expected != hash) {
    std::fprintf(stderr,
                 "\nslipx_conformance: DETERMINISM CHECK FAILED\n"
                 "  expected %s\n"
                 "  got      %s\n"
                 "Within one build this is a bug (NFR-02). Across builds, "
                 "compilers or architectures it may be the documented "
                 "limitation (NFR-03): compare the build block of the "
                 "manifest before concluding anything.\n",
                 expected.c_str(), hash.c_str());
    return 1;
  }

  // Last line, on its own, so a shell script can take it verbatim.
  std::printf("%s\n", hash.c_str());
  return 0;
}
