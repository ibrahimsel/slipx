// Copyright 2026 The SlipX Authors
// SPDX-License-Identifier: Apache-2.0

#include "slipx/sim/manifest.hpp"

#include <cstdio>
#include <fstream>
#include <sstream>

#include "slipx/sim/build_info.hpp"
#include "slipx/sim/hash.hpp"
#include "slipx/sim/libc_identity.hpp"
#include "slipx/version.hpp"

namespace slipx {
namespace sim {
namespace {

std::string escape(const std::string& s) {
  std::string out;
  out.reserve(s.size() + 8);
  for (const char c : s) {
    switch (c) {
      case '"': out += "\\\""; break;
      case '\\': out += "\\\\"; break;
      case '\n': out += "\\n"; break;
      case '\r': out += "\\r"; break;
      case '\t': out += "\\t"; break;
      default: out += c;
    }
  }
  return out;
}

std::string quote(const std::string& s) { return "\"" + escape(s) + "\""; }

// Seventeen significant digits round-trips an IEEE-754 double exactly. The
// manifest is meant to reproduce a run, so a step size printed to six figures
// would be a manifest that cannot do its job.
std::string number(double v) {
  char buf[64];
  std::snprintf(buf, sizeof(buf), "%.17g", v);
  return buf;
}

std::string number(std::uint64_t v) {
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%llu", static_cast<unsigned long long>(v));
  return buf;
}

}  // namespace

void RunManifest::capture_build_info() {
  slipx_core_version = kVersion;
  compiler_id = kBuildCompilerId;
  compiler_version = kBuildCompilerVersion;
  cxx_flags = kBuildCxxFlags;
  build_type = kBuildType;
  system_name = kBuildSystemName;
  system_processor = kBuildSystemProcessor;
  git_sha = kBuildGitSha;
  // Not from build_info.hpp: that file is generated when CMake configures,
  // and this value has to describe the process, not the machine that built
  // it (ADR-0033).
  const LibcIdentity libc = libc_identity();
  libc_id = libc.id;
  libc_version = libc.version;
}

std::string RunManifest::configuration_digest() const {
  // Fixed field order, as with the trajectory hash: the order is part of the
  // value, and reordering it invalidates every stored digest.
  TrajectoryHash h;
  h.update(slipx_core_version);
  h.update(schema_version);
  h.update(dt);
  h.update(integrator);
  h.update_u64(master_seed);
  // Two runs in different modes are not comparable, whatever else matches:
  // one is a function of its inputs and the other of a machine's scheduler.
  h.update(run_mode);
  h.update(contact_restitution);
  h.update(contact_friction);
  h.update(contact_restitution_min_speed);
  for (const AgentManifest& a : agents) {
    h.update(a.name);
    h.update(a.tier);
    h.update(a.params_digest);
    h.update_u64(a.seed);
    h.update(a.footprint_length);
    h.update(a.footprint_width);
  }
  h.update(compiler_id);
  h.update(compiler_version);
  h.update(cxx_flags);
  h.update(build_type);
  h.update(system_name);
  h.update(system_processor);
  h.update(git_sha);
  h.update(libc_id);
  h.update(libc_version);
  // Deliberately excludes steps and the trajectory hashes: the digest answers
  // "was this the same setup", and the trajectory hash answers "did it give
  // the same answer". Folding the result into the configuration digest would
  // make the two questions inseparable.
  return h.hex();
}

std::string RunManifest::to_json() const {
  std::ostringstream o;
  o << "{\n";
  o << "  \"slipx_core_version\": " << quote(slipx_core_version) << ",\n";
  o << "  \"schema_version\": " << quote(schema_version) << ",\n";
  o << "  \"configuration_digest\": " << quote(configuration_digest())
    << ",\n";

  o << "  \"run\": {\n";
  o << "    \"dt\": " << number(dt) << ",\n";
  o << "    \"steps\": " << number(steps) << ",\n";
  o << "    \"duration\": " << number(dt * static_cast<double>(steps))
    << ",\n";
  o << "    \"integrator\": " << quote(integrator) << ",\n";
  o << "    \"master_seed\": " << number(master_seed) << ",\n";
  o << "    \"contact_restitution\": " << number(contact_restitution)
    << ",\n";
  o << "    \"contact_friction\": " << number(contact_friction) << ",\n";
  o << "    \"contact_restitution_min_speed\": "
    << number(contact_restitution_min_speed) << ",\n";
  o << "    \"ground_truth_enabled\": "
    << (ground_truth_enabled ? "true" : "false") << "\n";
  o << "  },\n";

  o << "  \"agents\": [\n";
  for (std::size_t i = 0; i < agents.size(); ++i) {
    const AgentManifest& a = agents[i];
    o << "    {\n";
    o << "      \"name\": " << quote(a.name) << ",\n";
    o << "      \"tier\": " << quote(a.tier) << ",\n";
    o << "      \"params_digest\": " << quote(a.params_digest) << ",\n";
    o << "      \"seed\": " << number(a.seed) << ",\n";
    o << "      \"footprint_length\": " << number(a.footprint_length)
      << ",\n";
    o << "      \"footprint_width\": " << number(a.footprint_width) << ",\n";
    o << "      \"status\": " << quote(a.status) << ",\n";
    if (a.status == "dnf") {
      o << "      \"dnf_cause\": " << quote(a.dnf_cause) << ",\n";
      o << "      \"dnf_step\": " << number(a.dnf_step) << ",\n";
    }
    o << "      \"trajectory_hash\": "
      << quote(i < agent_trajectory_hashes.size()
                   ? agent_trajectory_hashes[i]
                   : std::string())
      << "\n";
    o << "    }" << (i + 1 < agents.size() ? "," : "") << "\n";
  }
  o << "  ],\n";

  o << "  \"build\": {\n";
  o << "    \"compiler_id\": " << quote(compiler_id) << ",\n";
  o << "    \"compiler_version\": " << quote(compiler_version) << ",\n";
  o << "    \"cxx_flags\": " << quote(cxx_flags) << ",\n";
  o << "    \"build_type\": " << quote(build_type) << ",\n";
  o << "    \"system_name\": " << quote(system_name) << ",\n";
  o << "    \"system_processor\": " << quote(system_processor) << ",\n";
  o << "    \"git_sha\": " << quote(git_sha) << ",\n";
  o << "    \"libc_id\": " << quote(libc_id) << ",\n";
  o << "    \"libc_version\": " << quote(libc_version) << "\n";
  o << "  },\n";

  // The scope of the promise, restated in every manifest rather than only in
  // the documentation, because this file is what somebody reads when a result
  // does not reproduce and they are deciding whether that is a bug or the
  // promise working. "Same binary and same C library" rather than "same
  // build": for a redistributed wheel those are different things, and the
  // hash tracks libm (ADR-0033).
  o << "  \"determinism\": {\n";
  if (run_mode == "validation") {
    // Said plainly, and not left to the documentation. This file is what
    // somebody reads when a result did not reproduce, and a manifest that
    // claimed bit-identity for a run paced by a laptop's scheduler would be
    // the most damaging single line in this library.
    o << "    \"within_build\": \"NOT REPRODUCIBLE: this run was paced "
         "against a wall clock in validation mode, so its trajectory depends "
         "on the machine it ran on\",\n";
    o << "    \"across_platforms\": \"not guaranteed, and not meaningful for "
         "a validation run\"\n";
  } else {
    o << "    \"within_build\": \"bit-identical for the same binary on the "
         "same C library\",\n";
    o << "    \"across_platforms\": \"not guaranteed; conformance is to a "
         "stated tolerance\"\n";
  }
  o << "  },\n";

  o << "  \"trajectory_hash\": " << quote(trajectory_hash) << "\n";
  o << "}\n";
  return o.str();
}

bool RunManifest::write(const std::string& path) const {
  std::ofstream out(path);
  if (!out) return false;
  out << to_json();
  return static_cast<bool>(out);
}

}  // namespace sim
}  // namespace slipx
