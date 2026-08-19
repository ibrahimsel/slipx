// Copyright 2026 The SlipX Authors
// SPDX-License-Identifier: Apache-2.0
//
// SIM-06: the run manifest.
//
// Every run emits one. It records what was simulated and what it was
// simulated with, and its hash is the thing two parties compare when they
// disagree about a result. The list of fields is not a wish list; it is the
// set of things that, if any one of them differs, make two runs
// incomparable:
//
//   schema and core versions   a model change is not a run difference
//   parameter digests          the same car, byte for byte
//   seeds                      the same noise
//   integrator and step        the same discretisation
//   git SHA                    the same source
//   compiler ID and flags      the same rounding (NFR-02)
//   platform and architecture  because NFR-03 does not promise across these
//   the C library              because the hash tracks libm (ADR-0033)
//
// The JSON writer is deliberately hand-rolled and about forty lines. A JSON
// dependency in slipx_sim would be harmless to CORE-01, but the manifest is
// the one artefact that has to be readable by a person adjudicating a
// leaderboard appeal five years from now, and a stable forty-line writer beats
// a library version they may not be able to install.

#ifndef SLIPX_SIM_MANIFEST_HPP
#define SLIPX_SIM_MANIFEST_HPP

#include <cstdint>
#include <string>
#include <vector>

#include "slipx/integrator.hpp"
#include "slipx/vehicle_model.hpp"

namespace slipx {
namespace sim {

// One entry per agent, so a race between differently configured cars is fully
// described rather than described on average.
struct AgentManifest {
  std::string name;              // caller-supplied label
  std::string tier;              // e.g. "L1_Bicycle"
  std::string params_digest;     // hash of the parameter set actually used
  std::uint64_t seed = 0;        // this agent's derived stream seed

  // How the agent's run ended: "running", or "dnf" with the cause and the
  // step spelled out (ADR-0042). A result, not configuration: a frozen car
  // in a trajectory whose manifest does not say why would send an
  // adjudicator hunting for a bug that is actually an event. Excluded from
  // configuration_digest for the same reason the trajectory hashes are.
  std::string status = "running";
  std::string dnf_cause;         // empty while running
  std::uint64_t dnf_step = 0;    // meaningless while running
};

struct RunManifest {
  // ------------------------------------------------------------- identity
  std::string slipx_core_version;
  // Version of the schema the parameters were parsed with, supplied by the
  // caller because the core and the sim never see slipx_schema (CORE-01).
  // Empty when the parameters were built in code rather than parsed, which is
  // itself worth recording.
  std::string schema_version;

  // --------------------------------------------------------- reproduction
  double dt = 1.0e-3;            // fixed step [s]                  (SIM-01)
  std::uint64_t steps = 0;       // steps advanced
  std::string integrator;        // "rk4" or "semi_implicit_euler"  (CORE-13)
  std::uint64_t master_seed = 0;

  // "deterministic" or "validation". A validation run is paced against a wall
  // clock, so its trajectory depends on how loaded the machine was, and the
  // determinism block below says so instead of repeating a promise that does
  // not hold for it.
  std::string run_mode = "deterministic";
  std::vector<AgentManifest> agents;

  // ------------------------------------------------------------ the build
  std::string compiler_id;
  std::string compiler_version;
  std::string cxx_flags;
  std::string build_type;
  std::string system_name;
  std::string system_processor;
  std::string git_sha;

  // The C library the run was executed against, read at run time rather than
  // captured at configure time: for a redistributed wheel the binary is fixed
  // when it is built and the C library is not chosen until it is installed.
  // libm is not correctly rounded and its results move between versions, so
  // two runs that differ here were never entitled to agree (ADR-0033).
  // libc_version is empty on platforms that offer no runtime version.
  std::string libc_id;
  std::string libc_version;

  // ----------------------------------------------------------- the result
  // Per-agent and whole-run trajectory hashes. The whole-run hash is what CI
  // compares; the per-agent hashes are what makes a failure diagnosable,
  // because they say which car diverged.
  std::vector<std::string> agent_trajectory_hashes;
  std::string trajectory_hash;

  // Whether ground truth was published during the run. Recorded because
  // ROS-03 requires a graded run to be able to prove it was not consuming it.
  // Always true in P0: there is no publishing layer yet.
  bool ground_truth_enabled = true;

  // Fills compiler_id, compiler_version, cxx_flags, build_type, system_name,
  // system_processor, git_sha and slipx_core_version from the generated build
  // info, and libc_id/libc_version by asking the C library at run time.
  // Called automatically by Simulation.
  void capture_build_info();

  // Digest over every field EXCEPT the result hashes. Two runs whose
  // configuration digests agree are runs that should produce the same
  // trajectory hash; when they do not, NFR-02 has been broken and this is how
  // that is told apart from someone having changed the car.
  std::string configuration_digest() const;

  // Pretty-printed JSON, two-space indent, stable key order.
  std::string to_json() const;

  // Writes to_json() to a path. Returns false if the file could not be
  // opened; does not throw, because a run that finished should not be lost to
  // a full disk without its result being reported first.
  bool write(const std::string& path) const;
};

}  // namespace sim
}  // namespace slipx

#endif  // SLIPX_SIM_MANIFEST_HPP
