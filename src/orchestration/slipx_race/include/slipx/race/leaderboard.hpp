// Copyright 2026 The SlipX Authors
// SPDX-License-Identifier: Apache-2.0
//
// The leaderboard harness: a seeded batch of head-to-head matches, standings
// computed from the event streams and from nothing else.
//
// Two design rules carry this file. First, the doctrine of the event stream
// task: anything a leaderboard needs must be an event or stream metadata, so
// `standings` takes parsed streams, not Match objects, and the harness holds
// itself to that by computing its own result the same way a third party
// would. Second, reproducibility is from the batch manifest and the seeds:
// every scenario's seed is derived from the master seed and the scenario's
// position, the manifest records all of it, and the same batch produces the
// same streams byte for byte.
//
// Entrants are code (a policy factory), because a control stack is not a
// value that fits in a file; what IS in the file is everything else.

#ifndef SLIPX_RACE_LEADERBOARD_HPP
#define SLIPX_RACE_LEADERBOARD_HPP

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include "slipx/race/event_stream.hpp"
#include "slipx/race/ruleset.hpp"
#include "slipx/scene/track.hpp"
#include "slipx/sim/simulation.hpp"

namespace slipx {
namespace race {

// One competitor: a name for the standings and a factory that builds its
// agent for one scenario. The factory is called once per scenario with the
// track, so a stateful controller starts every match fresh.
struct Entrant {
  std::string name;
  std::function<sim::AgentSpec(const scene::Track&)> make_agent;
};

struct BatchConfig {
  std::uint64_t master_seed = 0;
  // Each pair meets this many times, alternating who takes the left slot.
  int repetitions = 1;
  RaceConfig race{};
  double line_s = 0.0;                  // the start line's arc length    [m]
  std::uint64_t round_step_budget = 120000;
};

struct LeaderboardRow {
  std::string name;
  int matches = 0;
  int match_wins = 0;
  int round_wins = 0;
  int abandoned = 0;   // scenarios whose round budget ran out undecided
};

// Standings from parsed streams alone. Each stream's metadata names its two
// entrants ("entrant.0", "entrant.1"); wins are counted from kMatchWon and
// kRoundWon events. Rows are ordered by match wins, then round wins, then
// name, so the order is total and two runs can be compared with ==.
std::vector<LeaderboardRow> standings(
    const std::vector<EventStreamContents>& streams);

struct BatchResult {
  std::vector<LeaderboardRow> rows;
  // One stream per scenario, in scenario order, as written to disk.
  std::vector<std::string> stream_paths;
  std::string manifest_path;
  std::string leaderboard_path;
};

// Runs the full round-robin: every pair of entrants, `repetitions` times,
// each scenario seeded as derive_seed(master_seed, scenario_index) for both
// the simulation and the round-three coin flip. Writes into `directory`
// (which must exist): one event stream per scenario, `leaderboard.json`,
// and `batch_manifest.json` carrying the master seed, the per-scenario
// seeds and pairings, the ruleset and the race configuration, which is
// everything a re-run needs. Returns the standings computed from the
// streams it just wrote, read back from disk rather than trusted from
// memory. Throws std::runtime_error when a file cannot be written: a
// leaderboard that silently lost its evidence is worse than no leaderboard.
BatchResult run_round_robin(const scene::Track& track,
                            const std::vector<Entrant>& entrants,
                            const BatchConfig& config,
                            const std::string& directory);

}  // namespace race
}  // namespace slipx

#endif  // SLIPX_RACE_LEADERBOARD_HPP
