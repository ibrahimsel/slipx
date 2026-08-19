// Copyright 2026 The SlipX Authors
// SPDX-License-Identifier: Apache-2.0

#include "slipx/race/leaderboard.hpp"

#include <algorithm>
#include <fstream>
#include <map>
#include <sstream>
#include <stdexcept>

#include "slipx/race/head_to_head.hpp"
#include "slipx/sense/rng.hpp"

namespace slipx {
namespace race {
namespace {

std::string json_escape(const std::string& s) {
  std::string out;
  for (const char c : s) {
    if (c == '"' || c == '\\') out.push_back('\\');
    out.push_back(c);
  }
  return out;
}

}  // namespace

std::vector<LeaderboardRow> standings(
    const std::vector<EventStreamContents>& streams) {
  // An ordered map on purpose: iteration order feeds the rows, and an
  // unordered container would make the leaderboard depend on hashing.
  std::map<std::string, LeaderboardRow> rows;

  for (const EventStreamContents& stream : streams) {
    const std::string names[2] = {stream.metadata_value("entrant.0"),
                                  stream.metadata_value("entrant.1")};
    for (const std::string& name : names) {
      rows[name].name = name;
      ++rows[name].matches;
    }

    bool decided = false;
    for (const RaceEvent& event : stream.events) {
      if (event.type == EventType::kRoundWon && event.agent < 2) {
        ++rows[names[event.agent]].round_wins;
      } else if (event.type == EventType::kMatchWon && event.agent < 2) {
        ++rows[names[event.agent]].match_wins;
        decided = true;
      }
    }
    if (!decided) {
      for (const std::string& name : names) ++rows[name].abandoned;
    }
  }

  std::vector<LeaderboardRow> out;
  out.reserve(rows.size());
  for (const auto& entry : rows) out.push_back(entry.second);
  std::sort(out.begin(), out.end(),
            [](const LeaderboardRow& a, const LeaderboardRow& b) {
              if (a.match_wins != b.match_wins) {
                return a.match_wins > b.match_wins;
              }
              if (a.round_wins != b.round_wins) {
                return a.round_wins > b.round_wins;
              }
              return a.name < b.name;
            });
  return out;
}

BatchResult run_round_robin(const scene::Track& track,
                            const std::vector<Entrant>& entrants,
                            const BatchConfig& config,
                            const std::string& directory) {
  BatchResult result;

  struct Scenario {
    std::size_t a, b;
    std::uint64_t seed;
    bool a_on_left;
    std::string path;
  };
  std::vector<Scenario> scenarios;

  std::uint64_t index = 0;
  for (std::size_t i = 0; i < entrants.size(); ++i) {
    for (std::size_t j = i + 1; j < entrants.size(); ++j) {
      for (int repetition = 0; repetition < config.repetitions;
           ++repetition) {
        Scenario scenario;
        scenario.a = i;
        scenario.b = j;
        scenario.seed = sense::derive_seed(config.master_seed, index);
        scenario.a_on_left = (repetition % 2) == 0;
        scenario.path = directory + "/match_" + std::to_string(index) +
                        ".mcap";
        scenarios.push_back(scenario);
        ++index;
      }
    }
  }

  for (const Scenario& scenario : scenarios) {
    sim::SimulationConfig sim_config;
    sim_config.master_seed = scenario.seed;
    sim::Simulation sim(sim_config);
    sim.add_agent(entrants[scenario.a].make_agent(track));
    sim.add_agent(entrants[scenario.b].make_agent(track));

    Match match(sim, track, 0, 1, config.line_s, scenario.a_on_left,
                scenario.seed, config.race);
    match.run(config.round_step_budget);

    const bool written = write_event_stream(
        scenario.path, match.events(), config.race,
        {{"entrant.0", entrants[scenario.a].name},
         {"entrant.1", entrants[scenario.b].name},
         {"scenario_seed", std::to_string(scenario.seed)},
         {"master_seed", std::to_string(config.master_seed)},
         {"configuration_digest", sim.manifest().configuration_digest()}});
    if (!written) {
      throw std::runtime_error("slipx_race: could not write " +
                               scenario.path);
    }
    result.stream_paths.push_back(scenario.path);
  }

  // The standings, computed the way a third party would: from the files on
  // disk, not from the Match objects this function just held. If the two
  // ever disagreed, the files are the truth and this catches it early.
  std::vector<EventStreamContents> streams(scenarios.size());
  for (std::size_t k = 0; k < scenarios.size(); ++k) {
    if (!read_event_stream(scenarios[k].path, &streams[k])) {
      throw std::runtime_error("slipx_race: could not read back " +
                               scenarios[k].path);
    }
  }
  result.rows = standings(streams);

  // The batch manifest: everything a re-run needs, so "reproducible from
  // its manifest and seeds" is a property of the file, not of the caller's
  // memory.
  {
    std::ostringstream o;
    o << "{\n";
    o << "  \"ruleset_repository\": \"" << kRulesetRepository << "\",\n";
    o << "  \"ruleset_revision\": \"" << kRulesetRevision << "\",\n";
    o << "  \"master_seed\": " << config.master_seed << ",\n";
    o << "  \"repetitions\": " << config.repetitions << ",\n";
    o << "  \"round_step_budget\": " << config.round_step_budget << ",\n";
    o << "  \"entrants\": [";
    for (std::size_t i = 0; i < entrants.size(); ++i) {
      o << (i ? ", " : "") << "\"" << json_escape(entrants[i].name) << "\"";
    }
    o << "],\n";
    o << "  \"scenarios\": [\n";
    for (std::size_t k = 0; k < scenarios.size(); ++k) {
      const Scenario& s = scenarios[k];
      o << "    {\"a\": " << s.a << ", \"b\": " << s.b << ", \"seed\": "
        << s.seed << ", \"a_on_left\": " << (s.a_on_left ? "true" : "false")
        << ", \"stream\": \"" << json_escape(s.path) << "\"}"
        << (k + 1 < scenarios.size() ? "," : "") << "\n";
    }
    o << "  ]\n";
    o << "}\n";

    result.manifest_path = directory + "/batch_manifest.json";
    std::ofstream file(result.manifest_path, std::ios::binary);
    if (!file || !(file << o.str())) {
      throw std::runtime_error("slipx_race: could not write " +
                               result.manifest_path);
    }
  }

  {
    std::ostringstream o;
    o << "{\n  \"standings\": [\n";
    for (std::size_t k = 0; k < result.rows.size(); ++k) {
      const LeaderboardRow& row = result.rows[k];
      o << "    {\"name\": \"" << json_escape(row.name)
        << "\", \"matches\": " << row.matches
        << ", \"match_wins\": " << row.match_wins
        << ", \"round_wins\": " << row.round_wins
        << ", \"abandoned\": " << row.abandoned << "}"
        << (k + 1 < result.rows.size() ? "," : "") << "\n";
    }
    o << "  ]\n}\n";

    result.leaderboard_path = directory + "/leaderboard.json";
    std::ofstream file(result.leaderboard_path, std::ios::binary);
    if (!file || !(file << o.str())) {
      throw std::runtime_error("slipx_race: could not write " +
                               result.leaderboard_path);
    }
  }

  return result;
}

}  // namespace race
}  // namespace slipx
