// Copyright 2026 The SlipX Authors
// SPDX-License-Identifier: Apache-2.0

#include "slipx/race/ruleset.hpp"

#include "slipx/race/events.hpp"

namespace slipx {
namespace race {

std::string ruleset_statement() {
  return std::string("ruleset: ") + kRulesetRepository + " @ " +
         kRulesetRevision + " (" + kRulesetDate + ")";
}

const char* to_string(EventType type) {
  switch (type) {
    case EventType::kRoundStart: return "round_start";
    case EventType::kLap: return "lap";
    case EventType::kContactLight: return "contact_light";
    case EventType::kCrash: return "crash";
    case EventType::kWarning: return "warning";
    case EventType::kDisqualified: return "disqualified";
    case EventType::kBorderCrash: return "border_crash";
    case EventType::kRestart: return "restart";
    case EventType::kDnf: return "dnf";
    case EventType::kRoundWon: return "round_won";
    case EventType::kMatchWon: return "match_won";
    case EventType::kObstaclePassed: return "obstacle_passed";
    case EventType::kObstacleFailed: return "obstacle_failed";
    case EventType::kHeatEnd: return "heat_end";
  }
  return "unknown";
}

}  // namespace race
}  // namespace slipx
