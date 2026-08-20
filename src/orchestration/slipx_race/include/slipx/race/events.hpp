// Copyright 2026 The SlipX Authors
// SPDX-License-Identifier: Apache-2.0
//
// Race events: every race-control outcome as a flat, timestamped record.
//
// Flat on purpose. The event stream task encodes these as MCAP so the run
// sinks and the race outcomes are one format; a struct of plain numbers
// serialises without a schema argument, and anything that needs prose can
// derive it from the type. Any leaderboard, report or CI job is meant to
// consume this stream and nothing else, so if a fact about a race matters,
// it must be an event here rather than a printf somewhere.

#ifndef SLIPX_RACE_EVENTS_HPP
#define SLIPX_RACE_EVENTS_HPP

#include <cstdint>

namespace slipx {
namespace race {

enum class EventType {
  kRoundStart,       // agent = left car, other = right car
  kLap,              // agent completed lap `code`; value = lap time [s]
  kContactLight,     // agent/other touched; value = closing speed [m/s]
  kCrash,            // agent is AT FAULT, other is the victim;
                     // value = closing speed [m/s]
  kWarning,          // agent warned; code = warnings so far
  kDisqualified,     // agent disqualified (2.5.1.14.8)
  kBorderCrash,      // agent left the corridor; value = margin beyond [m]
  kRestart,          // agent placed at rest; value = arc length s [m]
  kDnf,              // agent's simulation-level DNF observed by the race
  kRoundWon,         // agent wins the round; code = 1 when won by the
                     // opponent's DNF or disqualification
  kMatchWon,         // agent wins the match
  kObstaclePassed,   // agent cleared the obstacle test
  kObstacleFailed,   // agent failed; code: 1 stopped, 2 hit the obstacle,
                     // 3 crashed the border
  kHeatEnd,          // time trial heat over for agent
  kWrongWay,         // agent is driving against the race direction;
                     // value = metres behind its furthest point [m]
};

const char* to_string(EventType type);

inline constexpr std::uint32_t kNoAgent = 0xffffffffu;

struct RaceEvent {
  EventType type = EventType::kRoundStart;
  std::uint64_t step = 0;   // simulation step count at emission
  double time = 0.0;        // step * dt                               [s]
  std::uint32_t agent = kNoAgent;
  std::uint32_t other = kNoAgent;
  double value = 0.0;       // per-type meaning, documented above
  int code = 0;             // per-type meaning, documented above
};

}  // namespace race
}  // namespace slipx

#endif  // SLIPX_RACE_EVENTS_HPP
