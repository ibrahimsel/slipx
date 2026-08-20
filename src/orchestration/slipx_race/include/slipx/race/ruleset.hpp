// Copyright 2026 The SlipX Authors
// SPDX-License-Identifier: Apache-2.0
//
// The ruleset this component implements, pinned (ADR-0046).
//
// The rules are the competition's, not this project's: they are revised for
// every event, and paraphrasing them invites drift. So the repository and
// the exact revision implemented are constants the build carries and can
// print, rule numbers are cited at each implementation site, and none of the
// rulebook's text is vendored (the licence lesson of ADR-0035 applies to
// documents exactly as it does to track geometry). Updating to a newer
// ruleset is a deliberate change to these constants and to every cited rule
// number, never a silent drift.
//
// Where the rulebook says "referee", this component substitutes a mechanised
// judgment, and every such substitution is a named field of RaceConfig
// rather than a buried constant: automated racing cannot read intent, and
// pretending otherwise would be a fidelity claim nobody can back.

#ifndef SLIPX_RACE_RULESET_HPP
#define SLIPX_RACE_RULESET_HPP

#include <string>

namespace slipx {
namespace race {

// The versioned dependency: which published rules this build implements.
inline constexpr char kRulesetRepository[] =
    "https://github.com/f1tenth/roboracer_rules";
inline constexpr char kRulesetRevision[] =
    "202c3771465b1690c0e28618271cca91d5c842c9";
inline constexpr char kRulesetDate[] = "2025-10-13";

// One line for a tool or a manifest to print: the "build states its ruleset
// revision" obligation, as a function so nobody re-assembles it differently.
std::string ruleset_statement();

// The knobs of the mechanised ruleset. Rule numbers cite the pinned
// revision above. The thresholds marked "operationalised" stand in for a
// human referee's judgment; they are plausible, calibrated against nothing,
// and a competition adopting this is expected to set its own.
struct RaceConfig {
  // Head-to-head: the first car to complete this many laps wins the round
  // (2.5.4.1), and the first team to win this many rounds wins the match
  // (2.5.1.8: best of three).
  int laps_to_win = 20;
  int rounds_to_win = 2;

  // Contact at or below this closing speed is "light side-bumps and
  // slow-speed nudges" (2.5.1.14.2): recorded, never penalised.
  // Operationalised.                                                  [m/s]
  double light_contact_speed = 0.5;

  // After an at-fault crash both cars restart at the crash, the at-fault
  // car this far behind (2.5.1.14.5), plus the bonus when the other car is
  // still running, which is this simulator's reading of "autonomously
  // detects and recovers" (2.5.1.14.9): a car nobody can reach into either
  // recovers autonomously or is DNF.                                    [m]
  double restart_gap = 2.0;
  double recovery_bonus = 1.0;

  // Every at-fault crash draws a warning, and this many disqualify
  // (2.5.1.14.7-8). Stricter than a referee reserving warnings for malice;
  // automated racing cannot read intent and does not pretend to.
  int warnings_to_disqualify = 3;

  // Track limits corridor widening, both sides (2.5.3.1: touching the
  // border is not penalised; beyond the tolerance the car has crashed the
  // border and is placed at rest where it left, 2.5.3.3).              [m]
  double limit_tolerance = 0.05;

  // Grid start: side by side, separated by one car width (2.5.1.9.1-2). [m]
  double grid_gap = 0.30;

  // Below this speed a car has "come to a complete stop" for the obstacle
  // test (2.5.1.6.3). Operationalised: floating point never reaches an
  // exact zero.                                                       [m/s]
  double stop_speed = 0.05;

  // Which way round. The direction is race control's announcement, not a
  // property of the venue: forward is increasing arc length along the
  // centreline as declared, and with this set every procedure races the
  // reversed track instead (same start line on a closed track), so nothing
  // downstream carries a sign (ADR-0056). The pinned revision assumes a
  // direction was announced and never says how; this field is the
  // announcement, mechanised.
  bool reversed = false;

  // A car whose progress along the race direction falls this far behind
  // its own furthest point is ruled to be driving the wrong way, once per
  // excursion. Recorded, never penalised: the pinned revision has no
  // wrong-way rule, so the event states what a referee would see and a
  // competition decides what it costs. Operationalised.               [m]
  double wrong_way_distance = 1.0;
};

}  // namespace race
}  // namespace slipx

#endif  // SLIPX_RACE_RULESET_HPP
