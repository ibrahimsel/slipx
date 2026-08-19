// Copyright 2026 The SlipX Authors
// SPDX-License-Identifier: Apache-2.0

#include "slipx/race/head_to_head.hpp"

#include <cmath>

#include "slipx/race/starts.hpp"
#include "slipx/sense/rng.hpp"

namespace slipx {
namespace race {

HeadToHeadRound::HeadToHeadRound(sim::Simulation& sim,
                                 const scene::Track& track, std::size_t car_a,
                                 std::size_t car_b, double line_s,
                                 bool a_on_left, std::array<int, 2> warnings,
                                 RaceConfig config)
    : sim_(sim),
      track_(track),
      car_a_(car_a),
      car_b_(car_b),
      config_(config),
      counters_{scene::LapCounter(track, config.limit_tolerance),
                scene::LapCounter(track, config.limit_tolerance)},
      warnings_(warnings) {
  const std::size_t left = a_on_left ? car_a : car_b;
  const std::size_t right = a_on_left ? car_b : car_a;
  grid_start(sim_, track_, left, right, line_s, config_.grid_gap);

  for (std::size_t which = 0; which < 2; ++which) {
    const VehicleState& state = sim_.state(sim_index(which));
    counters_[which].reset_to(state.pos.x, state.pos.y);
    lap_start_time_[which] = sim_.time();
  }
  emit(EventType::kRoundStart, static_cast<std::uint32_t>(left),
       static_cast<std::uint32_t>(right), line_s, 0);
}

int HeadToHeadRound::laps(std::size_t which) const {
  return counters_[which].laps();
}

void HeadToHeadRound::emit(EventType type, std::uint32_t agent,
                           std::uint32_t other, double value, int code) {
  RaceEvent event;
  event.type = type;
  event.step = sim_.step_count();
  event.time = sim_.time();
  event.agent = agent;
  event.other = other;
  event.value = value;
  event.code = code;
  events_.push_back(event);
}

void HeadToHeadRound::decide(std::size_t winner, int code) {
  finished_ = true;
  winner_ = winner;
  emit(EventType::kRoundWon, static_cast<std::uint32_t>(sim_index(winner)),
       static_cast<std::uint32_t>(sim_index(1 - winner)), 0.0, code);
}

// The border, enforced by rule rather than by physics (nothing in SlipX
// collides a car with a wall): beyond the tolerance is a border crash
// (2.5.3.1) and the car is placed at rest where it left (2.5.3.3). Not a
// warning: warnings are for cars that hit each other.
void HeadToHeadRound::handle_border(std::size_t which) {
  const bool inside = counters_[which].limits().inside;
  if (was_inside_[which] && !inside) {
    const std::uint32_t agent =
        static_cast<std::uint32_t>(sim_index(which));
    emit(EventType::kBorderCrash, agent, kNoAgent,
         counters_[which].limits().margin, 0);
    const double s = counters_[which].where().s;
    place_on_track(sim_, sim_index(which), track_, s, 0.0, 0.0);
    const VehicleState& placed = sim_.state(sim_index(which));
    counters_[which].update(placed.pos.x, placed.pos.y);
    emit(EventType::kRestart, agent, kNoAgent, s, 0);
  }
  was_inside_[which] = counters_[which].limits().inside;
}

void HeadToHeadRound::handle_contact() {
  const std::uint32_t lo =
      static_cast<std::uint32_t>(car_a_ < car_b_ ? car_a_ : car_b_);
  const std::uint32_t hi =
      static_cast<std::uint32_t>(car_a_ < car_b_ ? car_b_ : car_a_);

  bool touching_now = false;
  for (const sim::ContactEvent& contact : sim_.contacts()) {
    if (contact.a != lo || contact.b != hi) continue;
    touching_now = true;

    const double closing = contact.approach_a + contact.approach_b;
    if (closing <= config_.light_contact_speed) {
      // "Light side-bumps and slow-speed nudges are not penalised and do
      // not stop the race" (2.5.1.14.2). Recorded once per touch episode,
      // or side-by-side rubbing would flood the stream.
      if (!in_contact_) {
        emit(EventType::kContactLight, contact.a, contact.b, closing, 0);
      }
      continue;
    }

    // A crash. The referee's fault call (2.5.1.14.4), mechanised: the car
    // contributing more approach speed at the contact is at fault, and an
    // exact tie goes against the car behind on track, which is racing's
    // overtaker-responsibility convention.
    const double approach_of_a =
        (contact.a == static_cast<std::uint32_t>(car_a_)) ? contact.approach_a
                                                          : contact.approach_b;
    const double approach_of_b =
        (contact.a == static_cast<std::uint32_t>(car_a_)) ? contact.approach_b
                                                          : contact.approach_a;
    std::size_t fault;
    if (approach_of_a > approach_of_b) {
      fault = 0;
    } else if (approach_of_b > approach_of_a) {
      fault = 1;
    } else {
      fault = counters_[0].distance() <= counters_[1].distance() ? 0 : 1;
    }
    const std::size_t victim = 1 - fault;

    emit(EventType::kCrash, static_cast<std::uint32_t>(sim_index(fault)),
         static_cast<std::uint32_t>(sim_index(victim)), closing, 0);

    // Every at-fault crash draws a warning; the third disqualifies
    // (2.5.1.14.7-8). Stricter than a human referee, and said so where the
    // thresholds live (ruleset.hpp).
    ++warnings_[fault];
    emit(EventType::kWarning, static_cast<std::uint32_t>(sim_index(fault)),
         kNoAgent, 0.0, warnings_[fault]);
    if (warnings_[fault] >= config_.warnings_to_disqualify) {
      dq_[fault] = true;
      emit(EventType::kDisqualified,
           static_cast<std::uint32_t>(sim_index(fault)), kNoAgent, 0.0, 0);
      decide(victim, 1);
      return;
    }

    // The restart of 2.5.1.14.5: both cars stopped at the crash location,
    // the at-fault car set back, with the extra metre of 2.5.1.14.9 when
    // the victim is still running (this simulator's reading of "recovers
    // autonomously": a car nobody can reach into either recovers by itself
    // or is DNF, which the caller sees handled before contact).
    const bool victim_running = sim_.agent_running(sim_index(victim));
    const double s0 = counters_[victim].where().s;
    if (victim_running) {
      place_on_track(sim_, sim_index(victim), track_, s0, 0.0, 0.0);
      const VehicleState& placed = sim_.state(sim_index(victim));
      counters_[victim].update(placed.pos.x, placed.pos.y);
      emit(EventType::kRestart,
           static_cast<std::uint32_t>(sim_index(victim)), kNoAgent, s0, 0);
    }
    const double setback =
        config_.restart_gap + (victim_running ? config_.recovery_bonus : 0.0);
    place_on_track(sim_, sim_index(fault), track_, s0 - setback, 0.0, 0.0);
    const VehicleState& placed = sim_.state(sim_index(fault));
    counters_[fault].update(placed.pos.x, placed.pos.y);
    emit(EventType::kRestart, static_cast<std::uint32_t>(sim_index(fault)),
         kNoAgent, s0 - setback, 0);

    in_contact_ = false;   // the teleport separated them
    return;                // one crash per step settles the step
  }
  in_contact_ = touching_now;
}

void HeadToHeadRound::advance() {
  if (finished_) return;
  sim_.advance();

  // A DNF ends the round for the other car's benefit: the rulebook gives a
  // stopped car ten minutes to be repaired (2.5.1.8), and a rolled or
  // timed-out car in this simulator is not coming back inside any window.
  for (std::size_t which = 0; which < 2; ++which) {
    if (!sim_.agent_running(sim_index(which)) && !dnf_noted_[which]) {
      dnf_noted_[which] = true;
      emit(EventType::kDnf, static_cast<std::uint32_t>(sim_index(which)),
           kNoAgent, 0.0, 0);
      decide(1 - which, 1);
      return;
    }
  }

  for (std::size_t which = 0; which < 2; ++which) {
    const VehicleState& state = sim_.state(sim_index(which));
    counters_[which].update(state.pos.x, state.pos.y);
  }

  for (std::size_t which = 0; which < 2; ++which) handle_border(which);

  // Laps, and the win (2.5.4.1: the first car to complete the count). When
  // both cross on the same step, the one further along wins, and an exact
  // tie in distance goes to the lower index, deterministically.
  std::array<bool, 2> reached{false, false};
  for (std::size_t which = 0; which < 2; ++which) {
    if (counters_[which].laps() > laps_done_[which]) {
      laps_done_[which] = counters_[which].laps();
      const double lap_time = sim_.time() - lap_start_time_[which];
      emit(EventType::kLap, static_cast<std::uint32_t>(sim_index(which)),
           kNoAgent, lap_time, laps_done_[which]);
      lap_start_time_[which] = sim_.time();
    }
    reached[which] = laps_done_[which] >= config_.laps_to_win;
  }
  if (reached[0] || reached[1]) {
    std::size_t winner;
    if (reached[0] && reached[1]) {
      winner = counters_[0].distance() >= counters_[1].distance() ? 0 : 1;
    } else {
      winner = reached[0] ? 0 : 1;
    }
    decide(winner, 0);
    return;
  }

  handle_contact();
}

void HeadToHeadRound::run(std::uint64_t max_steps) {
  for (std::uint64_t i = 0; i < max_steps && !finished_; ++i) advance();
}

Match::Match(sim::Simulation& sim, const scene::Track& track,
             std::size_t car_a, std::size_t car_b, double line_s,
             bool a_on_left_first, std::uint64_t seed, RaceConfig config)
    : sim_(sim),
      track_(track),
      car_a_(car_a),
      car_b_(car_b),
      line_s_(line_s),
      a_on_left_first_(a_on_left_first),
      seed_(seed),
      config_(config) {}

void Match::run(std::uint64_t max_steps_per_round) {
  const int max_rounds = 2 * config_.rounds_to_win - 1;

  while (!finished_ && rounds_played_ < max_rounds) {
    // Sides: the higher qualifier's choice in round one (the caller's
    // a_on_left_first), swapped in round two (2.5.1.9.3), and the round
    // three coin flip (2.5.1.9.4) mechanised as a seeded draw, so a
    // replayed match is the same match.
    bool a_left = a_on_left_first_;
    if (rounds_played_ == 1) a_left = !a_left;
    if (rounds_played_ >= 2) {
      a_left = (sense::derive_seed(seed_, 3) & 1u) != 0u;
    }

    // The reset is the ten-minute repair window (2.5.1.8), compressed: a
    // car that rolled or timed out last round starts the next one whole.
    sim_.reset();
    HeadToHeadRound round(sim_, track_, car_a_, car_b_, line_s_, a_left,
                          warnings_, config_);
    round.run(max_steps_per_round);
    ++rounds_played_;

    events_.insert(events_.end(), round.events().begin(),
                   round.events().end());

    if (!round.finished()) {
      // The budget ran out with nobody at the lap count. The rulebook has
      // no rule for a race that never ends; the match is abandoned rather
      // than adjudicated by an invented one.
      return;
    }

    warnings_ = {round.warnings(0), round.warnings(1)};

    if (round.disqualified(0) || round.disqualified(1)) {
      // Disqualification is from the race, not the round (2.5.1.14.8).
      finished_ = true;
      winner_ = round.disqualified(0) ? 1 : 0;
      break;
    }

    ++round_wins_[round.winner()];
    if (round_wins_[round.winner()] >= config_.rounds_to_win) {
      finished_ = true;
      winner_ = round.winner();
    }
  }

  if (finished_) {
    RaceEvent event;
    event.type = EventType::kMatchWon;
    event.step = sim_.step_count();
    event.time = sim_.time();
    event.agent = static_cast<std::uint32_t>(winner_ == 0 ? car_a_ : car_b_);
    event.other = static_cast<std::uint32_t>(winner_ == 0 ? car_b_ : car_a_);
    event.code = rounds_played_;
    events_.push_back(event);
  }
}

}  // namespace race
}  // namespace slipx
