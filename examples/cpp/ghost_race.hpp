// Copyright 2026 The SlipX Authors
// SPDX-License-Identifier: Apache-2.0
//
// Twenty cars driving the shipped track at once. A ghost race, not a race.
//
// The distinction is the whole point of this file, so it is stated before
// anything else. There is no contact model in SlipX yet, and no race control:
// the cars here cannot touch, cannot be held up, cannot be overtaken in any
// sense that costs the car in front anything, and nothing decides who won.
// What they are is twenty independent time trials sharing a clock, a track
// and a lockstep barrier. That is a demonstration that the pieces built in P1
// compose to more than one car, and it is not a race result.
//
// What it does exercise, which the performance benchmark deliberately does
// not, is the whole chain per agent: the track geometry, the projection, the
// controller, the vehicle model and the lap counter, twenty times over in one
// orchestrated run. The benchmark answers "what does this cost"; this answers
// "does it drive".
//
// The grid is spaced around the lap rather than lined up along one straight,
// because with no contact model a line of cars 0.3 m apart is twenty cars
// occupying the same metre of tarmac, which looks like a start and is not
// one.
//
// Header-only and in examples/ for the same reason the reference stack is:
// nothing in the library depends on it and nothing should.

#ifndef SLIPX_EXAMPLES_GHOST_RACE_HPP
#define SLIPX_EXAMPLES_GHOST_RACE_HPP

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "reference_stack.hpp"
#include "slipx/scene/lap.hpp"
#include "slipx/scene/track.hpp"
#include "slipx/sim/simulation.hpp"
#include "slipx/vehicle_model.hpp"

namespace slipx {
namespace examples {

struct GhostRaceConfig {
  std::size_t agents = 20;

  // The spread of target speeds across the field, slowest first on the grid.
  // Pure pursuit understeers as the tyres start to matter, so the top of this
  // range is bounded by the controller rather than by the car: above about
  // 3.2 m/s it runs wide at the corner exits on a 1.5 m corridor. That is the
  // controller behaving as a geometric controller does and not a defect to
  // tune out, so the field is set inside it.
  double slowest = 2.4;   // [m/s]
  double fastest = 3.2;   // [m/s]

  double lookahead = 0.6;  // [m]
  double dt = 1.0e-3;      // [s]

  // Track-limit tolerance, per agent, in metres. Required rather than
  // defaulted by LapCounter, and passed on with the same intent.
  double limit_tolerance = 0.05;

  int laps = 2;
  double time_limit = 120.0;  // [s], so a car that never finishes still ends

  std::uint64_t master_seed = 1;

  // How often to record a frame for a viewer. Nothing in the simulation reads
  // this: it decides the size of the recording and nothing else.
  double record_hz = 25.0;
};

// One recorded frame of one agent. A recording, not a state: the state a
// simulation hands out is overwritten by the next step.
struct GhostFrame {
  double t = 0.0;
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  double speed = 0.0;
};

struct GhostResult {
  std::string name;
  double target_speed = 0.0;
  std::vector<double> lap_times;   // [s], in the order they were completed
  bool left_the_track = false;
  double worst_margin = 0.0;       // [m]
  int laps = 0;

  // The best lap, or NaN when the car completed none. NaN rather than zero,
  // for the reason every diagnostic in this library uses NaN: zero is a
  // plausible lap time and would be believed.
  double best_lap() const {
    double best = std::numeric_limits<double>::quiet_NaN();
    for (const double lap : lap_times) {
      if (!(best <= lap)) best = lap;
    }
    return best;
  }
};

struct GhostRaceRecording {
  std::vector<GhostResult> results;
  std::vector<std::vector<GhostFrame>> frames;  // per agent
  double duration = 0.0;                        // [s]
  double lap_length = 0.0;                      // [m]
};

// The centreline point at an arc length, and the direction of travel there.
// Wraps on a closed track, which is what putting a grid slot behind the line
// needs.
inline std::pair<double, double> centreline_pose(const scene::Track& track,
                                                 double s, double* heading) {
  const auto& points = track.centreline().points();
  const double length = track.length();
  if (points.size() < 2) throw std::invalid_argument("centreline too short");

  if (track.is_closed()) {
    s = std::fmod(s, length);
    if (s < 0.0) s += length;
  } else {
    if (s < 0.0) s = 0.0;
    if (s > points.back().s) s = points.back().s;
  }

  const std::size_t count = points.size();
  for (std::size_t i = 0; i + 1 < count; ++i) {
    if (s <= points[i + 1].s) {
      const double span = points[i + 1].s - points[i].s;
      const double f = span > 0.0 ? (s - points[i].s) / span : 0.0;
      const double dx = points[i + 1].x - points[i].x;
      const double dy = points[i + 1].y - points[i].y;
      if (heading) *heading = std::atan2(dy, dx);
      return {points[i].x + f * dx, points[i].y + f * dy};
    }
  }

  // The closing segment, from the last point back to the first.
  const double span = length - points.back().s;
  const double f = span > 0.0 ? (s - points.back().s) / span : 0.0;
  const double dx = points.front().x - points.back().x;
  const double dy = points.front().y - points.back().y;
  if (heading) *heading = std::atan2(dy, dx);
  return {points.back().x + f * dx, points.back().y + f * dy};
}

// Run the field. Deterministic: no clock is read, the policies are pure
// functions of the state they are handed, and the orchestrator collects every
// command before any car moves.
inline GhostRaceRecording run_ghost_race(const scene::Track& track,
                                         const VehicleParams& params,
                                         const GhostRaceConfig& config = {}) {
  if (config.agents == 0) throw std::invalid_argument("no agents");
  if (!track.is_closed()) {
    throw std::invalid_argument(
        "a ghost race counts laps, and an open track has none");
  }

  const double length = track.length();
  const double spacing = length / static_cast<double>(config.agents);

  // The controllers outlive the simulation because the policies capture them
  // by pointer: a std::function holding a dangling controller is a run that
  // reads freed memory rather than one that fails.
  std::vector<std::unique_ptr<PurePursuit>> controllers;
  std::vector<std::unique_ptr<scene::LapCounter>> counters;
  controllers.reserve(config.agents);
  counters.reserve(config.agents);

  sim::SimulationConfig sim_config;
  sim_config.dt = config.dt;
  sim_config.master_seed = config.master_seed;
  sim::Simulation simulation(sim_config);

  GhostRaceRecording recording;
  recording.lap_length = length;
  recording.results.resize(config.agents);
  recording.frames.resize(config.agents);

  const double span = config.agents > 1
                          ? (config.fastest - config.slowest) /
                                static_cast<double>(config.agents - 1)
                          : 0.0;

  for (std::size_t i = 0; i < config.agents; ++i) {
    const double speed = config.slowest + span * static_cast<double>(i);

    // Grid slot i is i spacings behind the start line, which on a closed
    // track is a wrap, and the speeds rise with the slot. Twenty cars evenly
    // spaced fill the whole lap, so there is no front and no back: what the
    // arrangement actually decides is that every car is directly behind a
    // marginally slower one, and the field therefore compresses into a queue
    // rather than dispersing. That is the interesting case to watch, because
    // it is where a contact model would have work to do and there is none.
    double heading = 0.0;
    const auto slot =
        centreline_pose(track, -spacing * static_cast<double>(i), &heading);

    controllers.push_back(std::make_unique<PurePursuit>(
        track, config.lookahead, params.lf + params.lr, speed));
    counters.push_back(
        std::make_unique<scene::LapCounter>(track, config.limit_tolerance));

    sim::AgentSpec agent;
    agent.name = "car" + std::to_string(i);
    agent.tier = Tier::L2_DoubleTrack;
    agent.params = params;
    agent.initial_state.pos.x = slot.first;
    agent.initial_state.pos.y = slot.second;
    agent.initial_state.yaw = heading;
    agent.initial_state.vel_body.x = speed;
    // L2 has no wheel rotational state (ADR-0027): omega_w is reported rather
    // than integrated, and the only thing the model reads it for is the point
    // on the ESC torque-speed curve. Seeding it costs nothing and buys the
    // first step the right point on that curve; leaving it zero gives one
    // step of stall-region torque, which ADR-0031 records as expected rather
    // than as a bug. One millisecond either way, and worth being deliberate
    // about rather than inheriting.
    for (auto& omega : agent.initial_state.omega_w) {
      omega = speed / params.wheel_radius;
    }

    const PurePursuit* controller = controllers.back().get();
    agent.policy = [controller](const VehicleState& state, double,
                                sim::Rng&) { return controller->drive(state); };
    simulation.add_agent(agent);

    counters.back()->reset_to(slot.first, slot.second);
    recording.results[i].name = agent.name;
    recording.results[i].target_speed = speed;
  }

  const auto steps = static_cast<std::uint64_t>(config.time_limit / config.dt);
  const auto stride = static_cast<std::uint64_t>(
      1.0 / config.record_hz / config.dt);

  std::vector<int> counted(config.agents, 0);

  for (std::uint64_t step = 0; step <= steps; ++step) {
    const double now = static_cast<double>(step) * config.dt;

    bool everyone_finished = true;
    for (std::size_t i = 0; i < config.agents; ++i) {
      if (counted[i] < config.laps) everyone_finished = false;
    }
    if (everyone_finished) {
      recording.duration = now;
      break;
    }

    if (stride > 0 && step % stride == 0) {
      for (std::size_t i = 0; i < config.agents; ++i) {
        const VehicleState& state = simulation.state(i);
        recording.frames[i].push_back(
            GhostFrame{now, state.pos.x, state.pos.y, state.yaw,
                       state.vel_body.x});
      }
    }

    recording.duration = now;
    if (step == steps) break;
    simulation.advance();

    for (std::size_t i = 0; i < config.agents; ++i) {
      const VehicleState& state = simulation.state(i);
      counters[i]->update(state.pos.x, state.pos.y);

      // A lap time is the gap between the crossings, not the time of the
      // crossing, so the first lap is measured from the start of the run and
      // every later one from the previous crossing.
      while (counters[i]->laps() > counted[i] && counted[i] < config.laps) {
        double previous = 0.0;
        for (const double lap : recording.results[i].lap_times) {
          previous += lap;
        }
        recording.results[i].lap_times.push_back(now + config.dt - previous);
        ++counted[i];
      }
    }
  }

  for (std::size_t i = 0; i < config.agents; ++i) {
    recording.results[i].laps = counters[i]->laps();
    recording.results[i].left_the_track = counters[i]->has_left_the_track();
    recording.results[i].worst_margin = counters[i]->worst_margin();
  }

  return recording;
}

}  // namespace examples
}  // namespace slipx

#endif  // SLIPX_EXAMPLES_GHOST_RACE_HPP
