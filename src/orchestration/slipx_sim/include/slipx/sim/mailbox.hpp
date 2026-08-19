// Copyright 2026 The SlipX Authors
// SPDX-License-Identifier: Apache-2.0
//
// The command mailbox: the barrier's doorway (ADR-0044).
//
// A thread-safe queue of step-tagged entries through which an external
// controller (another thread today; the ROS bridge and the multi-host race
// tomorrow) delivers commands into an otherwise single-threaded simulation.
// The step tag is the acknowledgement: an entry tagged N says either "here
// is my command for step N" (post) or "I am alive at step N, hold my last
// command" (ack). The simulation takes the entry tagged with exactly the
// step it is about to compute; anything else is a miss, answered by the
// agent's TimeoutPolicy in simulation.hpp.
//
// The tagging is deliberately strict. A controller slower than the step
// rate acknowledges the steps it has no new command for, one call each.
// The tempting alternative, hold-until-replaced with no acknowledgement,
// quietly converts "my controller crashed" into "my car drives on at its
// last command until the wall stops it"; the whole point of a barrier is
// that silence is visible.

#ifndef SLIPX_SIM_MAILBOX_HPP
#define SLIPX_SIM_MAILBOX_HPP

#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <mutex>
#include <optional>
#include <stdexcept>

#include "slipx/state.hpp"

namespace slipx {
namespace sim {

class CommandMailbox {
 public:
  // Deliver the command for one step. Refuses a NaN command by name: replay
  // uses NaN in the input log as the marker for a missed step (ADR-0044),
  // which is only sound because no NaN can enter through this door. Refuses
  // a tag that does not increase on the previous one: commands arrive in
  // step order or they are a protocol error worth hearing about early.
  void post(std::uint64_t step, const DriveInput& input) {
    if (std::isnan(input.steer_cmd) || std::isnan(input.accel_cmd)) {
      throw std::invalid_argument(
          "slipx_sim: a NaN command was posted to a mailbox. NaN is "
          "reserved as the input log's missed-step marker, so it is refused "
          "here rather than smuggled into a trajectory");
    }
    push(step, Entry{step, true, input});
  }

  // Acknowledge a step without a new command: "hold my last one". Before
  // any command has been posted, the held command is the neutral input,
  // which is a coast.
  void ack(std::uint64_t step) { push(step, Entry{step, false, {}}); }

  // The simulation's side: resolve the entry tagged exactly `step`.
  // Entries tagged earlier are stale and are discarded here, at the
  // barrier, because applying an old command to a new step would be the
  // barrier pretending an answer arrived that did not. Waits forever when
  // wait_forever is set (the kWait policy), otherwise up to
  // timeout_seconds of wall time; returns the command to apply, or nullopt
  // for a miss.
  std::optional<DriveInput> take(std::uint64_t step, bool wait_forever,
                                 double timeout_seconds) {
    std::unique_lock<std::mutex> lock(mutex_);
    const auto ready = [&] {
      while (!entries_.empty() && entries_.front().step < step) {
        entries_.pop_front();
      }
      return !entries_.empty();
    };

    if (wait_forever) {
      // Tags are strictly monotonic, so the first entry at or past `step`
      // settles the question either way.
      posted_.wait(lock, ready);
    } else if (timeout_seconds > 0.0) {
      posted_.wait_for(lock,
                       std::chrono::duration_cast<std::chrono::nanoseconds>(
                           std::chrono::duration<double>(timeout_seconds)),
                       ready);
    } else {
      ready();
    }

    if (entries_.empty() || entries_.front().step != step) {
      return std::nullopt;   // a miss: the timeout policy answers it
    }
    const Entry entry = entries_.front();
    entries_.pop_front();
    if (entry.has_input) held_ = entry.input;
    return held_;
  }

 private:
  struct Entry {
    std::uint64_t step = 0;
    bool has_input = false;
    DriveInput input{};
  };

  void push(std::uint64_t step, Entry entry) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (any_posted_ && step <= last_step_) {
        throw std::invalid_argument(
            "slipx_sim: mailbox tags must strictly increase; a command for "
            "an old step is stale by definition and refusing it here beats "
            "discovering it at the barrier");
      }
      any_posted_ = true;
      last_step_ = step;
      entries_.push_back(entry);
    }
    posted_.notify_all();
  }

  std::mutex mutex_;
  std::condition_variable posted_;
  std::deque<Entry> entries_;
  bool any_posted_ = false;
  std::uint64_t last_step_ = 0;
  DriveInput held_{};   // zero-order hold: the last posted command
};

}  // namespace sim
}  // namespace slipx

#endif  // SLIPX_SIM_MAILBOX_HPP
