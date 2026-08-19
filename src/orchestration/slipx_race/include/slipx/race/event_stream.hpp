// Copyright 2026 The SlipX Authors
// SPDX-License-Identifier: Apache-2.0
//
// The structured event stream: every race-control outcome, encoded as MCAP,
// so the event stream and the run sinks are one format rather than two.
//
// One channel, "/race/events", JSON messages against a JSON Schema, exactly
// the dialect the Python MCAP sink writes runs in (json messages, jsonschema
// schemas, absence over NaN); a Metadata record named "race" carries the
// pinned ruleset, the full RaceConfig and whatever run identifiers the
// caller adds. Any leaderboard, report or CI job is meant to consume this
// stream and nothing else: if a fact about a race matters, it is an event
// or metadata here, not a printf somewhere.
//
// The writer emits the minimal spec-valid shape: unchunked, uncompressed,
// no summary section (the footer says so honestly with zeroed offsets), CRC
// fields zero, which the format defines as "not computed". The official
// readers accept it, and the pytest suite holds that claim against the
// reference `mcap` library rather than trusting it. The reader half parses
// exactly what this writer emits, because the milestone's property is that
// a full race replays from its stream alone, and proving that needs a
// consumer with no access to anything else; it is not a general MCAP
// reader and does not pretend to be.
//
// Byte-determinism: two identical races write identical files. Doubles are
// printed at 17 significant digits, so every value round-trips exactly.

#ifndef SLIPX_RACE_EVENT_STREAM_HPP
#define SLIPX_RACE_EVENT_STREAM_HPP

#include <string>
#include <utility>
#include <vector>

#include "slipx/race/events.hpp"
#include "slipx/race/ruleset.hpp"

namespace slipx {
namespace race {

// Writes the stream. The metadata record always carries the ruleset
// repository, revision and date, and every RaceConfig field under
// "config."; `extra_metadata` is for run identifiers (seeds, digests,
// scenario names) and is written after them, in the order given. Returns
// false when the file cannot be written; does not throw, for the manifest
// writer's reason (a finished race should not be lost to a full disk
// without being reported first).
bool write_event_stream(
    const std::string& path, const std::vector<RaceEvent>& events,
    const RaceConfig& config,
    const std::vector<std::pair<std::string, std::string>>& extra_metadata =
        {});

// What a read gives back: everything a consumer needs and nothing it must
// go elsewhere for.
struct EventStreamContents {
  std::vector<std::pair<std::string, std::string>> metadata;
  std::vector<RaceEvent> events;

  // Convenience over `metadata`, empty string when absent.
  std::string metadata_value(const std::string& key) const;
};

// Reads a stream this writer wrote. Returns false on anything it does not
// recognise rather than guessing: a file this parser cannot read is a file
// somebody should look at, not one to be silently partially loaded.
bool read_event_stream(const std::string& path, EventStreamContents* out);

// The event type names used on the wire, shared by both directions so they
// cannot drift: to_string(EventType) writes them, this reads them. Returns
// false for an unknown name.
bool event_type_from_string(const std::string& name, EventType* out);

}  // namespace race
}  // namespace slipx

#endif  // SLIPX_RACE_EVENT_STREAM_HPP
