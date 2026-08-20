// Copyright 2026 The SlipX Authors
// SPDX-License-Identifier: Apache-2.0
//
// The MCAP framing is written out by hand, for the manifest writer's
// reason: this file is what somebody replays a disputed race from years
// later, and two hundred lines of stable encoder beat a library dependency
// they may not be able to install. The subset used is small and spelled out
// here: little-endian throughout; a record is opcode(1) + length(8) +
// payload; strings are u32-length-prefixed; a metadata map is a
// u32-byte-length-prefixed sequence of string pairs.

#include "slipx/race/event_stream.hpp"

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>

#include "slipx/version.hpp"

namespace slipx {
namespace race {
namespace {

constexpr char kMagic[8] = {'\x89', 'M', 'C', 'A', 'P', '0', '\r', '\n'};

constexpr std::uint8_t kOpHeader = 0x01;
constexpr std::uint8_t kOpFooter = 0x02;
constexpr std::uint8_t kOpSchema = 0x03;
constexpr std::uint8_t kOpChannel = 0x04;
constexpr std::uint8_t kOpMessage = 0x05;
constexpr std::uint8_t kOpMetadata = 0x0C;
constexpr std::uint8_t kOpDataEnd = 0x0F;

constexpr char kTopic[] = "/race/events";
constexpr char kSchemaName[] = "slipx.race.RaceEvent";

// The schema the messages are validated against by anyone who cares to.
// Absence semantics as everywhere in SlipX: `agent` and `other` are left
// out when an event has none, never written as a sentinel.
constexpr char kSchemaJson[] =
    "{\n"
    "  \"$schema\": \"http://json-schema.org/draft-07/schema#\",\n"
    "  \"title\": \"slipx.race.RaceEvent\",\n"
    "  \"description\": \"One race-control outcome. A key that is absent "
    "does not apply to this event type.\",\n"
    "  \"type\": \"object\",\n"
    "  \"properties\": {\n"
    "    \"type\": {\"type\": \"string\"},\n"
    "    \"step\": {\"type\": \"integer\"},\n"
    "    \"time\": {\"type\": \"number\"},\n"
    "    \"agent\": {\"type\": \"integer\"},\n"
    "    \"other\": {\"type\": \"integer\"},\n"
    "    \"value\": {\"type\": \"number\"},\n"
    "    \"code\": {\"type\": \"integer\"}\n"
    "  },\n"
    "  \"additionalProperties\": false\n"
    "}\n";

// ------------------------------------------------------------ wire writing

void put_u16(std::string& out, std::uint16_t v) {
  out.push_back(static_cast<char>(v & 0xff));
  out.push_back(static_cast<char>((v >> 8) & 0xff));
}

void put_u32(std::string& out, std::uint32_t v) {
  for (int shift = 0; shift < 32; shift += 8) {
    out.push_back(static_cast<char>((v >> shift) & 0xff));
  }
}

void put_u64(std::string& out, std::uint64_t v) {
  for (int shift = 0; shift < 64; shift += 8) {
    out.push_back(static_cast<char>((v >> shift) & 0xff));
  }
}

void put_string(std::string& out, const std::string& s) {
  put_u32(out, static_cast<std::uint32_t>(s.size()));
  out += s;
}

void put_map(std::string& out,
             const std::vector<std::pair<std::string, std::string>>& map) {
  std::string body;
  for (const auto& entry : map) {
    put_string(body, entry.first);
    put_string(body, entry.second);
  }
  put_u32(out, static_cast<std::uint32_t>(body.size()));
  out += body;
}

void put_record(std::string& out, std::uint8_t opcode,
                const std::string& payload) {
  out.push_back(static_cast<char>(opcode));
  put_u64(out, payload.size());
  out += payload;
}

std::string number(double v) {
  // Seventeen significant digits round-trips a double exactly, the
  // manifest writer's rule.
  char buffer[64];
  std::snprintf(buffer, sizeof(buffer), "%.17g", v);
  return buffer;
}

std::string event_json(const RaceEvent& event) {
  std::ostringstream o;
  o << "{\"type\":\"" << to_string(event.type) << "\"";
  o << ",\"step\":" << event.step;
  o << ",\"time\":" << number(event.time);
  if (event.agent != kNoAgent) o << ",\"agent\":" << event.agent;
  if (event.other != kNoAgent) o << ",\"other\":" << event.other;
  o << ",\"value\":" << number(event.value);
  o << ",\"code\":" << event.code;
  o << "}";
  return o.str();
}

// ------------------------------------------------------------ wire reading

struct Cursor {
  const char* data;
  std::size_t size;
  std::size_t at = 0;

  bool take(void* out, std::size_t n) {
    if (at + n > size) return false;
    std::memcpy(out, data + at, n);
    at += n;
    return true;
  }
  bool u16(std::uint16_t* v) { return take(v, 2); }
  bool u32(std::uint32_t* v) { return take(v, 4); }
  bool u64(std::uint64_t* v) { return take(v, 8); }
  bool str(std::string* v) {
    std::uint32_t length = 0;
    if (!u32(&length) || at + length > size) return false;
    v->assign(data + at, length);
    at += length;
    return true;
  }
};

// Pull one JSON value out of the writer's own flat output. Not a JSON
// parser: the keys are known, the values are unquoted numbers or a quoted
// type name, and the writer never nests or escapes.
bool json_field(const std::string& json, const std::string& key,
                std::string* out) {
  const std::string needle = "\"" + key + "\":";
  const std::size_t start = json.find(needle);
  if (start == std::string::npos) return false;
  std::size_t from = start + needle.size();
  if (from >= json.size()) return false;
  std::size_t end;
  if (json[from] == '"') {
    ++from;
    end = json.find('"', from);
  } else {
    end = json.find_first_of(",}", from);
  }
  if (end == std::string::npos) return false;
  out->assign(json, from, end - from);
  return true;
}

}  // namespace

std::string EventStreamContents::metadata_value(const std::string& key) const {
  for (const auto& entry : metadata) {
    if (entry.first == key) return entry.second;
  }
  return std::string();
}

bool event_type_from_string(const std::string& name, EventType* out) {
  for (int k = 0; k <= static_cast<int>(EventType::kWrongWay); ++k) {
    const EventType type = static_cast<EventType>(k);
    if (name == to_string(type)) {
      *out = type;
      return true;
    }
  }
  return false;
}

bool write_event_stream(
    const std::string& path, const std::vector<RaceEvent>& events,
    const RaceConfig& config,
    const std::vector<std::pair<std::string, std::string>>& extra_metadata) {
  std::string out;
  out.append(kMagic, sizeof(kMagic));

  {
    std::string header;
    put_string(header, "slipx_race");                      // profile
    put_string(header, std::string("slipx ") + kVersion);  // library
    put_record(out, kOpHeader, header);
  }
  {
    std::string schema;
    put_u16(schema, 1);                       // schema id
    put_string(schema, kSchemaName);
    put_string(schema, "jsonschema");
    put_string(schema, kSchemaJson);
    put_record(out, kOpSchema, schema);
  }
  {
    std::string channel;
    put_u16(channel, 1);                      // channel id
    put_u16(channel, 1);                      // schema id
    put_string(channel, kTopic);
    put_string(channel, "json");
    put_map(channel, {});
    put_record(out, kOpChannel, channel);
  }
  {
    // Everything a consumer needs to know what race this was: the pinned
    // ruleset, the full mechanised configuration, then the caller's run
    // identifiers.
    std::vector<std::pair<std::string, std::string>> map = {
        {"ruleset_repository", kRulesetRepository},
        {"ruleset_revision", kRulesetRevision},
        {"ruleset_date", kRulesetDate},
        {"config.laps_to_win", std::to_string(config.laps_to_win)},
        {"config.rounds_to_win", std::to_string(config.rounds_to_win)},
        {"config.light_contact_speed", number(config.light_contact_speed)},
        {"config.restart_gap", number(config.restart_gap)},
        {"config.recovery_bonus", number(config.recovery_bonus)},
        {"config.warnings_to_disqualify",
         std::to_string(config.warnings_to_disqualify)},
        {"config.limit_tolerance", number(config.limit_tolerance)},
        {"config.grid_gap", number(config.grid_gap)},
        {"config.stop_speed", number(config.stop_speed)},
        {"config.reversed", config.reversed ? "true" : "false"},
        {"config.wrong_way_distance", number(config.wrong_way_distance)},
    };
    map.insert(map.end(), extra_metadata.begin(), extra_metadata.end());

    std::string metadata;
    put_string(metadata, "race");
    put_map(metadata, map);
    put_record(out, kOpMetadata, metadata);
  }

  std::uint32_t sequence = 0;
  for (const RaceEvent& event : events) {
    const std::string json = event_json(event);
    const auto nanos =
        static_cast<std::uint64_t>(event.time * 1.0e9);
    std::string message;
    put_u16(message, 1);           // channel id
    put_u32(message, sequence++);
    put_u64(message, nanos);       // log time
    put_u64(message, nanos);       // publish time
    message += json;
    put_record(out, kOpMessage, message);
  }

  {
    std::string data_end;
    put_u32(data_end, 0);          // CRC not computed, which 0 means
    put_record(out, kOpDataEnd, data_end);
  }
  {
    std::string footer;
    put_u64(footer, 0);            // no summary section
    put_u64(footer, 0);
    put_u32(footer, 0);
    put_record(out, kOpFooter, footer);
  }
  out.append(kMagic, sizeof(kMagic));

  std::ofstream file(path, std::ios::binary);
  if (!file) return false;
  file.write(out.data(), static_cast<std::streamsize>(out.size()));
  return static_cast<bool>(file);
}

bool read_event_stream(const std::string& path, EventStreamContents* out) {
  std::ifstream file(path, std::ios::binary);
  if (!file) return false;
  std::string bytes((std::istreambuf_iterator<char>(file)),
                    std::istreambuf_iterator<char>());

  if (bytes.size() < 2 * sizeof(kMagic) ||
      std::memcmp(bytes.data(), kMagic, sizeof(kMagic)) != 0) {
    return false;
  }

  Cursor cursor{bytes.data(), bytes.size(), sizeof(kMagic)};
  out->metadata.clear();
  out->events.clear();

  while (cursor.at < cursor.size) {
    std::uint8_t opcode = 0;
    if (!cursor.take(&opcode, 1)) return false;
    std::uint64_t length = 0;
    if (!cursor.u64(&length) || cursor.at + length > cursor.size) {
      return false;
    }
    Cursor record{cursor.data + cursor.at, static_cast<std::size_t>(length),
                  0};
    cursor.at += static_cast<std::size_t>(length);

    if (opcode == kOpMetadata) {
      std::string name;
      std::uint32_t map_bytes = 0;
      if (!record.str(&name) || !record.u32(&map_bytes)) return false;
      while (record.at < record.size) {
        std::string key, value;
        if (!record.str(&key) || !record.str(&value)) return false;
        out->metadata.emplace_back(key, value);
      }
    } else if (opcode == kOpMessage) {
      std::uint16_t channel = 0;
      std::uint32_t sequence = 0;
      std::uint64_t log_time = 0, publish_time = 0;
      if (!record.u16(&channel) || !record.u32(&sequence) ||
          !record.u64(&log_time) || !record.u64(&publish_time)) {
        return false;
      }
      // Strict about the writer's own invariants: one channel, and the MCAP
      // log time agreeing with the JSON time to the nanosecond. A file that
      // breaks either was not written by this writer and deserves a refusal,
      // not a guess.
      if (channel != 1) return false;
      const std::string json(record.data + record.at,
                             record.size - record.at);

      RaceEvent event;
      std::string field;
      if (!json_field(json, "type", &field) ||
          !event_type_from_string(field, &event.type)) {
        return false;
      }
      if (!json_field(json, "step", &field)) return false;
      event.step = std::strtoull(field.c_str(), nullptr, 10);
      if (!json_field(json, "time", &field)) return false;
      event.time = std::strtod(field.c_str(), nullptr);
      if (log_time != static_cast<std::uint64_t>(event.time * 1.0e9)) {
        return false;
      }
      if (json_field(json, "agent", &field)) {
        event.agent =
            static_cast<std::uint32_t>(std::strtoul(field.c_str(), nullptr, 10));
      }
      if (json_field(json, "other", &field)) {
        event.other =
            static_cast<std::uint32_t>(std::strtoul(field.c_str(), nullptr, 10));
      }
      if (!json_field(json, "value", &field)) return false;
      event.value = std::strtod(field.c_str(), nullptr);
      if (!json_field(json, "code", &field)) return false;
      event.code = static_cast<int>(std::strtol(field.c_str(), nullptr, 10));
      out->events.push_back(event);
    } else if (opcode == kOpFooter) {
      break;
    }
    // Header, Schema, Channel and DataEnd carry nothing this consumer
    // needs back; they are validated by the pytest suite against the
    // reference library instead.
  }
  return true;
}

}  // namespace race
}  // namespace slipx
