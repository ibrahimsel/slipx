// Copyright 2026 The SlipX Authors
// SPDX-License-Identifier: Apache-2.0

#include "slipx/scene/centreline.hpp"

#include <cmath>
#include <fstream>
#include <locale>
#include <sstream>
#include <stdexcept>

namespace slipx {
namespace scene {
namespace {

// Reading a double without the ambient locale getting a vote.
//
// This is the one trap in the whole file. std::stod and a default-constructed
// istringstream both use the global locale, and on a machine whose locale has
// a comma decimal separator "1.5" reads as 1 with trailing rubbish, or as 1.5,
// depending on the library. Either way the track is silently a different
// shape, which is the worst failure a loader can have: it produces a plausible
// wrong answer rather than an error. Imbuing the classic locale explicitly
// costs one line and removes the whole class of problem.
//
// Trailing rubbish is rejected too, so "1.5abc" is a refusal rather than 1.5.
bool parse_double(const std::string& field, double& out) {
  std::istringstream in(field);
  in.imbue(std::locale::classic());
  in >> out;
  if (in.fail()) return false;
  char extra = 0;
  if (in >> extra) return false;  // fails at end of input, which is the pass
  return true;
}

std::string trim(const std::string& s) {
  const char* spaces = " \t\v\f";
  const std::size_t first = s.find_first_not_of(spaces);
  if (first == std::string::npos) return std::string();
  const std::size_t last = s.find_last_not_of(spaces);
  return s.substr(first, last - first + 1);
}

std::vector<std::string> split(const std::string& s, char separator) {
  std::vector<std::string> fields;
  std::size_t start = 0;
  while (true) {
    const std::size_t hit = s.find(separator, start);
    if (hit == std::string::npos) {
      fields.push_back(s.substr(start));
      return fields;
    }
    fields.push_back(s.substr(start, hit - start));
    start = hit + 1;
  }
}

std::string at(const std::string& origin, std::size_t line) {
  return origin + ":" + std::to_string(line);
}

// The distance between two points, by sqrt of the sum of squares rather than
// std::hypot. hypot is the better-behaved function in general, since it
// avoids intermediate overflow, but it is not required to be correctly
// rounded and its result therefore varies between C library versions
// (ADR-0033). sqrt is correctly rounded by IEEE-754, so this sum is the same
// on every libm, and a track a hundred metres across is nowhere near the
// overflow hypot exists to prevent.
double chord(const CentrelinePoint& a, const CentrelinePoint& b) {
  const double dx = b.x - a.x;
  const double dy = b.y - a.y;
  return std::sqrt(dx * dx + dy * dy);
}

}  // namespace

Centreline Centreline::from_csv(const std::string& text,
                                const std::string& origin) {
  Centreline out;
  out.origin_ = origin;

  // The source line of each accepted point, kept only so that a refusal can
  // name the two rows that collided rather than the two point indices, which
  // a person cannot find in the file.
  std::vector<std::size_t> source_lines;

  std::istringstream stream(text);
  std::string line;
  std::size_t line_number = 0;

  while (std::getline(stream, line)) {
    ++line_number;
    if (!line.empty() && line.back() == '\r') line.pop_back();  // CRLF files

    const std::string content = trim(line);
    if (content.empty()) continue;
    if (content[0] == '#') continue;  // the format's column header lives here

    const std::vector<std::string> fields = split(content, ',');
    if (fields.size() != 4) {
      throw std::invalid_argument(
          at(origin, line_number) + ": expected 4 comma-separated fields " +
          "(x_m, y_m, w_tr_right_m, w_tr_left_m), found " +
          std::to_string(fields.size()) + ". This is the centreline format " +
          "of the TUM racetrack database, read unextended; anything that is " +
          "not geometry belongs in the track manifest beside it.");
    }

    static const char* const names[4] = {"x_m", "y_m", "w_tr_right_m",
                                         "w_tr_left_m"};
    double value[4] = {0.0, 0.0, 0.0, 0.0};
    for (std::size_t i = 0; i < 4; ++i) {
      const std::string field = trim(fields[i]);
      if (!parse_double(field, value[i])) {
        throw std::invalid_argument(at(origin, line_number) + ": field " +
                                    names[i] + " is not a number (\"" + field +
                                    "\").");
      }
      if (!std::isfinite(value[i])) {
        throw std::invalid_argument(at(origin, line_number) + ": field " +
                                    names[i] +
                                    " is not finite. A track has no infinite "
                                    "or undefined coordinate.");
      }
    }

    CentrelinePoint point;
    point.x = value[0];
    point.y = value[1];
    // Right before left. That is the format's order, not a transcription
    // slip, and getting it backwards mirrors every track that is not
    // symmetric about its centreline.
    point.w_right = value[2];
    point.w_left = value[3];

    if (!(point.w_right > 0.0) || !(point.w_left > 0.0)) {
      throw std::invalid_argument(
          at(origin, line_number) +
          ": track widths must both be positive, found w_tr_right_m=" +
          std::to_string(point.w_right) +
          " w_tr_left_m=" + std::to_string(point.w_left) +
          ". A width of zero is not a narrow track, it is no track.");
    }

    out.points_.push_back(point);
    source_lines.push_back(line_number);
  }

  if (out.points_.size() < 2) {
    throw std::invalid_argument(
        origin + ": a centreline needs at least 2 points, found " +
        std::to_string(out.points_.size()) +
        ". Comment lines beginning with # and blank lines are skipped; a " +
        "file that is all header parses to nothing.");
  }

  // Two consecutive points at the same position is a zero-length segment. It
  // is harmless here and it is a division by zero in the first thing that
  // wants a heading or a normal, which is lap counting, one slice away. It is
  // refused where the line numbers are still known.
  for (std::size_t i = 1; i < out.points_.size(); ++i) {
    if (out.points_[i].x == out.points_[i - 1].x &&
        out.points_[i].y == out.points_[i - 1].y) {
      throw std::invalid_argument(
          origin + ": lines " + std::to_string(source_lines[i - 1]) + " and " +
          std::to_string(source_lines[i]) +
          " are the same point. A zero-length segment has no direction.");
    }
  }

  // Arc length, derived in file order. File order is the only ordering the
  // format guarantees, and summing in it makes the result the same on every
  // run without depending on anything but the geometry.
  out.points_[0].s = 0.0;
  for (std::size_t i = 1; i < out.points_.size(); ++i) {
    out.points_[i].s = out.points_[i - 1].s + chord(out.points_[i - 1],
                                                    out.points_[i]);
  }

  // The price of chord() being sqrt of a sum of squares rather than hypot is
  // that coordinates near the top of the double range overflow where hypot
  // would not. The price is paid here rather than passed on: every field can
  // be finite and the arc length still not be, and nothing downstream should
  // ever have to wonder whether an s it was given is a number.
  for (std::size_t i = 1; i < out.points_.size(); ++i) {
    if (!std::isfinite(out.points_[i].s)) {
      throw std::invalid_argument(
          origin + ": arc length is not finite by line " +
          std::to_string(source_lines[i]) +
          ". The coordinates are large enough to overflow the distance "
          "between two of them, which is a units error far more often than "
          "it is a track.");
    }
  }

  return out;
}

Centreline Centreline::from_file(const std::string& path) {
  std::ifstream file(path, std::ios::binary);
  if (!file) {
    throw std::invalid_argument(path +
                                ": cannot be opened. A track's geometry is "
                                "not shipped with SlipX and is converted into "
                                "place; check the track directory exists.");
  }
  std::ostringstream buffer;
  buffer << file.rdbuf();
  return from_csv(buffer.str(), path);
}

double Centreline::open_length() const { return points_.back().s; }

double Centreline::closing_chord() const {
  return chord(points_.back(), points_.front());
}

Centreline Centreline::reversed(bool keep_first) const {
  if (keep_first && points_.front().x == points_.back().x &&
      points_.front().y == points_.back().y) {
    throw std::invalid_argument(
        origin_ +
        ": cannot reverse keeping the first point while the last point "
        "repeats it; the repeat would become a zero-length segment, which "
        "has no direction. Delete the duplicate last row: on a closed "
        "track the closing chord is implied.");
  }

  Centreline out;
  out.origin_ = origin_ + " (reversed)";
  const std::size_t n = points_.size();
  out.points_.reserve(n);
  for (std::size_t i = 0; i < n; ++i) {
    // Walking backwards from the first point when it is kept, or from the
    // last when it is not: p0, p(n-1), ..., p1 against p(n-1), ..., p0.
    const std::size_t j = keep_first ? (n - i) % n : n - 1 - i;
    CentrelinePoint point = points_[j];
    const double w_left = point.w_left;
    point.w_left = point.w_right;
    point.w_right = w_left;
    out.points_.push_back(point);
  }

  out.points_[0].s = 0.0;
  for (std::size_t i = 1; i < n; ++i) {
    out.points_[i].s =
        out.points_[i - 1].s + chord(out.points_[i - 1], out.points_[i]);
  }
  return out;
}

}  // namespace scene
}  // namespace slipx
