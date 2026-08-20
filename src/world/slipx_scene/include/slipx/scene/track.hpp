// Copyright 2026 The SlipX Authors
// SPDX-License-Identifier: Apache-2.0
//
// A track: a centreline, plus everything about the track that is not
// geometry (ADR-0034).
//
// The manifest arrives here as a plain struct, filled by whoever parsed it,
// exactly as VehicleParams arrives at the core. slipx_scene therefore knows
// nothing about YAML, about slipx_schema, or about how a track directory is
// laid out, and the dependency direction holds without argument (ADR-0003).
//
// The one thing this component does enforce is the pairing that ADR-0010
// exists for. A track declares a surface identifier; a tyre file is a
// (compound, surface) pair; and the failure that decision was written to
// prevent is somebody carrying a car from one venue to another and running
// carpet tyres on asphalt with nothing in any file recording that the change
// of venue should have changed a number. That check has to happen where both
// facts meet, and it happens here, once, at construction.
//
// Note what is deliberately NOT here: friction. A surface identifier resolves
// to a tyre file, and the tyre file carries mu_y0, mu_x0 and the load
// sensitivity, because those are the parameters the identification programme
// is built to measure. A friction number in a track file would be a second
// source for the same quantity, supplied by somebody who measured nothing.

#ifndef SLIPX_SCENE_TRACK_HPP
#define SLIPX_SCENE_TRACK_HPP

#include <optional>
#include <string>
#include <vector>

#include "slipx/scene/centreline.hpp"

namespace slipx {
namespace scene {

// One tyre, as the schema references it: never coefficients, always the pair
// that identifies which file the coefficients came from.
struct TyrePair {
  std::string compound;  // e.g. "sponge"
  std::string surface;   // e.g. "carpet"
};

// Everything about a track that is not its geometry.
//
// Two fields are std::optional, and it is not stylistic. A plain bool has no
// "absent", so a caller who forgets to set `closed` silently asserts that the
// track is open, and a caller who never heard of banking silently asserts
// that the track is flat. Both are claims about a real place. Optional makes
// the omission visible and turns it into a refusal, which is the same rule
// the parameter loader follows (ADR-0025).
struct TrackManifest {
  std::string name;     // what this track is called, used in every refusal
  std::string surface;  // the surface identifier, e.g. "carpet"

  // Whether the last centreline point joins back to the first. Declared,
  // never inferred: a paddock straight that starts and finishes near the
  // same place is not a lap.
  std::optional<bool> closed;

  // Where the geometry came from and under what terms it may be passed on.
  // SlipX ships no third-party track geometry, so for any real venue these
  // were written by the converter that fetched it, and they are what a
  // published result cites (ADR-0035).
  std::string geometry_source;
  std::string geometry_licence;

  // measured | identified | provisional, the same three words used for every
  // other parameter set in the project, printed rather than documented
  // (ADR-0013). A generated track is provisional, and its source says it was
  // generated.
  std::string provenance_label;

  // Refused, not ignored. No tier consumes banking, so a manifest that
  // declares one is asking for a model that does not exist, and being told
  // so is better than having the number quietly dropped.
  std::optional<double> banking_rad;
};

// A track that is ready to run. Every refusal happens in build(), so a Track
// that exists is one whose surface has tyres and whose manifest is complete.
class Track {
 public:
  // `tyres_in_use` is the tyres the run will actually be driven on, normally
  // the car's front and rear entries, not a library of everything available.
  // Every one of them must declare this track's surface: the failure being
  // prevented is running a tyre identified on one surface over another, and
  // one matching entry somewhere in a directory does not prevent it.
  //
  // Throws std::invalid_argument, naming the field or the tyre, for an empty
  // name or surface, an unset `closed`, a provenance label that is not one of
  // the three, empty provenance, a declared banking, an empty tyre list, or
  // any tyre whose surface differs from the track's.
  static Track build(Centreline centreline, TrackManifest manifest,
                     const std::vector<TyrePair>& tyres_in_use);

  const Centreline& centreline() const { return centreline_; }
  const TrackManifest& manifest() const { return manifest_; }
  const std::vector<TyrePair>& tyres() const { return tyres_; }

  const std::string& name() const { return manifest_.name; }
  const std::string& surface() const { return manifest_.surface; }
  bool is_closed() const { return *manifest_.closed; }

  // The distance once round, closing chord included when the track is closed
  // and excluded when it is not.
  double length() const;

  // The same track raced the other way round: the centreline traversed in
  // reverse (keeping the start when the track closes, because reversing a
  // lap must not move the start line), widths swapped to match, manifest
  // and tyres carried unchanged. Everything downstream measures direction
  // as increasing arc length, so this one function is how a race runs the
  // other way and nothing else carries a direction flag.
  Track reversed() const;

 private:
  // Private, and taking everything it needs: a Track cannot be default
  // constructed, so there is no half-built one for a caller to hold.
  Track(Centreline centreline, TrackManifest manifest,
        std::vector<TyrePair> tyres);

  Centreline centreline_;
  TrackManifest manifest_;
  std::vector<TyrePair> tyres_;
};

}  // namespace scene
}  // namespace slipx

#endif  // SLIPX_SCENE_TRACK_HPP
