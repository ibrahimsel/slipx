// Copyright 2026 The SlipX Authors
// SPDX-License-Identifier: Apache-2.0
//
// The centreline: track geometry, and nothing that is not geometry.
//
// The file format is the four-column form used by the TUM racetrack database
// and by the F1TENTH set derived from it:
//
//     # x_m,y_m,w_tr_right_m,w_tr_left_m
//     0.0,0.0,1.5,1.5
//
// It is read unmodified and unextended. A file that loads here loads in every
// other tool in that ecosystem, and a file written for one of those tools
// loads here, which is worth more than any column we could add.
//
// Note the column order: the width to the RIGHT comes before the width to the
// left. It is the order the format uses and it is the wrong way round from
// the order anybody says out loud, so it is asserted in the tests rather than
// trusted.
//
// Three things a track has are deliberately absent from this class, because
// they are absent from the format and inventing them is the silent defaulting
// the loader is built to refuse (ADR-0034):
//
//   the surface     it lives in the track manifest, as an identifier, and
//                   friction is resolved from the tyre file (ADR-0010)
//   banking         nothing consumes it yet, so declaring one is a claim
//                   about a track that nobody measured
//   whether the
//   loop closes     the manifest says so; guessing it from the distance
//                   between the last point and the first is a heuristic that
//                   is silently wrong on a track that starts and ends near
//                   each other without closing
//
// Arc length is derived here rather than read, so it cannot disagree with the
// geometry it describes.

#ifndef SLIPX_SCENE_CENTRELINE_HPP
#define SLIPX_SCENE_CENTRELINE_HPP

#include <cstddef>
#include <string>
#include <vector>

namespace slipx {
namespace scene {

// One sample of the centreline. Positions are in the track's ground-plane
// frame, x forward and y left, as everywhere else in SlipX (ISO 8855).
struct CentrelinePoint {
  double s = 0.0;        // cumulative chord length from the first point   [m]
  double x = 0.0;        // ground-plane position                          [m]
  double y = 0.0;        // ground-plane position                          [m]
  double w_left = 0.0;   // drivable width to the left of the centreline   [m]
  double w_right = 0.0;  // drivable width to the right of the centreline  [m]
};

// A parsed centreline. Immutable once built; every refusal happens during
// construction, so a Centreline that exists is one that parsed cleanly.
class Centreline {
 public:
  // Parses the four-column form from text already in memory. `origin` is the
  // name that appears in refusals, normally the path the text came from; it
  // is a label, not a path, and is never opened.
  //
  // Throws std::invalid_argument, with a message naming the source line, for
  // a row that is not four fields, a field that is not a finite number, a
  // width that is not positive, a file with fewer than two points, and two
  // consecutive points at the same position.
  static Centreline from_csv(const std::string& text, const std::string& origin);

  // Reads a file and parses it. Throws std::invalid_argument if it cannot be
  // opened, naming the path.
  static Centreline from_file(const std::string& path);

  const std::vector<CentrelinePoint>& points() const { return points_; }
  std::size_t size() const { return points_.size(); }

  // Where this geometry was read from, carried through to refusals and to the
  // run manifest so a published result can say what it drove on.
  const std::string& origin() const { return origin_; }

  // Length of the polyline as written, first point to last, closing chord
  // excluded. Whether that chord counts is the manifest's business, not the
  // geometry's, so Track adds it and this does not.
  double open_length() const;

  // Length of the chord from the last point back to the first.
  double closing_chord() const;

  // The same geometry traversed the other way: point order reversed, the
  // left and right widths swapped (what lies to the left one way lies to
  // the right the other), and arc length re-derived in the new order. With
  // `keep_first` the original first point stays first, which is what a
  // closed lap wants: reversing the direction of racing must not move the
  // start line. The origin gains a "(reversed)" suffix so a manifest that
  // cites it says which way the geometry was walked.
  //
  // Throws std::invalid_argument when `keep_first` is set and the last
  // point repeats the first, because keeping the first point would turn
  // the repeat into a zero-length segment; the fix is deleting the
  // duplicate row, since a closed track's closing chord is implied.
  Centreline reversed(bool keep_first) const;

 private:
  Centreline() = default;

  std::vector<CentrelinePoint> points_;
  std::string origin_;
};

}  // namespace scene
}  // namespace slipx

#endif  // SLIPX_SCENE_CENTRELINE_HPP
