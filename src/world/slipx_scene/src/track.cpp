// Copyright 2026 The SlipX Authors
// SPDX-License-Identifier: Apache-2.0

#include "slipx/scene/track.hpp"

#include <stdexcept>
#include <utility>

namespace slipx {
namespace scene {
namespace {

// The refusals all name the track, because a run that loads several will
// otherwise report a problem without saying which file to open.
std::string prefix(const TrackManifest& manifest) {
  return manifest.name.empty() ? std::string("track: ")
                               : "track \"" + manifest.name + "\": ";
}

std::string offered(const std::vector<TyrePair>& tyres) {
  std::string out;
  for (std::size_t i = 0; i < tyres.size(); ++i) {
    if (i != 0) out += ", ";
    out += "(" + tyres[i].compound + ", " + tyres[i].surface + ")";
  }
  return out;
}

}  // namespace

Track::Track(Centreline centreline, TrackManifest manifest,
             std::vector<TyrePair> tyres)
    : centreline_(std::move(centreline)),
      manifest_(std::move(manifest)),
      tyres_(std::move(tyres)) {}

Track Track::build(Centreline centreline, TrackManifest manifest,
                   const std::vector<TyrePair>& tyres_in_use) {
  if (manifest.name.empty()) {
    throw std::invalid_argument(
        "track: the manifest has no name. Every refusal and every run "
        "manifest names the track, and an unnamed one cannot be cited.");
  }

  if (manifest.surface.empty()) {
    throw std::invalid_argument(
        prefix(manifest) +
        "the manifest declares no surface. A surface identifier is what "
        "resolves the tyre file, and the same car on carpet and on polished "
        "concrete is two different vehicles.");
  }

  if (!manifest.closed.has_value()) {
    throw std::invalid_argument(
        prefix(manifest) +
        "the manifest does not say whether the track closes. It is not "
        "inferred from the geometry: a straight that starts and finishes "
        "near the same place is not a lap, and guessing would make it one.");
  }

  if (manifest.banking_rad.has_value()) {
    throw std::invalid_argument(
        prefix(manifest) +
        "the manifest declares banking, and no tier consumes it. Banking is "
        "refused rather than dropped, because a number that is silently "
        "ignored is worse than one that is absent.");
  }

  if (manifest.provenance_label != "measured" &&
      manifest.provenance_label != "identified" &&
      manifest.provenance_label != "provisional") {
    throw std::invalid_argument(
        prefix(manifest) + "the provenance label is \"" +
        manifest.provenance_label +
        "\"; it must be one of measured, identified, provisional. A track "
        "whose geometry was generated rather than surveyed is provisional, "
        "and its source says so.");
  }

  if (manifest.geometry_source.empty()) {
    throw std::invalid_argument(
        prefix(manifest) +
        "the manifest does not say where its geometry came from. SlipX "
        "ships no third-party track geometry, so a track that cannot cite "
        "its source cannot be published with a result.");
  }

  if (manifest.geometry_licence.empty()) {
    throw std::invalid_argument(
        prefix(manifest) +
        "the manifest does not state the licence of its geometry. The "
        "converter records it when it fetches; an obligation nobody was told "
        "about is an obligation nobody meets.");
  }

  if (tyres_in_use.empty()) {
    throw std::invalid_argument(
        prefix(manifest) +
        "no tyres were offered, so the surface \"" + manifest.surface +
        "\" resolves to nothing. Friction comes from a (compound, surface) "
        "tyre file or the run does not start.");
  }

  // Every tyre, not the first match. A car with carpet tyres on one axle and
  // asphalt tyres on the other is a mistake that a search for one match
  // would wave through.
  for (const TyrePair& tyre : tyres_in_use) {
    if (tyre.surface != manifest.surface) {
      throw std::invalid_argument(
          prefix(manifest) + "declares the surface \"" + manifest.surface +
          "\", and the tyre (" + tyre.compound + ", " + tyre.surface +
          ") is not for it. Offered: " + offered(tyres_in_use) +
          ". Fit a tyre identified on \"" + manifest.surface +
          "\", or run on a track whose surface these tyres were measured on.");
    }
  }

  return Track(std::move(centreline), std::move(manifest), tyres_in_use);
}

double Track::length() const {
  const double open = centreline_.open_length();
  return is_closed() ? open + centreline_.closing_chord() : open;
}

Track Track::reversed() const {
  // Straight to the constructor: the manifest and the tyres already passed
  // build() once, and reversal preserves every geometric invariant the
  // parser enforces.
  return Track(centreline_.reversed(is_closed()), manifest_, tyres_);
}

}  // namespace scene
}  // namespace slipx
