# Copyright 2026 The SlipX Authors
# SPDX-License-Identifier: Apache-2.0

"""The track manifest: what it must say, and what it must not be allowed to say.

The interesting half of this file is the second one. A track schema that lets
an author state friction would quietly override the one parameter the whole
identification programme exists to measure, so the absence of that field is a
property worth a test rather than a property worth a comment.
"""

from __future__ import annotations

from dataclasses import dataclass

import pytest

from slipx_schema import (
    SchemaVersionError,
    SurfaceMismatchError,
    TrackDirectoryError,
    ValidationError,
    check_surface,
    load_track,
)


@dataclass(frozen=True)
class FakeTyre:
    """Only the two fields the pairing check reads."""

    compound: str
    surface: str


# ------------------------------------------------------------------ loading


def test_the_shipped_track_loads(reference_track) -> None:
    track = load_track(reference_track)

    assert track.name == "paddock_stadium"
    assert track.surface == "carpet"
    assert track.closed is True
    assert track.centreline.exists()
    assert track.geometry_licence == "Apache-2.0"


def test_the_summary_leads_with_the_label_and_states_the_licence(
    reference_track,
) -> None:
    # Both, for the same reason: neither is something a reader should have to
    # go and look up before deciding whether they may publish what they ran.
    summary = load_track(reference_track).summary()

    assert "PROVISIONAL" in summary
    assert "Apache-2.0" in summary
    assert "carpet" in summary


def test_a_manifest_can_be_named_directly(reference_track) -> None:
    assert load_track(reference_track / "track.yaml").name == "paddock_stadium"


def test_a_directory_without_a_manifest_is_refused(tmp_path) -> None:
    with pytest.raises(TrackDirectoryError, match="track.yaml"):
        load_track(tmp_path)


def test_a_manifest_naming_a_missing_centreline_is_refused(track_factory) -> None:
    path = track_factory()
    (path / "centreline.csv").unlink()

    with pytest.raises(TrackDirectoryError, match="centreline.csv"):
        load_track(path)


# ------------------------------------------------- what a track may not say


def test_a_track_may_not_state_friction(track_factory) -> None:
    # The decision this protects is ADR-0010: friction belongs to a
    # (compound, surface) tyre file, measured by somebody, and a number here
    # would be a second source for it supplied by a track author who measured
    # nothing. The schema has no field for it, so it arrives as an unknown
    # field and is refused rather than ignored.
    path = track_factory(lambda d: d.update(mu=0.9))

    with pytest.raises(ValidationError, match="mu"):
        load_track(path)


def test_a_track_may_not_declare_banking(track_factory) -> None:
    # Nothing consumes banking, so accepting it would mean dropping it.
    path = track_factory(lambda d: d.update(banking=0.05))

    with pytest.raises(ValidationError, match="banking"):
        load_track(path)


def test_closure_must_be_declared(track_factory) -> None:
    path = track_factory(lambda d: d.pop("closed"))

    with pytest.raises(ValidationError, match="closed"):
        load_track(path)


def test_the_geometry_licence_is_required(track_factory) -> None:
    path = track_factory(lambda d: d["geometry"].pop("licence"))

    with pytest.raises(ValidationError, match="licence"):
        load_track(path)


def test_the_geometry_source_is_required(track_factory) -> None:
    path = track_factory(lambda d: d["geometry"].pop("source"))

    with pytest.raises(ValidationError, match="source"):
        load_track(path)


def test_a_surface_identifier_is_lower_case(track_factory) -> None:
    # The identifier is matched exactly wherever it is used, so two spellings
    # of one surface would be two surfaces. The schema settles it at the point
    # the file is written rather than leaving it to be reconciled later.
    path = track_factory(lambda d: d.update(surface="Carpet"))

    with pytest.raises(ValidationError, match="surface"):
        load_track(path)


def test_provenance_is_required(track_factory) -> None:
    path = track_factory(lambda d: d.pop("provenance"))

    with pytest.raises(ValidationError, match="provenance"):
        load_track(path)


# ------------------------------------------------------------- the versions


def test_a_newer_minor_is_refused(track_factory) -> None:
    path = track_factory(lambda d: d.update(schema_version="0.6.0"))

    with pytest.raises(SchemaVersionError, match="0.6.0"):
        load_track(path)


def test_an_older_minor_migrates_forward(track_factory) -> None:
    # Neither 0.3.0 (which added the track kind), 0.4.0 (which loosened the
    # compound vocabulary, ADR-0041) nor 0.5.0 (which restructured the
    # sensor file, ADR-0048) changed any track field, so the steps are
    # identities and a file naming an older version is read and then judged
    # on its contents.
    path = track_factory(lambda d: d.update(schema_version="0.2.0"))

    assert load_track(path).schema_version == "0.5.0"


# --------------------------------------------------- the surface to tyre check


def test_tyres_for_the_declared_surface_are_accepted(reference_track) -> None:
    track = load_track(reference_track)
    check_surface(track, [FakeTyre("sponge", "carpet"), FakeTyre("sponge", "carpet")])


def test_a_tyre_for_another_surface_is_refused(reference_track) -> None:
    track = load_track(reference_track)

    with pytest.raises(SurfaceMismatchError) as caught:
        check_surface(track, [FakeTyre("slick", "asphalt")])

    message = str(caught.value)
    assert "carpet" in message and "asphalt" in message and "slick" in message


def test_every_tyre_is_checked_not_just_the_first(reference_track) -> None:
    # A car with one axle on carpet and one on asphalt is exactly the mistake
    # that a search for one match waves through.
    track = load_track(reference_track)

    with pytest.raises(SurfaceMismatchError, match="asphalt"):
        check_surface(
            track, [FakeTyre("sponge", "carpet"), FakeTyre("slick", "asphalt")]
        )


def test_no_tyres_at_all_is_refused(reference_track) -> None:
    track = load_track(reference_track)

    with pytest.raises(SurfaceMismatchError, match="no tyres"):
        check_surface(track, [])


def test_the_real_reference_car_can_run_on_the_shipped_track(
    reference_car, reference_track
) -> None:
    # The end of the loop: the car that ships and the track that ships agree
    # about the surface, so somebody following the README runs into none of
    # the refusals above.
    from slipx_schema import load_car

    car = load_car(reference_car)
    track = load_track(reference_track)

    check_surface(track, [car.tyre_front, car.tyre_rear])
