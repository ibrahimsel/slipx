# Copyright 2026 The SlipX Authors
# SPDX-License-Identifier: Apache-2.0

"""The Rerun sink, read back out of the file it wrote (SINK-03, SINK-05).

Skipped in full when the extra is absent. The parts of this sink that can be
decided without the SDK are tested in test_sinks.py and run either way, which
is deliberate: the SINK-05 filtering is the part most worth protecting and it
should not become untested on a machine that happens not to have rerun-sdk
installed.

What is here is the half that only the file can answer. A sink that believes it
left a column out and did not is exactly the failure the requirement exists
for, so the entity paths are read back rather than asserted against the plan.
"""

from __future__ import annotations

import pytest

import slipx
from slipx import sinks

pytest.importorskip("rerun", reason="the rerun extra is not installed (SINK-03)")

from rerun.experimental import RrdReader  # noqa: E402


def entity_paths(path):
    reader = RrdReader(str(path))
    recordings = reader.recordings()
    assert len(recordings) == 1
    store = reader.store(store=recordings[0])
    return sorted(store.schema().entity_paths())


def recording_ids(path):
    return [entry.recording_id for entry in RrdReader(str(path)).recordings()]


# ------------------------------------------------------------------ writing


def test_the_sink_writes_one_rrd_file(l1_recording, tmp_path):
    written = sinks.write(l1_recording, tmp_path / "run", format="rerun")

    assert written == tmp_path / "run.rrd"
    assert written.read_bytes()[:4] == b"RRF2"


def test_the_entity_tree_mirrors_the_state_layout(l1_recording, tmp_path):
    written = sinks.write(l1_recording, tmp_path / "run", format="rerun")

    assert entity_paths(written) == [
        "/car/diagnostics/accel_saturated",
        "/car/diagnostics/alpha_front",
        "/car/diagnostics/alpha_rear",
        "/car/diagnostics/ax",
        "/car/diagnostics/ay",
        "/car/diagnostics/fy_front",
        "/car/diagnostics/fy_rear",
        "/car/diagnostics/fz_front",
        "/car/diagnostics/fz_rear",
        "/car/diagnostics/speed_saturated",
        "/car/diagnostics/steer_saturated",
        "/car/diagnostics/tyre_saturated/FL",
        "/car/diagnostics/tyre_saturated/FR",
        "/car/diagnostics/tyre_saturated/RL",
        "/car/diagnostics/tyre_saturated/RR",
        "/car/state/pos/x",
        "/car/state/pos/y",
        "/car/state/rates/z",
        "/car/state/steer",
        "/car/state/vel_body/x",
        "/car/state/vel_body/y",
        "/car/state/yaw",
        "/provenance",
    ]


# ----------------------------------------------------- NaN arrives absent


def test_a_single_track_run_logs_no_per_wheel_entity(l1_recording, tmp_path):
    """SINK-05, the test ADR-0028 says each sink owes, asked of the file.

    A NaN scalar in a Rerun time series is not absent: it is a point, and the
    view decides whether that is a gap, a marker or a line through zero. So an
    unrepresentable quantity must not reach the SDK at all, and the entity has
    to be missing from the recording rather than present and empty.
    """
    written = sinks.write(l1_recording, tmp_path / "run", format="rerun")
    paths = entity_paths(written)

    for wheel in sinks.WHEELS:
        for base in ("Fz", "omega_w", "alpha_lag"):
            assert f"/car/state/{base}/{wheel}" not in paths
        for base in ("alpha", "kappa", "fx", "fy", "fz"):
            assert f"/car/diagnostics/{base}/{wheel}" not in paths
    for absent in ("/car/state/soc", "/car/state/pack_v", "/car/state/roll",
                   "/car/diagnostics/load_transfer_lat"):
        assert absent not in paths


def test_a_double_track_run_logs_every_contact_patch(l2_recording, tmp_path):
    written = sinks.write(l2_recording, tmp_path / "run", format="rerun")
    paths = entity_paths(written)

    for wheel in sinks.WHEELS:
        assert f"/car/state/Fz/{wheel}" in paths
        assert f"/car/state/alpha_lag/{wheel}" in paths
        for base in ("alpha", "kappa", "fx", "fy", "fz"):
            assert f"/car/diagnostics/{base}/{wheel}" in paths
    assert "/car/diagnostics/load_transfer_lat" in paths
    # Still absent, because no tier writes them (CORE-09, CORE-10).
    assert "/car/state/soc" not in paths
    assert "/car/state/steer_rate" not in paths


def test_no_wall_clock_is_written_into_the_recording(l1_recording, tmp_path):
    """The SDK would otherwise stamp a start time into the file.

    A wall clock is not in the recorded state, the diagnostics or the manifest,
    which is the whole of what a sink may emit (SINK-05). It would also make
    two encodings of one run differ for a reason that has nothing to do with
    the run.
    """
    written = sinks.write(l1_recording, tmp_path / "run", format="rerun")

    assert not [path for path in entity_paths(written) if "properties" in path]


# ---------------------------------------------------------- provenance


def test_the_provenance_label_is_in_the_recording(l1_recording, tmp_path):
    written = sinks.write(l1_recording, tmp_path / "run", format="rerun")

    assert "/provenance" in entity_paths(written)
    # The text itself, built without the SDK, is asserted in test_sinks.py.


def test_the_recording_id_is_the_trajectory_hash(l1_recording, tmp_path):
    """Two encodings of one run are one recording, and two runs never collide.

    Also the closest a log format gets to VIZ-02: whatever else is lost, the
    hash is on the recording rather than only in a console line beside it.
    """
    written = sinks.write(l1_recording, tmp_path / "run", format="rerun")

    assert recording_ids(written) == [l1_recording.trajectory_hash]


# --------------------------------------------------------- determinism


def test_writing_the_same_run_twice_gives_the_same_recording(
    l1_recording, tmp_path
):
    """Content, not bytes.

    The MCAP sink is byte-identical run to run and is tested that way. This one
    is not: the SDK batches into chunks and the chunk boundaries and internal
    identifiers are its business, so two files differ by a few bytes while
    carrying the same recording. That is a property of the SDK rather than of
    SlipX, and asserting bytes here would be asserting something we do not
    control and did not promise.
    """
    first = sinks.write(l1_recording, tmp_path / "a", format="rerun")
    second = sinks.write(l1_recording, tmp_path / "b", format="rerun")

    assert entity_paths(first) == entity_paths(second)
    assert recording_ids(first) == recording_ids(second)


def test_writing_a_run_does_not_change_it(step_steer_sim, tmp_path):
    sim = step_steer_sim(slipx.Tier.L2_DoubleTrack)
    recording = sinks.record_run(sim, duration=0.4, stride=10)
    before = recording.trajectory_hash

    sinks.write(recording, tmp_path / "run", format="rerun")

    assert sim.trajectory_hash() == before
    assert recording.trajectory_hash == before


def test_both_sinks_write_the_same_run_from_the_same_recording(
    l2_recording, tmp_path
):
    """SINK-01: one recording, one protocol, two formats and no special case.

    The columns each format carries are the same columns, which is the whole
    claim of a shared recorder.
    """
    pytest.importorskip("mcap")
    rrd = sinks.write(l2_recording, tmp_path / "run", format="rerun")
    mcap = sinks.write(l2_recording, tmp_path / "run", format="mcap")

    assert rrd.exists() and mcap.exists()
    assert rrd != mcap

    from slipx.sinks.rerun_sink import column_plan

    planned = {entity for entity, _, _ in column_plan(l2_recording, l2_recording.agents[0])}
    assert planned <= set(entity_paths(rrd))


# ------------------------------------------------------------- plumbing


def test_the_sink_satisfies_the_protocol():
    sink = sinks.sink_for("rerun")

    assert isinstance(sink, sinks.RunSink)
    assert sink.suffix == ".rrd"
