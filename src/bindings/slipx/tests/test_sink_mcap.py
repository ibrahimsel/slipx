# Copyright 2026 The SlipX Authors
# SPDX-License-Identifier: Apache-2.0

"""The MCAP sink (SINK-02, SINK-04, SINK-05).

Skipped in full when the extra is not installed, which is the point of an
extra: the distribution installs, imports and passes its tests with both
SDK-backed sinks absent (SINK-03). CI installs them in a job of its own,
because an optional path with no CI is an optional path that is broken.

Everything here reads the file back rather than inspecting the sink's
intentions. A sink that believes it left a key out and did not is the failure
mode the tests exist for.
"""

from __future__ import annotations

import json
import math

import pytest

import slipx
from slipx import sinks

pytest.importorskip("mcap", reason="the mcap extra is not installed (SINK-03)")

from mcap.reader import make_reader  # noqa: E402


def _reject_constant(name):
    raise AssertionError(
        f"an MCAP message carried the JSON token {name}. A quantity a tier "
        f"cannot represent is written by leaving the key out, not by a token "
        f"only Python's parser accepts (SINK-05)."
    )


def read_messages(path):
    """Every message as (topic, decoded payload), in log order."""
    out = []
    with open(path, "rb") as handle:
        for _, channel, message in make_reader(handle).iter_messages():
            out.append(
                (
                    channel.topic,
                    json.loads(message.data, parse_constant=_reject_constant),
                )
            )
    return out


def read_metadata(path):
    with open(path, "rb") as handle:
        return {record.name: record.metadata for record in make_reader(handle).iter_metadata()}


def read_summary(path):
    with open(path, "rb") as handle:
        return make_reader(handle).get_summary()


def topic_messages(messages, suffix):
    return [payload for topic, payload in messages if topic.endswith(suffix)]


# ------------------------------------------------------------------ writing


def test_the_sink_writes_one_mcap_file(l1_recording, tmp_path):
    written = sinks.write(l1_recording, tmp_path / "run")

    assert written == tmp_path / "run.mcap"
    assert written.read_bytes()[:8] == b"\x89MCAP0\r\n"


def test_mcap_is_what_you_get_without_asking(l1_recording, tmp_path):
    # SINK-02. The default is a decision, so it is asserted rather than left
    # to whichever sink happens to be first in the table.
    default = sinks.write(l1_recording, tmp_path / "default")
    named = sinks.write(l1_recording, tmp_path / "named", format="mcap")

    assert default.suffix == ".mcap"
    assert default.read_bytes() == named.read_bytes()


def test_an_explicit_suffix_is_left_alone(l1_recording, tmp_path):
    written = sinks.write(l1_recording, tmp_path / "run.bag")

    assert written.name == "run.bag"


def test_there_is_one_message_per_agent_per_frame(l1_recording, tmp_path):
    path = sinks.write(l1_recording, tmp_path / "run")
    messages = read_messages(path)

    assert len(topic_messages(messages, "/state")) == len(l1_recording)
    assert len(topic_messages(messages, "/diagnostics")) == len(l1_recording)
    assert len(topic_messages(messages, "/provenance")) == 1


def test_the_topics_name_the_agent(l2_recording, tmp_path):
    path = sinks.write(l2_recording, tmp_path / "run")
    topics = {channel.topic for channel in read_summary(path).channels.values()}

    assert topics == {
        "/slipx/car/state",
        "/slipx/car/diagnostics",
        "/slipx/provenance",
    }


def test_log_times_are_the_simulation_clock(l1_recording, tmp_path):
    path = sinks.write(l1_recording, tmp_path / "run")

    with open(path, "rb") as handle:
        stamps = sorted(
            {
                message.log_time
                for _, channel, message in make_reader(handle).iter_messages()
                if channel.topic.endswith("/state")
            }
        )

    assert stamps[0] == 10_000_000  # 0.01 s, the first recorded step
    assert stamps[-1] == 400_000_000
    assert len(stamps) == len(l1_recording)


# ----------------------------------------------------- NaN arrives absent


def test_a_single_track_run_carries_no_per_wheel_key(l1_recording, tmp_path):
    """SINK-05, the test ADR-0028 says each sink owes.

    L1 has one tyre per axle. Every per-wheel quantity is unrepresentable, and
    unrepresentable has to arrive at the viewer as nothing at all: not as zero,
    not as null, not as a key with a NaN in it.
    """
    path = sinks.write(l1_recording, tmp_path / "run")
    messages = read_messages(path)

    for payload in topic_messages(messages, "/state"):
        for absent in ("Fz", "omega_w", "alpha_lag", "soc", "pack_v", "pitch"):
            assert absent not in payload, absent
        assert payload["pos"]["x"] == pytest.approx(payload["pos"]["x"])
        assert "z" not in payload["pos"]

    for payload in topic_messages(messages, "/diagnostics"):
        for absent in ("alpha", "kappa", "fx", "fy", "fz",
                       "load_transfer_long", "load_transfer_lat"):
            assert absent not in payload, absent
        assert "alpha_front" in payload
        assert "ay" in payload


def test_absent_is_absent_and_never_a_zero_or_a_null(l1_recording, tmp_path):
    path = sinks.write(l1_recording, tmp_path / "run")

    for topic, payload in read_messages(path):
        for key, value in _walk(payload):
            assert value is not None, key
            assert not (isinstance(value, float) and math.isnan(value)), key


def test_a_double_track_run_carries_every_contact_patch(l2_recording, tmp_path):
    path = sinks.write(l2_recording, tmp_path / "run")
    messages = read_messages(path)

    for payload in topic_messages(messages, "/state"):
        assert set(payload["Fz"]) == set(sinks.WHEELS)
        assert set(payload["alpha_lag"]) == set(sinks.WHEELS)
    for payload in topic_messages(messages, "/diagnostics"):
        for base in ("alpha", "kappa", "fx", "fy", "fz"):
            assert set(payload[base]) == set(sinks.WHEELS), base
        assert "load_transfer_lat" in payload


def test_the_two_tiers_produce_different_keys_from_the_same_writer(
    l1_recording, l2_recording, tmp_path
):
    """The sink has no tier branch: what differs is the data it was given."""
    single = topic_messages(read_messages(
        sinks.write(l1_recording, tmp_path / "l1")), "/state")[0]
    double = topic_messages(read_messages(
        sinks.write(l2_recording, tmp_path / "l2")), "/state")[0]

    assert "Fz" not in single
    assert "Fz" in double
    assert set(single) < set(double)


def test_the_encoder_refuses_a_nan_rather_than_writing_one():
    """The guard behind the rule, tested directly.

    Python's json writes a bare NaN token by default. No other parser accepts
    it, and a file containing one would have put the unrepresentable quantity
    back in under a disguise.
    """
    from slipx.sinks.mcap_sink import _encode

    with pytest.raises(ValueError):
        _encode({"fz": float("nan")})


def test_the_schema_requires_nothing(l1_recording, tmp_path):
    path = sinks.write(l1_recording, tmp_path / "run")
    summary = read_summary(path)

    for schema in summary.schemas.values():
        assert schema.encoding == "jsonschema"
        document = json.loads(schema.data)
        assert "required" not in document
    names = {schema.name for schema in summary.schemas.values()}
    assert names == {
        "slipx.VehicleState", "slipx.StepDiagnostics", "slipx.Provenance"
    }


def test_the_schema_describes_the_whole_layout_not_one_tier(
    l1_recording, tmp_path
):
    # Generated from the recorder's column list, so the L1 file's schema still
    # describes the per-wheel fields it happens not to carry.
    path = sinks.write(l1_recording, tmp_path / "run")
    schemas = {s.name: json.loads(s.data) for s in read_summary(path).schemas.values()}
    state = schemas["slipx.VehicleState"]["properties"]

    assert set(state["Fz"]["properties"]) == set(sinks.WHEELS)
    assert state["yaw"]["type"] == "number"


def test_saturation_flags_are_booleans_and_always_present(l1_recording, tmp_path):
    path = sinks.write(l1_recording, tmp_path / "run")

    for payload in topic_messages(read_messages(path), "/diagnostics"):
        assert isinstance(payload["steer_saturated"], bool)
        assert set(payload["tyre_saturated"]) == set(sinks.WHEELS)


# ---------------------------------------------------------- provenance


def test_the_provenance_label_is_in_the_file(l1_recording, tmp_path):
    """NFR-08. A run that does not say what its numbers are worth is a
    stronger claim than this project is entitled to make."""
    path = sinks.write(l1_recording, tmp_path / "run")
    metadata = read_metadata(path)

    assert metadata["slipx"]["provenance"] == "provisional"
    assert l1_recording.trajectory_hash in metadata["slipx"]["provenance_line"]

    spoken = topic_messages(read_messages(path), "/provenance")[0]
    assert "provisional" in spoken["text"]
    assert spoken["trajectory_hash"] == l1_recording.trajectory_hash


def test_the_whole_manifest_travels_with_the_file(l1_recording, tmp_path):
    path = sinks.write(l1_recording, tmp_path / "run")
    manifest = json.loads(read_metadata(path)["slipx.manifest"]["json"])

    assert manifest["trajectory_hash"] == l1_recording.trajectory_hash
    assert manifest["run"]["dt"] == pytest.approx(l1_recording.dt)
    assert manifest["build"]["compiler_id"] != ""
    assert manifest["agents"][0]["params_digest"] != ""


def test_every_channel_names_its_agent_and_tier(l2_recording, tmp_path):
    path = sinks.write(l2_recording, tmp_path / "run")

    for channel in read_summary(path).channels.values():
        if channel.topic.endswith("/provenance"):
            continue
        assert channel.metadata["slipx.tier"] == "L2_DoubleTrack"
        assert channel.metadata["slipx.provenance"] == "provisional"
        assert channel.metadata["slipx.trajectory_hash"] != ""


# --------------------------------------------------------- determinism


def test_writing_the_same_run_twice_gives_the_same_bytes(l1_recording, tmp_path):
    """Nothing in the sink reads a clock, a hostname or a random number.

    An encoder that stamped wall time into the file would make two encodings
    of one run differ, and then a file could not be compared against a
    reference the way a trajectory hash can (NFR-02, in the spirit of it).
    """
    first = sinks.write(l1_recording, tmp_path / "a")
    second = sinks.write(l1_recording, tmp_path / "b")

    assert first.read_bytes() == second.read_bytes()


def test_writing_a_run_does_not_change_it(step_steer_sim, tmp_path):
    """SINK determinism: encoding is an observation."""
    sim = step_steer_sim(slipx.Tier.L1_Bicycle)
    recording = sinks.record_run(sim, duration=0.4, stride=10)
    before = recording.trajectory_hash

    sinks.write(recording, tmp_path / "run")

    assert recording.trajectory_hash == before
    assert sim.trajectory_hash() == before
    assert recording.agents[0].state["pos.x"][-1] == pytest.approx(
        recording.agents[0].state["pos.x"][-1]
    )


# ------------------------------------------------------------- plumbing


def test_an_unknown_compression_is_refused():
    with pytest.raises(ValueError):
        sinks.sink_for("mcap", compression="brotli")


def test_the_sink_satisfies_the_protocol():
    sink = sinks.sink_for("mcap")

    assert isinstance(sink, sinks.RunSink)
    assert sink.suffix == ".mcap"


def _walk(payload, prefix=""):
    for key, value in payload.items():
        name = f"{prefix}{key}"
        if isinstance(value, dict):
            yield from _walk(value, f"{name}.")
        else:
            yield name, value
