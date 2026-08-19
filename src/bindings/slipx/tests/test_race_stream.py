# Copyright 2026 The SlipX Authors
# SPDX-License-Identifier: Apache-2.0

"""The race event stream, read with the reference MCAP library.

The claim under test is "one format, not two": the C++ race layer writes its
event stream as MCAP so that the same tooling that reads the run sinks reads
the races. The C++ suite already proves the stream replays a race through its
own reader; what only this side can prove is that the file is real MCAP by
the reference implementation's judgment, not by ours.
"""

from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[4]


def _demo_binary() -> Path:
    suffix = ".exe" if sys.platform == "win32" else ""
    candidates = [
        REPO_ROOT / tree / "src" / "orchestration" / "slipx_race"
        / f"slipx_race_demo{suffix}"
        for tree in ("build", "build-win", "build-gcc")
    ]
    binary = next((c for c in candidates if c.exists()), None)
    if binary is None:
        pytest.skip(f"{candidates[0]} not built; run cmake --build build")
    return binary


def test_the_reference_library_reads_a_race_stream(tmp_path) -> None:
    mcap_reader = pytest.importorskip(
        "mcap.reader", reason="reading MCAP needs the `mcap` extra"
    )

    output = tmp_path / "race.mcap"
    result = subprocess.run(
        [str(_demo_binary()), str(output)],
        capture_output=True,
        text=True,
        check=True,
    )
    # The build states its ruleset revision, out loud, on every run.
    assert "roboracer_rules" in result.stdout

    events = []
    with output.open("rb") as handle:
        reader = mcap_reader.make_reader(handle)
        for schema, channel, message in reader.iter_messages():
            assert channel.topic == "/race/events"
            assert channel.message_encoding == "json"
            assert schema.name == "slipx.race.RaceEvent"
            assert schema.encoding == "jsonschema"
            # The schema itself is valid JSON, and strict JSON at that.
            json.loads(schema.data)
            payload = json.loads(message.data)
            assert isinstance(payload["type"], str)
            assert message.log_time == int(payload["time"] * 1e9)
            events.append(payload)

    # A whole match travelled: it started, laps were counted, somebody won.
    types = {event["type"] for event in events}
    assert "round_start" in types
    assert "lap" in types
    assert "match_won" in types

    # The metadata record carries the pinned ruleset and the mechanised
    # configuration, so the file answers "under what rules" by itself.
    with output.open("rb") as handle:
        reader = mcap_reader.make_reader(handle)
        records = list(reader.iter_metadata())
    assert len(records) == 1
    metadata = records[0].metadata
    assert metadata["ruleset_repository"].endswith("roboracer_rules")
    assert len(metadata["ruleset_revision"]) == 40
    assert "config.laps_to_win" in metadata
    assert metadata["scenario"] == "race_demo"
