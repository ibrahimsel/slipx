# Copyright 2026 The SlipX Authors
# SPDX-License-Identifier: Apache-2.0

"""The recorder and the sink protocol (SINK-01, SINK-05).

Nothing here needs an optional extra. The format-specific tests live beside
their sinks and skip when their SDK is absent; these are the rules that hold
for every sink because they are established before any sink sees the data.
"""

from __future__ import annotations

import math
import subprocess
import sys

import pytest

import slipx
from slipx import sinks

WHEEL_STATE = tuple(
    f"{base}.{wheel}"
    for base in ("Fz", "omega_w", "alpha_lag")
    for wheel in sinks.WHEELS
)
WHEEL_DIAGNOSTICS = tuple(
    f"{base}.{wheel}"
    for base in ("alpha", "kappa", "fx", "fy", "fz")
    for wheel in sinks.WHEELS
)


def all_nan(column) -> bool:
    return all(math.isnan(value) for value in column)


def no_nan(column) -> bool:
    return not any(math.isnan(value) for value in column)


# --------------------------------------------------------------- recording


def test_a_recording_has_one_frame_per_recorded_step(l1_recording):
    # 0.4 s at 1 kHz, every tenth step.
    assert len(l1_recording) == 40
    assert l1_recording.stride == 10
    assert l1_recording.times[0] == pytest.approx(0.01)
    assert l1_recording.times[-1] == pytest.approx(0.4)

    agent = l1_recording.agents[0]
    for column in agent.state.values():
        assert len(column) == 40
    for column in agent.diagnostics.values():
        assert len(column) == 40
    for column in agent.flags.values():
        assert len(column) == 40


def test_the_first_frame_is_after_the_first_step_not_before_it():
    """A pre-step frame would be one frame of fiction at the front of the run.

    Before anything has stepped, the diagnostics block is default constructed:
    zeros, and a tier field reading 0, which is L0. Recording that would put a
    frame in the run claiming an L2 car reported zero slip at every wheel.
    """
    sim = slipx.make_conformance_run(slipx.ConformanceSpec())
    run = sinks.record_run(sim, steps=3)

    assert run.times == pytest.approx((0.001, 0.002, 0.003))
    assert len(run) == 3


def test_recording_continues_from_wherever_the_simulation_is():
    sim = slipx.make_conformance_run(slipx.ConformanceSpec())
    sim.run(100)
    run = sinks.record_run(sim, steps=5)

    assert run.times[0] == pytest.approx(0.101)


def test_the_provenance_label_and_the_hash_are_on_the_recording(l1_recording):
    # NFR-08. Every sink writes this line, so it is built once here rather
    # than reassembled per format.
    line = l1_recording.provenance_line()

    assert "provisional" in line
    assert l1_recording.trajectory_hash in line
    assert l1_recording.slipx_version in line
    assert l1_recording.trajectory_hash != ""


def test_the_manifest_travels_with_the_recording(l1_recording):
    assert '"trajectory_hash"' in l1_recording.manifest_json
    assert l1_recording.trajectory_hash in l1_recording.manifest_json
    assert l1_recording.agents[0].params_digest != ""


# ----------------------------------------------------- NaN means absent

def test_a_single_track_run_records_no_per_wheel_quantity(l1_recording):
    """SINK-05, at the recorder. L1 has one tyre per axle and no load transfer,
    so every per-wheel column is NaN from end to end."""
    agent = l1_recording.agents[0]

    for name in WHEEL_DIAGNOSTICS:
        assert all_nan(agent.diagnostics[name]), name
    for name in WHEEL_STATE:
        assert all_nan(agent.state[name]), name
    for name in ("load_transfer_long", "load_transfer_lat"):
        assert all_nan(agent.diagnostics[name]), name


def test_a_single_track_run_still_records_what_it_does_represent(l1_recording):
    agent = l1_recording.agents[0]

    for name in ("alpha_front", "alpha_rear", "fy_front", "fy_rear", "ax", "ay"):
        assert no_nan(agent.diagnostics[name]), name
    for name in ("pos.x", "pos.y", "yaw", "vel_body.x", "vel_body.y",
                 "rates.z", "steer"):
        assert no_nan(agent.state[name]), name


def test_a_frame_drops_its_absent_columns_rather_than_zeroing_them(l1_recording):
    """The rule a sink applies, checked at the boundary the sinks share."""
    agent = l1_recording.agents[0]
    state = agent.state_frame(0)
    diagnostics = agent.diagnostics_frame(0)

    for name in WHEEL_STATE:
        assert name not in state
    for name in WHEEL_DIAGNOSTICS:
        assert name not in diagnostics
    assert "pos.x" in state
    assert "alpha_front" in diagnostics
    assert not any(math.isnan(v) for v in state.values())
    assert not any(math.isnan(v) for v in diagnostics.values())


def test_a_double_track_run_records_every_contact_patch(l2_recording):
    agent = l2_recording.agents[0]
    assert agent.tier == "L2_DoubleTrack"

    for name in ("alpha", "kappa", "fx", "fy", "fz"):
        for wheel in sinks.WHEELS:
            assert no_nan(agent.diagnostics[f"{name}.{wheel}"]), f"{name}.{wheel}"
    for wheel in sinks.WHEELS:
        assert no_nan(agent.state[f"Fz.{wheel}"])
        assert no_nan(agent.state[f"alpha_lag.{wheel}"])
        assert no_nan(agent.state[f"omega_w.{wheel}"])
    for name in ("load_transfer_long", "load_transfer_lat"):
        assert no_nan(agent.diagnostics[name])


def test_a_double_track_run_still_hides_what_no_tier_writes(l2_recording):
    """The rows of the table that are about the future, asserted so that
    landing CORE-09 or CORE-10 breaks a test rather than quietly shipping a
    battery reading nobody modelled."""
    agent = l2_recording.agents[0]

    for name in ("soc", "pack_v", "steer_rate", "pitch", "roll",
                 "pos.z", "vel_body.z", "rates.x", "rates.y"):
        assert all_nan(agent.state[name]), name


def test_the_zeros_a_lower_tier_leaves_behind_are_not_recorded_as_zeros():
    """The specific lie this rule exists to stop.

    state.hpp parks an unrepresented field at zero rather than NaN, and says
    why: the state is hashed, and a NaN in a trajectory is evidence the run is
    broken. The live state therefore really does read 0.0 for the front-left
    vertical load of an L1 car, and a sink that wrote it would plot a tyre
    carrying no load for the whole run (ADR-0006).
    """
    sim = slipx.make_conformance_run(slipx.ConformanceSpec())
    sim.advance()

    assert sim.state(0).Fz[0] == 0.0  # what the core holds
    run = sinks.record_run(sim, steps=1)
    assert math.isnan(run.agents[0].state["Fz.FL"][0])  # what a sink is given


@pytest.mark.parametrize(
    "column,tier,expected",
    [
        ("pos.x", 0, True),
        ("Fz.FL", 1, False),
        ("Fz.FL", 2, True),
        ("alpha_lag.RR", 1, False),
        ("alpha_lag.RR", 2, True),
        ("omega_w.FL", 2, True),
        ("soc", 2, False),
        ("steer_rate", 2, False),
        ("roll", 2, False),
        ("rates.z", 0, True),
        ("vel_body.y", 0, True),
        ("vel_body.z", 2, False),
    ],
)
def test_the_representability_table_says_what_the_tiers_say(column, tier, expected):
    assert sinks.represented(column, tier) is expected


def test_flags_are_recorded_as_booleans_and_not_normalised(l1_recording):
    # A bool has no NaN, so it cannot carry the absent rule and is kept apart
    # from the floats rather than being given a third state.
    agent = l1_recording.agents[0]

    assert set(agent.flags) == set(sinks.FLAG_COLUMNS)
    for column in agent.flags.values():
        assert all(isinstance(value, bool) for value in column)


# ------------------------------------------------------------ determinism


def test_recording_a_run_does_not_change_it():
    """NFR-02. Encoding is an observation, and an observation that moved the
    trajectory would make every published hash conditional on whether anybody
    was watching."""
    plain = slipx.make_conformance_run(slipx.ConformanceSpec())
    plain.run(400)

    recorded = slipx.make_conformance_run(slipx.ConformanceSpec())
    run = sinks.record_run(recorded, steps=400)

    assert recorded.trajectory_hash() == plain.trajectory_hash()
    assert run.trajectory_hash == plain.trajectory_hash()


def test_the_stride_does_not_change_the_run():
    """Keeping fewer frames must not step the car differently."""
    every = sinks.record_run(
        slipx.make_conformance_run(slipx.ConformanceSpec()), steps=400
    )
    sparse = sinks.record_run(
        slipx.make_conformance_run(slipx.ConformanceSpec()), steps=400, stride=40
    )

    assert every.trajectory_hash == sparse.trajectory_hash
    assert len(sparse) == 10
    assert sparse.agents[0].state["pos.x"][-1] == every.agents[0].state["pos.x"][-1]


def test_two_recordings_of_the_same_run_agree_column_for_column(
    l2_recording, step_steer_sim
):
    again = sinks.record_run(
        step_steer_sim(slipx.Tier.L2_DoubleTrack), duration=0.4, stride=10
    )

    assert again.trajectory_hash == l2_recording.trajectory_hash
    for name, column in l2_recording.agents[0].state.items():
        assert again.agents[0].state[name] == column, name


# ------------------------------------------------------------- the protocol


def test_the_format_table_is_the_only_place_a_format_is_named():
    assert sinks.formats() == ("mcap", "rerun")
    assert sinks.DEFAULT_FORMAT == "mcap"


def test_a_format_with_no_extra_installed_is_still_a_known_format():
    # Reporting it as unknown would send somebody looking for a spelling
    # mistake instead of an install.
    assert "rerun" in sinks.formats()
    assert "mcap" in sinks.formats()


def test_an_unknown_format_names_the_ones_that_exist():
    with pytest.raises(ValueError) as raised:
        sinks.sink_for("rosbag")

    assert "mcap" in str(raised.value)
    assert "rerun" in str(raised.value)


def test_neither_sdk_is_imported_when_slipx_is():
    """SINK-03, checked the only way that means anything.

    In a subprocess, because this test process may well have imported an SDK
    already for one of the format tests, and then the assertion would pass or
    fail on test ordering rather than on the import graph.
    """
    program = (
        "import sys, slipx, slipx.sinks;"
        "bad = [m for m in ('mcap', 'rerun', 'numpy', 'pyarrow')"
        " if m in sys.modules];"
        "print(bad);"
        "sys.exit(1 if bad else 0)"
    )
    result = subprocess.run(
        [sys.executable, "-c", program],
        capture_output=True,
        text=True,
        env=_child_env(),
    )

    assert result.returncode == 0, (
        f"importing slipx pulled in {result.stdout.strip()}. Both SDK-backed "
        f"sinks are optional extras and neither may be imported at package "
        f"import time (SINK-03)."
    )


def test_no_sink_opens_a_window():
    """SINK-04, as a property of the source rather than of one run.

    Rerun's SDK will happily spawn a viewer process, connect to a running one
    or serve over a socket, and the MCAP tooling has a CLI that will open
    things. A sink calls none of it: it writes a file and stops, so NFR-04
    holds by construction rather than by care (ADR-0028).

    A source scan rather than a behavioural test because the behaviour under
    test is one that must never happen, and a test that only fails when
    somebody has already opened a window is not a guard.
    """
    import io
    import tokenize
    from pathlib import Path

    forbidden = (
        "spawn", "connect", "connect_grpc", "serve", "serve_web",
        "notebook_show", "show", "subprocess", "webbrowser", "socket",
        "system", "startfile", "popen", "Popen",
    )
    directory = Path(sinks.__file__).parent

    offences = []
    for module in sorted(directory.glob("*.py")):
        with module.open(encoding="utf-8") as handle:
            names = {
                token.string
                for token in tokenize.generate_tokens(handle.readline)
                if token.type == tokenize.NAME
            }
        # Names only: comments and docstrings are excluded by construction,
        # which matters because this file and those docstrings both have to be
        # able to say the word "spawn".
        offences.extend(
            f"{module.name}: {name}" for name in sorted(names & set(forbidden))
        )

    assert offences == [], (
        f"a sink reaches for {offences}. A sink writes a file and never opens "
        f"a window, even where the SDK offers one (SINK-04, NFR-04)."
    )


def _child_env():
    import os
    from pathlib import Path

    root = Path(__file__).resolve().parents[4]
    env = dict(os.environ)
    env["PYTHONPATH"] = os.pathsep.join(
        [
            str(root / "src" / "bindings" / "slipx"),
            str(root / "src" / "core" / "slipx_schema"),
        ]
    )
    return env


# ------------------------------------------------------------- refusals


def test_record_run_refuses_an_ambiguous_length():
    sim = slipx.make_conformance_run(slipx.ConformanceSpec())

    with pytest.raises(ValueError):
        sinks.record_run(sim, steps=10, duration=1.0)
    with pytest.raises(ValueError):
        sinks.record_run(sim)


def test_record_run_refuses_a_run_of_nothing():
    sim = slipx.make_conformance_run(slipx.ConformanceSpec())

    with pytest.raises(ValueError):
        sinks.record_run(sim, steps=0)
    with pytest.raises(ValueError):
        sinks.record_run(sim, steps=10, stride=0)


def test_record_run_refuses_a_simulation_with_no_agents():
    with pytest.raises(ValueError):
        sinks.record_run(slipx.Simulation(), steps=10)
