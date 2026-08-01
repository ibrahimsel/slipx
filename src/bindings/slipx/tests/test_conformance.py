# Copyright 2026 The SlipX Authors
# SPDX-License-Identifier: Apache-2.0

"""The P0 exit gate, and the simulation API underneath it.

The gate, in the SRS's words: a third party can pip install the package, load a
car directory, integrate a step-steer manoeuvre and get the same trajectory
hash as CI. Three of those four are testable here; the fourth needs CI, and the
workflow runs the same conformance run against the same reference file.
"""

from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path

import pytest

import slipx

REPO_ROOT = Path(__file__).resolve().parents[4]
REFERENCE_CAR = REPO_ROOT / "examples" / "cars" / "reference_1_10"


def _subprocess_env() -> dict[str, str]:
    """An environment in which a child process can import slipx.

    Working in the tree, the packages are importable only because pytest was
    configured to put them on the path, and a subprocess does not inherit
    that. Rebuilding PYTHONPATH from this interpreter's own sys.path makes the
    console-script tests work both here and against an installed wheel, which
    is the configuration the exit gate is actually claimed for.
    """
    env = dict(os.environ)
    existing = env.get("PYTHONPATH", "")
    entries = [p for p in sys.path if p]
    env["PYTHONPATH"] = os.pathsep.join(entries + ([existing] if existing else []))
    return env


def test_the_conformance_run_is_reproducible_within_the_process() -> None:
    first = slipx.make_conformance_run()
    first.run_for(5.0)
    second = slipx.make_conformance_run()
    second.run_for(5.0)

    assert first.trajectory_hash() == second.trajectory_hash()
    assert len(first.trajectory_hash()) == 16


def test_the_python_hash_matches_the_cpp_binary() -> None:
    """Both bindings compute the same hash, or neither claim means anything.

    The extension and the standalone binary are the same code compiled twice
    with the same flags, so agreement is expected; it is asserted because the
    moment it stops being true, every published reference hash silently becomes
    ambiguous about which one it referred to.
    """
    binary = REPO_ROOT / "build" / "src" / "orchestration" / "slipx_sim" / "slipx_conformance"
    if not binary.exists():
        pytest.skip(f"{binary} not built; run cmake --build build")

    result = subprocess.run(
        [str(binary), "--quiet"], capture_output=True, text=True, check=True
    )
    from_cpp = result.stdout.strip().splitlines()[-1]

    sim = slipx.make_conformance_run()
    sim.run_for(5.0)
    assert sim.trajectory_hash() == from_cpp


def test_the_console_script_prints_the_hash_on_its_own_last_line() -> None:
    # A shell script has to be able to take the last line verbatim.
    result = subprocess.run(
        [sys.executable, "-m", "slipx.conformance", "--quiet"],
        capture_output=True,
        text=True,
        check=True,
        env=_subprocess_env(),
    )
    last = result.stdout.strip().splitlines()[-1]
    assert len(last) == 16
    assert all(c in "0123456789abcdef" for c in last)

    sim = slipx.make_conformance_run()
    sim.run_for(5.0)
    assert last == sim.trajectory_hash()


def test_the_console_script_fails_on_a_mismatch() -> None:
    result = subprocess.run(
        [sys.executable, "-m", "slipx.conformance", "--quiet", "--expect", "0" * 16],
        capture_output=True,
        text=True,
        env=_subprocess_env(),
    )
    assert result.returncode == 1
    assert "DETERMINISM CHECK FAILED" in result.stderr
    # And it must point at NFR-03 rather than letting somebody conclude the
    # library is broken when they have merely changed compiler.
    assert "NFR-03" in result.stderr


def test_the_exit_gate_end_to_end() -> None:
    """Load a car directory, drive a step steer, get a hash. Twice, identically."""

    def run() -> str:
        car = slipx.load_car(REFERENCE_CAR)

        config = slipx.SimulationConfig()
        config.master_seed = 20260801
        config.schema_version = car.spec.schema_version
        sim = slipx.Simulation(config)

        spec = slipx.AgentSpec()
        spec.name = car.name
        spec.tier = slipx.Tier.L1_Bicycle
        spec.params = car.params
        spec.initial_state.vel_body.x = 5.0
        spec.policy = slipx.step_steer()
        sim.add_agent(spec)

        sim.run_for(5.0)
        return sim.trajectory_hash()

    assert run() == run()

    # The manifest records the schema version the parameters came through, so
    # a result cannot be compared against one produced from a different file
    # format (SIM-06).
    car = slipx.load_car(REFERENCE_CAR)
    assert car.spec.schema_version


def test_a_python_policy_is_just_a_callable() -> None:
    # SIM-02: in-process mode where agent policies are callables and no
    # middleware is present.
    sim = slipx.Simulation()
    spec = slipx.AgentSpec()
    spec.tier = slipx.Tier.L1_Bicycle
    spec.initial_state.vel_body.x = 5.0

    seen = []

    def policy(state, time, rng):
        seen.append(time)
        return slipx.DriveInput(steer_cmd=0.05, accel_cmd=slipx.hold_speed(state, 5.0))

    spec.policy = policy
    sim.add_agent(spec)
    sim.run(100)

    assert len(seen) == 100
    assert seen[0] == 0.0
    assert seen[-1] == pytest.approx(0.099)
    assert sim.state(0).yaw > 0.0


def test_time_is_steps_times_dt() -> None:
    sim = slipx.make_conformance_run()
    sim.run(10000)
    assert sim.step_count == 10000
    assert sim.time == 10.0  # exactly, not approximately


def test_replay_from_the_input_log_is_bit_identical() -> None:
    # SIM-07, from Python: the adjudication path, where the policies may be
    # gone or may have been the thing under dispute.
    sim = slipx.make_conformance_run()
    sim.set_input_logging(True)
    sim.run_for(2.0)

    original = sim.trajectory_hash()
    log = sim.input_log()
    assert len(log) == 2000

    sim.replay(log)
    assert sim.trajectory_hash() == original


def test_the_manifest_records_the_build_it_ran_on() -> None:
    # NFR-02 scopes bit-identity to a fixed (platform, compiler, flag set), so
    # a hash without that context is not evidence of anything.
    sim = slipx.make_conformance_run()
    sim.run_for(1.0)
    manifest = sim.manifest()

    assert manifest.compiler_id
    assert manifest.system_processor
    assert "-ffp-contract=off" in manifest.cxx_flags
    assert manifest.integrator == "rk4"
    assert manifest.steps == 1000
    assert len(manifest.configuration_digest()) == 16
    assert "not guaranteed" in manifest.to_json()


def test_agents_are_added_without_an_upper_bound() -> None:
    # SIM-09.
    sim = slipx.Simulation()
    for i in range(100):
        spec = slipx.AgentSpec()
        spec.tier = slipx.Tier.L1_Bicycle
        spec.initial_state.vel_body.x = 2.0 + 0.01 * i
        sim.add_agent(spec)

    assert sim.agent_count == 100
    sim.run_for(0.2)
    assert sim.state(99).pos.x > sim.state(0).pos.x


def test_the_rng_matches_the_published_splitmix64_sequence() -> None:
    # The same pinned vectors the C++ suite asserts. Any future reimplementation
    # in another binding has these to check against.
    rng = slipx.Rng(0)
    assert rng.next_u64() == 0xE220A8397B1DCDAF
    assert rng.next_u64() == 0x6E789E6AA1B965F4


def test_the_hash_matches_the_published_fnv1a_vectors() -> None:
    assert slipx.hash_text("foobar") == "85944171f73967e8"
    assert slipx.hash_text("") == "cbf29ce484222325"
