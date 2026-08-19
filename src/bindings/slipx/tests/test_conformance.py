# Copyright 2026 The SlipX Authors
# SPDX-License-Identifier: Apache-2.0

"""The P0 exit gate, and the simulation API underneath it.

The gate, in the SRS's words: a third party can pip install the package, load a
car directory, integrate a step-steer manoeuvre and get the same trajectory
hash as CI. Three of those four are testable here; the fourth needs CI, and the
workflow runs the same conformance run against the same reference file.
"""

from __future__ import annotations

import importlib.util
import os
import subprocess
import sys
from pathlib import Path

import pytest

import slipx

REPO_ROOT = Path(__file__).resolve().parents[4]
REFERENCE_CAR = REPO_ROOT / "examples" / "cars" / "reference_1_10"


def _check_conformance_module():
    """Import tools/check_conformance.py by path.

    It is a script rather than a package, and it is not installed with the
    wheel, so the tests that need it skip when the checkout is not there.
    """
    path = REPO_ROOT / "tools" / "check_conformance.py"
    if not path.exists():
        pytest.skip(f"{path} not present; not running from a checkout")
    spec = importlib.util.spec_from_file_location("_slipx_check_conformance", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


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
    # The binary this interpreter can actually execute. A checkout that builds
    # under more than one toolchain holds more than one build tree, and on
    # Windows the Linux binary from a WSL build still *exists*, so existence
    # alone picks a file CreateProcess then refuses. The platform-correct
    # suffix is the discriminator: an ELF binary is never named .exe.
    suffix = ".exe" if sys.platform == "win32" else ""
    candidates = [
        REPO_ROOT / tree / "src" / "orchestration" / "slipx_sim"
        / f"slipx_conformance{suffix}"
        for tree in ("build", "build-win", "build-gcc")
    ]
    binary = next((c for c in candidates if c.exists()), None)
    if binary is None:
        pytest.skip(f"{candidates[0]} not built; run cmake --build build")

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

    # And the C library, because it is libm the hash tracks and libm is not
    # chosen until the wheel is installed (ADR-0033).
    assert manifest.libc_id
    if sys.platform.startswith("linux") and manifest.libc_id == "glibc":
        assert manifest.libc_version


def test_the_reference_key_separates_two_c_libraries() -> None:
    """A run on another glibc must miss the row rather than fail against it.

    The measured case: one wheel, byte for byte the same, hashed differently on
    glibc 2.28 and glibc 2.39. Before the C library entered the key, that run
    matched a published row it was never entitled to match, and the checker
    called it a determinism bug (ADR-0033).
    """
    check = _check_conformance_module()

    build = {
        "system_processor": "x86_64",
        "compiler_id": "GNU",
        "compiler_version": "13.3.0",
        "build_type": "RelWithDebInfo",
        "libc_id": "glibc",
        "libc_version": "2.39",
    }
    assert check.libc_column(build) == "glibc-2.39"

    reference = check.read_reference()
    key_columns = check.COLUMNS[:-1]

    def row_for(key):
        return next(
            (r for r in reference if all(r[c] == key[c] for c in key_columns)), None
        )

    recorded = check.key_of({"build": build}, "L1", "rk4")
    assert row_for(recorded) is not None, "this build is one of the published ones"

    older = check.key_of(
        {"build": dict(build, libc_version="2.28")}, "L1", "rk4"
    )
    assert row_for(older) is None, "nothing was ever claimed about glibc 2.28"

    # An id with no version stands alone rather than gaining a trailing dash:
    # the Windows UCRT and musl offer no version to ask for, and an invented
    # one would be worse than an absent one.
    assert check.libc_column(dict(build, libc_id="musl", libc_version="")) == "musl"

    # A manifest written before the field existed is not silently keyed as
    # though it came from this machine.
    assert check.libc_column({"compiler_id": "GNU"}) == "unrecorded"


def test_the_c_library_version_is_asked_for_at_run_time() -> None:
    """A source check, because no behavioural test on one machine can do it.

    Replacing `gnu_get_libc_version()` with `__GLIBC__.__GLIBC_MINOR__` is a
    mutation that survives the whole suite: the headers a build compiles
    against and the library it runs against are the same here, so both spell
    2.39 and every assertion still passes. It is wrong anyway, and wrong in
    precisely the case the column exists for, which is a wheel built against
    one glibc and installed against another. The macros would record the
    build machine and the field would be a confident lie.

    So this asserts the shape of the source rather than its behaviour. It is
    the honest way to pin something a single machine cannot observe.
    """
    source = REPO_ROOT / "src" / "orchestration" / "slipx_sim" / "src" / "libc_identity.cpp"
    if not source.exists():
        pytest.skip(f"{source} not present; not running from a checkout")
    # Comments stripped: the file's own comment names the macro in order to
    # say it is not used, and a check that cannot tell those two apart would
    # be a check that fires on its own documentation.
    code = "\n".join(
        line.split("//", 1)[0] for line in source.read_text(encoding="utf-8").splitlines()
    )

    assert "gnu_get_libc_version()" in code
    # __GLIBC__ itself is fine and necessary: it is how the platform is
    # detected. It is the MINOR that would be a version.
    assert "__GLIBC_MINOR__" not in code


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
