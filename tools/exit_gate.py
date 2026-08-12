#!/usr/bin/env python3
# Copyright 2026 The SlipX Authors
# SPDX-License-Identifier: Apache-2.0

"""The P0 exit gate, run as one command.

The gate is written in the SRS as a sentence about a person:

    a third party can `pip install` the package, load a car directory,
    integrate a step-steer manoeuvre and get the same trajectory hash as CI.

This script is that sentence, executable. It is deliberately the only check in
the repository that touches nothing in the repository: it imports the installed
`slipx`, loads the car that shipped inside it, and compares against a hash
passed in on the command line. Run it from a directory that is not a SlipX
checkout, or it proves less than it appears to, because Python puts the working
directory on sys.path and the checkout would satisfy the imports on its own.

    python3 tools/exit_gate.py --expect 0d8f69a1e3b58038

The hash is an argument rather than something read from
conformance/reference_hashes.tsv, because the point is to check an
independently built artefact against a published number. Reading the number out
of the tree the artefact was built from would make the comparison circular.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument(
        "--expect",
        help="the published trajectory hash for the canonical L1/rk4 run. "
             "Omit to print what this install produces without judging it.",
    )
    args = parser.parse_args()

    try:
        import slipx
    except ImportError as exc:
        print(f"slipx is not installed: {exc}", file=sys.stderr)
        return 2

    package_dir = Path(slipx.__file__).resolve().parent
    print(f"slipx {slipx.__version__} from {package_dir}")
    print(f"core  {slipx.core_version}")

    if Path.cwd() == package_dir.parent or (Path.cwd() / "pyproject.toml").exists():
        print(
            "\nWARNING: running from what looks like a source checkout. The "
            "imports above may be satisfied by the working directory rather "
            "than by the install, which is the thing being tested.",
            file=sys.stderr,
        )

    # Clause two: load a car directory. The reference car travels in the wheel,
    # so this reaches nothing outside the install.
    car = slipx.load_reference_car()
    print(f"\nloaded car directory: {slipx.reference_car_path()}")
    # summary() already carries the provenance label, the notes and the
    # warnings (NFR-08), so there is nothing to print alongside it.
    print(car.summary())

    # Clause three: integrate a step steer. This one uses the loaded car's
    # parameters, which the canonical conformance run does not: that run is
    # pinned to the VehicleParams defaults so its hash cannot move when a car
    # file is edited. Both are run here, because the gate names both loading a
    # car and reproducing the published hash, and one artefact cannot do both.
    config = slipx.SimulationConfig()
    spec = slipx.AgentSpec()
    spec.name = car.name
    spec.tier = slipx.Tier.L1_Bicycle
    spec.params = car.params
    spec.initial_state = slipx.VehicleState()
    spec.initial_state.vel_body.x = 5.0
    spec.policy = slipx.step_steer()

    sim = slipx.Simulation(config)
    sim.add_agent(spec)
    sim.run_for(5.0)
    state = sim.state(0)
    print(
        f"\nstep steer with the loaded car: "
        f"x={state.pos.x:.9f} y={state.pos.y:.9f} yaw={state.yaw:.9f}"
    )
    print(f"  trajectory hash: {sim.trajectory_hash()}")

    # Clause four: the canonical run, whose hash is the published one.
    canonical = slipx.make_conformance_run()
    canonical.run_for(5.0)
    digest = canonical.trajectory_hash()
    manifest = canonical.manifest()
    print(f"\ncanonical conformance run: {digest}")
    # The build this hash belongs to, printed whether or not anything is
    # compared. The C library is on that line because it is libm the hash
    # tracks: this same wheel produces a different hash on a different glibc,
    # and that is the first thing to check before calling a hash wrong.
    libc = manifest.libc_id + (
        f" {manifest.libc_version}" if manifest.libc_version else ""
    )
    print(
        f"  build: {manifest.system_processor} {manifest.compiler_id} "
        f"{manifest.compiler_version} {manifest.build_type}, {libc}"
    )

    if args.expect is None:
        print("\nno --expect given, so nothing was compared.")
        return 0

    if digest != args.expect:
        print(
            f"\nEXIT GATE FAILED: expected {args.expect}, got {digest}.\n\n"
            "If this is an architecture, compiler or C library with no "
            "published reference row, that is documented behaviour and not a "
            "bug (NFR-03): cross-platform bit-identity is not promised. Check "
            "conformance/reference_hashes.tsv for a row matching the build "
            "printed above, C library included, before treating it as a "
            f"determinism failure. This run was on {libc}.",
            file=sys.stderr,
        )
        return 1

    print("\nP0 exit gate met: installed wheel, car loaded, hash matches.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
