# Copyright 2026 The SlipX Authors
# SPDX-License-Identifier: Apache-2.0

"""``slipx-conformance``: run the canonical step steer and print its hash.

This is the P0 exit gate, in one command:

    pip install slipx
    slipx-conformance

A third party installs the package, integrates a step steer, and compares the
trajectory hash against the one CI published for the same build. That is the
whole test of whether the determinism claim is real, and it has to be runnable
by somebody who has never opened the repository.

What a mismatch means depends on the build block of the manifest. Within one
(platform, compiler, flag set) a mismatch is a bug (NFR-02). Across a different
compiler or architecture, it is the documented limitation: SlipX promises
bit-identity within a build and conformance to a stated tolerance across
architectures, and says so rather than glossing it (NFR-03).
"""

from __future__ import annotations

import argparse
import sys

from . import Integrator, Tier, make_conformance_run
from ._slipx import ConformanceSpec

_TIERS = {"L0": Tier.L0_Kinematic, "L1": Tier.L1_Bicycle}
_INTEGRATORS = {
    "rk4": Integrator.RK4,
    "semi_implicit_euler": Integrator.SemiImplicitEuler,
}


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        prog="slipx-conformance",
        description=__doc__.split("\n")[0],
    )
    parser.add_argument(
        "--tier",
        choices=sorted(_TIERS),
        default="L1",
        help="L2 and L3 are not implemented yet (CORE-02, P1)",
    )
    parser.add_argument(
        "--integrator", choices=sorted(_INTEGRATORS), default="rk4"
    )
    parser.add_argument(
        "--expect", default="", help="exit non-zero unless the hash matches"
    )
    parser.add_argument("--manifest", default="", help="write the manifest as JSON")
    parser.add_argument("--quiet", action="store_true")
    args = parser.parse_args(argv)

    spec = ConformanceSpec()
    spec.tier = _TIERS[args.tier]
    spec.integrator = _INTEGRATORS[args.integrator]

    sim = make_conformance_run(spec)
    sim.run_for(spec.duration)
    manifest = sim.manifest()

    if args.manifest and not manifest.write(args.manifest):
        print(f"slipx-conformance: could not write {args.manifest}", file=sys.stderr)
        return 3

    if not args.quiet:
        print(manifest.to_json())
        state = sim.state(0)
        print(
            f"\nfinal state: x={state.pos.x:.9f} y={state.pos.y:.9f} "
            f"yaw={state.yaw:.9f} vx={state.vx:.9f} vy={state.vy:.9f} "
            f"r={state.yaw_rate:.9f}"
        )

    if args.expect and args.expect != manifest.trajectory_hash:
        print(
            "\nslipx-conformance: DETERMINISM CHECK FAILED\n"
            f"  expected {args.expect}\n"
            f"  got      {manifest.trajectory_hash}\n"
            "Within one build this is a bug (NFR-02). Across builds, compilers "
            "or architectures it may be the documented limitation (NFR-03): "
            "compare the build block of the manifest before concluding "
            "anything.",
            file=sys.stderr,
        )
        return 1

    # Last line, alone, so a shell script can take it verbatim.
    print(manifest.trajectory_hash)
    return 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
