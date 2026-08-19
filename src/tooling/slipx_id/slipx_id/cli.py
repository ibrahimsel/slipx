# Copyright 2026 The SlipX Authors
# SPDX-License-Identifier: Apache-2.0

"""``slipx-id``: bags in, a car directory out.

One command from a session file to a loadable, provenance-carrying
parameter set, which is the shape the registry contribution flow needs
(a submission should be a by-product of fitting, not a separate act).
"""

from __future__ import annotations

import argparse
import sys
from typing import Optional, Sequence

from .emit import check_provenance, emit_car_directory
from .session import load_session, run_session


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        prog="slipx-id",
        description=(
            "Fit a SlipX parameter set from rosbag2 recordings of the "
            "manoeuvre library (docs/identification) and emit it as a car "
            "directory with residuals and provenance."
        ),
    )
    parser.add_argument("session", help="the session YAML file")
    parser.add_argument(
        "--sample-stride",
        type=int,
        default=2,
        help="keep every nth steady sample in the lateral fit (default 2)",
    )
    arguments = parser.parse_args(argv)

    try:
        session = load_session(arguments.session)
        # Refuse an unemittable session before the expensive part.
        check_provenance(session.provenance)
        outcome = run_session(session, sample_stride=arguments.sample_stride)
        emitted = emit_car_directory(outcome)
    except (ValueError, FileNotFoundError, FileExistsError) as failure:
        print(f"slipx-id: {failure}", file=sys.stderr)
        return 2

    print(f"IDENTIFIED: parameter set emitted to {emitted.directory}")
    for name, report in outcome.reports().items():
        print(f"\n[{name}]")
        print(report.summary())
    if outcome.mu_x0 is not None:
        value, spread = outcome.mu_x0
        print(f"\n[mu_x0]\n  mu_x0 = {value:.4g} (top-decile spread {spread:.2g})")
    for note in emitted.notes:
        print(f"note: {note}")
    for warning in emitted.warnings:
        print(f"warning: {warning}")
    return 0


if __name__ == "__main__":  # pragma: no cover
    sys.exit(main())
