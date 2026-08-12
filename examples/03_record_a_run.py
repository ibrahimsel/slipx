#!/usr/bin/env python3
# Copyright 2026 The SlipX Authors
# SPDX-License-Identifier: Apache-2.0

"""Run a car, record it, and write a file you can look at.

SlipX writes files. It never opens a window and never launches a viewer:
scrubbing a timeline is a solved problem, and a simulator that insists on
being the thing you look at cannot run on a build machine.

A finished run is recorded once, into one format-neutral Recording, and handed
to a sink. This example uses the SVG sink, which needs nothing installed
beyond SlipX itself: it is the standard library and a string, and it writes one
self-contained animated document with the car's provenance label and the run's
trajectory hash drawn into the picture, so a plot pasted into a slide carries
its own label.

    pip install slipx              # this example
    pip install "slipx[mcap]"      # then format="mcap" also works
    pip install "slipx[rerun]"     # and format="rerun"

Run it:

    python3 examples/03_record_a_run.py [output_directory]
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import slipx
from slipx import sinks


def main() -> int:
    out_dir = Path(sys.argv[1] if len(sys.argv) > 1 else ".")
    out_dir.mkdir(parents=True, exist_ok=True)

    car = slipx.load_reference_car()

    config = slipx.SimulationConfig()
    config.master_seed = 20260813
    config.schema_version = car.spec.schema_version
    sim = slipx.Simulation(config)

    agent = slipx.AgentSpec()
    agent.name = car.name
    agent.tier = slipx.Tier.L2_DoubleTrack
    agent.params = car.params_for_tier(slipx.Tier.L2_DoubleTrack)
    agent.initial_state.vel_body.x = 5.0

    # A policy is a callable of (state, time, rng) and nothing else. Anything
    # it reads that is not one of those three, a wall clock or a global
    # counter, is a way for the run to stop replaying, so the orchestrator
    # hands it everything it is allowed to see.
    def slalom(state, time, rng):
        return slipx.DriveInput(
            steer_cmd=0.12 * math.sin(2.0 * math.pi * 0.5 * time),
            accel_cmd=slipx.hold_speed(state, 5.0),
        )

    agent.policy = slalom
    sim.add_agent(agent)

    # Recording does not perturb the run. The sim is stepped exactly as it
    # would be without a recorder attached, nothing is fed back, and the
    # trajectory hash is the same either way. `stride` decides how much is
    # kept, never how much is simulated.
    run = sinks.record_run(sim, duration=8.0, stride=10)

    print(run.provenance_line())
    print(f"recorded {len(run)} frames of {run.dt * run.stride:.3f} s")
    print(f"trajectory hash: {sim.trajectory_hash()}")

    path = sinks.write(run, out_dir / "slalom", format="svg")
    print(f"\nwrote {path}")
    print("open it in a browser: it animates on its own, in light or dark.")

    # What a sink is allowed to draw is exactly what the run contained. There
    # is no track in the picture, because nothing in SlipX has a track yet and
    # a drawn kerb would assert one. Quantities the tier could not compute are
    # absent rather than plotted as zero, which is why the same call at L0
    # produces a shorter document rather than one full of flat lines at the
    # origin.
    agent.tier = slipx.Tier.L0_Kinematic
    agent.params = car.params_for_tier(slipx.Tier.L0_Kinematic)
    kinematic = slipx.Simulation(config)
    kinematic.add_agent(agent)
    l0_run = sinks.record_run(kinematic, duration=8.0, stride=10)
    l0_path = sinks.write(l0_run, out_dir / "slalom_l0", format="svg")

    l2_bytes = Path(path).stat().st_size
    l0_bytes = Path(l0_path).stat().st_size
    print(f"\nthe same manoeuvre at L0: {l0_path}")
    print(f"  L2 document {l2_bytes:6d} bytes")
    print(f"  L0 document {l0_bytes:6d} bytes, because a kinematic car has no")
    print("  tyre forces, no wheel loads and no battery to draw. Those panels")
    print("  are missing, not empty.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
