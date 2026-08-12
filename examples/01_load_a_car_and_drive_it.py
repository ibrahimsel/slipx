#!/usr/bin/env python3
# Copyright 2026 The SlipX Authors
# SPDX-License-Identifier: Apache-2.0

"""Load a car directory, drive a step steer, read the result.

The shortest complete thing SlipX does. Everything else in this directory is
this example with one idea added.

Run it:

    pip install slipx
    python3 examples/01_load_a_car_and_drive_it.py

No extras are needed. The reference car ships inside the package, so this runs
from an installed wheel with no checkout present.
"""

from __future__ import annotations

import slipx


def main() -> int:
    # The car is a directory of YAML files, not a Python object literal. It is
    # loaded, validated and labelled, and the label is printed rather than
    # documented: these numbers are provisional and no plot made from them may
    # imply otherwise.
    car = slipx.load_reference_car()
    print(car.summary())

    # A tier is chosen, never inferred. L2 is the double-track model: four
    # contact patches, load transfer, MF-lite tyres, a differential, an ESC, a
    # battery and a steering servo. Asking for a tier the build does not
    # implement raises rather than quietly giving you a simpler one.
    params = car.params_for_tier(slipx.Tier.L2_DoubleTrack)
    model = slipx.VehicleModel.create(slipx.Tier.L2_DoubleTrack, params)

    state = slipx.VehicleState()
    state.vel_body.x = 5.0  # m/s. A standing start would divide by zero.

    diagnostics = slipx.StepDiagnostics()
    dt = 1.0e-3  # fixed, always. A variable step is not reproducible.

    # Half a second straight, then half a second at 0.1 rad of steering.
    for step in range(1000):
        steer = 0.0 if step < 500 else 0.1
        command = slipx.DriveInput(
            steer_cmd=steer,
            accel_cmd=slipx.hold_speed(state, 5.0),
        )
        model.step(state, command, dt, diagnostics)

    print(f"\nafter {1000 * dt:.1f} s of a {0.1:.1f} rad step steer at 5 m/s:")
    print(f"  position       x={state.pos.x:7.3f} m   y={state.pos.y:7.3f} m")
    print(f"  heading        {state.yaw:7.3f} rad")
    print(f"  yaw rate       {state.yaw_rate:7.3f} rad/s")
    print(f"  body slip      {state.sideslip():7.3f} rad")
    print(f"  lateral accel  {diagnostics.ay:7.3f} m/s^2")

    # The command is what was asked for; the state is what the car achieved.
    # From L2 they differ, because the steering servo has a slew limit and a
    # second-order lag, and that gap is the point of modelling it.
    print(f"\n  commanded steer {0.1:.4f} rad, achieved {state.steer:.4f} rad")

    # Per-wheel numbers exist at L2 and are NaN below it, never zero: a zero
    # slip angle is a number somebody would plot and believe.
    print("\n  per-wheel slip angle [rad] and vertical load [N]:")
    for wheel, name in enumerate(("front left", "front right",
                                  "rear left", "rear right")):
        print(f"    {name:<12} alpha={diagnostics.alpha[wheel]:7.4f}"
              f"  Fz={diagnostics.fz[wheel]:6.2f}")

    # Longitudinal load transfer under steady speed is nil; the lateral
    # transfer is what leans the car on its outside tyres.
    print(f"\n  lateral load transfer   {diagnostics.load_transfer_lat:6.2f} N")
    print(f"  longitudinal transfer   {diagnostics.load_transfer_long:6.2f} N")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
