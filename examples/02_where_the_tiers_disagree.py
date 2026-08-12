#!/usr/bin/env python3
# Copyright 2026 The SlipX Authors
# SPDX-License-Identifier: Apache-2.0

"""Drive the same manoeuvre at three fidelity tiers and measure the gap.

A fidelity tier is a choice about what you are willing to be wrong about. The
useful question is never "which tier is correct" but "up to what lateral
acceleration does the cheap one give the same answer as the expensive one",
and that is measurable rather than a matter of opinion.

This holds a steering angle until the transient has died, at a series of
angles, and compares the settled path radius the three tiers report. The
controller never changes: one state struct serves every tier, so the same code
points at L0, L1 and L2 without an edit. That is the whole reason the state is
one type.

Run it:

    pip install slipx
    python3 examples/02_where_the_tiers_disagree.py

Takes a few seconds: it settles 18 rollouts of six seconds each at 1 kHz.
"""

from __future__ import annotations

import slipx

SPEED = 5.0            # m/s, held by the controller throughout
SETTLE_STEPS = 6000    # six seconds at 1 kHz, many yaw time constants
STEERS = (0.01, 0.03, 0.06, 0.09, 0.12, 0.15)   # rad at the road wheel


def settled(params, tier, steer):
    """Hold a steer angle and a speed until nothing is changing any more.

    Returns the settled path radius [m] and the magnitude of the lateral
    acceleration [m/s^2]: the pair a skidpad measures, and therefore the pair
    two models should be compared on.
    """
    model = slipx.VehicleModel.create(tier, params)
    state = slipx.VehicleState()
    state.vel_body.x = SPEED
    diagnostics = slipx.StepDiagnostics()

    for _ in range(SETTLE_STEPS):
        command = slipx.DriveInput(steer, slipx.hold_speed(state, SPEED))
        model.step(state, command, 1.0e-3, diagnostics)

    return state.speed() / abs(state.yaw_rate), abs(diagnostics.ay)


def main() -> int:
    car = slipx.load_reference_car()

    # The reference car file runs a spool, a locked rear axle, because most
    # 1/10 competition cars do. A spool fights the car's turn-in with a drive
    # couple that a single-track model has no way to represent at all, and
    # that drivetrain disagreement would swamp the tyre effect this example is
    # about. So the comparison is made on an open differential, and the last
    # section shows what the spool does to it.
    params = car.params_for_tier(slipx.Tier.L2_DoubleTrack)
    params.differential = slipx.Differential.Open

    print(f"{car.name}: settled skidpad, {SPEED:.0f} m/s, open differential")
    print(f"{'steer':>7} {'ay':>8} {'L0 R':>8} {'L1 R':>8} {'L2 R':>8} "
          f"{'L1 vs L2':>9}")
    print(f"{'[rad]':>7} {'[m/s^2]':>8} {'[m]':>8} {'[m]':>8} {'[m]':>8} "
          f"{'':>9}")

    agreed_to = None    # the highest ay at which the two are still within 1%
    departed_at = None  # and the first sampled point where they are not
    for steer in STEERS:
        r0, _ = settled(params, slipx.Tier.L0_Kinematic, steer)
        r1, _ = settled(params, slipx.Tier.L1_Bicycle, steer)
        r2, ay = settled(params, slipx.Tier.L2_DoubleTrack, steer)
        error = abs(r1 - r2) / r2
        if error <= 0.01 and departed_at is None:
            agreed_to = ay
        elif departed_at is None:
            departed_at = (ay, error)
        print(f"{steer:7.3f} {ay:8.2f} {r0:8.2f} {r1:8.2f} {r2:8.2f} "
              f"{error * 100:8.1f}%")

    print()
    # L0 has no tyres, so its radius is pure geometry: wheelbase over the
    # steer angle, the same number at any speed and any grip level. It is not
    # converging on the others as the corner tightens; it never was answering
    # the same question.
    print("L0 is the kinematic bicycle: radius = wheelbase / steer, with no")
    print("tyre, no slip and no speed dependence. At 0.15 rad that is")
    print(f"{params.wheelbase / 0.15:.2f} m whatever the car is made of.")
    print()
    if departed_at is None:
        print("L1 and L2 agree within 1% across this whole range. The sweep")
        print("stops below the tyre's limit; push it further and they part.")
    else:
        ay, error = departed_at
        print(f"L1 and L2 agree within 1% up to at least "
              f"{agreed_to:.2f} m/s^2 ({agreed_to / 9.81:.2f} g),")
        print(f"and by {ay:.2f} m/s^2 ({ay / 9.81:.2f} g) they differ by "
              f"{error * 100:.1f}%. The sweep is")
        print("coarse, so the crossing lies between those two points.")
        print()
        print("The single-track model is the optimistic one: with one tyre")
        print("per axle it has no load transfer, and load transfer costs")
        print("grip, because a tyre's friction coefficient falls as its")
        print("vertical load rises. The pair of tyres on a real axle is")
        print("therefore worth less than the single tyre L1 replaces them")
        print("with, and the gap opens exactly where the tyres start to")
        print("matter.")

    # And the drivetrain, which is the other half of what a single-track model
    # cannot see. Same tyres, same geometry, one parameter changed.
    spool = car.params_for_tier(slipx.Tier.L2_DoubleTrack)
    assert spool.differential == slipx.Differential.Spool
    print()
    print("The same car with its shipped spool, at L2:")
    print(f"{'steer':>7} {'open R':>9} {'spool R':>9} {'difference':>11}")
    for steer in (0.03, 0.09, 0.15):
        r_open, _ = settled(params, slipx.Tier.L2_DoubleTrack, steer)
        r_spool, _ = settled(spool, slipx.Tier.L2_DoubleTrack, steer)
        print(f"{steer:7.3f} {r_open:9.2f} {r_spool:9.2f} "
              f"{(r_spool - r_open) / r_open * 100:10.1f}%")
    print()
    print("A locked axle forces both rear wheels to one speed, so the inner")
    print("one drags and the outer one pushes. That couple resists the turn")
    print("from the first degree of steering, which is why the gap does not")
    print("shrink as the corner opens out. No single-track model has a")
    print("differential to represent, at any lateral acceleration.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
