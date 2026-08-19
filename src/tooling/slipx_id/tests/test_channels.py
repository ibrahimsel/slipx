# Copyright 2026 The SlipX Authors
# SPDX-License-Identifier: Apache-2.0

"""Channels and the reconstruction, against hand-derivable answers."""

from __future__ import annotations

import math

import pytest

from slipx_id.channels import Channel, unwrap_angles, wrap_angle
from slipx_id.reconstruct import (
    Bench,
    GRAVITY,
    SteadySample,
    body_kinematics,
    wheel_loads,
    wheel_slip_angles,
)


def _bench() -> Bench:
    return Bench(
        mass=3.5,
        lf=0.16,
        lr=0.16,
        h_cog=0.06,
        track_front=0.24,
        track_rear=0.24,
        wheel_radius=0.05,
        izz=0.05,
    )


class TestChannel:
    def test_interpolates_between_samples(self) -> None:
        ch = Channel((0.0, 1.0, 2.0), (0.0, 10.0, 30.0))
        assert ch.value_at(0.5) == pytest.approx(5.0)
        assert ch.value_at(1.5) == pytest.approx(20.0)
        assert ch.value_at(1.0) == 10.0

    def test_refuses_to_extrapolate(self) -> None:
        ch = Channel((0.0, 1.0), (0.0, 1.0))
        with pytest.raises(ValueError, match="outside"):
            ch.value_at(1.5)

    def test_value_before_is_a_hold_not_a_ramp(self) -> None:
        # A command step must stay a step: just before the edge the old
        # value, at and after the edge the new one.
        ch = Channel((0.0, 1.0, 2.0), (0.0, 1.0, 1.0))
        assert ch.value_before(0.999) == 0.0
        assert ch.value_before(1.0) == 1.0
        assert ch.value_before(5.0) == 1.0  # held past the last sample

    def test_derivative_of_a_parabola(self) -> None:
        times = tuple(0.1 * i for i in range(11))
        ch = Channel(times, tuple(t * t for t in times))
        d = ch.derivative()
        # Central differences are exact for a parabola.
        for t, v in zip(d.times, d.values):
            assert v == pytest.approx(2.0 * t, abs=1e-12)

    def test_refuses_non_increasing_timestamps(self) -> None:
        with pytest.raises(ValueError, match="increase strictly"):
            Channel((0.0, 1.0, 1.0), (0.0, 1.0, 2.0))

    def test_angle_unwrap_and_wrap(self) -> None:
        wrapped = [3.1, -3.1, 3.05]  # crossing pi twice
        unwrapped = unwrap_angles(wrapped)
        for a, b in zip(unwrapped, unwrapped[1:]):
            assert abs(b - a) < math.pi
        assert wrap_angle(3.0 * math.pi) == pytest.approx(math.pi)
        assert wrap_angle(-2.5 * math.pi) == pytest.approx(-0.5 * math.pi)


class TestWheelLoads:
    def test_static_split_is_the_textbook_one(self) -> None:
        bench = _bench()
        fl, fr, rl, rr = wheel_loads(bench, 0.0, 0.0)
        assert fl == fr == rl == rr == pytest.approx(3.5 * GRAVITY / 4.0)

    def test_a_left_turn_loads_the_right_wheels(self) -> None:
        # ISO 8855: positive ay is a left turn and the outer wheels are on
        # the right. The classic sign trap, held here as it is in the core.
        fl, fr, rl, rr = wheel_loads(_bench(), 0.0, 5.0)
        assert fr > fl and rr > rl

    def test_braking_loads_the_front(self) -> None:
        fl, fr, rl, rr = wheel_loads(_bench(), -6.0, 0.0)
        assert fl + fr > rl + rr

    def test_load_is_conserved_even_at_wheel_lift(self) -> None:
        bench = _bench()
        for ax, ay in ((0.0, 0.0), (5.0, 3.0), (-15.0, 0.0), (0.0, 25.0)):
            loads = wheel_loads(bench, ax, ay)
            assert sum(loads) == pytest.approx(bench.weight, rel=1e-12)
            assert all(load >= 0.0 for load in loads)


class TestKinematics:
    def test_a_perfect_circle_reconstructs_its_own_motion(self) -> None:
        # A car on a 3 m circle at 2 m/s with 5 degrees of sideslip: the
        # pose is written down analytically and the reconstruction must
        # recover speed, course and sideslip.
        radius, speed, beta = 3.0, 2.0, math.radians(5.0)
        omega = speed / radius
        times = tuple(0.01 * i for i in range(400))
        xs, ys, yaws = [], [], []
        for t in times:
            angle = omega * t
            xs.append(radius * math.cos(angle))
            ys.append(radius * math.sin(angle))
            # Course leads the position angle by 90 degrees; the heading
            # trails the course by the sideslip.
            yaws.append(wrap_angle(angle + 0.5 * math.pi - beta))
        kin = body_kinematics(
            Channel(times, tuple(xs)),
            Channel(times, tuple(ys)),
            Channel(times, tuple(yaws)),
        )
        mid = len(kin.speed) // 2
        assert kin.speed.values[mid] == pytest.approx(speed, rel=1e-4)
        assert kin.sideslip.values[mid] == pytest.approx(beta, rel=1e-3)
        assert kin.vx.values[mid] == pytest.approx(
            speed * math.cos(beta), rel=1e-3
        )
        assert kin.vy.values[mid] == pytest.approx(
            speed * math.sin(beta), rel=1e-2
        )

    def test_slip_angles_follow_the_double_track_geometry(self) -> None:
        # Hand-computed from the wheel-centre velocities: vx = 3, vy = 0.1,
        # r = 0.9, steer = 0.1 on the bench above.
        bench = _bench()
        sample = SteadySample(
            time=0.0,
            vx=3.0,
            vy=0.1,
            yaw_rate=0.9,
            steer=0.1,
            ax=0.0,
            ay=2.7,
            bench=bench,
        )
        fl, fr, rl, rr = wheel_slip_angles(sample)
        # Front left: wheel at (0.16, 0.12): vxw = 3 - 0.9*0.12 = 2.892,
        # vyw = 0.1 + 0.9*0.16 = 0.244; alpha = atan2(0.244, 2.892) - 0.1.
        assert fl == pytest.approx(math.atan2(0.244, 2.892) - 0.1, abs=1e-12)
        # Rear right: wheel at (-0.16, -0.12): vxw = 3.108, vyw = -0.044.
        assert rr == pytest.approx(math.atan2(-0.044, 3.108), abs=1e-12)
