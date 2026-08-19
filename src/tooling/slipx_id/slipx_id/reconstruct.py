# Copyright 2026 The SlipX Authors
# SPDX-License-Identifier: Apache-2.0

"""From bag-level signals to the quantities the fits consume.

Everything here works from what a car actually records: a localisation pose,
an IMU, wheel encoders and the commands. Body velocities, sideslip, per-wheel
slip angles and per-wheel vertical loads are reconstructed, not read, and
the reconstruction states its assumptions where it makes them.

The load formulae mirror ``load_transfer.hpp``, which documents them as the
public quasi-static law; the slip geometry mirrors the double-track
kinematics (wheel-centre velocity from body velocity and yaw rate). Both are
textbook rigid-body statements rather than model internals, and the
synthetic round-trip test holds this module to the model's own numbers.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List, Mapping, Sequence, Tuple

from .channels import Channel, unwrap_angles, wrap_angle

#: Standard gravity, as the core's kGravity (math.hpp).
GRAVITY = 9.80665


@dataclass(frozen=True)
class Bench:
    """The bench-measured constants of one run's car (docs/identification).

    These are `measured` inputs to every fit, never outputs: a fitter that
    adjusts the mass to improve a tyre fit has stopped identifying anything.
    """

    mass: float  # [kg]
    lf: float  # [m]
    lr: float  # [m]
    h_cog: float  # [m]
    track_front: float  # [m]
    track_rear: float  # [m]
    wheel_radius: float  # [m]
    izz: float  # [kg m^2], from the bifilar pendulum

    @property
    def wheelbase(self) -> float:
        return self.lf + self.lr

    @property
    def weight(self) -> float:
        return self.mass * GRAVITY

    @property
    def static_front_per_tyre(self) -> float:
        return 0.5 * self.weight * self.lr / self.wheelbase

    @property
    def static_rear_per_tyre(self) -> float:
        return 0.5 * self.weight * self.lf / self.wheelbase


def wheel_loads(
    bench: Bench, ax: float, ay: float
) -> Tuple[float, float, float, float]:
    """Quasi-static per-wheel loads (FL, FR, RL, RR) under specific forces.

    Mirrors ``quasi_static_loads`` in load_transfer.hpp, including the
    redistribution rule at wheel lift: an axle or a wheel clamped to zero
    hands its load to the other side, so the four always sum to the weight.
    """
    weight = bench.weight
    static_front = weight * bench.lr / bench.wheelbase
    static_rear = weight * bench.lf / bench.wheelbase
    dfz_long = bench.mass * ax * bench.h_cog / bench.wheelbase

    fz_front = static_front - dfz_long
    fz_rear = static_rear + dfz_long
    if fz_front < 0.0:
        fz_front, fz_rear = 0.0, weight
    elif fz_rear < 0.0:
        fz_front, fz_rear = weight, 0.0

    mass_front = bench.mass * bench.lr / bench.wheelbase
    mass_rear = bench.mass * bench.lf / bench.wheelbase
    dfz_lat_front = mass_front * ay * bench.h_cog / bench.track_front
    dfz_lat_rear = mass_rear * ay * bench.h_cog / bench.track_rear

    fl = 0.5 * fz_front - dfz_lat_front
    fr = 0.5 * fz_front + dfz_lat_front
    rl = 0.5 * fz_rear - dfz_lat_rear
    rr = 0.5 * fz_rear + dfz_lat_rear

    if fl < 0.0:
        fl, fr = 0.0, fz_front
    elif fr < 0.0:
        fl, fr = fz_front, 0.0
    if rl < 0.0:
        rl, rr = 0.0, fz_rear
    elif rr < 0.0:
        rl, rr = fz_rear, 0.0

    return fl, fr, rl, rr


@dataclass(frozen=True)
class BodyKinematics:
    """Body-frame velocity and sideslip, reconstructed from the pose."""

    speed: Channel
    course: Channel  # world-frame direction of travel        [rad]
    sideslip: Channel  # course minus heading, wrapped        [rad]
    vx: Channel
    vy: Channel


def body_kinematics(
    pose_x: Channel, pose_y: Channel, yaw: Channel
) -> BodyKinematics:
    """Differentiate the pose into speed, course and sideslip.

    Central differences, so the pose rate bounds the bandwidth: at 100 Hz on
    a 1/10-scale car the truncation error is parts in 10^5, far below any
    localiser's noise floor. The three channels must share timestamps; a
    real bag is resampled onto the pose's stamps before it gets here.
    """
    if pose_x.times != pose_y.times or pose_x.times != yaw.times:
        raise ValueError("pose channels must share timestamps")

    vx_world = pose_x.derivative()
    vy_world = pose_y.derivative()
    times = vx_world.times

    speeds = []
    courses = []
    for wx, wy in zip(vx_world.values, vy_world.values):
        speeds.append(math.hypot(wx, wy))
        courses.append(math.atan2(wy, wx))

    unwrapped_course = unwrap_angles(tuple(courses))
    betas = []
    vxs = []
    vys = []
    for t, speed, course in zip(times, speeds, unwrapped_course):
        beta = wrap_angle(course - yaw.value_at(t))
        betas.append(beta)
        vxs.append(speed * math.cos(beta))
        vys.append(speed * math.sin(beta))

    return BodyKinematics(
        speed=Channel(times, tuple(speeds)),
        course=Channel(times, tuple(unwrapped_course)),
        sideslip=Channel(times, tuple(betas)),
        vx=Channel(times, tuple(vxs)),
        vy=Channel(times, tuple(vys)),
    )


@dataclass(frozen=True)
class SteadySample:
    """One steady-state observation, everything a lateral residual needs."""

    time: float
    vx: float
    vy: float
    yaw_rate: float
    steer: float  # commanded; equal to achieved once the servo settles
    ax: float
    ay: float
    bench: Bench


# Wheel positions in the body frame, in the fixed FL, FR, RL, RR order:
# x forward of the CoG, y to the left.
def _wheel_positions(bench: Bench) -> Tuple[Tuple[float, float], ...]:
    return (
        (bench.lf, 0.5 * bench.track_front),
        (bench.lf, -0.5 * bench.track_front),
        (-bench.lr, 0.5 * bench.track_rear),
        (-bench.lr, -0.5 * bench.track_rear),
    )


def wheel_slip_angles(sample: SteadySample) -> Tuple[float, float, float, float]:
    """Per-wheel geometric slip angles from the reconstructed kinematics.

    The double-track statement: each wheel centre moves with the body plus
    the yaw rate crossed with its position, and its slip angle is the angle
    of that velocity in the wheel's own frame. Steering applies to the front
    wheels; the servo is assumed settled, which holds in every steady window
    the stages select (a 0.02 rad/s ramp lags the 45 rad/s servo by under a
    milliradian).
    """
    out: List[float] = []
    for index, (xw, yw) in enumerate(_wheel_positions(sample.bench)):
        vxw = sample.vx - sample.yaw_rate * yw
        vyw = sample.vy + sample.yaw_rate * xw
        steer = sample.steer if index < 2 else 0.0
        out.append(math.atan2(vyw, vxw) - steer)
    return out[0], out[1], out[2], out[3]


def wheel_slip_ratios(
    omega: Sequence[float],
    wheel_radius: float,
    vx: float,
    yaw_rate: float,
    bench: Bench,
) -> Tuple[float, float, float, float]:
    """Per-wheel slip ratios from encoder speeds against reconstructed motion.

    kappa = (omega R - v_wheel) / v_wheel, with each wheel's own centre
    velocity in the denominator, so a circling car does not read its inner
    and outer wheels as slipping against a single body speed.
    """
    out: List[float] = []
    for (xw, yw), w in zip(_wheel_positions(bench), omega):
        vxw = vx - yaw_rate * yw
        if abs(vxw) < 0.05:
            raise ValueError(
                "slip ratio is not defined this close to standstill; gate "
                "the samples before asking"
            )
        out.append((w * wheel_radius - vxw) / vxw)
    return out[0], out[1], out[2], out[3]
