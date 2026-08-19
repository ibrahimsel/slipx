# Copyright 2026 The SlipX Authors
# SPDX-License-Identifier: Apache-2.0

"""The manoeuvre library, driven in simulation (ADR-0038, M6.2).

Generates the recordings the fitter consumes, by driving the forward model
through the procedures of ``docs/identification/`` and keeping only what a
real car would have recorded: the localisation pose, the IMU, the wheel
encoders and the commands. Ground-truth body velocities, tyre forces and
slip angles exist in the simulation and are deliberately not extracted,
because a fitter tested against signals no car can log is a fitter that has
never been tested.

This module is the synthetic half of the self-test and is also the honest
statement of what each manoeuvre records; the bag reader that replaces it
for real data produces the same :class:`ManoeuvreRecording` shape.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Callable, Dict, Mapping, Optional, Tuple

import slipx
from slipx import sinks

from .channels import Channel
from .reconstruct import Bench

#: The sensor-level sampling of a synthetic recording: pose, IMU and encoder
#: channels are kept every this-many simulation steps (100 Hz at the 1 kHz
#: default), which is the order of a real localisation rate. Commands are
#: kept at every step, because the step-steer fit measures lags of tens of
#: milliseconds and a command edge blurred by resampling would poison it.
SENSOR_STRIDE = 10


@dataclass(frozen=True)
class ManoeuvreRecording:
    """What one run of one manoeuvre leaves behind.

    ``channels`` carries exactly the bag-available signals:

    - ``pose.x``, ``pose.y``, ``pose.yaw``: the localisation pose [m, rad]
    - ``imu.ax``, ``imu.ay``: specific force at the CoG [m/s^2]
    - ``imu.yaw_rate``: [rad/s]
    - ``wheel.FL``, ``wheel.FR``, ``wheel.RL``, ``wheel.RR``: encoder wheel
      speeds [rad/s]
    - ``cmd.steer`` [rad], ``cmd.accel`` [m/s^2]: the commands, at full rate
    """

    name: str
    dt: float
    bench: Bench
    channels: Mapping[str, Channel]

    def channel(self, name: str) -> Channel:
        try:
            return self.channels[name]
        except KeyError:
            raise KeyError(
                f"recording '{self.name}' has no channel '{name}'; it has "
                f"{sorted(self.channels)}"
            ) from None


def bench_of(params: "slipx.VehicleParams") -> Bench:
    """The bench constants a real team would have measured, read off the
    parameter set the synthetic car was built from."""
    return Bench(
        mass=params.mass,
        lf=params.lf,
        lr=params.lr,
        h_cog=params.h_cog,
        track_front=params.track_front,
        track_rear=params.track_rear,
        wheel_radius=params.wheel_radius,
        izz=params.izz,
    )


def ballasted(params: "slipx.VehicleParams", extra_mass: float) -> "slipx.VehicleParams":
    """The same car with ballast fixed over the CoG.

    The mass rises; the tyre is the same physical object, so its
    coefficients are restated at the new static load by the ADR-0039 power
    laws, exactly as the loader would restate the same tyre file under the
    heavier car. Restating is what makes the ballast manoeuvre identify
    ``k_mu``: a synthetic ballast run that instead re-derived the tyre at
    the new load would, by construction, show no ballast effect at all.
    """
    if extra_mass <= 0.0:
        raise ValueError("ballast must add mass")
    out = params.copy()
    ratio = (params.mass + extra_mass) / params.mass
    out.mass = params.mass + extra_mass
    for tyre in (out.tyre_front, out.tyre_rear):
        k = tyre.k_mu
        tyre.mu_y0 *= ratio**-k
        tyre.mu_x0 *= ratio**-k
    out.c_alpha_f *= ratio ** (1.0 - out.tyre_front.k_mu)
    out.c_alpha_r *= ratio ** (1.0 - out.tyre_rear.k_mu)
    out.c_kappa *= ratio ** (1.0 - out.tyre_front.k_mu)
    return out


def record_manoeuvre(
    name: str,
    params: "slipx.VehicleParams",
    policy: Callable,
    duration: float,
    *,
    initial_speed: float = 0.0,
    dt: float = 1.0e-3,
    seed: int = 20260819,
) -> ManoeuvreRecording:
    """Drive one manoeuvre and keep what a car would have recorded."""
    config = slipx.SimulationConfig()
    config.dt = dt
    config.master_seed = seed
    sim = slipx.Simulation(config)

    agent = slipx.AgentSpec()
    agent.name = name
    agent.tier = slipx.Tier.L2_DoubleTrack
    agent.params = params
    agent.initial_state.vel_body.x = initial_speed
    if initial_speed > 0.0:
        # Wheels rolling to match, so the first step does not read a launch.
        # Assigned as a whole list: the binding converts the array by value,
        # so indexing into it would mutate a temporary and be lost.
        rolling = initial_speed / params.wheel_radius
        agent.initial_state.omega_w = [rolling] * 4
    agent.policy = policy
    sim.add_agent(agent)

    sim.set_input_logging(True)
    recording = sinks.record_run(sim, duration=duration, stride=SENSOR_STRIDE)
    agent_record = recording.agents[0]

    times = recording.times
    state = agent_record.state
    diagnostics = agent_record.diagnostics

    def channel(column_values) -> Channel:
        return Channel(tuple(times), tuple(column_values))

    channels: Dict[str, Channel] = {
        "pose.x": channel(state["pos.x"]),
        "pose.y": channel(state["pos.y"]),
        "pose.yaw": channel(state["yaw"]),
        "imu.ax": channel(diagnostics["ax"]),
        "imu.ay": channel(diagnostics["ay"]),
        "imu.yaw_rate": channel(state["rates.z"]),
        "wheel.FL": channel(state["omega_w.FL"]),
        "wheel.FR": channel(state["omega_w.FR"]),
        "wheel.RL": channel(state["omega_w.RL"]),
        "wheel.RR": channel(state["omega_w.RR"]),
    }

    # Commands at full rate from the input log: entry k was applied during
    # step k, so it is stamped at k*dt, the instant it was issued.
    log = sim.input_log()
    command_times = tuple(k * dt for k in range(len(log)))
    channels["cmd.steer"] = Channel(
        command_times, tuple(entry.steer_cmd for entry in log)
    )
    channels["cmd.accel"] = Channel(
        command_times, tuple(entry.accel_cmd for entry in log)
    )

    return ManoeuvreRecording(
        name=name, dt=dt, bench=bench_of(params), channels=channels
    )


# ------------------------------------------------------------- the library


def coastdown(
    params: "slipx.VehicleParams", entry_speed: float, duration: float = 8.0
) -> ManoeuvreRecording:
    """Zero drive torque from an entry speed, steering centred."""

    def policy(state, time, rng):
        return slipx.DriveInput(steer_cmd=0.0, accel_cmd=0.0)

    return record_manoeuvre(
        f"coastdown_{entry_speed:g}",
        params,
        policy,
        duration,
        initial_speed=entry_speed,
    )


def launch(
    params: "slipx.VehicleParams", duration: float = 6.0
) -> ManoeuvreRecording:
    """Full throttle from rest, straight ahead."""
    full = params.accel_max

    def policy(state, time, rng):
        return slipx.DriveInput(steer_cmd=0.0, accel_cmd=full)

    return record_manoeuvre("launch", params, policy, duration)


def skidpad(
    params: "slipx.VehicleParams",
    speed: float,
    steer: float,
    duration: float = 6.0,
) -> ManoeuvreRecording:
    """A held circle: constant steer, speed held closed loop.

    The radius is whatever the car settles onto; the fit reads the pose and
    does not need to know the circle that was intended, which is also true
    in a car park.
    """

    def policy(state, time, rng):
        return slipx.DriveInput(
            steer_cmd=steer, accel_cmd=slipx.hold_speed(state, speed)
        )

    return record_manoeuvre(
        f"skidpad_{speed:g}_{steer:g}",
        params,
        policy,
        duration,
        initial_speed=speed,
    )


def ramp_steer(
    params: "slipx.VehicleParams",
    speed: float,
    rate: float = 0.01,
    peak: float = 0.35,
) -> ManoeuvreRecording:
    """Constant speed, steering wound slowly in and then back out.

    Triangular rather than one-way, as the procedure doc instructs ("unwind
    gently"): the tyre's lateral force lags its slip angle by the relaxation
    length, so a one-way sweep reads the whole curve shifted by the lag, and
    the shift does not cancel between a left and a right sweep because both
    walk outward. The inbound and outbound branches of a triangle carry the
    lag in opposite directions along the curve, so together they straddle
    the true one, which is also why the doc says the two branches lying on
    top of each other is the health check.
    """
    half = abs(peak / rate)
    sign = 1.0 if rate >= 0.0 else -1.0
    magnitude = abs(rate)

    def policy(state, time, rng):
        if time <= half:
            steer = sign * magnitude * time
        else:
            steer = sign * magnitude * (2.0 * half - time)
        return slipx.DriveInput(
            steer_cmd=steer, accel_cmd=slipx.hold_speed(state, speed)
        )

    return record_manoeuvre(
        f"ramp_{speed:g}_{rate:g}",
        params,
        policy,
        2.0 * half,
        initial_speed=speed,
    )


def slalom(
    params: "slipx.VehicleParams",
    speed: float,
    amplitude: float = 0.15,
    frequency: float = 0.5,
    duration: float = 8.0,
) -> ManoeuvreRecording:
    """Sinusoidal steering at held speed: the validation manoeuvre.

    Not one of the six identification manoeuvres, on purpose: a validation
    run should exercise the model with an input the fit never saw, and a
    slalom sweeps through the transients and both signs of the curve at
    once.
    """

    def policy(state, time, rng):
        steer = amplitude * math.sin(2.0 * math.pi * frequency * time)
        return slipx.DriveInput(
            steer_cmd=steer, accel_cmd=slipx.hold_speed(state, speed)
        )

    return record_manoeuvre(
        f"slalom_{speed:g}",
        params,
        policy,
        duration,
        initial_speed=speed,
    )


def step_steer(
    params: "slipx.VehicleParams",
    speed: float,
    amplitude: float,
    step_time: float = 1.0,
    duration: float = 3.0,
) -> ManoeuvreRecording:
    """Straight running, then a true step to a fixed steer command."""

    def policy(state, time, rng):
        steer = amplitude if time >= step_time else 0.0
        return slipx.DriveInput(
            steer_cmd=steer, accel_cmd=slipx.hold_speed(state, speed)
        )

    return record_manoeuvre(
        f"step_{speed:g}_{amplitude:g}",
        params,
        policy,
        duration,
        initial_speed=speed,
    )
