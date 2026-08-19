# Copyright 2026 The SlipX Authors
# SPDX-License-Identifier: Apache-2.0

"""The staged fits, in the manoeuvre library's order (ADR-0038).

Each stage consumes earlier results as constants and returns a
:class:`~slipx_id.optimise.FitReport`. The steady-state stages fit
algebraically reconstructed quantities; the transient stage replays the
recorded commands through the forward model, because a lag can only be
measured against the thing that lags.

Every residual here is an exact rigid-body statement, not a steady-state
approximation: the lateral force balance uses the measured specific force
(Newton's law holds in any state), and the yaw moment balance carries the
measured ``izz * r_dot`` term rather than assuming it away, with ``izz``
from the bench. What the steady gates actually protect is the two settled
assumptions: the tyre's lagged slip equals its geometric slip, and the servo
has caught up with the command.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List, Sequence, Tuple

import slipx
from slipx import sinks

from .channels import Channel
from .optimise import FitReport, levenberg_marquardt, linear_least_squares
from .reconstruct import (
    Bench,
    GRAVITY,
    SteadySample,
    body_kinematics,
    wheel_loads,
    wheel_slip_angles,
)
from .synthetic import SENSOR_STRIDE, ManoeuvreRecording


# ------------------------------------------------------------ sample gates


@dataclass(frozen=True)
class GatedSample(SteadySample):
    """A steady sample with the measured yaw acceleration attached."""

    yaw_accel: float = 0.0


def quasi_steady_samples(
    rec: ManoeuvreRecording,
    *,
    settle: float = 1.0,
    min_speed: float = 1.0,
    max_yaw_accel: float = 0.35,
    max_sideslip: float = 0.35,
    stride: int = 1,
) -> List[GatedSample]:
    """The samples on which the settled assumptions hold.

    The gates: a settle period at the start (initial transients), a speed
    floor (slip angles diverge at standstill), a yaw-acceleration ceiling
    (rung transitions and breakaway), and a sideslip ceiling (post-breakaway
    data describes a slide, not a tyre curve). A rung of a skidpad ladder
    and the whole pre-breakaway body of a ramp sweep pass; the transitions
    and the spin do not.
    """
    kinematics = body_kinematics(
        rec.channel("pose.x"), rec.channel("pose.y"), rec.channel("pose.yaw")
    )
    yaw_rate = rec.channel("imu.yaw_rate")
    yaw_accel = yaw_rate.derivative()
    ax = rec.channel("imu.ax")
    ay = rec.channel("imu.ay")
    steer = rec.channel("cmd.steer")

    out: List[GatedSample] = []
    for i, t in enumerate(kinematics.speed.times):
        if stride > 1 and i % stride:
            continue
        if t < settle:
            continue
        if t < yaw_accel.times[0] or t > yaw_accel.times[-1]:
            continue
        speed = kinematics.speed.values[i]
        beta = kinematics.sideslip.values[i]
        r_dot = yaw_accel.value_at(t)
        if speed < min_speed:
            continue
        if abs(r_dot) > max_yaw_accel:
            continue
        if abs(beta) > max_sideslip:
            continue
        out.append(
            GatedSample(
                time=t,
                vx=kinematics.vx.values[i],
                vy=kinematics.vy.values[i],
                yaw_rate=yaw_rate.value_at(t),
                steer=steer.value_before(t),
                ax=ax.value_at(t),
                ay=ay.value_at(t),
                bench=rec.bench,
                yaw_accel=r_dot,
            )
        )
    return out


# --------------------------------------------------------------- coastdown


def fit_coastdown(
    recordings: Sequence[ManoeuvreRecording],
    *,
    min_speed: float = 2.0,
) -> FitReport:
    """``roll_resist`` and ``drag_coeff`` from coasting deceleration.

    Linear in both parameters, so this is the closed form and not an
    iteration. The speed floor is not a nicety: below walking pace the
    model's rolling resistance rolls off smoothly through zero speed
    (``tanh(vx / v_eps)``), and a sample down there would bias the fit of
    the constant it is trying to measure.
    """
    rows: List[List[float]] = []
    rhs: List[float] = []
    for rec in recordings:
        radius = rec.bench.wheel_radius
        wheels = [rec.channel(f"wheel.{w}") for w in ("FL", "FR", "RL", "RR")]
        times = wheels[0].times
        speeds = [
            radius * sum(ch.values[i] for ch in wheels) / 4.0
            for i in range(len(times))
        ]
        speed = Channel(times, tuple(speeds))
        accel = speed.derivative()
        mass = rec.bench.mass
        for t, a in zip(accel.times, accel.values):
            v = speed.value_at(t)
            if v < min_speed:
                continue
            rows.append([-GRAVITY, -v * v / mass])
            rhs.append(a)
    return linear_least_squares(rows, rhs, ("roll_resist", "drag_coeff"))


# ----------------------------------------------------------------- lateral


@dataclass(frozen=True)
class LateralFit:
    """The lateral stage's result plus what later stages consume."""

    report: FitReport

    @property
    def c_alpha_f(self) -> float:
        return self.report.value("c_alpha_f")

    @property
    def c_alpha_r(self) -> float:
        return self.report.value("c_alpha_r")

    @property
    def mu_y0(self) -> float:
        return self.report.value("mu_y0")

    @property
    def shape_c(self) -> float:
        return self.report.value("shape_c")

    @property
    def curvature_e(self) -> float:
        return self.report.value("curvature_e")

    @property
    def k_mu(self) -> float:
        return self.report.value("k_mu")


def _axle_ratio(run: Bench, reference: Bench, front: bool) -> float:
    if front:
        return run.static_front_per_tyre / reference.static_front_per_tyre
    return run.static_rear_per_tyre / reference.static_rear_per_tyre


def _candidate_tyres(
    values: Sequence[float], run: Bench, reference: Bench
) -> Tuple[object, object]:
    """Build this run's MF-lite pair from candidate parameters.

    Candidates are stated at the reference bench (the car as raced,
    unballasted). A run at a different mass gets the same physical tyre
    restated at its own static loads by the ADR-0039 power laws, which is
    exactly what makes the ballast runs carry the ``k_mu`` signal.
    """
    c_alpha_f, c_alpha_r, mu_y0, shape_c, curvature_e, k_mu = values
    tyres = []
    for front, c_alpha_axle in ((True, c_alpha_f), (False, c_alpha_r)):
        ratio = _axle_ratio(run, reference, front)
        coefficients = slipx.TyreCoefficients()
        coefficients.mu_y0 = mu_y0 * ratio**-k_mu
        coefficients.mu_x0 = 1.0  # not consumed by a pure lateral force
        coefficients.k_mu = k_mu
        coefficients.relax_length = 0.05  # not consumed in steady state
        coefficients.shape_c = shape_c
        coefficients.curvature_e = curvature_e
        static = (
            run.static_front_per_tyre if front else run.static_rear_per_tyre
        )
        tyres.append(
            slipx.make_mf_lite(
                coefficients,
                0.5 * c_alpha_axle * ratio ** (1.0 - k_mu),
                static,
            )
        )
    return tyres[0], tyres[1]


def _linear_stiffness_start(
    samples: Sequence[GatedSample],
) -> Tuple[float, float]:
    """Warm start for the axle stiffnesses from the low-slip samples.

    In the linear range each axle's force is minus its stiffness times its
    slip angle, and the force and moment balances give each axle's force
    from the measurements alone. Only samples below a small slip angle
    enter, so the curve's bend cannot flatten the estimate.
    """
    front_rows: List[List[float]] = []
    front_rhs: List[float] = []
    rear_rows: List[List[float]] = []
    rear_rhs: List[float] = []
    for s in samples:
        fl, fr, rl, rr = wheel_slip_angles(s)
        alpha_front = 0.5 * (fl + fr)
        alpha_rear = 0.5 * (rl + rr)
        if abs(alpha_front) > 0.03 or abs(alpha_rear) > 0.03:
            continue
        mass = s.bench.mass
        lf, lr = s.bench.lf, s.bench.lr
        wheelbase = s.bench.wheelbase
        force_front = (
            (mass * s.ay * lr + s.bench.izz * s.yaw_accel)
            / wheelbase
            / math.cos(s.steer)
        )
        force_rear = (mass * s.ay * lf - s.bench.izz * s.yaw_accel) / wheelbase
        front_rows.append([-alpha_front])
        front_rhs.append(force_front)
        rear_rows.append([-alpha_rear])
        rear_rhs.append(force_rear)
    if len(front_rows) < 4:
        raise ValueError(
            "not enough low-slip samples for a stiffness estimate; the "
            "skidpad ladder needs rungs in the linear range"
        )
    front = linear_least_squares(front_rows, front_rhs, ("c",)).values[0]
    rear = linear_least_squares(rear_rows, rear_rhs, ("c",)).values[0]
    return front, rear


def fit_lateral(
    recordings: Sequence[ManoeuvreRecording],
    reference: Bench,
    *,
    max_iterations: int = 80,
    sample_stride: int = 1,
) -> LateralFit:
    """The lateral curve: both stiffnesses, the peak, its shape, and the
    load sensitivity.

    Feed it the skidpad ladder and the ramp sweeps, both directions, plus
    the ballasted circle runs; the ballast is where ``k_mu`` comes from and
    without it the report will say so through the confidence interval.
    """
    samples: List[GatedSample] = []
    for rec in recordings:
        samples.extend(quasi_steady_samples(rec, stride=sample_stride))
    if len(samples) < 40:
        raise ValueError(
            f"only {len(samples)} usable steady samples across "
            f"{len(recordings)} recordings; the fit wants the full ladder "
            f"and sweep"
        )

    c_alpha_f0, c_alpha_r0 = _linear_stiffness_start(samples)

    def residuals(values: Sequence[float]) -> List[float]:
        tyre_by_bench: Dict[Tuple[float, float], Tuple[object, object]] = {}
        out: List[float] = []
        force_scale = reference.weight
        moment_scale = 0.5 * reference.wheelbase * force_scale
        for s in samples:
            key = (s.bench.mass, s.bench.lf)
            if key not in tyre_by_bench:
                tyre_by_bench[key] = _candidate_tyres(values, s.bench, reference)
            front_tyre, rear_tyre = tyre_by_bench[key]

            fl_a, fr_a, rl_a, rr_a = wheel_slip_angles(s)

            # The model closes the load-force loop with a fixed two-pass
            # evaluation (ADR-0027): forces at static loads give the
            # acceleration the load transfer is computed from, and the final
            # forces are evaluated at those transferred loads. The
            # prediction mirrors that structure rather than feeding the
            # measured acceleration into the load law, because the loads the
            # model actually used came from its first pass, and the
            # difference is exactly the size of the shape-factor signal this
            # stage exists to fit. Only the candidate tyres and the
            # reconstructed kinematics enter: nothing here reads a signal a
            # car cannot log. The measured ax stands in for the first
            # pass's longitudinal acceleration; its share of the transfer
            # is milli-newtons in any gated sample.
            static_f = s.bench.static_front_per_tyre
            static_r = s.bench.static_rear_per_tyre
            cos_s = math.cos(s.steer)
            first_ay = (
                (
                    slipx.mf_lite_fy(front_tyre, fl_a, static_f)
                    + slipx.mf_lite_fy(front_tyre, fr_a, static_f)
                )
                * cos_s
                + slipx.mf_lite_fy(rear_tyre, rl_a, static_r)
                + slipx.mf_lite_fy(rear_tyre, rr_a, static_r)
            ) / s.bench.mass
            fl_z, fr_z, rl_z, rr_z = wheel_loads(s.bench, s.ax, first_ay)

            fy_fl = slipx.mf_lite_fy(front_tyre, fl_a, fl_z)
            fy_fr = slipx.mf_lite_fy(front_tyre, fr_a, fr_z)
            fy_rl = slipx.mf_lite_fy(rear_tyre, rl_a, rl_z)
            fy_rr = slipx.mf_lite_fy(rear_tyre, rr_a, rr_z)

            # Body-frame components of the steered wheels' forces. The
            # lateral component carries cos(steer); the induced body-x
            # component, -fy sin(steer), matters only through the yaw
            # moment, where its left-right asymmetry under load transfer is
            # a couple the axle-level balance would silently miss (it is
            # exactly the couple that made the first version of this stage
            # misread the shape factors). Rear drive forces cancel from the
            # moment on an open differential and are neglected, which the
            # stage's documentation owns.
            sin_s = math.sin(s.steer)
            body_front = (fy_fl + fy_fr) * cos_s
            force_rear = fy_rl + fy_rr

            lateral = s.bench.mass * s.ay - (body_front + force_rear)
            front_couple = (
                0.5 * s.bench.track_front * sin_s * (fy_fl - fy_fr)
            )
            moment = s.bench.izz * s.yaw_accel - (
                s.bench.lf * body_front
                - s.bench.lr * force_rear
                + front_couple
            )
            out.append(lateral / force_scale)
            out.append(moment / moment_scale)
        return out

    report = levenberg_marquardt(
        residuals,
        [c_alpha_f0, c_alpha_r0, 1.0, 1.5, 0.0, 0.1],
        ("c_alpha_f", "c_alpha_r", "mu_y0", "shape_c", "curvature_e", "k_mu"),
        lower=[50.0, 50.0, 0.3, 1.05, -2.0, 0.0],
        upper=[2000.0, 2000.0, 2.5, 2.5, 0.98, 0.6],
        max_iterations=max_iterations,
    )
    return LateralFit(report=report)


# ------------------------------------------------------------ longitudinal


def fit_c_kappa(
    rec: ManoeuvreRecording,
    resistances: FitReport,
    k_mu: float,
    *,
    min_speed: float = 0.5,
    max_speed: float = 8.0,
) -> FitReport:
    """Longitudinal slip stiffness from the launch, rear axle against front.

    On a rear-wheel-drive car the front encoders read ground speed and the
    rear encoders read wheel speed; the gap is the slip ratio, the delivered
    force comes from the measured acceleration plus the stage-one
    resistances, and the relation is linear through the origin. The load
    correction uses the lateral stage's exponent: a launching car has moved
    load onto exactly the axle being measured.
    """
    bench = rec.bench
    radius = bench.wheel_radius
    roll_resist = resistances.value("roll_resist")
    drag_coeff = resistances.value("drag_coeff")

    front = [rec.channel("wheel.FL"), rec.channel("wheel.FR")]
    rear = [rec.channel("wheel.RL"), rec.channel("wheel.RR")]
    ax = rec.channel("imu.ax")

    rows: List[List[float]] = []
    rhs: List[float] = []
    fz_nom_rear = bench.static_rear_per_tyre
    for i, t in enumerate(front[0].times):
        ground = radius * 0.5 * (front[0].values[i] + front[1].values[i])
        if ground < min_speed or ground > max_speed:
            continue
        wheel = radius * 0.5 * (rear[0].values[i] + rear[1].values[i])
        kappa = (wheel - ground) / ground
        a = ax.value_at(t)
        resist = roll_resist * bench.weight + drag_coeff * ground * ground
        force_per_tyre = 0.5 * (bench.mass * a + resist)
        _, _, rl_z, rr_z = wheel_loads(bench, a, 0.0)
        fz = 0.5 * (rl_z + rr_z)
        load_factor = (fz / fz_nom_rear) ** (1.0 - k_mu)
        rows.append([kappa * load_factor])
        rhs.append(force_per_tyre)
    if len(rows) < 10:
        raise ValueError(
            "not enough launch samples in the speed window for a slip "
            "stiffness fit"
        )
    return linear_least_squares(rows, rhs, ("c_kappa",))


def fit_mu_x0(
    rec: ManoeuvreRecording,
    resistances: FitReport,
    k_mu: float,
    window: Tuple[float, float],
) -> Tuple[float, float]:
    """Peak longitudinal friction from a traction-limited window, with a
    spread.

    The caller names the window, because telling a traction plateau from a
    current-limit plateau is a judgement about the run (the procedure doc
    says whose): inside it, the delivered force per tyre over the load law
    is the friction in use, and its largest value is the ceiling the tyre
    demonstrated. Without a window that actually saturates, the honest
    output is "not observable on this surface", which the caller expresses
    by not calling this.

    Returns ``(mu_x0, spread)``: an envelope estimate has no Gauss-Newton
    interval, so the spread of the top decile of in-use friction inside the
    window stands in for one. On a plateau it is small; a window that was
    not really a plateau shows up as a wide spread rather than a confident
    number.
    """
    bench = rec.bench
    radius = bench.wheel_radius
    roll_resist = resistances.value("roll_resist")
    drag_coeff = resistances.value("drag_coeff")
    front = [rec.channel("wheel.FL"), rec.channel("wheel.FR")]
    ax = rec.channel("imu.ax")
    fz_nom_rear = bench.static_rear_per_tyre

    in_use: List[float] = []
    for i, t in enumerate(front[0].times):
        if not window[0] <= t <= window[1]:
            continue
        ground = radius * 0.5 * (front[0].values[i] + front[1].values[i])
        a = ax.value_at(t)
        resist = roll_resist * bench.weight + drag_coeff * ground * ground
        force_per_tyre = 0.5 * (bench.mass * a + resist)
        _, _, rl_z, rr_z = wheel_loads(bench, a, 0.0)
        fz = 0.5 * (rl_z + rr_z)
        if fz <= 0.0:
            continue
        in_use.append(force_per_tyre / (fz ** (1.0 - k_mu) * fz_nom_rear**k_mu))
    if not in_use or max(in_use) <= 0.0:
        raise ValueError("no delivered force in the stated window")
    top = sorted(in_use, reverse=True)[: max(3, len(in_use) // 10)]
    return top[0], top[0] - top[-1]


def fit_esc(
    rec: ManoeuvreRecording,
    resistances: FitReport,
    *,
    min_speed: float = 0.3,
) -> FitReport:
    """The ESC's torque-speed curve from the launch: stall torque, free
    speed, and the flat cap that binds at low speed.

    The delivered force against speed walks the three regimes of the
    straight-line procedure: a plateau (the cap: the current limit, or the
    tyre, whichever binds lower), the knee, and the falling line whose
    intercepts are the stall torque and the free speed. Which cap was
    identified is the caller's judgement, exactly as for ``fit_mu_x0``: the
    curve cannot tell a current limit from a traction limit by itself.

    The curve is identified at the session's pack state: open-circuit
    voltage above nominal scales the whole curve up, and without pack
    telemetry the fit cannot separate that scale from the stall torque. Run
    the launch on a pack near its nominal voltage, or read the stall torque
    as including the session's voltage ratio; the procedure doc lists sag
    across a session as a failure mode for the same reason.
    """
    bench = rec.bench
    radius = bench.wheel_radius
    roll_resist = resistances.value("roll_resist")
    drag_coeff = resistances.value("drag_coeff")
    front = [rec.channel("wheel.FL"), rec.channel("wheel.FR")]
    ax = rec.channel("imu.ax")

    samples: List[Tuple[float, float]] = []
    for i, t in enumerate(front[0].times):
        v = radius * 0.5 * (front[0].values[i] + front[1].values[i])
        if v < min_speed:
            continue
        a = ax.value_at(t)
        if a < 0.0:
            continue  # braking or coasting at the end of the run
        resist = roll_resist * bench.weight + drag_coeff * v * v
        samples.append((v, bench.mass * a + resist))
    if len(samples) < 20:
        raise ValueError(
            "not enough launch samples above the speed floor for an ESC fit"
        )

    top_force = max(force for _, force in samples)
    top_speed = max(v for v, _ in samples)

    def residuals(values: Sequence[float]) -> List[float]:
        stall_force, free_speed, cap = values
        out = []
        for v, force in samples:
            curve = stall_force * (1.0 - v / free_speed)
            out.append(min(cap, curve) - force)
        return out

    fit = levenberg_marquardt(
        residuals,
        [1.5 * top_force, 2.0 * top_speed, top_force],
        ("stall_force", "free_speed", "cap_force"),
        lower=[top_force, top_speed, 0.5 * top_force],
        upper=[50.0 * top_force, 50.0 * top_speed, 1.2 * top_force],
    )
    return fit


# -------------------------------------------------------------- transient


def fit_transient(
    recordings: Sequence[ManoeuvreRecording],
    base_params: "slipx.VehicleParams",
    *,
    fit_shape: bool = True,
    max_iterations: int = 40,
    fit_window: float = 1.2,
) -> FitReport:
    """The step-steer family: the delays, and the far side of the peak.

    The three delays are separated by the speed ladder (the procedure doc
    carries the argument); the fit replays each recording's commands through
    the forward model with candidate values and matches the measured yaw
    rate trace. Everything else in ``base_params`` is held fixed: this stage
    runs last and consumes the earlier stages' values as constants.

    With ``fit_shape`` the Magic Formula shape pair rides along, and the
    reason is physical rather than convenient: steady-state driving cannot
    sit on the falling branch of the tyre curve (a car balanced past the
    peak is unstable and departs), so the steady stages pin the curve up to
    the peak and leave C and E trading off along everything after it: the
    lateral stage reports exactly that correlation. A step past the limit
    rides the falling branch for a few tenths of a second on the way to its
    excursion, and the replay is the only fit structure that can read those
    tenths, because the tyre lag is in the model rather than assumed away.
    The lateral stage's values seed the pair.
    """

    def predict(values: Sequence[float], rec: ManoeuvreRecording) -> Channel:
        relax, bandwidth, damping = values[0], values[1], values[2]
        params = base_params.copy()
        params.tyre_front.relax_length = relax
        params.tyre_rear.relax_length = relax
        params.steer_bandwidth = bandwidth
        params.steer_damping = damping
        if fit_shape:
            shape_c, curvature_e = values[3], values[4]
            for tyre in (params.tyre_front, params.tyre_rear):
                tyre.shape_c = shape_c
                tyre.curvature_e = curvature_e

        steer_cmd = rec.channel("cmd.steer")
        accel_cmd = rec.channel("cmd.accel")

        def playback(state, time, rng):
            return slipx.DriveInput(
                steer_cmd=steer_cmd.value_before(time),
                accel_cmd=accel_cmd.value_before(time),
            )

        config = slipx.SimulationConfig()
        config.dt = rec.dt
        sim = slipx.Simulation(config)
        agent = slipx.AgentSpec()
        agent.name = "candidate"
        agent.tier = slipx.Tier.L2_DoubleTrack
        agent.params = params
        wheels = rec.channel("wheel.FL")
        initial_speed = wheels.values[0] * rec.bench.wheel_radius
        agent.initial_state.vel_body.x = initial_speed
        agent.initial_state.omega_w = [
            initial_speed / rec.bench.wheel_radius
        ] * 4
        agent.policy = playback
        sim.add_agent(agent)

        duration = rec.channel("imu.yaw_rate").times[-1]
        recording = sinks.record_run(
            sim, duration=duration, stride=SENSOR_STRIDE
        )
        record = recording.agents[0]
        return Channel(tuple(recording.times), tuple(record.state["rates.z"]))

    # The step instant per recording, read off the command channel so the
    # window follows the data rather than a convention.
    windows: List[Tuple[ManoeuvreRecording, float, float]] = []
    for rec in recordings:
        cmd = rec.channel("cmd.steer")
        step_at = next(
            (t for t, v in zip(cmd.times, cmd.values) if v != cmd.values[0]),
            None,
        )
        if step_at is None:
            raise ValueError(f"recording '{rec.name}' contains no step")
        windows.append((rec, step_at - 0.1, step_at + fit_window))

    def residuals(values: Sequence[float]) -> List[float]:
        out: List[float] = []
        for rec, start, end in windows:
            measured = rec.channel("imu.yaw_rate")
            predicted = predict(values, rec)
            for t, r in zip(measured.times, measured.values):
                if start <= t <= end:
                    out.append(predicted.value_at(t) - r)
        return out

    names: List[str] = ["relax_length", "steer_bandwidth", "steer_damping"]
    initial: List[float] = [0.05, 30.0, 0.8]
    lower: List[float] = [0.005, 5.0, 0.2]
    upper: List[float] = [0.5, 200.0, 2.0]
    if fit_shape:
        names += ["shape_c", "curvature_e"]
        initial += [
            base_params.tyre_front.shape_c,
            base_params.tyre_front.curvature_e,
        ]
        lower += [1.05, -2.0]
        upper += [2.5, 0.98]

    return levenberg_marquardt(
        residuals,
        initial,
        names,
        lower=lower,
        upper=upper,
        max_iterations=max_iterations,
    )
