# Copyright 2026 The SlipX Authors
# SPDX-License-Identifier: Apache-2.0

"""The validation report (M6.4).

Replay the measured commands of a run the fit never saw through the fitted
model, and show the divergence in yaw rate, lateral acceleration and speed,
with one headline number. The drawing borrows the SVG sink's own helpers
and theme (ADR-0024's rules apply unchanged: a file, never a window, and
nothing drawn that was not measured or replayed).

What the report is and is not: it compares the model against the manoeuvres
that were driven, and says nothing about manoeuvres that were not. A set
that validates on a slalom has validated on a slalom. The headline is the
worst channel of the worst run, so the number cannot be improved by adding
easy runs.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple, Union

import slipx
from slipx import sinks
from slipx.sinks.svg_sink import _STYLE, _esc, _n, _polyline

from .channels import Channel
from .synthetic import SENSOR_STRIDE, ManoeuvreRecording

#: The compared channels: what the car measured against what the replay
#: produced, like against like. Speed is the encoder speed on both sides,
#: because that is the signal a car actually has.
_CHANNELS = (
    ("yaw rate", "rad/s"),
    ("lateral acceleration", "m/s2"),
    ("speed", "m/s"),
)

#: Samples before this time are the replay finding its feet from the
#: constructed straight start, not model divergence.
_SETTLE_S = 0.5

_WIDTH = 960.0
_PANEL_X = 130.0
_PANEL_W = 780.0
_PANEL_H = 96.0
_PANEL_GAP = 34.0


@dataclass(frozen=True)
class ChannelDivergence:
    name: str
    unit: str
    rms_error: float
    scale: float

    @property
    def percent(self) -> float:
        return 100.0 * self.rms_error / self.scale


@dataclass(frozen=True)
class RunComparison:
    name: str
    times: Tuple[float, ...]
    measured: Tuple[Tuple[float, ...], ...]  # per channel
    replayed: Tuple[Tuple[float, ...], ...]
    divergences: Tuple[ChannelDivergence, ...]


def _measured_channels(rec: ManoeuvreRecording) -> Tuple[Channel, Channel, Channel]:
    radius = rec.bench.wheel_radius
    wheels = [rec.channel(f"wheel.{w}") for w in ("FL", "FR", "RL", "RR")]
    speed = Channel(
        wheels[0].times,
        tuple(
            radius * sum(ch.values[i] for ch in wheels) / 4.0
            for i in range(len(wheels[0]))
        ),
    )
    return rec.channel("imu.yaw_rate"), rec.channel("imu.ay"), speed


def _replay(params: "slipx.VehicleParams", rec: ManoeuvreRecording):
    """The recorded commands through the fitted model, sampled like the
    sensors. Starts from straight steady running at the recording's initial
    encoder speed, which is why a validation run begins on a straight."""
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
    agent.name = "validation"
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
    recording = sinks.record_run(sim, duration=duration, stride=SENSOR_STRIDE)
    record = recording.agents[0]
    times = tuple(recording.times)
    radius = rec.bench.wheel_radius
    speed = tuple(
        radius
        * 0.25
        * (
            record.state["omega_w.FL"][i]
            + record.state["omega_w.FR"][i]
            + record.state["omega_w.RL"][i]
            + record.state["omega_w.RR"][i]
        )
        for i in range(len(times))
    )
    return (
        Channel(times, tuple(record.state["rates.z"])),
        Channel(times, tuple(record.diagnostics["ay"])),
        Channel(times, speed),
    )


def _rms(values: Sequence[float]) -> float:
    if not values:
        return 0.0
    return math.sqrt(sum(v * v for v in values) / len(values))


def compare(
    params: "slipx.VehicleParams", rec: ManoeuvreRecording
) -> RunComparison:
    measured = _measured_channels(rec)
    replayed = _replay(params, rec)

    times = [
        t
        for t in measured[0].times
        if t >= _SETTLE_S and t <= replayed[0].times[-1]
    ]
    measured_columns: List[Tuple[float, ...]] = []
    replayed_columns: List[Tuple[float, ...]] = []
    divergences: List[ChannelDivergence] = []
    for index, (name, unit) in enumerate(_CHANNELS):
        m = tuple(measured[index].value_at(t) for t in times)
        r = tuple(replayed[index].value_at(t) for t in times)
        errors = [b - a for a, b in zip(m, r)]
        mean = sum(m) / len(m)
        if name == "speed":
            # A held speed barely varies, so deviation about the mean would
            # divide by nearly nothing; the mean speed is the honest scale.
            scale = max(abs(mean), 0.1)
        else:
            scale = max(_rms([v - mean for v in m]), 1e-3)
        divergences.append(
            ChannelDivergence(name, unit, _rms(errors), scale)
        )
        measured_columns.append(m)
        replayed_columns.append(r)

    return RunComparison(
        name=rec.name,
        times=tuple(times),
        measured=tuple(measured_columns),
        replayed=tuple(replayed_columns),
        divergences=tuple(divergences),
    )


def headline(comparisons: Sequence[RunComparison]) -> float:
    """The worst channel of the worst run, in percent. Adding easy runs
    cannot improve it."""
    return max(
        divergence.percent
        for comparison in comparisons
        for divergence in comparison.divergences
    )


def _panel(
    comparison: RunComparison, index: int, x: float, y: float
) -> List[str]:
    name, unit = _CHANNELS[index]
    measured = comparison.measured[index]
    replayed = comparison.replayed[index]
    lo = min(min(measured), min(replayed))
    hi = max(max(measured), max(replayed))
    if hi - lo < 1e-9:
        hi = lo + 1.0
    t0, t1 = comparison.times[0], comparison.times[-1]

    def place(t: float, v: float) -> Tuple[float, float]:
        px = x + (t - t0) / (t1 - t0) * _PANEL_W
        py = y + _PANEL_H - (v - lo) / (hi - lo) * _PANEL_H
        return px, py

    parts = [
        f'<rect class="frame" x="{_n(x)}" y="{_n(y)}" '
        f'width="{_n(_PANEL_W)}" height="{_n(_PANEL_H)}"/>',
        f'<text class="t" x="{_n(x - 104.0)}" y="{_n(y + 14.0)}">'
        f"{_esc(name)}</text>",
        f'<text class="tk" x="{_n(x - 104.0)}" y="{_n(y + 28.0)}">'
        f"[{_esc(unit)}]</text>",
        # The y extent, written down: a panel auto-scaled to a hair's
        # breadth of variation would otherwise make a 0.1 per cent
        # divergence look like a gulf.
        f'<text class="tk" x="{_n(x + _PANEL_W + 6.0)}" y="{_n(y + 10.0)}">'
        f"{_esc(f'{hi:.3g}')}</text>",
        f'<text class="tk" x="{_n(x + _PANEL_W + 6.0)}" '
        f'y="{_n(y + _PANEL_H)}">{_esc(f"{lo:.3g}")}</text>',
    ]
    divergence = comparison.divergences[index]
    parts.append(
        f'<text class="tm" x="{_n(x - 104.0)}" y="{_n(y + 46.0)}">'
        f"diverges {_esc(f'{divergence.percent:.1f}')}%</text>"
    )
    measured_points = [
        place(t, v) for t, v in zip(comparison.times, measured)
    ]
    replayed_points = [
        place(t, v) for t, v in zip(comparison.times, replayed)
    ]
    parts.append(
        f'<polyline class="s1" points="{_polyline(measured_points)}"/>'
    )
    parts.append(
        f'<polyline class="s0 dash" points="{_polyline(replayed_points)}"/>'
    )
    return parts


def render(
    comparisons: Sequence[RunComparison],
    *,
    car_name: str,
    label: str,
    date: str,
) -> str:
    """The report as one theme-aware SVG document."""
    if not comparisons:
        raise ValueError("a validation report needs at least one run")

    panels_per_run = len(_CHANNELS)
    header_h = 96.0
    run_title_h = 30.0
    footer_h = 88.0
    runs_h = sum(
        run_title_h + panels_per_run * (_PANEL_H + _PANEL_GAP)
        for _ in comparisons
    )
    height = header_h + runs_h + footer_h

    worst = headline(comparisons)
    parts = [
        '<svg xmlns="http://www.w3.org/2000/svg" '
        f'viewBox="0 0 {_n(_WIDTH)} {_n(height)}" role="img" '
        'aria-label="SlipX validation report">',
        f"<style>{_STYLE}\n  .dash {{ stroke-dasharray: 6 4; }}</style>",
        f'<rect class="card" x="0" y="0" width="{_n(_WIDTH)}" '
        f'height="{_n(height)}"/>',
        f'<text class="tb" x="26" y="34">Validation report: '
        f"{_esc(car_name)}</text>",
        f'<text class="t" x="26" y="56">{_esc(label.upper())} parameter '
        f"set, replayed against {len(comparisons)} recorded run(s), "
        f"{_esc(date)}</text>",
        f'<text class="tb" x="26" y="80">worst-channel divergence: '
        f"{_esc(f'{worst:.1f}')}%</text>",
        f'<text class="tm" x="{_n(_WIDTH - 26.0)}" y="56" '
        'text-anchor="end">measured</text>',
        f'<line class="s1" x1="{_n(_WIDTH - 90.0)}" y1="52" '
        f'x2="{_n(_WIDTH - 116.0)}" y2="52"/>',
        f'<text class="tm" x="{_n(_WIDTH - 26.0)}" y="74" '
        'text-anchor="end">replayed</text>',
        f'<line class="s0 dash" x1="{_n(_WIDTH - 90.0)}" y1="70" '
        f'x2="{_n(_WIDTH - 116.0)}" y2="70"/>',
    ]

    y = header_h
    for comparison in comparisons:
        parts.append(
            f'<text class="t" x="26" y="{_n(y + 18.0)}">'
            f"{_esc(comparison.name)}</text>"
        )
        y += run_title_h
        for index in range(panels_per_run):
            parts.extend(_panel(comparison, index, _PANEL_X, y))
            y += _PANEL_H + _PANEL_GAP

    parts.extend(
        [
            f'<text class="tm" x="26" y="{_n(y + 10.0)}">'
            "Replay: the run's recorded commands through the fitted model, "
            "from straight steady running at the run's initial speed."
            "</text>",
            f'<text class="tm" x="26" y="{_n(y + 26.0)}">'
            "Divergence: RMS(replayed - measured) over RMS variation of the "
            "measured channel (over mean speed, for speed), after a "
            f"{_esc(f'{_SETTLE_S:g}')} s settle.</text>",
            f'<text class="tm" x="26" y="{_n(y + 42.0)}">'
            "A set that validates on these runs has validated on these "
            "runs, and on nothing else.</text>",
            "</svg>",
        ]
    )
    return "\n".join(parts)


def generate(
    car: Union[str, Path, "slipx.VehicleParams"],
    recordings: Sequence[ManoeuvreRecording],
    output: Union[str, Path],
    *,
    date: str = "",
) -> Tuple[Path, float]:
    """Write the report and return its path and the headline percentage."""
    if isinstance(car, (str, Path)):
        loaded = slipx.load_car(car)
        params = loaded.params_for_tier(slipx.Tier.L2_DoubleTrack)
        car_name = loaded.name
        label = loaded.provenance.label
    else:
        params = car
        car_name = "parameters"
        label = str(params.provenance).rsplit(".", 1)[-1].lower()

    comparisons = [compare(params, rec) for rec in recordings]
    path = Path(output)
    path.write_text(
        render(comparisons, car_name=car_name, label=label, date=date),
        encoding="utf-8",
    )
    return path, headline(comparisons)
