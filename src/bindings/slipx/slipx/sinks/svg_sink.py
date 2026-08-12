# Copyright 2026 The SlipX Authors
# SPDX-License-Identifier: Apache-2.0

"""The SVG sink: one self-contained animated file, and no dependencies at all.

This is the sink that needs nothing installed. The MCAP and Rerun sinks are
optional extras because they encode with somebody else's library; this one is
the Python standard library and a string, so it is always available and it is
the one you can attach to an issue, drop into a slide or open on a machine that
has a browser and nothing else.

It writes a file and it never opens a window (ADR-0024, ADR-0028). There is no
display server here, no GPU, no viewer process and nothing to serve: the output
is a document, and a browser is somebody else's program that happens to render
documents.

Three properties are worth stating because they are easy to lose.

**It draws nothing the run did not contain** (ADR-0024). No track, no kerbs, no
car body. The last one is the one people ask about: a car outline needs a
length and a width, and neither is in the recording. The manifest carries a
digest of the parameters, not the parameters, so a body drawn here would be a
body invented here. What moves along the path is a marker with a heading tick,
sized in screen pixels, which is a cursor and not a vehicle.

**A quantity a tier cannot represent is absent, never a plotted zero**
(ADR-0006, ADR-0028). The recording has already turned every unrepresentable
field into NaN, so the rule reduces to: a panel whose columns are all NaN is
not drawn, and a gap inside a column breaks the trace rather than joining
across it. At L0 and L1 that is most of this file's panels, and the picture
being shorter is the honest outcome.

**Two encodings of one run are byte-identical.** No timestamp is written, no
identifier is generated, every dictionary is walked in a fixed order and every
number goes through one formatter. A viewer diffing two runs should see the run
differ, not the writer.

Theme awareness comes from an embedded ``prefers-color-scheme`` stylesheet and
an opaque background card, the same pattern the tutorial series' figures use.
An SVG with a transparent background and dark strokes is invisible on a dark
page, which is the single most common way a diagram breaks after somebody
pastes it somewhere.
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Dict, List, Mapping, Optional, Sequence, Tuple, Union

from .protocol import resolve_path
from .recording import AgentRecord, Recording

# ------------------------------------------------------------------- layout
#
# Fixed geometry, in user units. The map is square because a path plotted on
# unequal axes is a lie about the shape of the corner; the panel column grows
# downwards with however many panels survive the absence rule, and the document
# height follows it.

_WIDTH = 960.0
_HEADER_H = 92.0
_FOOTER_H = 46.0
_MAP = (26.0, _HEADER_H, 438.0, 438.0)  # x, y, w, h
_PANEL_X = 502.0
_PANEL_W = 400.0
_PANEL_H = 56.0
_PANEL_GAP = 20.0
_MARKER_R = 5.0
_HEADING_PX = 17.0

#: Per-wheel traces are drawn in this order, always, so the legend and the
#: colours mean the same thing in every file.
_WHEEL_ORDER = ("FL", "FR", "RL", "RR")


class Panel:
    """One time-series panel and the recorded columns it is drawn from.

    Kept as an explicit object rather than a tuple because it is the unit the
    element-inventory test reasons about: every stroke this sink makes inside a
    panel comes from ``sources``, and ``sources`` are recording column names.
    """

    __slots__ = ("title", "unit", "group", "columns", "series")

    def __init__(
        self,
        title: str,
        unit: str,
        group: str,
        columns: Sequence[str],
        series: Sequence[Tuple[str, Tuple[float, ...]]],
    ):
        self.title = title
        self.unit = unit
        self.group = group          # "state" or "diagnostics"
        self.columns = tuple(columns)
        self.series = tuple(series)  # (label, values), in drawing order

    @property
    def sources(self) -> Tuple[str, ...]:
        """``("diagnostics.fz.FL", ...)``: fully qualified recorded columns."""
        return tuple(f"{self.group}.{name}" for name in self.columns)


# The panels this sink knows how to draw, in the order they are stacked.
# A row is (title, unit, group, column names). Per-wheel rows name all four
# and are drawn as four traces; a row survives only if at least one of its
# columns has a value somewhere in the run.
_PANEL_SPECS: Tuple[Tuple[str, str, str, Tuple[str, ...]], ...] = (
    ("forward speed", "m/s", "state", ("vel_body.x",)),
    ("yaw rate", "rad/s", "state", ("rates.z",)),
    ("lateral acceleration", "m/s^2", "diagnostics", ("ay",)),
    ("steer angle, achieved", "rad", "state", ("steer",)),
    ("axle lateral force", "N", "diagnostics", ("fy_front", "fy_rear")),
    ("vertical load per wheel", "N", "diagnostics",
     tuple(f"fz.{w}" for w in _WHEEL_ORDER)),
    ("slip angle per wheel", "rad", "diagnostics",
     tuple(f"alpha.{w}" for w in _WHEEL_ORDER)),
    ("slip ratio per wheel", "-", "diagnostics",
     tuple(f"kappa.{w}" for w in _WHEEL_ORDER)),
    ("drive torque", "N m", "diagnostics", ("drive_torque",)),
    ("state of charge", "-", "state", ("soc",)),
)


def _label(column: str) -> str:
    """The legend text for one column: what distinguishes it, and no more.

    ``fz.FL`` is ``FL`` and ``fy_front`` is ``front``. The panel title already
    says which quantity it is, and repeating it in four legend entries is how
    a legend runs off the edge of the page.
    """
    last = column.split(".")[-1]
    return last.rsplit("_", 1)[-1] if "_" in last else last


def _has_any(values: Sequence[float]) -> bool:
    """Is there a single non-NaN in this column?

    ``value == value`` is False for NaN and for nothing else, which is the one
    test that needs no import and no per-element function call.
    """
    return any(value == value for value in values)


def panel_plan(agent: AgentRecord) -> Tuple[Panel, ...]:
    """Every panel this sink will draw for one agent, and nothing else.

    Pure: no file, no string building, no side effect. This is the whole of the
    absence decision, so a test can call it and compare it against what the
    tier is entitled to report, without parsing an SVG to find out.
    """
    panels: List[Panel] = []
    for title, unit, group, columns in _PANEL_SPECS:
        source: Mapping[str, Tuple[float, ...]] = (
            agent.state if group == "state" else agent.diagnostics
        )
        series = [
            (_label(name), source[name])
            for name in columns
            if name in source and _has_any(source[name])
        ]
        if series:
            kept = tuple(name for name in columns
                         if name in source and _has_any(source[name]))
            panels.append(Panel(title, unit, group, kept, series))
    return tuple(panels)


def drawn_sources(agent: AgentRecord) -> Tuple[str, ...]:
    """Every recorded column any stroke in the drawing comes from.

    The panels, plus the three the map is built from. If a name appears here
    that is not a column of the recording, this sink has invented something.
    """
    names = ["state.pos.x", "state.pos.y", "state.yaw"]
    for panel in panel_plan(agent):
        names.extend(panel.sources)
    return tuple(names)


# -------------------------------------------------------------------- style

_STYLE = """
  :root {
    --paper:#ffffff; --fg:#14181d; --muted:#5b6672; --grid:#e3e7ec;
    --path:#5b6672; --c0:#c2410c; --c1:#1d4ed8; --c2:#15803d; --c3:#a16207;
  }
  @media (prefers-color-scheme: dark) {
    :root {
      --paper:#11161c; --fg:#e8ecf1; --muted:#93a0ad; --grid:#2a323b;
      --path:#93a0ad; --c0:#fb923c; --c1:#7aa7ff; --c2:#4ade80; --c3:#eab308;
    }
  }
  .card { fill: var(--paper); }
  .frame { fill: none; stroke: var(--grid); stroke-width: 1; }
  .path { fill: none; stroke: var(--path); stroke-width: 1.6;
          stroke-linejoin: round; stroke-linecap: round; }
  .cursor { stroke: var(--fg); stroke-width: 1; opacity: 0.45; }
  .mark { fill: var(--c0); }
  .head { stroke: var(--c0); stroke-width: 2; fill: none; }
  .s0 { fill: none; stroke: var(--c0); stroke-width: 1.5; }
  .s1 { fill: none; stroke: var(--c1); stroke-width: 1.5; }
  .s2 { fill: none; stroke: var(--c2); stroke-width: 1.5; }
  .s3 { fill: none; stroke: var(--c3); stroke-width: 1.5; }
  text { font-family: ui-sans-serif, -apple-system, "Segoe UI", Roboto,
         "Helvetica Neue", Arial, sans-serif; fill: var(--fg); }
  .t  { font-size: 12px; }
  .tb { font-size: 16px; font-weight: 600; }
  .tm { font-size: 11px; fill: var(--muted); }
  .tk { font-size: 10px; fill: var(--muted); }
  .mono { font-family: ui-monospace, SFMono-Regular, Menlo, Consolas,
          monospace; font-size: 11px; fill: var(--muted); }
"""

_SERIES_CLASS = ("s0", "s1", "s2", "s3")


def _n(value: float) -> str:
    """One number formatter, so two runs of the same recording agree exactly.

    Rounding to three decimals in user units is well inside a pixel and keeps
    the file free of the seventeen-digit repr that would otherwise make every
    diff unreadable. ``-0.000`` is normalised because the sign of a rounded
    zero is a property of the input's last bit and not of the drawing.
    """
    text = "%.3f" % value
    return "0.000" if text == "-0.000" else text


def _sig(value: float) -> str:
    """Four significant figures, for a label a reader is meant to read.

    The drawing coordinates use :func:`_n`, which is three decimals because
    that is well inside a pixel. A range label is not a coordinate: three
    decimals reports a state of charge falling from 1.000 to 1.000.
    """
    text = "%.4g" % value
    return "0" if text == "-0" else text


def _esc(text: str) -> str:
    return (
        text.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")
    )


class _Extent:
    """The value range of a set of columns, ignoring NaN, never degenerate."""

    def __init__(self, columns: Sequence[Sequence[float]]):
        values = [v for column in columns for v in column if v == v]
        self.lo = min(values)
        self.hi = max(values)
        if self.hi - self.lo < 1e-12:
            # A constant trace still has to be drawn somewhere, and a zero
            # span divides by zero. Centre it in a unit band.
            pad = max(abs(self.hi), 1.0) * 0.05
            self.lo -= pad
            self.hi += pad

    def frac(self, value: float) -> float:
        return (value - self.lo) / (self.hi - self.lo)


def _polyline(points: Sequence[Tuple[float, float]]) -> str:
    return " ".join(f"{_n(x)},{_n(y)}" for x, y in points)


def _traces(
    values: Sequence[float],
    times: Sequence[float],
    x_of: "callable",
    y_of: "callable",
) -> List[List[Tuple[float, float]]]:
    """Split a column into runs of consecutive non-NaN samples.

    A break rather than a bridge: a wheel that lifts has no slip ratio for as
    long as it is off the ground, and a line drawn across that gap asserts a
    value nobody computed.
    """
    runs: List[List[Tuple[float, float]]] = []
    current: List[Tuple[float, float]] = []
    for time, value in zip(times, values):
        if value == value:
            current.append((x_of(time), y_of(value)))
        elif current:
            runs.append(current)
            current = []
    if current:
        runs.append(current)
    return [run for run in runs if len(run) > 1]


class SvgSink:
    """Write a run as one self-contained animated SVG.

    The trajectory as a path, a marker animated along it, and one panel per
    recorded quantity the tier actually produced, each with a time cursor
    moving with the marker. The provenance label and the trajectory hash are
    drawn into the image rather than printed beside it, because a rendered run
    ends up pasted into a slide with everything around it discarded.

    Writes a file and stops.
    """

    suffix = ".svg"

    def __init__(self, *, agent: Optional[str] = None):
        #: Which agent to draw, by name. One picture is one car; a multi-agent
        #: run writes one file per agent rather than overlaying traces whose
        #: axes have nothing to do with each other.
        self._agent = agent

    def write(self, recording: Recording, path: Union[str, Path]) -> Path:
        resolved = resolve_path(path, self.suffix)
        resolved.write_text(self.render(recording), encoding="utf-8")
        return resolved

    # --------------------------------------------------------------- render

    def render(self, recording: Recording) -> str:
        """The whole document as a string, so a test never needs a file."""
        agent = self._select(recording)
        panels = panel_plan(agent)

        panels_bottom = _HEADER_H + len(panels) * (_PANEL_H + _PANEL_GAP)
        height = max(_MAP[1] + _MAP[3], panels_bottom) + _FOOTER_H
        duration = recording.times[-1] if recording.times else 0.0

        parts: List[str] = [
            '<svg xmlns="http://www.w3.org/2000/svg" '
            f'width="{_n(_WIDTH)}" height="{_n(height)}" '
            f'viewBox="0 0 {_n(_WIDTH)} {_n(height)}" role="img" '
            f'aria-label="SlipX run {_esc(recording.trajectory_hash)}">',
            f"<title>SlipX run {_esc(recording.trajectory_hash)}</title>",
            f"<style>{_STYLE}</style>",
            f'<rect class="card" x="0" y="0" width="{_n(_WIDTH)}" '
            f'height="{_n(height)}" rx="10"/>',
        ]
        parts.extend(self._header(recording, agent))
        parts.extend(self._map(recording, agent, duration))
        for index, panel in enumerate(panels):
            parts.extend(self._panel(recording, panel, index, duration))
        parts.extend(self._footer(recording, height))
        parts.append("</svg>")
        return "\n".join(parts) + "\n"

    # ------------------------------------------------------------ internals

    def _select(self, recording: Recording) -> AgentRecord:
        if self._agent is None:
            return recording.agents[0]
        for agent in recording.agents:
            if agent.name == self._agent:
                return agent
        known = ", ".join(agent.name for agent in recording.agents)
        raise KeyError(
            f"no agent named {self._agent!r} in this recording; it has: {known}"
        )

    def _header(self, recording: Recording, agent: AgentRecord) -> List[str]:
        return [
            f'<text class="tb" x="26" y="34">SlipX run: {_esc(agent.name)} '
            f"at {_esc(agent.tier)}</text>",
            f'<text class="tm" x="26" y="55">'
            f"{_esc(recording.provenance_line())}</text>",
            f'<text class="tm" x="26" y="73">integrator '
            f"{_esc(recording.integrator)}, dt {_n(recording.dt)} s, every "
            f"{recording.stride} steps, {len(recording.times)} frames</text>",
        ]

    def _map(
        self, recording: Recording, agent: AgentRecord, duration: float
    ) -> List[str]:
        x0, y0, w, h = _MAP
        xs = agent.state["pos.x"]
        ys = agent.state["pos.y"]
        yaws = agent.state["yaw"]

        # One scale for both axes, so the shape of the path is the shape it
        # was driven in. Centred in the box with a margin.
        ex, ey = _Extent([xs]), _Extent([ys])
        span = max(ex.hi - ex.lo, ey.hi - ey.lo)
        scale = (min(w, h) - 44.0) / span
        cx, cy = 0.5 * (ex.lo + ex.hi), 0.5 * (ey.lo + ey.hi)

        def px(value: float) -> float:
            return x0 + 0.5 * w + (value - cx) * scale

        def py(value: float) -> float:
            # ISO 8855 y points left; SVG y points down. One negation, here,
            # so a left turn on the road is a left turn on the page.
            return y0 + 0.5 * h - (value - cy) * scale

        points = [(px(x), py(y)) for x, y in zip(xs, ys)]
        parts = [
            f'<rect class="frame" x="{_n(x0)}" y="{_n(y0)}" width="{_n(w)}" '
            f'height="{_n(h)}" rx="6"/>',
            f'<text class="tk" x="{_n(x0 + 8)}" y="{_n(y0 + 16)}">path of '
            f"pos.x, pos.y   [m], {_n(1.0 / scale * 100.0)} m per 100 px"
            "</text>",
            f'<polyline class="path" points="{_polyline(points)}"/>',
        ]

        # The marker: a dot for position and a tick for heading, both animated
        # from the recorded columns and nothing else. Sized in screen pixels
        # because it is a cursor, not a car (see the module docstring).
        if duration > 0.0:
            keytimes = ";".join(
                _n(time / duration) for time in recording.times
            )
            translate = ";".join(
                f"{_n(x)} {_n(y)}" for x, y in points
            )
            # SVG rotates clockwise, ISO yaw counter-clockwise: one negation,
            # for the same reason as py().
            rotate = ";".join(
                _n(-math.degrees(yaw)) for yaw in yaws
            )
            first_x, first_y = points[0]
            parts.append(
                f'<g transform="translate({_n(first_x)} {_n(first_y)})">'
                f'<animateTransform attributeName="transform" '
                f'type="translate" dur="{_n(duration)}s" '
                f'repeatCount="indefinite" keyTimes="{keytimes}" '
                f'values="{translate}" calcMode="linear"/>'
                f'<g transform="rotate({_n(-math.degrees(yaws[0]))})">'
                f'<animateTransform attributeName="transform" '
                f'type="rotate" dur="{_n(duration)}s" '
                f'repeatCount="indefinite" keyTimes="{keytimes}" '
                f'values="{rotate}" calcMode="linear"/>'
                f'<circle class="mark" cx="0" cy="0" r="{_n(_MARKER_R)}"/>'
                f'<line class="head" x1="0" y1="0" x2="{_n(_HEADING_PX)}" '
                f'y2="0"/></g></g>'
            )
        return parts

    def _panel(
        self,
        recording: Recording,
        panel: Panel,
        index: int,
        duration: float,
    ) -> List[str]:
        x0 = _PANEL_X
        y0 = _HEADER_H + index * (_PANEL_H + _PANEL_GAP)
        w, h = _PANEL_W, _PANEL_H
        extent = _Extent([values for _, values in panel.series])

        def px(time: float) -> float:
            span = recording.times[-1] - recording.times[0]
            if span <= 0.0:
                return x0
            return x0 + (time - recording.times[0]) / span * w

        def py(value: float) -> float:
            return y0 + h - extent.frac(value) * h

        parts = [
            f'<rect class="frame" x="{_n(x0)}" y="{_n(y0)}" width="{_n(w)}" '
            f'height="{_n(h)}" rx="4"/>',
            f'<text class="tk" x="{_n(x0)}" y="{_n(y0 - 5)}">'
            f"{_esc(panel.title)}   [{_esc(panel.unit)}]</text>",
            f'<text class="tk" x="{_n(x0 + w)}" y="{_n(y0 - 5)}" '
            f'text-anchor="end">{_sig(extent.lo)} to {_sig(extent.hi)}</text>',
        ]
        for order, (label, values) in enumerate(panel.series):
            cls = _SERIES_CLASS[order % len(_SERIES_CLASS)]
            for run in _traces(values, recording.times, px, py):
                parts.append(
                    f'<polyline class="{cls}" points="{_polyline(run)}"/>'
                )
            if len(panel.series) > 1:
                parts.append(
                    f'<text class="tk" x="{_n(x0 + w + 6)}" '
                    f'y="{_n(y0 + 11 + order * 12)}">{_esc(label)}</text>'
                )

        if duration > 0.0:
            parts.append(
                f'<line class="cursor" y1="{_n(y0)}" y2="{_n(y0 + h)}">'
                f'<animate attributeName="x1" dur="{_n(duration)}s" '
                f'repeatCount="indefinite" values="{_n(x0)};{_n(x0 + w)}"/>'
                f'<animate attributeName="x2" dur="{_n(duration)}s" '
                f'repeatCount="indefinite" values="{_n(x0)};{_n(x0 + w)}"/>'
                f"</line>"
            )
        return parts

    def _footer(self, recording: Recording, height: float) -> List[str]:
        y = height - 26.0
        # The hash on its own line as well as inside the provenance line: it is
        # the one string somebody retypes to check a result, and a reader
        # should not have to find it inside a sentence.
        return [
            f'<text class="mono" x="26" y="{_n(y)}">trajectory '
            f"{_esc(recording.trajectory_hash)}   configuration "
            f"{_esc(_configuration_digest(recording))}</text>",
            f'<text class="tk" x="{_n(_WIDTH - 26)}" y="{_n(y)}" '
            'text-anchor="end">nothing is drawn here that the run did not '
            "record: no track, no car body</text>",
        ]


def _configuration_digest(recording: Recording) -> str:
    """The manifest's own digest, read out of the manifest rather than rebuilt.

    Parsed with a string search instead of ``json`` for one reason: if the
    field is ever absent, the drawing should say so rather than raise while
    encoding somebody's run.
    """
    key = '"configuration_digest"'
    start = recording.manifest_json.find(key)
    if start < 0:
        return "absent"
    opening = recording.manifest_json.find('"', start + len(key))
    closing = recording.manifest_json.find('"', opening + 1)
    if opening < 0 or closing < 0:
        return "absent"
    return recording.manifest_json[opening + 1:closing]
