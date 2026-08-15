#!/usr/bin/env python3
# Copyright 2026 The SlipX Authors
# SPDX-License-Identifier: Apache-2.0
"""Render a ghost race as an animated SVG.

    ./build/examples/cpp/slipx_ghost_race /tmp/race
    python3 examples/ghost_race_figure.py /tmp/race

Reads the three CSVs `slipx_ghost_race` writes and draws them. The only
dependency is the standard library, and the only input is the recording: this
script has no vehicle model, no track geometry and no opinion about either.

This is NOT the SVG sink, and the difference matters. `slipx.sinks.svg_sink`
implements the sink protocol over a `Recording` and deliberately draws one car
per file, because overlaying traces whose relationship nobody has defined is a
picture that asserts something the library does not (ADR-0028). A ghost race
is the case where the relationship IS defined, because the cars share a track,
a clock and a barrier and cannot interact. So the multi-car drawing lives here
in an example, where it can say so, rather than being smuggled into the sink.

What it draws is only what was recorded. There is no invented kerb, no racing
line and no grid marking, for the same reason (ADR-0024).
"""

import csv
import math
import os
import sys
from collections import defaultdict

# The car, at the default VehicleParams: wheelbase 0.32 m (lf 0.16 + lr 0.16)
# and 0.24 m of track width. Drawn a little larger than both, as a shell is.
CAR_L, CAR_W = 0.42, 0.26

# How long the animation takes on screen. The recording is played at a fixed
# rate rather than compressed to a target duration, so a longer race is a
# longer animation and the speeds on screen stay comparable between figures.
PLAY_RATE = 1.0

STYLE = """
  :root {
    --paper:#ffffff; --fg:#14181d; --muted:#5b6672; --grid:#e3e7ec;
    --accent:#c2410c; --accent2:#1d4ed8; --ok:#15803d;
    --road:#e7ebf0; --panel:#f7f9fb;
  }
  @media (prefers-color-scheme: dark) {
    :root {
      --paper:#11161c; --fg:#e8ecf1; --muted:#93a0ad; --grid:#2a323b;
      --accent:#fb923c; --accent2:#7aa7ff; --ok:#4ade80;
      --road:#222b34; --panel:#161c23;
    }
  }
  .card { fill: var(--paper); }
  .panel { fill: var(--panel); stroke: var(--grid); stroke-width: 1; }
  .road { fill: var(--road); stroke: var(--fg); stroke-width: 1.8; }
  .mut { stroke: var(--muted); stroke-width: 1.4; fill: none; }
  .bar { fill: var(--accent2); }
  .barb { fill: var(--accent); }
  text { font-family: ui-sans-serif, -apple-system, "Segoe UI", Roboto,
         "Helvetica Neue", Arial, sans-serif; fill: var(--fg); }
  .t  { font-size: 13px; }
  .ts { font-size: 11.5px; fill: var(--muted); }
  .tb { font-size: 15px; font-weight: 600; }
  .th { font-size: 21px; font-weight: 600; }
  .tm { font-size: 12px; fill: var(--muted); }
  .tn { font-size: 10.5px; fill: var(--muted); }
"""


def read_walls(directory):
    left, right = [], []
    with open(os.path.join(directory, "ghost_race_walls.csv"),
              newline="") as fh:
        for row in csv.DictReader(fh):
            (left if row["wall"] == "left" else right).append(
                (float(row["x"]), float(row["y"])))
    return left, right


def read_states(directory):
    by_agent = defaultdict(list)
    with open(os.path.join(directory, "ghost_race_states.csv"),
              newline="") as fh:
        for row in csv.DictReader(fh):
            by_agent[int(row["agent"])].append(
                (float(row["t"]), float(row["x"]), float(row["y"]),
                 float(row["yaw"]), float(row["speed"])))
    return by_agent


def read_laps(directory):
    times = defaultdict(list)
    speeds = {}
    with open(os.path.join(directory, "ghost_race_laps.csv"),
              newline="") as fh:
        for row in csv.DictReader(fh):
            agent = int(row["agent"])
            times[agent].append(float(row["lap_time"]))
            speeds[agent] = float(row["target_speed"])
    return times, speeds


def closest_approach(states):
    """The nearest two cars ever came, over the recorded frames.

    Measured rather than asserted, because it is the number that says what a
    contact model would have had to arbitrate. It is a sampled minimum: the
    recording is 25 Hz and two cars could pass closer between frames.
    """
    best = float("inf")
    when = 0.0
    frames = len(next(iter(states.values())))
    agents = sorted(states)
    for f in range(frames):
        for i, a in enumerate(agents):
            for b in agents[i + 1:]:
                if f >= len(states[a]) or f >= len(states[b]):
                    continue
                gap = math.hypot(states[a][f][1] - states[b][f][1],
                                 states[a][f][2] - states[b][f][2])
                if gap < best:
                    best, when = gap, states[a][f][0]
    return best, when


def hue_for(agent, count):
    """A distinct hue per car. Lightness is fixed at a value that reads on
    both a light and a dark card, which is why this is not a palette lookup:
    twenty entries chosen by hand would drift."""
    return f"hsl({(agent * 360.0 / count) % 360:.0f} 70% 52%)"


class View:
    """A window onto the world, in metres, mapped into an SVG box."""

    def __init__(self, x, y, w, h, bounds, pad=18):
        self.x, self.y, self.w, self.h = x, y, w, h
        (x0, x1), (y0, y1) = bounds
        span_x = max(x1 - x0, 1e-6)
        span_y = max(y1 - y0, 1e-6)
        self.scale = min((w - 2 * pad) / span_x, (h - 2 * pad) / span_y)
        self.cx, self.cy = (x0 + x1) / 2.0, (y0 + y1) / 2.0

    def px(self, wx):
        return self.x + self.w / 2 + (wx - self.cx) * self.scale

    def py(self, wy):
        # World y is up (ISO 8855); SVG y is down.
        return self.y + self.h / 2 - (wy - self.cy) * self.scale

    def pt(self, wx, wy):
        return self.px(wx), self.py(wy)


def road_path(view, left, right):
    outer = "M " + " L ".join(f"{view.px(x):.2f} {view.py(y):.2f}"
                              for x, y in left) + " Z"
    inner = "M " + " L ".join(f"{view.px(x):.2f} {view.py(y):.2f}"
                              for x, y in reversed(right)) + " Z"
    return f'<path class="road" fill-rule="evenodd" d="{outer} {inner}"/>'


def car_defs(view, count):
    lx, wy = CAR_L * view.scale, CAR_W * view.scale
    nose = max(1.6, lx * 0.2)
    out = []
    for agent in range(count):
        colour = hue_for(agent, count)
        out.append(
            f'<g id="car{agent}">'
            f'<rect x="{-lx / 2:.2f}" y="{-wy / 2:.2f}" width="{lx:.2f}" '
            f'height="{wy:.2f}" rx="{min(2.2, wy / 3):.2f}" fill="{colour}"/>'
            f'<rect x="{lx / 2 - nose:.2f}" y="{-wy / 2 + 0.9:.2f}" '
            f'width="{nose:.2f}" height="{max(1.2, wy - 1.8):.2f}" '
            f'fill="var(--paper)" opacity="0.85"/></g>')
    return "".join(out)


def main():
    directory = sys.argv[1] if len(sys.argv) > 1 else "."
    output = sys.argv[2] if len(sys.argv) > 2 else os.path.join(
        directory, "ghost_race.svg")

    left, right = read_walls(directory)
    states = read_states(directory)
    lap_times, speeds = read_laps(directory)
    count = len(states)
    if count == 0:
        raise SystemExit("no states recorded: run slipx_ghost_race first")

    frames = len(states[0])
    duration = states[0][-1][0] if frames else 0.0
    play = max(duration / PLAY_RATE, 0.1)
    gap, gap_t = closest_approach(states)

    xs = [point[0] for point in left + right]
    ys = [point[1] for point in left + right]
    bounds = ((min(xs), max(xs)), (min(ys), max(ys)))

    W, H = 1080, 760
    parts = []
    parts.append('<text class="th" x="24" y="36">A ghost race</text>')
    parts.append(
        f'<text class="tm" x="24" y="58">{count} cars, {len(lap_times[0])} '
        f'laps of paddock_stadium, recorded by examples/cpp/ghost_race_main'
        f'.cpp. They cannot touch: there is no contact model, so this is '
        f'{count} time trials sharing a track and a clock, not a race.</text>')

    # ------------------------------------------------------------ the track
    track = View(24, 76, 700, 560, bounds)
    parts.append(f'<clipPath id="trackclip"><rect x="{track.x}" '
                 f'y="{track.y}" width="{track.w}" height="{track.h}" '
                 f'rx="6"/></clipPath>')
    parts.append(f'<rect class="panel" x="{track.x}" y="{track.y}" '
                 f'width="{track.w}" height="{track.h}" rx="6"/>')
    parts.append('<g clip-path="url(#trackclip)">')
    parts.append(road_path(track, left, right))
    parts.append(f'<defs>{car_defs(track, count)}</defs>')

    for agent in sorted(states):
        path = states[agent]
        xs_ = ";".join(f"{track.px(f[1]):.2f},{track.py(f[2]):.2f}"
                       for f in path)
        # SVG rotates clockwise with y down, so a left turn (yaw increasing,
        # ISO 8855) is a decreasing screen angle.
        rots = ";".join(f"{-math.degrees(f[3]):.2f}" for f in path)
        parts.append(
            f'<g><use href="#car{agent}"/>'
            f'<animateTransform attributeName="transform" type="translate" '
            f'values="{xs_}" dur="{play:.2f}s" calcMode="linear" '
            f'repeatCount="indefinite"/>'
            f'<animateTransform attributeName="transform" type="rotate" '
            f'values="{rots}" dur="{play:.2f}s" calcMode="linear" '
            f'repeatCount="indefinite" additive="sum"/></g>')

    parts.append('</g>')

    # A clock, so the animation is readable as time rather than as motion.
    parts.append(f'<text class="ts" x="{track.x + 14}" '
                 f'y="{track.y + track.h - 14}">'
                 f'{duration:.1f} s of simulation, played once through, '
                 f'looping</text>')
    bar_x, bar_w = track.x + 14, track.w - 28
    bar_y = track.y + 20
    parts.append(f'<line class="mut" x1="{bar_x}" y1="{bar_y}" '
                 f'x2="{bar_x + bar_w}" y2="{bar_y}" stroke-width="2"/>')
    parts.append(f'<circle r="4.5" fill="var(--accent)" cy="{bar_y}">'
                 f'<animate attributeName="cx" values="{bar_x};'
                 f'{bar_x + bar_w}" dur="{play:.2f}s" '
                 f'repeatCount="indefinite"/></circle>')

    # ------------------------------------------------------ the lap times
    panel_x, panel_w = 740, 316
    parts.append(f'<rect class="panel" x="{panel_x}" y="76" '
                 f'width="{panel_w}" height="560" rx="6"/>')
    parts.append(f'<text class="tb" x="{panel_x + 14}" y="101">'
                 f'Best lap</text>')
    parts.append(f'<text class="ts" x="{panel_x + 14}" y="119">'
                 f'a time trial ranking, which is the only result</text>')
    parts.append(f'<text class="ts" x="{panel_x + 14}" y="134">'
                 f'this simulator can honestly produce</text>')

    best = {a: min(t) for a, t in lap_times.items() if t}
    order = sorted(best, key=lambda a: best[a])
    fastest, slowest = min(best.values()), max(best.values())
    span = max(slowest - fastest, 1e-9)

    row_y, row_h = 156, 22
    for rank, agent in enumerate(order):
        y = row_y + rank * row_h
        colour = hue_for(agent, count)
        parts.append(f'<rect x="{panel_x + 14}" y="{y - 9}" width="9" '
                     f'height="9" rx="2" fill="{colour}"/>')
        parts.append(f'<text class="tn" x="{panel_x + 30}" y="{y}">'
                     f'car{agent}</text>')
        parts.append(f'<text class="tn" x="{panel_x + 72}" y="{y}">'
                     f'{speeds[agent]:.2f} m/s</text>')
        # The bar is the lap time above the fastest, so the scale is the
        # spread of the field rather than the time itself: at 11 to 15 s a
        # bar from zero would be twenty bars of the same length.
        width = 110 * (best[agent] - fastest) / span
        parts.append(f'<rect class="bar" x="{panel_x + 136}" y="{y - 8}" '
                     f'width="{max(width, 1.0):.1f}" height="8" rx="2" '
                     f'opacity="0.55"/>')
        parts.append(f'<text class="tn" x="{panel_x + panel_w - 14}" '
                     f'y="{y}" text-anchor="end">{best[agent]:.3f} s</text>')

    foot = row_y + len(order) * row_h + 16
    parts.append(f'<text class="tn" x="{panel_x + 14}" y="{foot}">'
                 f'Bars are time lost to the fastest car,</text>')
    parts.append(f'<text class="tn" x="{panel_x + 14}" y="{foot + 14}">'
                 f'not lap time from zero.</text>')

    # ------------------------------------------------------------ footnotes
    notes = [
        ("the controller", "pure pursuit against the centreline, from "
                           "examples/cpp/reference_stack.hpp. It is a "
                           "geometric controller and understeers as the tyres "
                           "start to matter, which is what it should do."),
        ("the spread", "the only difference between the cars is the target "
                       "speed. Same parameters, same line, same tyres, so the "
                       "ranking is the speed demand and nothing else."),
        ("what is missing", f"contact, race control, attribution and "
                            f"penalties are P3 (M7). Nothing here decides an "
                            f"overtake, and two cars in the same place would "
                            f"simply be in the same place."),
        ("how close it came", f"the closest two cars came in this run was "
                              f"{gap:.3f} m centre to centre, at t = "
                              f"{gap_t:.1f} s, against a {CAR_L} m car: "
                              f"{100 * (gap - CAR_L):.0f} cm of clear air, "
                              f"sampled at {len(states[0]) / duration:.0f} Hz."),
        ("provenance", "every parameter here is labelled provisional. Nothing "
                       "in this picture has been measured against a car."),
    ]
    y = 668
    parts.append(f'<line class="mut" x1="24" y1="{y - 16}" x2="1056" '
                 f'y2="{y - 16}" opacity="0.35"/>')
    for i, (key, value) in enumerate(notes):
        parts.append(f'<text class="ts" x="24" y="{y + i * 16}">'
                     f'<tspan fill="var(--fg)">{key}</tspan>&#160;&#160;'
                     f'{value}</text>')

    svg = (f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" '
           f'height="{H}" viewBox="0 0 {W} {H}" role="img" '
           f'aria-label="A ghost race: {count} cars lapping the shipped '
           f'track">\n<title>A ghost race</title>\n'
           f'<style>{STYLE}</style>\n'
           f'<rect class="card" x="0" y="0" width="{W}" height="{H}" '
           f'rx="10"/>\n' + "\n".join(parts) + "\n</svg>\n")

    with open(output, "w", encoding="utf-8") as fh:
        fh.write(svg)
    print(f"wrote {output} ({len(svg) / 1024:.0f} kB, {count} cars, "
          f"{frames} frames, {duration:.1f} s)")


if __name__ == "__main__":
    main()
