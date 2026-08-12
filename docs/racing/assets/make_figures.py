#!/usr/bin/env python3
# Copyright 2026 The SlipX Authors
# SPDX-License-Identifier: Apache-2.0
"""Render the diagrams for the autonomous racing tutorial series.

    python3 docs/racing/assets/make_figures.py

Writes one SVG per figure into this directory. The only dependency is SlipX
itself, which is the point: every plotted curve is evaluated by `slipx_core`
through the Python bindings rather than by a second tyre model living here. A
diagram nobody can regenerate goes stale the first time a number in it is
questioned; a diagram regenerated from a model that is not the library's goes
wrong more quietly than that.

Figures carry labels, not paragraphs. Anything that needs a sentence belongs in
the article beside the figure, where it can be edited, translated and searched.

Two kinds of figure live here and they are not the same kind of claim.

SCHEMATICS (slip-angle, load-transfer-*, vehicle-models, understeer-oversteer,
racing-line) are geometry. They show how quantities are defined and how a
picture is labelled, and there is no model behind them to be right or wrong.
The left panel of differential-speeds is one too: it is a pair of concentric
arcs and nothing else.

PLOTS are computed rather than drawn. The tyre plots (tyre-curve,
load-sensitivity, peak-location, friction-ellipse, combined-slip, gg-diagram,
speed-profile, cross-tier-crossover) come out of `slipx_core` at the
parameters of the reference car in `examples/cars/reference_1_10`. The
drivetrain and actuator plots (differential-speeds' right panel, torque-speed,
servo-step) are closed-form expressions written out below at plausible values,
because the quantities they show are not tyre curves and the tier does not
expose them pointwise.

Either way the numbers are ILLUSTRATIVE: no parameter set SlipX ships has been
measured against a vehicle, and a figure being consistent with the library is
not a figure being right about a car.
"""

import math
import os
import sys

try:
    import slipx
except ImportError as exc:  # pragma: no cover - an install problem, not a bug
    raise SystemExit(
        "make_figures.py evaluates every plotted curve with slipx_core "
        "through the Python bindings, so `import slipx` has to work. Build "
        "the extension in place with `make build`, and run this script from "
        "the repository root."
    ) from exc

OUT_DIR = os.path.dirname(os.path.abspath(__file__))

# --------------------------------------------------------------------- style
#
# Every figure carries its own stylesheet and its own background card, so it
# renders legibly whether the page around it is light or dark and whether or
# not the viewer honours prefers-color-scheme. The card is not decoration: an
# SVG with a transparent background and dark strokes disappears entirely on a
# dark page, which is the single most common way a repository diagram breaks.

STYLE = """
  :root {
    --paper:#ffffff; --fg:#14181d; --muted:#5b6672; --grid:#e3e7ec;
    --accent:#c2410c; --accent2:#1d4ed8; --ok:#15803d; --warn:#a16207;
    --body:#cfd6de; --road:#eef1f5;
  }
  @media (prefers-color-scheme: dark) {
    :root {
      --paper:#11161c; --fg:#e8ecf1; --muted:#93a0ad; --grid:#2a323b;
      --accent:#fb923c; --accent2:#7aa7ff; --ok:#4ade80; --warn:#eab308;
      --body:#2b333c; --road:#1a2027;
    }
  }
  .card { fill: var(--paper); }
  .grid { stroke: var(--grid); stroke-width: 1; fill: none; }
  .axis { stroke: var(--muted); stroke-width: 1.3; fill: none; }
  .mut  { stroke: var(--muted); stroke-width: 1.6; fill: none; }
  .fg   { stroke: var(--fg); stroke-width: 1.6; fill: none; }
  .body { fill: var(--body); stroke: var(--muted); stroke-width: 1.1; }
  .road { fill: var(--road); stroke: var(--grid); stroke-width: 1.1; }
  .a1   { stroke: var(--accent);  stroke-width: 2.2; fill: none; }
  .a2   { stroke: var(--accent2); stroke-width: 2.2; fill: none; }
  .ok   { stroke: var(--ok);      stroke-width: 2.2; fill: none; }
  .a1f  { fill: var(--accent);  stroke: none; }
  .a2f  { fill: var(--accent2); stroke: none; }
  .okf  { fill: var(--ok);      stroke: none; }
  .dash { stroke-dasharray: 7 5; }
  .dot  { stroke-dasharray: 2 4; }
  .thin { stroke-width: 1.2; }
  text  { font-family: ui-sans-serif, -apple-system, "Segoe UI", Roboto,
          "Helvetica Neue", Arial, sans-serif; fill: var(--fg); }
  .t    { font-size: 13px; }
  .ts   { font-size: 11.5px; fill: var(--muted); }
  .tb   { font-size: 15px; font-weight: 600; }
  .tm   { font-size: 12px; fill: var(--muted); }
  .k1   { font-size: 12px; fill: var(--accent); }
  .k2   { font-size: 12px; fill: var(--accent2); }
  .k3   { font-size: 12px; fill: var(--ok); }
  .sb   { font-size: 0.72em; }
"""

ARROWS = """
  <marker id="ah" viewBox="0 0 10 10" refX="8.5" refY="5" markerWidth="7"
          markerHeight="7" orient="auto-start-reverse">
    <path d="M 0 0 L 10 5 L 0 10 z" fill="context-stroke"/>
  </marker>
"""


def sub(s, tail=""):
    """A real subscript.

    Unicode's Latin subscript block has no 'y' and no 'z', so the obvious
    approach of pasting a codepoint gives Fₐ or Fₖ where Fy was meant. It is a
    silent error: the glyph renders, it just says something else. A tspan says
    exactly what it means and works in every renderer.
    """
    out = f'<tspan class="sb" dy="3">{s}</tspan>'
    if tail:
        out += f'<tspan dy="-3">{tail}</tspan>'
    return out


FX = "F" + sub("x")
FY = "F" + sub("y")
FZ = "F" + sub("z")
AX = "a" + sub("x")
AY = "a" + sub("y")
ALPHA, DELTA, MU, KAPPA = "&#945;", "&#948;", "&#956;", "&#954;"


class Fig:
    """A minimal SVG canvas. y grows downward, as in SVG itself."""

    def __init__(self, w, h, title=""):
        self.w, self.h = w, h
        self.parts = []
        self.title = title

    def add(self, s):
        self.parts.append(s)
        return self

    def line(self, x1, y1, x2, y2, cls="fg", arrow=False):
        m = ' marker-end="url(#ah)"' if arrow else ""
        return self.add(f'<line class="{cls}" x1="{x1:.2f}" y1="{y1:.2f}" '
                        f'x2="{x2:.2f}" y2="{y2:.2f}"{m}/>')

    def path(self, d, cls="fg", arrow=False):
        m = ' marker-end="url(#ah)"' if arrow else ""
        return self.add(f'<path class="{cls}" d="{d}"{m}/>')

    def poly(self, pts, cls="fg", close=False, arrow=False):
        d = "M " + " L ".join(f"{x:.2f} {y:.2f}" for x, y in pts)
        if close:
            d += " Z"
        return self.path(d, cls, arrow)

    def circle(self, x, y, r, cls="a1f"):
        return self.add(f'<circle class="{cls}" cx="{x:.2f}" cy="{y:.2f}" '
                        f'r="{r:.2f}"/>')

    def rect(self, x, y, w, h, cls="body", rx=0):
        return self.add(f'<rect class="{cls}" x="{x:.2f}" y="{y:.2f}" '
                        f'width="{w:.2f}" height="{h:.2f}" rx="{rx}"/>')

    def rot_rect(self, x, y, w, h, ang, cls="body", rx=3):
        return self.add(f'<rect class="{cls}" x="{x - w / 2:.2f}" '
                        f'y="{y - h / 2:.2f}" width="{w}" height="{h}" '
                        f'rx="{rx}" transform="rotate({ang} {x:.2f} '
                        f'{y:.2f})"/>')

    def text(self, x, y, s, cls="t", anchor=None, rot=None):
        a = f' text-anchor="{anchor}"' if anchor else ""
        t = f' transform="rotate({rot} {x:.2f} {y:.2f})"' if rot else ""
        return self.add(f'<text class="{cls}" x="{x:.2f}" y="{y:.2f}"{a}{t}>'
                        f'{s}</text>')

    def head(self, title, subtitle=""):
        self.text(22, 28, title, "tb")
        if subtitle:
            self.text(22, 47, subtitle, "tm")
        return self

    def arc(self, cx, cy, r, a0, a1, cls="fg", arrow=False):
        """Arc by centre and degree range, in screen coordinates (y down)."""
        x0 = cx + r * math.cos(math.radians(a0))
        y0 = cy + r * math.sin(math.radians(a0))
        x1 = cx + r * math.cos(math.radians(a1))
        y1 = cy + r * math.sin(math.radians(a1))
        large = 1 if abs(a1 - a0) > 180 else 0
        sweep = 1 if a1 > a0 else 0
        return self.path(f"M {x0:.2f} {y0:.2f} A {r:.2f} {r:.2f} 0 "
                         f"{large} {sweep} {x1:.2f} {y1:.2f}", cls, arrow)

    def save(self, name):
        head = (f'<svg xmlns="http://www.w3.org/2000/svg" width="{self.w}" '
                f'height="{self.h}" viewBox="0 0 {self.w} {self.h}" '
                f'role="img" aria-label="{self.title}">')
        svg = (f'{head}\n<title>{self.title}</title>\n<defs>{ARROWS}</defs>\n'
               f'<style>{STYLE}</style>\n'
               f'<rect class="card" x="0" y="0" width="{self.w}" '
               f'height="{self.h}" rx="10"/>\n' + "\n".join(self.parts) +
               "\n</svg>\n")
        with open(os.path.join(OUT_DIR, name), "w", encoding="utf-8") as fh:
            fh.write(svg)
        print(f"  {name}")


class Axes:
    """A plot box mapping data coordinates onto a Fig."""

    def __init__(self, fig, x0, y0, w, h, xlim, ylim):
        self.f, self.x0, self.y0, self.w, self.h = fig, x0, y0, w, h
        self.xlim, self.ylim = xlim, ylim

    def px(self, x):
        lo, hi = self.xlim
        return self.x0 + (x - lo) / (hi - lo) * self.w

    def py(self, y):
        lo, hi = self.ylim
        return self.y0 + self.h - (y - lo) / (hi - lo) * self.h

    def pt(self, x, y):
        return self.px(x), self.py(y)

    def frame(self, xticks, yticks, xfmt="{:g}", yfmt="{:g}"):
        f = self.f
        for t in xticks:
            f.line(self.px(t), self.y0, self.px(t), self.y0 + self.h, "grid")
        for t in yticks:
            f.line(self.x0, self.py(t), self.x0 + self.w, self.py(t), "grid")
        # Axis lines sit at zero when zero is inside the range. A y axis drawn
        # at the left of a plot whose data straddles zero invites reading the
        # sign off the wrong line.
        ax = self.px(0) if self.xlim[0] < 0 < self.xlim[1] else self.x0
        ay = self.py(0) if self.ylim[0] < 0 < self.ylim[1] else self.y0 + self.h
        f.line(self.x0, ay, self.x0 + self.w, ay, "axis")
        f.line(ax, self.y0, ax, self.y0 + self.h, "axis")
        for t in xticks:
            if t == 0:
                continue
            f.text(self.px(t), ay + 16, xfmt.format(t), "ts", "middle")
        for t in yticks:
            if t == 0:
                continue
            f.text(ax - 7, self.py(t) + 4, yfmt.format(t), "ts", "end")
        return self

    def curve(self, xs, ys, cls="a1", arrow=False):
        return self.f.poly([self.pt(x, y) for x, y in zip(xs, ys)], cls,
                           arrow=arrow)

    def clipped(self, xs, ys, cls="a1"):
        """Draw only the part of a curve that lies inside the y range."""
        run = []
        for x, y in zip(xs, ys):
            if self.ylim[0] <= y <= self.ylim[1]:
                run.append(self.pt(x, y))
            elif run:
                self.f.poly(run, cls)
                run = []
        if run:
            self.f.poly(run, cls)
        return self


# ------------------------------------------------------------ the tyre model
#
# There is no tyre model in this file. Every curve below is evaluated by
# `slipx_core` through the Python bindings, at the parameters of the reference
# car that ships in the repository, so a figure here and a run of the library
# cannot disagree. They used to: this file carried its own Magic Formula with
# its own coefficients, and when the reference car's cornering stiffness was
# corrected (ADR-0032) the two described tyres that peaked twenty degrees
# apart.
#
# The reference car's parameters are PROVISIONAL and describe no measured
# vehicle. That caveat is unchanged and is the one thing about these figures
# that has not moved: they are now consistent as well as illustrative, which is
# not the same as being measured.

G = 9.80665                       # slipx::kGravity

_CAR = slipx.load_reference_car()
_PARAMS = _CAR.params_for_tier(slipx.Tier.L2_DoubleTrack)

#: Static vertical load on one tyre, from the car's own mass and its 50/50
#: weight distribution. This is the load MF-lite states its coefficients at.
FZ_NOM = _PARAMS.mass * G / 4.0

#: The front tyre, built the way the tier builds it: the stiffness factor B
#: derived from the cornering stiffness and this load, never read from a file.
TYRE = slipx.make_mf_lite(_PARAMS.tyre_front, _PARAMS.c_alpha_f / 2.0, FZ_NOM)

MU_Y0 = TYRE.mu_y0                # peak lateral friction at FZ_NOM      [-]
MU_X0 = TYRE.mu_x0                # peak longitudinal friction           [-]
K_MU = TYRE.k_mu                  # load sensitivity exponent            [-]


def mu_y(fz):
    """Load-sensitive peak friction: mu falls as the tyre is loaded."""
    return slipx.peak_lateral_force(TYRE, fz) / fz


def fy_mf(alpha, fz=FZ_NOM):
    """Lateral force from MF-lite, ISO 8855 sign, straight from the library.

    A positive slip angle means the tyre is running to the left of where it
    points, and the force it makes opposes that, so it is negative. Under SAE
    the slip angle carries the other sign and the same curve is written
    without the minus.
    """
    return slipx.mf_lite_fy(TYRE, alpha, fz)


def c_alpha(fz=FZ_NOM):
    """Cornering stiffness: the slope of the curve at the origin, positive."""
    return slipx.cornering_stiffness_at_load(TYRE, fz)


def tyre_shaped(shape_c, curvature_e):
    """The same tyre with different shape factors, built by the library.

    Used only by the figure about where a peak lands, which needs a family of
    curves that share a cornering stiffness and a peak force and differ in
    nothing else. Deriving B here rather than in the figure keeps that
    derivation in one place, and it is the library's.
    """
    coefficients = slipx.TyreCoefficients()
    coefficients.mu_y0 = MU_Y0
    coefficients.mu_x0 = MU_X0
    coefficients.k_mu = K_MU
    coefficients.shape_c = shape_c
    coefficients.curvature_e = curvature_e
    return slipx.make_mf_lite(coefficients, c_alpha(), FZ_NOM)


def frange(lo, hi, n):
    return [lo + (hi - lo) * i / (n - 1) for i in range(n)]


def spline(points):
    """Catmull-Rom through the given points, as an SVG cubic path.

    Used for the racing lines, which are schematic: a real minimum-time line
    has continuously varying radius and comes out of an optimiser, not out of
    a drawing. What this has to show correctly is where the apex falls and how
    straight the exit is, and a spline through waypoints taken off the track
    geometry shows exactly that without pretending to be an optimal result.
    """
    p = [points[0]] + list(points) + [points[-1]]
    d = f"M {p[1][0]:.2f} {p[1][1]:.2f}"
    for i in range(1, len(p) - 2):
        p0, p1, p2, p3 = p[i - 1], p[i], p[i + 1], p[i + 2]
        c1 = (p1[0] + (p2[0] - p0[0]) / 6.0, p1[1] + (p2[1] - p0[1]) / 6.0)
        c2 = (p2[0] - (p3[0] - p1[0]) / 6.0, p2[1] - (p3[1] - p1[1]) / 6.0)
        d += (f" C {c1[0]:.2f} {c1[1]:.2f}, {c2[0]:.2f} {c2[1]:.2f}, "
              f"{p2[0]:.2f} {p2[1]:.2f}")
    return d


# ============================================================ 1. slip angle

def fig_slip_angle():
    f = Fig(640, 300, "Slip angle: the wheel points one way and travels "
                      "another")
    f.head("Slip angle", "Top view of one wheel. ISO 8855, so y is to the "
                         "left and a positive " + ALPHA + " makes a negative "
                         + FY + ".")

    cx, cy, ang = 300, 180, -20

    r = math.radians(ang)
    f.line(cx - 190 * math.cos(r), cy - 190 * math.sin(r),
           cx + 150 * math.cos(r), cy + 150 * math.sin(r), "fg thin dash")
    f.text(cx - 200 * math.cos(r), cy - 200 * math.sin(r) - 6,
           "wheel plane: where it points", "ts", "middle")
    f.rot_rect(cx, cy, 88, 27, ang)

    va = math.radians(ang + 17)
    f.line(cx, cy, cx + 190 * math.cos(va), cy + 190 * math.sin(va),
           "a2", arrow=True)
    f.text(cx + 200 * math.cos(va), cy + 190 * math.sin(va) + 16,
           "v: where it goes", "k2", "middle")

    f.arc(cx, cy, 104, ang, ang + 17, "a1")
    f.text(cx + 128 * math.cos(math.radians(ang + 8.5)) - 4,
           cy + 128 * math.sin(math.radians(ang + 8.5)) + 5, ALPHA, "k1")

    fa = math.radians(ang - 90)
    f.line(cx, cy, cx + 112 * math.cos(fa), cy + 112 * math.sin(fa),
           "ok", arrow=True)
    f.text(cx + 124 * math.cos(fa) - 10, cy + 124 * math.sin(fa), FY, "k3")

    f.text(44, 268, ALPHA + " = atan2(v" + sub("lat", ", v")
           + sub("lon", ")") + " &#8722; " + DELTA, "t")
    f.text(160, 268, "no slip angle, no lateral force", "ts")
    f.save("slip-angle.svg")


# ============================================================ 2. tyre curve

def fig_tyre_curve():
    f = Fig(640, 400, "Lateral force against slip angle")
    f.head("The tyre curve", "One tyre, constant vertical load. Magnitude "
                             "plotted; the ISO sign of " + FY + " is "
                             "negative.")

    ax = Axes(f, 66, 82, 500, 262, (0, 20), (0, 11.4))
    ax.frame([0, 5, 10, 15, 20], [0, 2, 4, 6, 8, 10])
    f.text(316, 380, "slip angle " + ALPHA + "   [degrees]", "ts", "middle")
    f.text(20, 213, "|" + FY + "|   [N]", "ts", "middle", rot=-90)

    degs = frange(0.0, 20.0, 260)
    ys = [abs(fy_mf(math.radians(d))) for d in degs]
    peak_i = max(range(len(ys)), key=lambda i: ys[i])
    xp, yp = degs[peak_i], ys[peak_i]

    # The linear model, for comparison: the tangent at the origin, which is
    # what a bicycle model with a linear tyre believes all the way up.
    ax.clipped(degs, [c_alpha() * math.radians(d) for d in degs], "a2 dash")
    ax.curve(degs, ys, "a1")

    f.line(*ax.pt(xp, 0), *ax.pt(xp, yp), "grid dot")
    f.circle(*ax.pt(xp, yp), 4.5, "a1f")
    f.text(*ax.pt(xp + 0.5, yp + 0.55), f"peak, at {xp:.1f}&#176;", "k1")

    f.text(*ax.pt(2.6, 1.6), "linear region", "ts")
    f.line(*ax.pt(2.4, 1.9), *ax.pt(1.1, 3.0), "mut thin")
    f.text(*ax.pt(14.6, 10.9), "falling branch", "ts", "middle")
    f.line(*ax.pt(11.0, 10.5), *ax.pt(18.4, 10.5), "mut thin")
    f.text(*ax.pt(4.4, 7.1), "linear model", "k2")
    f.text(*ax.pt(4.4, 6.4), "(no peak, so no spin)", "ts")
    f.line(*ax.pt(4.2, 7.4), *ax.pt(2.7, 9.6), "a2 thin")
    f.save("tyre-curve.svg")


# ====================================================== 3. load sensitivity

def fig_load_sensitivity():
    f = Fig(640, 330, "Load sensitivity: grip per newton falls as load rises")
    f.head("Load sensitivity", "A tyre carrying twice the load does not make "
                               "twice the grip.")

    a1 = Axes(f, 62, 92, 220, 190, (0, 20), (0.8, 1.4))
    a1.frame([5, 10, 15, 20], [0.9, 1.0, 1.1, 1.2, 1.3], yfmt="{:.1f}")
    fzs = frange(1.2, 20.0, 140)
    a1.curve(fzs, [mu_y(z) for z in fzs], "a1")
    f.text(172, 82, "peak friction " + MU, "ts", "middle")
    f.text(172, 308, "vertical load " + FZ + "   [N]", "ts", "middle")
    f.line(*a1.pt(FZ_NOM, 0.8), *a1.pt(FZ_NOM, mu_y(FZ_NOM)), "grid dot")
    f.circle(*a1.pt(FZ_NOM, mu_y(FZ_NOM)), 4, "a1f")
    f.text(*a1.pt(FZ_NOM + 0.8, mu_y(FZ_NOM) + 0.04), "nominal load", "ts")

    a2 = Axes(f, 392, 92, 220, 190, (0, 20), (0, 24))
    a2.frame([5, 10, 15, 20], [5, 10, 15, 20])
    a2.clipped(frange(0, 20, 60), [MU_Y0 * z for z in frange(0, 20, 60)],
               "a2 dash")
    a2.curve(fzs, [mu_y(z) * z for z in fzs], "a1")
    f.text(502, 82, "peak lateral force " + MU + FZ, "ts", "middle")
    f.text(502, 308, "vertical load " + FZ + "   [N]", "ts", "middle")
    f.text(*a2.pt(8.2, 21.4), "constant " + MU, "k2")
    f.text(*a2.pt(12.4, 10.2), "actual", "k1")
    f.save("load-sensitivity.svg")


# ======================================================= 4. where a peak lands

def fig_peak_location():
    """Three tyres with the same slope and the same peak, peaking elsewhere.

    The point of the figure is a fact that is invisible until you try to fit
    the Magic Formula: once the cornering stiffness and the peak friction are
    fixed by measurement, B is no longer free, and C and E between them decide
    one remaining thing, which is how far out the peak sits.

    Each curve here is built by DERIVING B from the shared cornering stiffness,
    exactly as an identification would, so all three leave the origin along the
    same tangent and all three reach the same height. Nothing else about them
    is the same.
    """
    f = Fig(640, 400, "Where the peak lands, for the same slope and peak")
    f.head("C and E decide where the peak lands",
           "Same cornering stiffness, same peak force. Only C and E differ.")

    ca = c_alpha()                       # shared, as a skidpad would give it
    peak_force = slipx.peak_lateral_force(TYRE, FZ_NOM)
    alpha_lin = peak_force / ca          # the linear tyre's crossing point

    ax = Axes(f, 66, 92, 500, 250, (0, 30), (0, 11.4))
    ax.frame([0, 5, 10, 15, 20, 25, 30], [0, 2, 4, 6, 8, 10])
    f.text(316, 380, "slip angle " + ALPHA + "   [degrees]", "ts", "middle")
    f.text(20, 217, "|" + FY + "|   [N]", "ts", "middle", rot=-90)

    degs = frange(0.0, 30.0, 400)

    # (C, E, css, label). The last pair is legal under the SlipX tyre schema
    # and is not a tyre: its peak is out where no car goes.
    # The third pair is legal under the SlipX tyre schema, which bounds C and E
    # independently, and is not a tyre: its peak is out where no car goes.
    cases = [(1.90, 0.00, "ok", "okf", "k3"),
             (1.50, -0.20, "a1", "a1f", "k1"),
             (1.43, 0.87, "a2", "a2f", "k2")]

    # The legend sits in the empty lower right of the plot. Its rows are placed
    # in data coordinates so that moving the axis limits moves them with it.
    row_y = [3.5, 2.4, 1.3]

    for (shape_c, curve_e, cls, dot, txt), ly in zip(cases, row_y):
        # A real tyre each time, built by the library from the shared
        # cornering stiffness. B is derived there and not here, which is what
        # makes "same slope, same peak" a property of the construction rather
        # than of this loop.
        variant = tyre_shaped(shape_c, curve_e)
        ys = [abs(slipx.mf_lite_fy(variant, math.radians(d), FZ_NOM))
              for d in degs]
        ax.curve(degs, ys, cls)

        i = max(range(len(ys)), key=lambda j: ys[j])
        ratio = math.radians(degs[i]) / alpha_lin
        f.circle(*ax.pt(degs[i], ys[i]), 4.5, dot)

        f.line(*ax.pt(11.5, ly), *ax.pt(13.5, ly), cls)
        f.text(*ax.pt(14.3, ly - 0.28), f"C = {shape_c:g}, E = {curve_e:g}",
               txt)
        f.text(*ax.pt(22.4, ly - 0.28),
               f"peak at {degs[i]:.0f}&#176; = {ratio:.1f}&#215; "
               + ALPHA + sub("lin"), "ts")

    # The shared tangent, and the slip angle where it would have reached the
    # peak. That angle is the natural yardstick: every peak above is a multiple
    # of it.
    ax.clipped(degs, [ca * math.radians(d) for d in degs], "mut dash thin")
    f.line(*ax.pt(math.degrees(alpha_lin), 0),
           *ax.pt(math.degrees(alpha_lin), peak_force), "grid dot")
    f.text(*ax.pt(math.degrees(alpha_lin) + 0.5, 1.2), ALPHA + sub("lin")
           + f" = {math.degrees(alpha_lin):.1f}&#176;", "ts")
    f.text(*ax.pt(0.6, 10.6), "shared tangent, slope C" + sub("&#945;"), "ts")
    f.save("peak-location.svg")


# ======================================================= 5. friction ellipse

def fig_relaxation():
    """The same transient plotted against time and against distance.

    The whole point of the relaxation length is that the second panel collapses
    the two speeds onto one curve and the first does not, so the two panels are
    the argument and the caption only names them.
    """
    SIGMA = 0.08          # relaxation length                              [m]
    SPEEDS = (3.0, 12.0)  # slow and fast                                [m/s]

    f = Fig(880, 400, "Tyre relaxation, in time and in distance")
    f.head("Grip arrives late, and how late depends on speed",
           f"first-order lag, relaxation length {SIGMA} m, illustrative")

    # --- left: response against time -------------------------------------
    a = Axes(f, 70, 90, 330, 240, (0.0, 0.06), (0.0, 1.05))
    a.frame([0, 0.02, 0.04, 0.06], [0, 0.5, 1.0])
    f.text(235, 368, "time after the steering step [s]", "ts", "middle")
    f.text(70, 78, "fraction of the steady-state " + FY, "ts")

    for speed, cls in zip(SPEEDS, ("a1", "a2")):
        ts = frange(0.0, 0.06, 300)
        ys = [1.0 - math.exp(-t * speed / SIGMA) for t in ts]
        a.curve(ts, ys, cls)

    f.line(a.px(0), a.py(1.0), a.px(0.06), a.py(1.0), "mut thin dash")
    f.text(a.px(0.06) - 4, a.py(1.0) - 7, "steady state", "ts", "end")

    f.text(a.px(0.026), a.py(0.88), "12 m/s", "k2")
    f.text(a.px(0.030), a.py(0.55), "3 m/s", "k1")

    # --- right: response against distance rolled -------------------------
    b = Axes(f, 500, 90, 330, 240, (0.0, 0.30), (0.0, 1.05))
    b.frame([0, 0.1, 0.2, 0.3], [0, 0.5, 1.0])
    f.text(665, 368, "distance rolled since the step [m]", "ts", "middle")
    f.text(500, 78, "the same two runs, one curve", "ts")

    ss = frange(0.0, 0.30, 300)
    ys = [1.0 - math.exp(-d / SIGMA) for d in ss]
    b.curve(ss, ys, "ok")

    f.line(b.px(0), b.py(1.0), b.px(0.30), b.py(1.0), "mut thin dash")
    f.line(b.px(SIGMA), b.y0, b.px(SIGMA), b.y0 + b.h, "mut thin dot")
    f.text(b.px(SIGMA) + 6, b.py(0.20), "one " + "&#963;", "ts")
    f.line(b.px(0), b.py(0.632), b.px(SIGMA), b.py(0.632), "mut thin dot")
    f.text(b.px(SIGMA) - 8, b.py(0.632) - 8, "63%", "ts", "end")

    f.save("relaxation.svg")


def fig_friction_ellipse():
    f = Fig(560, 400, "The friction ellipse: one budget, two demands")
    f.head("The friction ellipse", "Braking and cornering spend the same "
                                   "contact patch.")

    cx, cy, rx, ry = 268, 224, 158, 108   # rx is mu_y Fz, ry is mu_x Fz

    f.add(f'<ellipse class="a1" cx="{cx}" cy="{cy}" rx="{rx}" ry="{ry}"/>')
    f.line(cx - rx - 42, cy, cx + rx + 46, cy, "axis")
    f.line(cx, cy - ry - 46, cx, cy + ry + 46, "axis")
    f.text(cx + rx + 54, cy + 4, FY, "ts")
    f.text(cx + 12, cy - ry - 52, FX, "ts")
    # No left/right on the lateral axis. In ISO 8855 positive Fy points left,
    # so labelling the right of the plot "right" would be exactly wrong, and
    # labelling it "left" reads as an error. Direction is not what this figure
    # is about: the budget is.
    f.text(cx - 12, cy - ry - 52, "drive", "ts", "end")
    f.text(cx - 12, cy + ry + 50, "brake", "ts", "end")

    f.line(cx, cy, cx + rx, cy, "a2", arrow=True)
    f.circle(cx + rx, cy, 4.5, "a2f")
    f.text(cx + rx - 14, cy - 14, "pure cornering", "k2", "end")

    t = math.radians(56)
    px, py = cx + rx * math.cos(t), cy + ry * math.sin(t)
    f.line(cx, cy, px, py, "ok", arrow=True)
    f.circle(px, py, 4.5, "okf")
    f.line(px, py, px, cy, "grid dot")
    f.line(px, py, cx, py, "grid dot")
    f.text(px + 14, py + 6, "combined:", "k3")
    f.text(px + 14, py + 21, "braking and cornering", "ts")

    f.circle(cx - 74, cy + 38, 4.5, "a1f")
    f.text(cx - 84, cy + 42, "grip in hand", "ts", "end")
    f.save("friction-ellipse.svg")


def fig_combined_slip():
    """Clipping each axis against projecting onto the budget.

    The geometry is asserted rather than eyeballed. A figure whose whole point
    is that one construction lands on the boundary and the other does not is
    worthless if the drawn point misses the curve, and three versions of the
    racing line figure looked plausible and were wrong.
    """
    fy_max = slipx.peak_lateral_force(TYRE, FZ_NOM)
    fx_max = slipx.peak_longitudinal_force(TYRE, FZ_NOM)
    # Each legal on its own axis, with room to spare on both: the figure's
    # claim is that two individually-fine demands are jointly impossible, and
    # a demand sitting at 96% of one axis muddles that with "nearly too much".
    fy_demand, fx_demand = 8.0, -7.0

    u, v = fy_demand / fy_max, fx_demand / fx_max
    radius = math.hypot(u, v)
    assert abs(u) < 1.0 and abs(v) < 1.0, "the demand must clear both axes"
    assert radius > 1.0, "the demand must lie outside the ellipse"
    assert abs(math.hypot(u / radius, v / radius) - 1.0) < 1e-12

    # The kept pair is the library's, not this file's: the figure claims a
    # specific thing about what SlipX does when a demand leaves the ellipse,
    # so it had better be what SlipX does.
    kept = slipx.friction_ellipse(fx_demand, fy_demand, fx_max, fy_max)
    assert kept.saturated
    assert abs(kept.fy - fy_demand / radius) < 1e-12
    demand = math.hypot(fy_demand, fx_demand)
    kept_y, kept_x = kept.fy, -kept.fx

    f = Fig(640, 470, "Clipping each axis against projecting onto the budget")
    f.head("Combined slip: two ways to run out of grip",
           "Each demand is legal on its own axis. Together they are not.")

    cx, cy, rx, ry = 270, 230, 150, 118

    def at(un, vn):
        return cx + rx * un, cy - ry * vn

    f.add(f'<ellipse class="a1" cx="{cx}" cy="{cy}" rx="{rx}" ry="{ry}"/>')
    f.line(cx - rx - 40, cy, cx + rx + 40, cy, "axis")
    f.line(cx, cy - ry - 36, cx, cy + ry + 40, "axis")
    f.text(cx + rx + 46, cy + 22, FY, "ts")
    f.text(cx + 12, cy - ry - 42, FX, "ts")
    f.text(cx - 14, cy + ry + 30, "brake", "ts", "end")

    dx, dy = at(u, v)
    px, py = at(u / radius, v / radius)

    # The per-axis limits the naive rule checks against, and passes.
    f.line(dx, cy, dx, dy, "grid dot")
    f.line(cx, dy, dx, dy, "grid dot")
    f.text(dx + 6, cy - 10, f"{fy_demand:.1f} N of {fy_max:.1f}", "k2")
    f.text(cx - 8, dy + 4, f"{-fx_demand:.1f} N of {fx_max:.1f}", "k2", "end")

    # Green from the origin to the boundary, blue for the part beyond it. The
    # two demands point the same way, so drawing both from the origin would
    # draw one arrow on top of the other and hide the only thing in question,
    # which is the overshoot.
    f.line(cx, cy, px, py, "ok", arrow=True)
    f.line(px, py, dx, dy, "a2 dash", arrow=True)
    f.circle(dx, dy, 5, "a2f")
    f.circle(px, py, 5, "okf")
    f.text(dx + 14, dy + 10, "clipped per axis", "k2")
    f.text(dx + 14, dy + 26, f"asks {demand:.1f} N of the patch", "ts")
    f.text(cx, cy + ry + 56, "projected onto the boundary", "k3", "middle")
    f.text(cx, cy + ry + 72, f"{kept_y:.1f} N and {kept_x:.1f} N, "
                             f"in the direction asked for", "ts", "middle")

    f.text(22, 448,
           "Both components scale by the same factor, so the direction "
           "survives and the magnitude does not.", "ts")
    f.save("combined-slip.svg")


# ================================================ 5. longitudinal transfer

def fig_load_transfer_long():
    f = Fig(640, 320, "Longitudinal load transfer under braking")
    f.head("Longitudinal load transfer", "Side view, braking. The lever is "
                                         "the CoG height; the base is the "
                                         "wheelbase.")

    gy, x_r, x_f = 232, 176, 436
    cog_x, cog_y = 0.5 * (x_r + x_f) + 4, gy - 62

    f.line(76, gy, 566, gy, "axis")
    f.rect(x_r - 4, gy - 92, x_f - x_r + 8, 46, "body", rx=8)
    for x in (x_r, x_f):
        f.circle(x, gy - 21, 21, "body")

    f.circle(cog_x, cog_y, 6.5, "a1f")
    f.text(cog_x - 12, cog_y + 5, "CoG", "ts", "end")

    f.line(cog_x, cog_y, cog_x, cog_y + 46, "fg", arrow=True)
    f.text(cog_x + 8, cog_y + 42, "m g", "ts")
    f.line(cog_x, cog_y, cog_x + 92, cog_y, "a1", arrow=True)
    f.text(cog_x + 46, cog_y - 10, "inertia", "k1", "middle")

    f.line(x_f, gy, x_f, gy - 122, "ok", arrow=True)
    f.text(x_f + 12, gy - 118, "front gains", "k3")
    f.line(x_r, gy, x_r, gy - 84, "a2", arrow=True)
    f.text(x_r - 12, gy - 80, "rear loses", "k2", "end")

    f.line(x_f + 62, gy, x_f + 62, cog_y, "grid")
    f.line(x_f + 56, gy, x_f + 68, gy, "grid")
    f.line(x_f + 56, cog_y, x_f + 68, cog_y, "grid")
    f.text(x_f + 74, (gy + cog_y) / 2 + 4, "h", "ts")
    f.line(x_r, gy + 26, x_f, gy + 26, "grid")
    f.line(x_r, gy + 20, x_r, gy + 32, "grid")
    f.line(x_f, gy + 20, x_f, gy + 32, "grid")
    f.text((x_r + x_f) / 2, gy + 44, "L", "ts", "middle")

    f.text(76, 300, "&#916;" + FZ + " = m " + AX + " h / L", "t")
    f.save("load-transfer-long.svg")


# ===================================================== 6. lateral transfer

def fig_load_transfer_lat():
    f = Fig(640, 320, "Lateral load transfer in a corner")
    f.head("Lateral load transfer", "Rear view, cornering left. Same "
                                    "mechanism; the track replaces the "
                                    "wheelbase.")

    gy, x_l, x_r = 236, 210, 420
    cog_x, cog_y = 0.5 * (x_l + x_r), gy - 66

    f.line(112, gy, 520, gy, "axis")
    for x in (x_l, x_r):
        f.rect(x - 13, gy - 44, 26, 44, "body", rx=4)
    f.rect(x_l + 16, gy - 98, x_r - x_l - 32, 48, "body", rx=8)

    f.circle(cog_x, cog_y, 6.5, "a1f")
    f.text(cog_x - 12, cog_y + 5, "CoG", "ts", "end")

    f.line(cog_x, cog_y, cog_x, cog_y + 48, "fg", arrow=True)
    f.text(cog_x + 8, cog_y + 44, "m g", "ts")
    # ISO: cornering left is positive ay, so the inertial force acts to the
    # right and the right-hand wheels are the loaded ones.
    f.line(cog_x, cog_y, cog_x + 84, cog_y, "a1", arrow=True)
    f.text(cog_x + 42, cog_y - 10, "inertia", "k1", "middle")

    f.line(x_r, gy, x_r, gy - 124, "ok", arrow=True)
    f.text(x_r + 12, gy - 120, "outer gains", "k3")
    f.line(x_l, gy, x_l, gy - 74, "a2", arrow=True)
    f.text(x_l - 12, gy - 70, "inner loses", "k2", "end")

    f.line(x_r + 62, gy, x_r + 62, cog_y, "grid")
    f.line(x_r + 56, gy, x_r + 68, gy, "grid")
    f.line(x_r + 56, cog_y, x_r + 68, cog_y, "grid")
    f.text(x_r + 74, (gy + cog_y) / 2 + 4, "h", "ts")
    f.line(x_l, gy + 26, x_r, gy + 26, "grid")
    f.line(x_l, gy + 20, x_l, gy + 32, "grid")
    f.line(x_r, gy + 20, x_r, gy + 32, "grid")
    f.text(cog_x, gy + 44, "t", "ts", "middle")

    f.text(112, 300, "&#916;" + FZ + " = m " + AY + " h / t", "t")
    f.text(250, 300, "inner wheel reaches zero load at " + AY + " = g t / 2h",
           "ts")
    f.save("load-transfer-lat.svg")


# ====================================================== 7. the model ladder

def fig_vehicle_models():
    f = Fig(700, 330, "Three vehicle models of increasing fidelity")
    f.head("The modelling ladder", "Each rung adds a mechanism, and a reason "
                                   "not to trust the rung below it.")

    oy = 186

    def wheel(x, y, ang):
        f.rot_rect(x, y, 12, 30, ang)

    ox = 116
    f.text(ox, 92, "Kinematic bicycle", "t", "middle")
    f.text(ox, 110, "4 states", "ts", "middle")
    f.line(ox, oy + 52, ox, oy - 52, "fg thin")
    wheel(ox, oy + 52, 0)
    wheel(ox, oy - 52, -22)
    f.line(ox, oy, ox + 70, oy - 38, "a2", arrow=True)
    f.text(ox + 40, oy - 46, "v", "k2")
    f.text(ox, oy + 100, "velocity follows the", "ts", "middle")
    f.text(ox, oy + 115, "wheels; no tyres at all", "ts", "middle")

    ox = 350
    f.text(ox, 92, "Dynamic bicycle", "t", "middle")
    f.text(ox, 110, "6 states", "ts", "middle")
    f.line(ox, oy + 52, ox, oy - 52, "fg thin")
    wheel(ox, oy + 52, 0)
    wheel(ox, oy - 52, -22)
    f.line(ox, oy, ox + 70, oy - 22, "a2", arrow=True)
    f.text(ox + 42, oy - 30, "v", "k2")
    f.line(ox, oy - 52, ox - 44, oy - 52, "ok", arrow=True)
    f.line(ox, oy + 52, ox - 32, oy + 52, "ok", arrow=True)
    f.text(ox - 50, oy - 58, FY, "k3", "end")
    f.text(ox, oy + 100, "one tyre per axle, making", "ts", "middle")
    f.text(ox, oy + 115, "force from slip", "ts", "middle")

    ox = 586
    f.text(ox, 92, "Double track", "t", "middle")
    f.text(ox, 110, "~15 states", "ts", "middle")
    f.rect(ox - 24, oy - 46, 48, 92, "body", rx=8)
    for dx, dy, a, r in ((-38, -52, -22, 5.0), (38, -52, -22, 8.5),
                         (-38, 52, 0, 5.5), (38, 52, 0, 9.0)):
        wheel(ox + dx, oy + dy, a)
        f.circle(ox + dx, oy + dy, r, "a1f")
    f.text(ox, oy + 100, "four tyres, four loads;", "ts", "middle")
    f.text(ox, oy + 115, "disc size is " + FZ, "ts", "middle")
    f.save("vehicle-models.svg")


# ================================================= 8. understeer/oversteer

def fig_understeer_oversteer():
    f = Fig(560, 340, "Understeer, neutral steer and oversteer")
    f.head("Understeer and oversteer", "One steering angle, held. Three cars, "
                                       "the same distance travelled.")

    # Equal ARC LENGTH, not equal sweep angle. The three cars cover the same
    # ground; the only difference is how much of it went into turning. Sweeping
    # each by the same angle instead would make the understeering car travel
    # furthest, which confuses "ran wide" with "went faster".
    sx, sy, arc_len = 380, 300, 230.0
    for r, cls, dotcls, lab in ((132.0, "a2", "a2f", "oversteer, tighter"),
                                (196.0, "ok", "okf", "neutral, as asked"),
                                (296.0, "a1", "a1f", "understeer, wider")):
        sweep = math.degrees(arc_len / r)
        f.arc(sx - r, sy, r, 0, -sweep, cls, arrow=True)
        ex = sx - r + r * math.cos(math.radians(-sweep))
        ey = sy - r * math.sin(math.radians(sweep))
        f.circle(ex, ey, 4, dotcls)
        f.text(ex + 20, ey - 12, lab, "ts")

    f.circle(sx, sy, 5, "a1f")
    f.text(sx - 12, sy + 5, "same start, same " + DELTA, "ts", "end")
    f.save("understeer-oversteer.svg")


# ======================================================== 9. the racing line

def fig_racing_line():
    f = Fig(660, 470, "The geometric line and the late apex")
    f.head("Two lines through one corner", "Track width is radius, and radius "
                                           "is speed.")

    # A ninety degree left-hander. The car comes up the entry straight on the
    # right, turns left, and leaves along the exit straight at the top left.
    # The inside of a left turn is the smaller radius from the corner centre.
    xc, yc, ri, ro = 410, 320, 88, 170
    y_bot, x_left = 452, 60

    for radius in (ri, ro):
        pts = [(xc + radius, y_bot)]
        pts += [(xc + radius * math.cos(math.radians(-90 * k / 36.0)),
                 yc + radius * math.sin(math.radians(-90 * k / 36.0)))
                for k in range(37)]
        pts.append((x_left, yc - radius))
        f.poly(pts, "fg thin")

    f.text(x_left + 18, yc - ro - 14, "exit straight", "ts")
    f.text(xc + ro + 14, y_bot - 34, "entry", "ts")
    f.line(xc + ro + 26, y_bot - 16, xc + ro + 26, y_bot - 62, "mut thin",
           arrow=True)

    def arc_pts(cx2, cy2, r, a0, a1, n=48):
        return [(cx2 + r * math.cos(math.radians(a0 + (a1 - a0) * k / n)),
                 cy2 + r * math.sin(math.radians(a0 + (a1 - a0) * k / n)))
                for k in range(n + 1)]

    # --- the geometric line: one circle, and it is fully determined.
    # Tangent to the outer edge of the entry straight (x = xc + ro), tangent to
    # the outer edge of the exit straight (y = yc - ro), and touching the inner
    # boundary at the geometric middle of the corner. Out, in, out.
    x_ent, y_ext = xc + ro, yc - ro
    apex_g = (xc + ri * math.cos(math.radians(-45)),
              yc + ri * math.sin(math.radians(-45)))
    d = x_ent - apex_g[0]                       # = apex_g[1] - y_ext, by symmetry
    r_geo = d * math.sqrt(2.0) / (math.sqrt(2.0) - 1.0)
    cg = (x_ent - r_geo, y_ext + r_geo)
    a_start = math.degrees(math.atan2(y_bot - cg[1],
                                      math.sqrt(max(r_geo ** 2 -
                                                    (y_bot - cg[1]) ** 2, 0.0))))
    geo = arc_pts(cg[0], cg[1], r_geo, a_start, -90)
    geo.append((x_left, y_ext))
    f.poly(geo, "a2 dash")

    # --- the late apex: two arcs, tight then opening.
    # A single circle cannot express a late apex, because the whole point of one
    # is that the radius TIGHTENS on entry and OPENS on exit. Two arcs sharing a
    # tangent at the apex is the textbook construction and is monotone by
    # design, which the naive single-circle drawing is not.
    th = math.radians(-68)                       # apex, later round the corner
    apex_l = (xc + ri * math.cos(th), yc + ri * math.sin(th))
    n_in = (-math.cos(th), -math.sin(th))        # unit vector, apex towards C

    # Entry radius set so the car turns in from hard against the outer edge.
    r1 = (x_ent - apex_l[0]) / (1.0 + n_in[0])
    c1 = (apex_l[0] + r1 * n_in[0], apex_l[1] + r1 * n_in[1])
    r2 = 420.0                                   # exit radius, deliberately open
    c2 = (apex_l[0] + r2 * n_in[0], apex_l[1] + r2 * n_in[1])

    late = [(x_ent, y_bot)]
    late += arc_pts(c1[0], c1[1], r1, 0, math.degrees(th))
    late += arc_pts(c2[0], c2[1], r2, math.degrees(th), -90)
    late.append((x_left, c2[1] - r2))
    f.poly(late, "a1")

    # Both lines are checked against the track rather than eyeballed.
    def check(line, name):
        for i, (x, y) in enumerate(line):
            if x >= xc and y >= yc:
                ok = xc + ri - 1 <= x <= xc + ro + 1        # entry straight
            elif x <= xc and y <= yc:
                ok = yc - ro - 1 <= y <= yc - ri + 1        # exit straight
            else:
                r = math.hypot(x - xc, y - yc)
                ok = ri - 1 <= r <= ro + 1                  # the corner
            assert ok, f"{name}: point {i} {(x, y)} is off the track"
            if i:
                assert (x <= line[i - 1][0] + 0.5
                        and y <= line[i - 1][1] + 0.5), \
                    f"{name}: doubles back at point {i}"

    check(geo, "geometric")
    check(late, "late apex")

    f.circle(*apex_g, 5, "a2f")
    f.text(apex_g[0] + 14, apex_g[1] + 6, "geometric apex", "k2")
    f.circle(*apex_l, 5, "a1f")
    f.text(apex_l[0] - 16, apex_l[1] - 14, "late apex", "k1", "end")

    f.text(52, 316, "geometric", "k2")
    f.text(52, 334, "the largest radius that fits between", "ts")
    f.text(52, 349, "the boundaries, so the highest", "ts")
    f.text(52, 364, "constant speed through the corner", "ts")
    f.text(52, 396, "late apex", "k1")
    f.text(52, 414, "slower in, but the exit straightens", "ts")
    f.text(52, 429, "sooner, so the following straight", "ts")
    f.text(52, 444, "is driven faster all the way down", "ts")
    f.save("racing-line.svg")


# ========================================================= 10. g-g diagram

def fig_gg_diagram():
    f = Fig(600, 430, "The g-g diagram and one corner traced through it")
    f.head("The g-g diagram", "Everything the car can do, and the path one "
                              "corner traces around it.")

    ax = Axes(f, 168, 96, 286, 286, (-16, 16), (-16, 16))
    ax.frame([-10, -5, 5, 10], [-10, -5, 5, 10])
    f.text(311, 410, "lateral " + AY + "   [m/s&#178;]", "ts", "middle")
    f.text(120, 239, "longitudinal " + AX + "   [m/s&#178;]", "ts", "middle",
           rot=-90)
    f.text(311, 86, "drive", "ts", "middle")
    f.text(311, 396, "brake", "ts", "middle")

    mu_g, mux_g, accel_max = MU_Y0 * G, MU_X0 * G, 8.0

    # The grip ellipse, then the envelope the car can actually reach: the same
    # ellipse with a flat top, because forward acceleration runs out of motor
    # before it runs out of grip.
    ell = [(mu_g * math.cos(math.radians(a)), mux_g * math.sin(math.radians(a)))
           for a in frange(0, 360, 241)]
    ax.curve([p[0] for p in ell], [p[1] for p in ell], "mut dash thin")
    ax.curve([p[0] for p in ell], [min(p[1], accel_max) for p in ell], "a1")
    f.text(*ax.pt(-15.8, accel_max + 1.0), "drivetrain limit", "k1")
    f.text(*ax.pt(-15.8, -8.6), "grip limit", "ts")
    f.line(*ax.pt(-11.6, -9.2), *ax.pt(-8.4, -8.0), "mut thin")

    # One left-hand corner: straight-line braking, trail braking in, apex at
    # pure lateral, then unwinding onto the power. Drawn inside the envelope,
    # because no real lap sits exactly on it.
    k = 0.88
    trace = [(0, -mux_g * k)]
    for a in frange(-90, 0, 40):
        trace.append((mu_g * math.cos(math.radians(a)) * k,
                      mux_g * math.sin(math.radians(a)) * k))
    for a in frange(0, 90, 40):
        trace.append((mu_g * math.cos(math.radians(a)) * k,
                      min(accel_max * k,
                          mux_g * math.sin(math.radians(a)) * 1.6)))
    ax.curve([p[0] for p in trace], [p[1] for p in trace], "a2", arrow=True)

    f.circle(*ax.pt(0, -mux_g * k), 4, "a2f")
    f.text(*ax.pt(-1.4, -mux_g * k - 1.1), "brake", "k2", "end")
    f.circle(*ax.pt(mu_g * k, 0), 4, "a2f")
    f.text(*ax.pt(mu_g * k + 0.8, 1.2), "apex", "k2")
    f.circle(*ax.pt(0, accel_max * k), 4, "a2f")
    f.text(*ax.pt(-1.4, accel_max * k - 1.2), "full throttle", "k2", "end")
    f.save("gg-diagram.svg")


# ====================================================== 11. speed profile

def fig_speed_profile():
    f = Fig(660, 350, "Building a speed profile with a forward and a "
                      "backward pass")
    f.head("From curvature to a speed profile", "Grip sets the limit. Motor "
                                                "and brakes decide how much "
                                                "of it you reach.")

    n, s_end = 400, 100.0
    s = [s_end * i / (n - 1) for i in range(n)]

    def curvature(x):
        # Two corners: a fast one and a much tighter one. Radii of about 2.2 m
        # and 0.9 m, which is the range a 1/10 car actually turns in.
        k = 0.0
        for centre, width, peak in ((30.0, 5.0, 0.45), (68.0, 3.5, 1.10)):
            k += peak * math.exp(-((x - centre) / width) ** 2)
        return k

    v_cap = 11.0
    v_grip = [min(v_cap, math.sqrt(MU_Y0 * G / k) if k > 1e-6 else v_cap)
              for k in map(curvature, s)]

    ds, a_acc, a_brk = s[1] - s[0], 6.0, 11.0
    fwd = list(v_grip)
    for i in range(1, n):
        fwd[i] = min(fwd[i], math.sqrt(fwd[i - 1] ** 2 + 2 * a_acc * ds))
    bwd = list(fwd)
    for i in range(n - 2, -1, -1):
        bwd[i] = min(bwd[i], math.sqrt(bwd[i + 1] ** 2 + 2 * a_brk * ds))

    ax = Axes(f, 68, 92, 448, 192, (0, 100), (0, 12.6))
    ax.frame([20, 40, 60, 80, 100], [2, 4, 6, 8, 10, 12])
    f.text(292, 318, "distance along the track s   [m]", "ts", "middle")
    f.text(22, 190, "speed   [m/s]", "ts", "middle", rot=-90)

    ax.curve(s, v_grip, "mut dash thin")
    ax.curve(s, fwd, "ok dot")
    ax.curve(s, bwd, "a2")

    f.text(*ax.pt(1.5, 11.9), "grip limit  &#8730;(" + MU + "g/" + KAPPA
           + ")", "ts")
    f.text(*ax.pt(37, 7.4), "forward pass", "k3")
    f.text(*ax.pt(78, 6.0), "final profile", "k2")
    f.text(*ax.pt(50, 2.0), "braking zone", "ts", "middle")
    f.line(*ax.pt(56, 2.4), *ax.pt(62, 4.4), "a2 thin")

    f.text(532, 132, "the tighter corner", "ts")
    f.text(532, 147, "needs the longer", "ts")
    f.text(532, 162, "braking zone, drawn", "ts")
    f.text(532, 177, "backwards from it", "ts")
    f.save("speed-profile.svg")


# ================================================== 12. differential geometry

# One axle, one corner. The two panels are the same fact stated twice: a
# geometric speed difference the axle has to absorb somehow, and the slip
# ratio a locked axle is therefore forced into.

TRACK = 0.24        # track width of a 1/10-scale car                     [m]
KAPPA_PEAK = 0.10   # where a rubber tyre's longitudinal force peaks      [-]


def fig_differential_speeds():
    f = Fig(880, 400, "Why a driven axle needs a differential")
    f.head("One axle, two path radii",
           f"track {TRACK} m; the outer wheel travels the longer arc")

    # --- left: the geometry ----------------------------------------------
    #
    # A quarter turn about a centre placed at the panel's bottom-left corner.
    # Scale is chosen so a 1 m corner radius fills the panel; the two arcs are
    # therefore drawn to scale, and the visible gap between them IS the track
    # width, not an exaggeration for clarity.
    cx, cy, scale = 62.0, 372.0, 240.0
    r_mid = 1.0                                 # corner radius            [m]
    r_in = (r_mid - TRACK / 2) * scale
    r_out = (r_mid + TRACK / 2) * scale
    assert abs(r_out - r_in - TRACK * scale) < 1e-9, "arcs must differ by t"
    assert cy - r_out > 90.0, "the outer arc must stay inside the panel"

    f.arc(cx, cy, r_in, -90, 0, "mut thin dash")
    f.arc(cx, cy, r_out, -90, 0, "mut thin dash")

    theta = math.radians(-45.0)
    ix, iy = cx + r_in * math.cos(theta), cy + r_in * math.sin(theta)
    ox, oy = cx + r_out * math.cos(theta), cy + r_out * math.sin(theta)

    f.line(cx, cy, ox + 26 * math.cos(theta), oy + 26 * math.sin(theta),
           "grid thin")
    f.line(cx, cy, cx + (r_out + 26), cy, "grid thin")
    f.line(cx, cy, cx, cy - (r_out + 26), "grid thin")

    # The axle, straddling both arcs, with a wheel at each end. The wheels sit
    # square to the axle, which is the tangent direction at this point.
    tangent = math.degrees(theta) + 90.0
    f.line(ix, iy, ox, oy, "fg")
    f.rot_rect(ix, iy, 9, 22, tangent, "body")
    f.rot_rect(ox, oy, 9, 22, tangent, "body")
    f.circle(cx, cy, 3.5, "a1f")

    f.text(cx + 8, cy - 8, "turn centre", "ts")
    f.text(ix - 10, iy + 16, "inner wheel", "k1", "end")
    f.text(ox + 12, oy - 8, "outer wheel", "k2")

    # The two radii, labelled where the arcs cross the vertical radius: the
    # annulus is widest to read there and nothing else is drawn in it.
    f.text(cx + 10, cy - r_out - 8, "R + t/2", "ts")
    f.text(cx + 10, cy - r_in - 8, "R &#8722; t/2", "ts")

    ratio = (r_mid + TRACK / 2) / (r_mid - TRACK / 2)
    f.text(312, 200, f"R = {r_mid:g} m:", "ts")
    f.text(312, 219, "the outer wheel must turn", "ts")
    f.text(312, 238, f"{100 * (ratio - 1):.0f}% faster than the inner", "k2")

    # --- right: the slip ratio a locked axle forces -----------------------
    a = Axes(f, 545, 100, 300, 230, (0.0, 6.0), (0.0, 0.30))
    a.frame([1, 2, 3, 4, 5, 6], [0.05, 0.10, 0.15, 0.20, 0.25, 0.30])
    f.text(695, 366, "corner radius R   [m]", "ts", "middle")
    f.text(545, 88, "slip ratio " + KAPPA + " forced on each wheel of a "
                    "locked axle", "ts")

    rs = frange(0.42, 6.0, 400)
    a.clipped(rs, [TRACK / (2 * r) for r in rs], "a1")

    f.line(a.px(0), a.py(KAPPA_PEAK), a.px(6.0), a.py(KAPPA_PEAK),
           "mut thin dash")
    f.text(a.px(6.0), a.py(KAPPA_PEAK) - 8, "tyre's peak " + KAPPA, "ts",
           "end")

    r_cross = TRACK / (2 * KAPPA_PEAK)
    assert abs(TRACK / (2 * r_cross) - KAPPA_PEAK) < 1e-12
    f.circle(*a.pt(r_cross, KAPPA_PEAK), 4.5, "a1f")
    f.line(a.px(r_cross), a.py(KAPPA_PEAK), a.px(r_cross), a.y0 + a.h,
           "mut thin dot")
    f.text(a.px(r_cross) + 8, a.py(0.235),
           f"tighter than {r_cross:.1f} m and a", "ts")
    f.text(a.px(r_cross) + 8, a.py(0.215),
           "locked axle is past the peak", "ts")

    f.save("differential-speeds.svg")


# ====================================================== 13. the torque budget

# A permanent-magnet DC drive, stated at the wheel so no gear ratio appears.
# Illustrative, like every plot here.

T_STALL = 2.0       # wheel torque at zero speed, full pack             [N m]
V_FREE = 24.0       # road speed at which the drive torque reaches zero [m/s]
T_CURRENT = 1.2     # the current limit, expressed as wheel torque      [N m]
V_SAG = 0.85        # a depleted-and-sagging pack, as a voltage fraction  [-]


def drive_torque(v, s=1.0):
    """The torque-speed line at voltage fraction s, capped by current."""
    line = T_STALL * s * (1.0 - v / (V_FREE * s))
    return max(0.0, min(line, T_CURRENT))


def fig_torque_speed():
    f = Fig(880, 400, "The torque a small electric drivetrain can deliver")
    f.head("Torque, power, and the two things that limit them",
           "a current limit clips the launch; the pack voltage scales "
           "everything")

    # --- left: torque against speed --------------------------------------
    a = Axes(f, 70, 100, 330, 230, (0.0, 26.0), (0.0, 2.2))
    a.frame([5, 10, 15, 20, 25], [0.5, 1.0, 1.5, 2.0])
    f.text(235, 366, "road speed   [m/s]", "ts", "middle")
    f.text(70, 88, "wheel torque   [N m]", "ts")

    vs = frange(0.0, 26.0, 400)
    # The uncapped line, so the reader can see what the cap removes.
    a.clipped(vs, [T_STALL * (1.0 - v / V_FREE) for v in vs], "mut thin dot")
    a.curve(vs, [drive_torque(v) for v in vs], "a1")
    a.curve(vs, [drive_torque(v, V_SAG) for v in vs], "a2 dash")

    v_knee = V_FREE * (1.0 - T_CURRENT / T_STALL)
    assert abs(T_STALL * (1.0 - v_knee / V_FREE) - T_CURRENT) < 1e-12
    f.circle(*a.pt(v_knee, T_CURRENT), 4.5, "a1f")
    f.text(a.px(0.6), a.py(T_CURRENT) + 16, "current limit", "k1")
    f.text(a.px(v_knee) + 8, a.py(T_CURRENT) + 4,
           f"{v_knee:.1f} m/s", "ts")
    f.text(a.px(13.5), a.py(1.02), "full pack", "k1")
    f.text(a.px(9.0), a.py(0.30), f"pack at {100 * V_SAG:.0f}% voltage", "k2")

    # --- right: the same thing as power ----------------------------------
    b = Axes(f, 545, 100, 300, 230, (0.0, 26.0), (0.0, 280.0))
    b.frame([5, 10, 15, 20, 25], [50, 100, 150, 200, 250])
    f.text(695, 366, "road speed   [m/s]", "ts", "middle")
    f.text(545, 88, "mechanical power at the wheels   [W]", "ts")

    def power(v, s=1.0):
        return drive_torque(v, s) * v / 0.05      # T omega, wheel radius 5 cm

    b.curve(vs, [power(v) for v in vs], "a1")
    b.curve(vs, [power(v, V_SAG) for v in vs], "a2 dash")

    # Peak power of an uncapped line is at half the free speed. The cap is to
    # the left of that here, so it costs launch torque and no peak power at
    # all, which is the panel's whole point.
    v_peak = V_FREE / 2
    assert v_peak > v_knee, "the cap must not touch the power peak"
    p_full, p_sag = power(v_peak), power(V_SAG * v_peak, V_SAG)
    f.circle(*b.pt(v_peak, p_full), 4.5, "a1f")
    f.circle(*b.pt(V_SAG * v_peak, p_sag), 4.5, "a2f")
    f.text(b.px(v_peak) + 8, b.py(p_full) - 8, f"{p_full:.0f} W", "k1")
    f.text(b.px(V_SAG * v_peak) - 8, b.py(p_sag) - 12, f"{p_sag:.0f} W",
           "k2", "end")
    f.text(b.px(1.0), b.py(268.0),
           f"{100 * (1 - V_SAG):.0f}% less voltage, "
           f"{100 * (1 - p_sag / p_full):.0f}% less power", "ts")

    f.save("torque-speed.svg")


# ======================================================== 14. actuator lag

WN = 45.0          # servo natural frequency                          [rad/s]
ZETA = 0.7         # damping ratio                                        [-]
RATE_MAX = 10.0    # slew limit                                       [rad/s]
STEER_STEP = 0.40  # a full-travel steering command                     [rad]
SIGMA = 0.08       # tyre relaxation length, from article 8               [m]


def second_order_step(t, amp, wn, zeta):
    """Unit-step response of a linear second-order lag, underdamped."""
    wd = wn * math.sqrt(1.0 - zeta * zeta)
    return amp * (1.0 - math.exp(-zeta * wn * t) *
                  (math.cos(wd * t) +
                   zeta / math.sqrt(1.0 - zeta * zeta) * math.sin(wd * t)))


def peak_rate_factor(zeta):
    """Peak of the step response's derivative, in units of amplitude * wn.

    Differentiating the response above leaves a single sine lobe,

        dx/dt = A (wn / r) exp(-zeta wn t) sin(wd t),   r = sqrt(1 - zeta^2)

    whose peak sits where tan(wd t) = r / zeta. At that point sin(wd t) is
    exactly r, which cancels the leading 1/r and leaves a factor depending on
    the damping ratio alone. That is what makes "does the slew limit bind?"
    answerable without simulating anything: compare amplitude * wn * this
    against the limit.
    """
    r = math.sqrt(1.0 - zeta * zeta)
    return math.exp(-zeta * math.atan2(r, zeta) / r)


def fig_servo_step():
    f = Fig(880, 400, "The two limits of a steering actuator, and how they "
                      "compare with the tyre's own lag")
    f.head("A servo has two limits, and they look different",
           f"second-order lag at {WN:g} rad/s and damping {ZETA:g}, against "
           f"a {RATE_MAX:g} rad/s slew limit")

    # --- left: bandwidth-limited against slew-limited ---------------------
    a = Axes(f, 70, 100, 330, 230, (0.0, 0.20), (0.0, 0.46))
    a.frame([0.05, 0.10, 0.15, 0.20], [0.1, 0.2, 0.3, 0.4],
            xfmt="{:.2f}")
    f.text(235, 366, "time after the command   [s]", "ts", "middle")
    f.text(70, 88, "road wheel angle   [rad]", "ts")

    f.line(a.px(0), a.py(STEER_STEP), a.px(0.20), a.py(STEER_STEP),
           "mut thin dash")
    f.text(a.px(0.008), a.py(STEER_STEP) - 8, "commanded", "ts")

    # The soft servo's own peak rate stays inside the slew limit even for a
    # full-travel step, so this curve is honestly unlimited: drawing the
    # second-order response here would be wrong if it were not.
    assert STEER_STEP * WN * peak_rate_factor(ZETA) < RATE_MAX

    ts = frange(0.0, 0.20, 400)
    a.curve(ts, [second_order_step(t, STEER_STEP, WN, ZETA) for t in ts], "a1")

    # A stiffer servo asks for more rate than it has, so its angle is a
    # straight ramp at the limit until it arrives. Nothing curved about it.
    t_ramp = STEER_STEP / RATE_MAX
    a.curve([0.0, t_ramp, 0.20], [0.0, STEER_STEP, STEER_STEP], "a2")

    over = math.exp(-math.pi * ZETA / math.sqrt(1.0 - ZETA * ZETA))
    t_peak = math.pi / (WN * math.sqrt(1.0 - ZETA * ZETA))
    assert abs(second_order_step(t_peak, 1.0, WN, ZETA) - (1.0 + over)) < 1e-12
    f.circle(*a.pt(t_peak, STEER_STEP * (1 + over)), 4.5, "a1f")
    f.text(a.px(t_peak) + 8, a.py(STEER_STEP * (1 + over)) - 6,
           f"overshoot {100 * over:.1f}%", "k1")
    f.text(a.px(t_ramp) + 6, a.py(0.145), "slew-limited:", "k2")
    f.text(a.px(t_ramp) + 6, a.py(0.105), f"a straight {RATE_MAX:g} rad/s "
                                          "ramp", "k2")
    f.text(a.px(0.052), a.py(0.055), "bandwidth-limited", "k1")

    # --- right: which lag dominates, and where ---------------------------
    b = Axes(f, 545, 100, 300, 230, (0.0, 15.0), (0.0, 0.30))
    b.frame([3, 6, 9, 12, 15], [0.05, 0.10, 0.15, 0.20, 0.25, 0.30],
            yfmt="{:.2f}")
    f.text(695, 366, "speed   [m/s]", "ts", "middle")
    f.text(545, 88, "time to come within 5% of the command   [s]", "ts")

    # Both curves are the same threshold: the servo's envelope reaches 5% at
    # 3/(zeta wn), and the tyre reaches 95% of its force after three
    # relaxation lengths. Comparing a 2% number against a 95% one would make
    # the crossing an artefact of the definitions.
    t_servo = 3.0 / (ZETA * WN)
    b.curve([0.0, 15.0], [t_servo, t_servo], "a2")
    f.text(b.px(8.4), b.py(t_servo) - 10, "the servo, fixed in seconds", "k2")

    vs = frange(0.9, 15.0, 400)
    b.clipped(vs, [3.0 * SIGMA / v for v in vs], "a1")
    f.text(b.px(3.4), b.py(0.200), "the tyre, " + "3&#963;/v", "k1")

    v_cross = 3.0 * SIGMA / t_servo
    assert abs(3.0 * SIGMA / v_cross - t_servo) < 1e-12
    f.circle(*b.pt(v_cross, t_servo), 4.5, "a1f")
    f.line(*b.pt(v_cross + 0.4, t_servo - 0.008), *b.pt(5.0, 0.079),
           "mut thin")
    f.text(b.px(5.2), b.py(0.082),
           f"above {v_cross:.1f} m/s the servo", "ts")
    f.text(b.px(5.2), b.py(0.060), "is the slower of the two", "ts")

    f.save("servo-step.svg")



# ============================================== 15. the cross-tier crossover

# The released artefact of SRS 7: how far up the lateral-acceleration range the
# single-track and double-track tiers agree, measured rather than asserted.
# Both curves are rollouts of `slipx_core`, one per tier, at the reference
# car's own parameters.

# Finely spaced through the crossing, because the figure reports where the
# curve leaves the 1% band and a coarse grid would report the first sample
# past it instead. The README quotes this number, so the sweep has to resolve
# it rather than round it up to the next steer angle.
_CROSSOVER_STEERS = (0.005, 0.01, 0.02, 0.03, 0.045, 0.06, 0.075, 0.09,
                     0.10, 0.105, 0.11, 0.115, 0.12, 0.135, 0.15)
_CROSSOVER_SPEED = 5.0
_SETTLE_STEPS = 6000        # six seconds at 1 kHz, many yaw time constants


def _settled(params, tier, steer):
    """Hold a steer angle and a speed until the transient has died.

    Returns the settled path radius and lateral acceleration, which is the
    pair a skidpad measures and the pair the two tiers are compared on.
    """
    model = slipx.VehicleModel.create(tier, params)
    state = slipx.VehicleState()
    state.vel_body.x = _CROSSOVER_SPEED
    diagnostics = slipx.StepDiagnostics()
    for _ in range(_SETTLE_STEPS):
        model.step(state,
                   slipx.DriveInput(steer,
                                    slipx.hold_speed(state, _CROSSOVER_SPEED)),
                   0.001, diagnostics)
    return state.speed() / abs(state.rates.z), abs(diagnostics.ay)


def fig_cross_tier_crossover():
    """Where the single-track model stops being allowed to answer.

    Two differentials, because the reference car ships with a spool and the
    struct defaults are open, and the difference between them dwarfs the
    tyre effect this figure is otherwise about. Drawing only the shipped car
    would attribute a drivetrain disagreement to the tyre model.
    """
    open_params = _CAR.params_for_tier(slipx.Tier.L2_DoubleTrack)
    open_params.differential = slipx.Differential.Open
    spool_params = _CAR.params_for_tier(slipx.Tier.L2_DoubleTrack)
    assert spool_params.differential == slipx.Differential.Spool, (
        "the reference car is expected to ship a locked rear axle"
    )

    rows = []
    for steer in _CROSSOVER_STEERS:
        r1, _ = _settled(open_params, slipx.Tier.L1_Bicycle, steer)
        r_open, ay = _settled(open_params, slipx.Tier.L2_DoubleTrack, steer)
        r_spool, _ = _settled(spool_params, slipx.Tier.L2_DoubleTrack, steer)
        rows.append((ay, r1, r_open, r_spool,
                     100.0 * abs(r_open - r1) / r1,
                     100.0 * abs(r_spool - r1) / r1))

    ays = [row[0] for row in rows]
    assert ays == sorted(ays), "the sweep must be monotone in lateral g"

    f = Fig(880, 400, "Where the single-track and double-track tiers stop "
                      "agreeing")
    f.head("How far up the range one model can answer for the other",
           "Steady-state skidpad at 5 m/s, both tiers, reference car "
           "parameters (provisional)")

    # --- left: the radius each tier settles at ---------------------------
    a = Axes(f, 70, 100, 330, 230, (0.0, 11.0), (0.0, 9.0))
    a.frame([2, 4, 6, 8, 10], [2, 4, 6, 8])
    f.text(235, 366, "lateral acceleration   [m/s&#178;]", "ts",
           "middle")
    f.text(70, 88, "settled path radius   [m]", "ts")

    a.clipped(ays, [row[1] for row in rows], "mut dash")
    a.clipped(ays, [row[2] for row in rows], "a1")
    a.clipped(ays, [row[3] for row in rows], "a2")
    f.text(a.px(6.6), a.py(4.9), "L1, and L2 with an open diff", "ts")
    f.text(a.px(4.4), a.py(7.4), "L2, spool", "k2")

    # --- right: the disagreement, which is the actual claim --------------
    b = Axes(f, 545, 100, 300, 230, (0.0, 11.0), (0.0, 30.0))
    b.frame([2, 4, 6, 8, 10], [5, 10, 15, 20, 25, 30])
    f.text(695, 366, "lateral acceleration   [m/s&#178;]", "ts",
           "middle")
    f.text(545, 88, "disagreement in path radius   [%]", "ts")

    b.clipped(ays, [row[5] for row in rows], "a2")
    b.clipped(ays, [row[4] for row in rows], "a1")

    f.line(b.px(0), b.py(1.0), b.px(11.0), b.py(1.0), "mut thin dash")
    f.text(b.px(10.7), b.py(2.0), "1%", "ts", "end")

    # The crossing: the first sampled point above one per cent, reported
    # rather than assumed, because it moves whenever the tyre does.
    crossing = next((row[0] for row in rows if row[4] > 1.0), None)
    assert crossing is not None, "the sweep never leaves the agreement band"
    f.circle(*b.pt(crossing, 1.0), 4.5, "a1f")
    f.line(*b.pt(crossing, 1.6), *b.pt(crossing - 1.6, 4.4), "mut thin")
    f.text(b.px(0.4), b.py(9.6),
           f"open diff: past {crossing:.1f} m/s&#178; ({crossing / G:.2f} g)",
           "k1")
    f.text(b.px(0.4), b.py(7.2), "L1 stops answering for L2", "k1")

    f.text(b.px(0.4), b.py(27.4), "spool: L1 cannot represent a", "k2")
    f.text(b.px(0.4), b.py(25.0), "locked axle at all, so it", "k2")
    f.text(b.px(0.4), b.py(22.6), "disagrees from the start", "k2")

    f.save("cross-tier-crossover.svg")

def main():
    print("writing figures to docs/racing/assets/")
    for fn in (fig_slip_angle, fig_tyre_curve, fig_load_sensitivity,
               fig_peak_location,
               fig_relaxation,
               fig_friction_ellipse, fig_combined_slip,
               fig_load_transfer_long,
               fig_load_transfer_lat, fig_vehicle_models,
               fig_understeer_oversteer, fig_racing_line, fig_gg_diagram,
               fig_speed_profile,
               fig_differential_speeds, fig_torque_speed, fig_servo_step,
               fig_cross_tier_crossover):
        fn()
    print("done")


if __name__ == "__main__":
    main()
