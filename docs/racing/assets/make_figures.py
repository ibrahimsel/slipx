#!/usr/bin/env python3
# Copyright 2026 The SlipX Authors
# SPDX-License-Identifier: Apache-2.0
"""Render the diagrams for the autonomous racing tutorial series.

    python3 docs/racing/assets/make_figures.py

Writes one SVG per figure into this directory. No dependencies beyond the
standard library, deliberately: a diagram nobody can regenerate is a diagram
that goes stale the first time a number in it is questioned.

Figures carry labels, not paragraphs. Anything that needs a sentence belongs in
the article beside the figure, where it can be edited, translated and searched.

Two kinds of figure live here and they are not the same kind of claim.

SCHEMATICS (slip-angle, load-transfer-*, vehicle-models, understeer-oversteer,
racing-line) are geometry. They show how quantities are defined and how a
picture is labelled, and there is no model behind them to be right or wrong.

PLOTS (tyre-curve, load-sensitivity, friction-ellipse, gg-diagram,
speed-profile) are computed from the formulae written out below, at parameter
values chosen to be plausible for a 1/10-scale car. They are ILLUSTRATIVE. They
are not output from `slipx_core`, and no parameter set in this file has been
measured against a vehicle (NFR-08). This is the same caveat that
`docs/assets/make_banner.py` carries and for the same reason: at the time of
writing, the tier that owns a Magic Formula tyre is not built yet.

When the double-track tier lands, the plotted figures should be regenerated
from `slipx_core` through the Python bindings and the local formulae here
deleted. That is a real task, not an aspiration: a tutorial whose tyre curve
disagrees with the library it ships beside teaches the wrong thing twice.
"""

import math
import os

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
# Illustrative parameters for one tyre of a 3.5 kg 1/10-scale car at rest, so
# the vertical load below is a quarter of its weight. See the module docstring:
# these numbers are plausible and unmeasured.

FZ_NOM = 3.5 * 9.80665 / 4.0      # nominal vertical load per tyre       [N]
MU_Y0 = 1.10                      # peak lateral friction at that load   [-]
MU_X0 = 1.15                      # peak longitudinal friction           [-]
K_MU = 0.15                       # load sensitivity exponent            [-]
MF_B, MF_C, MF_E = 13.0, 1.50, -0.20   # Magic Formula shape factors     [-]
G = 9.80665


def mu_y(fz):
    """Load-sensitive peak friction: mu falls as the tyre is loaded."""
    return MU_Y0 * (fz / FZ_NOM) ** (-K_MU)


def fy_mf(alpha, fz=FZ_NOM):
    """Lateral force from the reduced Magic Formula, ISO 8855 sign.

    Fy = -mu(Fz) Fz sin(C atan(B a - E (B a - atan(B a))))

    The leading minus is the ISO convention: a positive slip angle means the
    tyre is running to the left of where it points, and the force it makes
    opposes that, so it is negative. Under SAE the slip angle carries the
    other sign and the same curve is written without the minus.
    """
    ba = MF_B * alpha
    inner = ba - MF_E * (ba - math.atan(ba))
    return -mu_y(fz) * fz * math.sin(MF_C * math.atan(inner))


def c_alpha(fz=FZ_NOM):
    """Cornering stiffness: the slope of the curve at the origin, positive."""
    return MF_B * MF_C * mu_y(fz) * fz


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


# ======================================================= 4. friction ellipse

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


def main():
    print("writing figures to docs/racing/assets/")
    for fn in (fig_slip_angle, fig_tyre_curve, fig_load_sensitivity,
               fig_friction_ellipse, fig_load_transfer_long,
               fig_load_transfer_lat, fig_vehicle_models,
               fig_understeer_oversteer, fig_racing_line, fig_gg_diagram,
               fig_speed_profile):
        fn()
    print("done")


if __name__ == "__main__":
    main()
