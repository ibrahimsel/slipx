#!/usr/bin/env python3
# Copyright 2026 The SlipX Authors
# SPDX-License-Identifier: Apache-2.0
"""Render the animated README banner: a 1/10-scale car drifting on a skidpad.

Every frame is a rollout of `slipx_core` at the double-track tier, through the
Python bindings. There is no model in this file: the positions, the slip
angles, the axle forces and the vertical loads drawn on the panel all come out
of a `VehicleState` and a `StepDiagnostics`, and the only thing written here is
the controller that holds the skidpad, which is a controller and not physics.

This used to carry its own single-track model and its own Magic Formula,
written before the library existed. It was replaced when MF-lite and the
double-track tier landed, because a banner advertising a physics library ought
to be output from it.

The car drifts because its rear tyre is a lower-grip compound than its front,
which is a parameter choice the tier supports and not a special case in the
drawing. The parameters are the reference car's and are PROVISIONAL: this is a
picture of the library running, not a measurement of a vehicle.

Needs Pillow and a built `slipx`. Run from the repository root:

    python3 docs/assets/make_banner.py

Output: docs/assets/slipx-banner.gif
"""

import math
import os

from PIL import Image, ImageDraw, ImageFont

try:
    import slipx
except ImportError as exc:  # pragma: no cover - an install problem, not a bug
    raise SystemExit(
        "make_banner.py rolls the banner out of slipx_core, so `import slipx` "
        "has to work. Build the extension in place with `make build`, and run "
        "this from the repository root."
    ) from exc

# ----------------------------------------------------------------- parameters
#
# The reference car, with two deliberate edits. Neither is a fudge for the
# drawing: both are ordinary parameter choices the tier already supports.

G = 9.80665                        # slipx::kGravity

R_PAD = 2.85                       # skidpad radius                      [m]
V_REF = 6.0                        # target speed                      [m/s]
DT = 1.0 / 1000.0                  # physics step                        [s]


def _params():
    p = slipx.load_reference_car().params_for_tier(slipx.Tier.L2_DoubleTrack)

    # A grippier front compound than rear. A car with matched tyres and a
    # neutral balance holds the skidpad without ever reaching the rear's
    # limit, which makes for a correct and completely undramatic picture.
    # Different compounds at the two ends is what the schema's per-axle tyre
    # reference exists for.
    p.tyre_front.mu_y0 = 1.30
    p.tyre_rear.mu_y0 = 0.86

    # Enough motor to hold the speed through the drift. The provisional ESC
    # is sized for a car that is not spending most of its rear grip sideways.
    p.torque_per_amp = 0.05
    return p


PARAMS = _params()
M = PARAMS.mass
LF, LR = PARAMS.lf, PARAMS.lr

# ------------------------------------------------------------------- dynamics
#
# There is none here. `slipx_core` is the model; what follows only unpacks it
# into the tuple the drawing code has always used.


def _unpack(state):
    return (state.vel_body.x, state.vel_body.y, state.rates.z, state.yaw,
            state.pos.x, state.pos.y)


def _diagnostics(d):
    """The panel's fields, named as the drawing has always named them.

    Copied out rather than held by reference: `StepDiagnostics` handed back
    from a step is overwritten by the next one, and a frame list full of
    aliases to the last frame is a trap this project has paid for once.
    """
    return dict(alpha_f=d.alpha_front, alpha_r=d.alpha_rear,
                fy_f=d.fy_front, fy_r=d.fy_rear,
                fz_f=d.fz_front, fz_r=d.fz_rear, ay=d.ay)


def drive(s):
    """Hold the skidpad radius and the reference speed. Deliberately simple."""
    vx, vy, r, yaw, x, y = s
    radius = math.hypot(x, y)
    theta = math.atan2(y, x)
    heading_err = math.atan2(math.sin(theta + math.pi / 2 - yaw),
                             math.cos(theta + math.pi / 2 - yaw))
    delta = 1.15 * heading_err + 0.75 * (radius - R_PAD) / R_PAD
    delta = max(-0.38, min(0.38, delta))  # servo travel limit
    v_target = V_REF * (1.0 + 0.05 * math.sin(2.0 * theta))
    # A commanded acceleration, which is what DriveInput carries: the ESC and
    # the tyres decide between them how much of it arrives.
    return delta, 5.5 * (v_target - vx)


def simulate(n_frames):
    """Warm up onto the limit cycle, then sample one revolution."""
    model = slipx.VehicleModel.create(slipx.Tier.L2_DoubleTrack, PARAMS)

    state = slipx.VehicleState()
    state.vel_body.x = V_REF
    state.rates.z = V_REF / R_PAD
    state.yaw = math.pi / 2
    state.pos.x = R_PAD
    diagnostics = slipx.StepDiagnostics()

    def advance():
        delta, accel = drive(_unpack(state))
        model.step(state, slipx.DriveInput(delta, accel), DT, diagnostics)

    for _ in range(int(12.0 / DT)):
        advance()

    frames, unwrapped = [], 0.0
    prev = math.atan2(state.pos.y, state.pos.x)
    while unwrapped < 2 * math.pi:
        advance()
        th = math.atan2(state.pos.y, state.pos.x)
        unwrapped += math.atan2(math.sin(th - prev), math.cos(th - prev))
        prev = th
        frames.append((_unpack(state), state.steer, _diagnostics(diagnostics),
                       unwrapped))

    out = []
    for i in range(n_frames):
        target = 2 * math.pi * i / n_frames
        j = min(range(len(frames)), key=lambda k: abs(frames[k][3] - target))
        out.append(frames[j][:3])
    return out


# -------------------------------------------------------------------- drawing

W, H = 900, 300
SS = 2  # supersampling factor
BG = (10, 14, 22)
PANEL = (15, 21, 32)
GRID = (28, 38, 54)
TRACK = (46, 60, 82)
INK = (232, 240, 252)
DIM = (122, 140, 168)
CYAN = (56, 211, 235)  # heading
AMBER = (255, 194, 62)  # velocity
ROSE = (244, 94, 122)  # tyre force


# panel geometry, in unscaled pixels
PX0, PY0 = 452, 18
PW, PH = W - 22 - PX0, H - 18 - PY0
CX, CY = PW * 0.66, PH * 0.56  # camera focus, panel-local
PPM = 116.0  # pixels per metre; the camera chases the car

FONT_DIR = "/usr/share/fonts/truetype/dejavu"


def font(name, size):
    return ImageFont.truetype(os.path.join(FONT_DIR, name), size * SS)


F_TITLE = font("DejaVuSans-Bold.ttf", 58)
F_TAG = font("DejaVuSans.ttf", 15)
F_SUB = font("DejaVuSans.ttf", 11)
F_HUD = font("DejaVuSansMono.ttf", 10)
F_HUDB = font("DejaVuSansMono-Bold.ttf", 11)


def arrow(d, x0, y0, x1, y1, colour, width, head=8):
    d.line([x0, y0, x1, y1], fill=colour, width=width * SS)
    ang = math.atan2(y1 - y0, x1 - x0)
    h = head * SS
    d.polygon([
        (x1, y1),
        (x1 - h * math.cos(ang - 0.42), y1 - h * math.sin(ang - 0.42)),
        (x1 - h * math.cos(ang + 0.42), y1 - h * math.sin(ang + 0.42)),
    ], fill=colour)


def blend(a, b, t):
    return tuple(int(a[i] + (b[i] - a[i]) * t) for i in range(3))


def draw_panel(states, i):
    """The simulation view: chase camera, clipped to its own rounded panel."""
    (vx, vy, r, yaw, x, y), delta, diag = states[i]
    pan = Image.new("RGB", (PW * SS, PH * SS), PANEL)
    d = ImageDraw.Draw(pan)

    def w2s(wx, wy):
        return (CX + (wx - x) * PPM) * SS, (CY - (wy - y) * PPM) * SS

    # world grid at 0.5 m, scrolling under the car
    for k in range(-8, 9):
        gx = (math.floor(x / 0.5) + k) * 0.5
        d.line([w2s(gx, 0)[0], 0, w2s(gx, 0)[0], PH * SS], fill=GRID, width=1)
        gy = (math.floor(y / 0.5) + k) * 0.5
        d.line([0, w2s(0, gy)[1], PW * SS, w2s(0, gy)[1]], fill=GRID, width=1)

    # skidpad centreline and its 1 m lane edges
    ox, oy = w2s(0.0, 0.0)
    r_line = sum(math.hypot(s[0][4], s[0][5]) for s in states) / len(states)
    for rad, col, wd in ((r_line - 0.6, TRACK, 2), (r_line + 0.6, TRACK, 2)):
        rr = rad * PPM * SS
        d.ellipse([ox - rr, oy - rr, ox + rr, oy + rr], outline=col, width=wd * SS)
    rr = r_line * PPM * SS
    d.ellipse([ox - rr, oy - rr, ox + rr, oy + rr],
              outline=(34, 46, 64), width=1 * SS)

    # tyre marks already laid down by the rear axle
    trail = 26
    for k in range(trail, 0, -1):
        pa = states[(i - k) % len(states)][0]
        pb = states[(i - k + 1) % len(states)][0]
        ra = (pa[4] - LR * math.cos(pa[3]), pa[5] - LR * math.sin(pa[3]))
        rb = (pb[4] - LR * math.cos(pb[3]), pb[5] - LR * math.sin(pb[3]))
        t = 1.0 - k / trail
        d.line([*w2s(*ra), *w2s(*rb)],
               fill=blend(PANEL, (128, 58, 74), t), width=int(3 + 4 * t) * SS)

    # chassis
    hl, hw = 0.26, 0.115
    cs, sn = math.cos(yaw), math.sin(yaw)

    def body_pt(px, py):
        return w2s(x + px * cs - py * sn, y + px * sn + py * cs)

    # wheels first, so the body sits on top of them
    for wx, wy, st in ((LF, hw, delta), (LF, -hw, delta),
                       (-LR, hw, 0.0), (-LR, -hw, 0.0)):
        wa, hub = yaw + st, body_pt(wx, wy)
        wc, ws = math.cos(wa) * 0.06 * PPM * SS, math.sin(wa) * 0.06 * PPM * SS
        d.line([hub[0] - wc, hub[1] + ws, hub[0] + wc, hub[1] - ws],
               fill=(12, 16, 26), width=9 * SS)

    d.polygon([body_pt(*p) for p in
               ((hl, hw * 0.86), (hl * 0.55, hw), (-hl, hw),
                (-hl, -hw), (hl * 0.55, -hw), (hl, -hw * 0.86))],
              fill=(228, 236, 250))
    d.polygon([body_pt(*p) for p in
               ((0.06, hw * 0.8), (0.06, -hw * 0.8),
                (-0.13, -hw * 0.8), (-0.13, hw * 0.8))],
              fill=(96, 112, 142))
    d.line([body_pt(hl, 0.0), body_pt(hl * 0.6, 0.0)], fill=CYAN, width=3 * SS)

    # body slip: the wedge between where the car points and where it goes
    speed = math.hypot(vx, vy)
    beta = math.atan2(vy, vx)
    hx, hy = math.cos(yaw), math.sin(yaw)
    vhx, vhy = math.cos(yaw + beta), math.sin(yaw + beta)
    a0, a1 = math.degrees(yaw), math.degrees(yaw + beta)
    for rad, wd in ((0.46, 2), (0.50, 2)):
        bb = [*w2s(x - rad, y + rad), *w2s(x + rad, y - rad)]
        d.arc(bb, -max(a0, a1), -min(a0, a1), fill=(150, 112, 44), width=wd * SS)
    arrow(d, *w2s(x, y), *w2s(x + hx * 0.72, y + hy * 0.72), CYAN, 2, 7)
    arrow(d, *w2s(x, y), *w2s(x + vhx * 0.95, y + vhy * 0.95), AMBER, 3, 9)

    # lateral tyre force at each axle, scaled by the friction limit
    for lx, fy in ((LF, diag["fy_f"]), (-LR, diag["fy_r"])):
        ax_, ay_ = x + lx * cs, y + lx * sn
        mag = fy / (PARAMS.tyre_front.mu_y0 * M * G) * 0.85
        arrow(d, *w2s(ax_, ay_), *w2s(ax_ - hy * mag, ay_ + hx * mag), ROSE, 2, 7)

    mask = Image.new("L", (PW * SS, PH * SS), 0)
    ImageDraw.Draw(mask).rounded_rectangle(
        [0, 0, PW * SS - 1, PH * SS - 1], radius=10 * SS, fill=255)
    return pan, mask, (beta, speed, delta, diag)


def render(states, i):
    img = Image.new("RGB", (W * SS, H * SS), BG)
    d = ImageDraw.Draw(img)

    # --- left: wordmark -----------------------------------------------------
    d.line([48 * SS, 76 * SS, 96 * SS, 76 * SS], fill=CYAN, width=3 * SS)
    d.text((44 * SS, 90 * SS), "Slip", font=F_TITLE, fill=INK)
    d.text((44 * SS + d.textlength("Slip", font=F_TITLE), 90 * SS), "X",
           font=F_TITLE, fill=CYAN)
    d.text((48 * SS, 166 * SS), "Vehicle dynamics for 1/10-scale racecars",
           font=F_TAG, fill=(200, 214, 236))
    d.text((48 * SS, 192 * SS),
           "C++17 core. Every tyre parameter is one you can fit from a rosbag",
           font=F_SUB, fill=DIM)
    d.text((48 * SS, 208 * SS),
           "recorded on a car park skidpad, using the sensors already on the car.",
           font=F_SUB, fill=DIM)

    # --- right: simulation panel -------------------------------------------
    pan, mask, (beta, speed, delta, diag) = draw_panel(states, i)
    img.paste(pan, (PX0 * SS, PY0 * SS), mask)
    d.rounded_rectangle([PX0 * SS, PY0 * SS, (PX0 + PW) * SS - 1, (PY0 + PH) * SS - 1],
                        radius=10 * SS, outline=(38, 50, 70), width=1 * SS)

    # --- HUD ----------------------------------------------------------------
    scrim = Image.new("RGBA", img.size, (0, 0, 0, 0))
    ds = ImageDraw.Draw(scrim)
    ds.rounded_rectangle([(PX0 + 6) * SS, (PY0 + 6) * SS,
                          (PX0 + 138) * SS, (PY0 + 116) * SS],
                         radius=6 * SS, fill=(9, 13, 21, 205))
    ds.rounded_rectangle([(PX0 + 6) * SS, (PY0 + PH - 58) * SS,
                          (PX0 + 178) * SS, (PY0 + PH - 3) * SS],
                         radius=6 * SS, fill=(9, 13, 21, 205))
    img = Image.alpha_composite(img.convert("RGBA"), scrim).convert("RGB")
    d = ImageDraw.Draw(img)

    rows = [
        ("beta", math.degrees(beta), "deg"),
        ("a_f", math.degrees(diag["alpha_f"]), "deg"),
        ("a_r", math.degrees(diag["alpha_r"]), "deg"),
        ("a_y", diag["ay"] / G, "g"),
        ("v", speed, "m/s"),
        ("delta", math.degrees(delta), "deg"),
    ]
    hx0, hy0 = (PX0 + 14) * SS, (PY0 + 12) * SS
    d.text((hx0, hy0), "StepDiagnostics", font=F_HUDB, fill=(158, 176, 204))
    for n, (label, val, unit) in enumerate(rows):
        yy = hy0 + int((17 + n * 13) * SS)
        d.text((hx0, yy), label, font=F_HUD, fill=DIM)
        d.text((hx0 + 42 * SS, yy), f"{val:>6.2f}", font=F_HUD, fill=INK)
        d.text((hx0 + 92 * SS, yy), unit, font=F_HUD, fill=DIM)

    d.text((hx0, (PY0 + PH - 50) * SS), "L2 double-track - MF-lite - 1 kHz",
           font=F_HUD, fill=(158, 176, 204))
    for n, (col, label) in enumerate(((AMBER, "velocity"), (CYAN, "heading"),
                                      (ROSE, "lateral tyre force"))):
        yy = (PY0 + PH - 37 + n * 11) * SS
        d.rectangle([hx0, yy + 3 * SS, hx0 + 7 * SS, yy + 7 * SS], fill=col)
        d.text((hx0 + 13 * SS, yy), label, font=F_HUD, fill=DIM)

    return img.resize((W, H), Image.LANCZOS)


def main():
    n = 48
    states = simulate(n)
    frames = [render(states, i) for i in range(n)]
    pal = frames[0].quantize(colors=64, method=Image.MEDIANCUT)
    frames = [f.quantize(palette=pal, dither=Image.FLOYDSTEINBERG) for f in frames]
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "slipx-banner.gif")
    frames[0].save(out, save_all=True, append_images=frames[1:],
                   duration=62, loop=0, optimize=True, disposal=1)
    print(f"{out}  {os.path.getsize(out) / 1e6:.2f} MB  {n} frames")


if __name__ == "__main__":
    main()
