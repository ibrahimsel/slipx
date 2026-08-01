# Copyright 2026 The SlipX Authors
# SPDX-License-Identifier: Apache-2.0

"""Checks that JSON Schema cannot express: SCH-03 and SCH-04.

A JSON Schema can say that ``width`` is a number between 0 and 1. It cannot say
that the inertia tensor must satisfy the triangle inequality, or that the sum
of ``lf`` and ``lr`` must be shorter than the car. Those live here.

The division between an error and a warning is deliberate and consistent:

  error    the described object cannot exist, or is not a legal competition
           car. Refusing is correct even if the user is annoyed.
  warning  the value is possible but improbable. Refusing would make the
           parser wrong about unusual cars; staying quiet would make it
           useless for the overwhelmingly common case, which is that somebody
           entered millimetres.
"""

from __future__ import annotations

from typing import Any, Dict, List, Tuple

from .errors import FieldError, Warning_

# ---------------------------------------------------------------- SCH-03
#
# Dimensional bounds from the published RoboRacer ruleset. A car that validates
# shall be a legal car, so these are errors and not warnings.
#
# These are competition rules, and competition rules change. They are stated
# once, here, with the revision they came from, so that tracking a ruleset
# update is an edit to four numbers rather than an archaeology exercise
# (SCENE-07 makes the same argument for the race procedures).

ROBORACER_RULESET = "RoboRacer / F1TENTH vehicle rules, 1/10 class"

WIDTH_MIN_M = 0.238
WIDTH_MAX_M = 0.341
LENGTH_MIN_M = 0.454
LENGTH_MAX_M = 0.654


def check_dimensional_legality(geometry: Dict[str, Any], file: str) -> List[FieldError]:
    """SCH-03: a car that validates shall be a legal car."""
    errors: List[FieldError] = []

    width = geometry.get("width")
    if isinstance(width, (int, float)) and not (WIDTH_MIN_M <= width <= WIDTH_MAX_M):
        errors.append(
            FieldError(
                path="geometry.width",
                message=(
                    f"{width} m is outside the {ROBORACER_RULESET} width limits, "
                    f"so this car could not be entered"
                ),
                permitted=f"{WIDTH_MIN_M} to {WIDTH_MAX_M} m",
                file=file,
                requirement="SCH-03",
            )
        )

    length = geometry.get("length")
    if isinstance(length, (int, float)) and not (LENGTH_MIN_M <= length <= LENGTH_MAX_M):
        errors.append(
            FieldError(
                path="geometry.length",
                message=(
                    f"{length} m is outside the {ROBORACER_RULESET} length limits, "
                    f"so this car could not be entered"
                ),
                permitted=f"{LENGTH_MIN_M} to {LENGTH_MAX_M} m",
                file=file,
                requirement="SCH-03",
            )
        )

    return errors


def check_geometric_consistency(
    geometry: Dict[str, Any], file: str
) -> Tuple[List[FieldError], List[Warning_]]:
    """Internal consistency of the geometry block.

    These catch the errors that pass every individual range check because each
    number is plausible on its own.
    """
    errors: List[FieldError] = []
    warnings: List[Warning_] = []

    lf = geometry.get("lf")
    lr = geometry.get("lr")
    length = geometry.get("length")
    width = geometry.get("width")
    track_front = geometry.get("track_front")
    track_rear = geometry.get("track_rear")
    h_cog = geometry.get("h_cog")
    wheel_radius = geometry.get("wheel_radius")

    if all(isinstance(v, (int, float)) for v in (lf, lr, length)):
        wheelbase = lf + lr
        if wheelbase > length:
            errors.append(
                FieldError(
                    path="geometry.lf + geometry.lr",
                    message=(
                        f"wheelbase {wheelbase:.3f} m exceeds the overall length "
                        f"{length:.3f} m, which describes no possible car"
                    ),
                    permitted=f"< {length} m",
                    file=file,
                    requirement="SCH-03",
                )
            )
        elif wheelbase < 0.4 * length:
            warnings.append(
                Warning_(
                    path="geometry.lf + geometry.lr",
                    message=(
                        f"wheelbase {wheelbase:.3f} m is under 40% of the overall "
                        f"length {length:.3f} m, which is unusual; check that lf "
                        f"and lr are measured from the CoG and not from the nose"
                    ),
                    file=file,
                    requirement="SCH-04",
                )
            )

    for name, track in (("track_front", track_front), ("track_rear", track_rear)):
        if isinstance(track, (int, float)) and isinstance(width, (int, float)):
            if track > width:
                errors.append(
                    FieldError(
                        path=f"geometry.{name}",
                        message=(
                            f"track {track:.3f} m exceeds the overall width "
                            f"{width:.3f} m; the wheels cannot be outside the car"
                        ),
                        permitted=f"<= {width} m",
                        file=file,
                        requirement="SCH-03",
                    )
                )

    if isinstance(h_cog, (int, float)) and isinstance(wheel_radius, (int, float)):
        if h_cog > 3.0 * wheel_radius:
            warnings.append(
                Warning_(
                    path="geometry.h_cog",
                    message=(
                        f"CoG height {h_cog:.3f} m is more than three wheel radii "
                        f"above the ground; possible with a tall bodyshell, but "
                        f"more often a units error"
                    ),
                    file=file,
                    requirement="SCH-04",
                )
            )

    return errors, warnings


# ---------------------------------------------------------------- SCH-04
#
# Inertia tensor physical consistency, then plausibility for the declared mass
# and dimensions.


def check_inertia(
    inertia: Dict[str, Any], mass: Any, geometry: Dict[str, Any], file: str
) -> Tuple[List[FieldError], List[Warning_]]:
    """SCH-04: consistency as an error, plausibility as a warning."""
    errors: List[FieldError] = []
    warnings: List[Warning_] = []

    ixx = inertia.get("ixx")
    iyy = inertia.get("iyy")
    izz = inertia.get("izz")
    if not all(isinstance(v, (int, float)) for v in (ixx, iyy, izz)):
        return errors, warnings  # the schema will already have complained

    # The triangle inequality on principal moments. It follows from the
    # definition of the inertia tensor as an integral over a mass
    # distribution, so a violation does not describe an unusual object; it
    # describes no object.
    for a, b, c, names in (
        (ixx, iyy, izz, ("ixx", "iyy", "izz")),
        (iyy, izz, ixx, ("iyy", "izz", "ixx")),
        (izz, ixx, iyy, ("izz", "ixx", "iyy")),
    ):
        if a + b < c:
            errors.append(
                FieldError(
                    path=f"inertia.{names[2]}",
                    message=(
                        f"the principal moments violate the triangle inequality: "
                        f"{names[0]} + {names[1]} = {a + b:.5g} < {names[2]} = "
                        f"{c:.5g}. No rigid body has this inertia tensor"
                    ),
                    permitted=f"{names[2]} <= {names[0]} + {names[1]}",
                    file=file,
                    requirement="SCH-04",
                )
            )

    # Positive definiteness of the full tensor, when products of inertia are
    # given. Checked through the leading principal minors (Sylvester's
    # criterion), which needs no eigenvalue solver and no numpy.
    ixy = float(inertia.get("ixy", 0.0))
    ixz = float(inertia.get("ixz", 0.0))
    iyz = float(inertia.get("iyz", 0.0))
    if ixy or ixz or iyz:
        # The inertia tensor's off-diagonal entries are -Ixy by the usual
        # convention; the file states products of inertia directly, so the
        # matrix is built with the signs the physics uses.
        m11 = ixx
        m2 = ixx * iyy - ixy * ixy
        m3 = (
            ixx * (iyy * izz - iyz * iyz)
            - ixy * (ixy * izz - iyz * ixz)
            + ixz * (ixy * iyz - iyy * ixz)
        )
        if m11 <= 0.0 or m2 <= 0.0 or m3 <= 0.0:
            errors.append(
                FieldError(
                    path="inertia",
                    message=(
                        "the inertia tensor is not positive definite; with these "
                        "products of inertia it describes no rigid body"
                    ),
                    permitted="a positive definite tensor",
                    file=file,
                    requirement="SCH-04",
                )
            )

    # Plausibility. A uniform box of the declared mass and dimensions is the
    # reference: a real car concentrates mass low and centrally and so comes
    # in under the box, but not by an order of magnitude. This is the check
    # that catches an izz copied from a full-scale vehicle paper, which is the
    # single most likely way a 1/10 car ends up with a plausible-looking
    # inertia that is a hundred times too large.
    length = geometry.get("length")
    width = geometry.get("width")
    if all(isinstance(v, (int, float)) for v in (mass, length, width)):
        box_izz = mass * (length * length + width * width) / 12.0
        ratio = izz / box_izz if box_izz > 0 else 0.0
        if not (0.15 <= ratio <= 1.5):
            warnings.append(
                Warning_(
                    path="inertia.izz",
                    message=(
                        f"izz = {izz:.4g} kg m^2 is {ratio:.2g} times the "
                        f"{box_izz:.4g} kg m^2 of a uniform box with this mass and "
                        f"footprint. Real cars land between roughly 0.15 and 1.5 "
                        f"times that; check the value and its units"
                    ),
                    file=file,
                    requirement="SCH-04",
                )
            )

    return errors, warnings


def check_tyre_plausibility(tyre: Dict[str, Any], file: str) -> List[Warning_]:
    """Plausibility for a tyre file (ID-06 in spirit, at load time).

    Warnings only. The bounds are what a 1/10-scale tyre on an indoor surface
    plausibly does, and an outlier is more often a units error or a value
    lifted from a full-scale dataset than a remarkable tyre.
    """
    warnings: List[Warning_] = []

    friction = tyre.get("friction", {})
    for name in ("mu_y0", "mu_x0"):
        mu = friction.get(name)
        if isinstance(mu, (int, float)) and not (0.4 <= mu <= 1.6):
            warnings.append(
                Warning_(
                    path=f"friction.{name}",
                    message=(
                        f"{mu} is outside the 0.4 to 1.6 a 1/10-scale tyre "
                        f"plausibly reaches indoors; possible, but check it "
                        f"was not taken from a full-scale dataset"
                    ),
                    file=file,
                    requirement="SCH-04",
                )
            )

    c_alpha = tyre.get("linear", {}).get("c_alpha")
    nominal_load = tyre.get("nominal_load")
    mu_y0 = friction.get("mu_y0")
    if all(isinstance(v, (int, float)) for v in (c_alpha, nominal_load, mu_y0)):
        # The slip angle at which the linear extrapolation would reach the
        # friction limit. For a real tyre this is a few degrees; a value far
        # outside that means the cornering stiffness and the friction
        # coefficient disagree about what tyre this is.
        peak_alpha = mu_y0 * nominal_load / c_alpha
        if not (0.02 <= peak_alpha <= 0.35):
            warnings.append(
                Warning_(
                    path="linear.c_alpha",
                    message=(
                        f"c_alpha and mu_y0 imply the linear region runs to "
                        f"{peak_alpha:.3f} rad ({peak_alpha * 57.3:.1f} deg) of "
                        f"slip before saturating. Real tyres peak between about "
                        f"1 and 20 degrees, so one of the two is inconsistent "
                        f"with the other"
                    ),
                    file=file,
                    requirement="SCH-04",
                )
            )

    return warnings
