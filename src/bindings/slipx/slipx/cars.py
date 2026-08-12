# Copyright 2026 The SlipX Authors
# SPDX-License-Identifier: Apache-2.0

"""Loading a car directory into something the core can step.

The whole of this module is the join between two layers that do not know about
each other: ``slipx_schema`` turns a directory into a dataclass, and
``slipx_core`` takes a plain struct. Neither imports the other, and this is
where the two meet.

That is more than a formality. ``slipx_core`` must build and pass its full test
suite with ``slipx_schema`` absent (CORE-01), so this file is also the only
place in the Python API where a missing schema install is allowed to be a
problem, and it says so rather than raising an ImportError from three frames
down.
"""

from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING

from ._slipx import Provenance, VehicleParams

if TYPE_CHECKING:  # pragma: no cover
    from slipx_schema.model import Car

_PROVENANCE_LABELS = {
    "provisional": Provenance.Provisional,
    "identified": Provenance.Identified,
    "measured": Provenance.Measured,
}


def _require_schema():
    try:
        import slipx_schema
    except ImportError as exc:  # pragma: no cover - environment-specific
        raise ImportError(
            "loading a car directory needs slipx_schema, which is not "
            "installed. The core itself does not: parameters enter it as a "
            "plain VehicleParams struct, and you can fill one in directly if "
            "you would rather not depend on the parser (CORE-01)."
        ) from exc
    return slipx_schema


def to_vehicle_params(parameters) -> VehicleParams:
    """Convert a ``slipx_schema.VehicleParameters`` into the core's struct.

    Field for field, with no arithmetic. Any unit conversion or derived
    quantity belongs in slipx_schema, where it can be validated and where its
    inputs are still named; doing it here would put a calculation in the one
    place nothing tests.

    ``v_eps`` is the single field that may be absent from a car file with the
    core's default standing in. When it is, ``Car.notes`` already recorded
    that (SCH-02 forbids silent defaulting, not defaulting). ``c_kappa`` may
    also be absent, but it never defaults: it is consumed only through
    :meth:`Car.params_for_tier`, which refuses L2 by name without it.
    """
    params = VehicleParams()
    params.mass = parameters.mass
    params.izz = parameters.izz
    params.ixx = parameters.ixx
    params.iyy = parameters.iyy
    params.lf = parameters.lf
    params.lr = parameters.lr
    params.track_front = parameters.track_front
    params.track_rear = parameters.track_rear
    params.h_cog = parameters.h_cog
    params.wheel_radius = parameters.wheel_radius
    params.c_alpha_f = parameters.c_alpha_f
    params.c_alpha_r = parameters.c_alpha_r
    params.mu_clip = parameters.mu_clip
    params.accel_max = parameters.accel_max
    params.decel_max = parameters.decel_max
    params.v_max = parameters.v_max
    params.steer_max = parameters.steer_max
    params.drag_coeff = parameters.drag_coeff
    params.roll_resist = parameters.roll_resist
    params.provenance = _PROVENANCE_LABELS[parameters.provenance_label]
    if parameters.v_eps is not None:
        params.v_eps = parameters.v_eps

    # The core's own physical sanity check, run here so that a bad car fails at
    # load rather than at the first step. It is not schema validation and does
    # not duplicate it: slipx_schema has already checked the rules and the
    # ranges, and this checks that the struct describes a possible object.
    reason = params.validate()
    if reason is not None:
        raise ValueError(f"slipx_core rejected the parameters: {reason}")

    return params



def _copy_params(source: VehicleParams) -> VehicleParams:
    """A field-for-field copy, because the binding exposes no copy constructor.

    Only the fields a tier could care about; the tyre blocks are copied as
    values rather than shared, so mutating one car's parameters cannot reach
    another's.
    """
    out = VehicleParams()
    for name in (
        "mass", "izz", "ixx", "iyy", "lf", "lr", "track_front", "track_rear",
        "h_cog", "wheel_radius", "c_alpha_f", "c_alpha_r", "mu_clip",
        "c_kappa", "accel_max", "decel_max", "v_max", "steer_max",
        "drag_coeff", "roll_resist", "provenance", "v_eps",
    ):
        setattr(out, name, getattr(source, name))
    for axle in ("tyre_front", "tyre_rear"):
        src = getattr(source, axle)
        dst = getattr(out, axle)
        for name in ("mu_y0", "mu_x0", "k_mu", "relax_length", "shape_c",
                     "curvature_e"):
            setattr(dst, name, getattr(src, name))
    return out


class Car:
    """A loaded car directory, with its parameters ready for the core.

    Thin by design. Everything about the files is ``slipx_schema``'s and stays
    reachable through :attr:`spec`; what this adds is the converted
    :attr:`params`.
    """

    def __init__(self, spec: "Car") -> None:
        self.spec = spec
        self.params = to_vehicle_params(spec.params)

    @property
    def name(self) -> str:
        return self.spec.name

    @property
    def provenance(self):
        """How these numbers were obtained (NFR-08)."""
        return self.spec.provenance

    @property
    def notes(self) -> list[str]:
        """Everything that was decided rather than read."""
        return self.spec.notes

    @property
    def warnings(self) -> list[str]:
        """Values that validated but look wrong (SCH-04)."""
        return self.spec.warnings

    def params_for_tier(self, tier) -> VehicleParams:
        """Parameters for one tier, refusing rather than defaulting.

        ``params`` carries everything L0 and L1 need and is what most callers
        want. L2 additionally needs the MF-lite block, the load sensitivity,
        the relaxation length and the longitudinal slip stiffness
        ``linear.c_kappa``; schema 0.2.0 carries all of them (ADR-0030).

        A tyre file that lacks any of those, which every file written at
        schema 0.1.0 does for ``c_kappa``, produces a refusal naming each
        absent field. Silent defaulting is forbidden, and a parameter nobody
        identified is exactly the failure ADR-0009 exists to prevent: a
        default would produce a trajectory that looks right, is labelled L2,
        and rests on a number that came from nowhere.

        This is NOT the ADR-0005 failure of substituting a simpler tier: on
        refusal no model is returned at all.
        """
        from . import Tier  # local, to keep the module import graph flat

        if tier is not Tier.L2_DoubleTrack:
            return self.params

        params = _copy_params(self.params)
        front = self.spec.tyre_front
        rear = self.spec.tyre_rear

        missing = []
        for label, tyre, target in (
            ("front", front, params.tyre_front),
            ("rear", rear, params.tyre_rear),
        ):
            if tyre.mf_lite is None:
                missing.append(f"{label} tyre mf_lite block")
            else:
                target.shape_c = float(tyre.mf_lite["C"])
                target.curvature_e = float(tyre.mf_lite["E"])
            if tyre.k_mu is None:
                missing.append(f"{label} tyre friction.k_mu")
            else:
                target.k_mu = float(tyre.k_mu)
            if tyre.sigma is None:
                missing.append(f"{label} tyre relaxation.sigma")
            else:
                target.relax_length = float(tyre.sigma)
            if tyre.c_kappa is None:
                missing.append(f"{label} tyre linear.c_kappa")
            target.mu_y0 = float(tyre.mu_y0)
            target.mu_x0 = float(tyre.mu_x0)

        if not missing:
            # Reduced to one whole-car value at load, with a note when the
            # two files disagreed; see the loader.
            params.c_kappa = float(self.spec.params.c_kappa)
            return params

        raise ValueError(
            "this car cannot parameterise tier L2: "
            + ", ".join(missing)
            + " missing. Nothing here is defaulted, because a parameter "
            "nobody identified is worse than one that does not exist "
            "(ADR-0025, ADR-0009). A tyre file written at schema 0.1.0 has "
            "no linear.c_kappa field; schema 0.2.0 adds it, and migration "
            "cannot invent the value, so the file needs the measurement and "
            "not just the version bump. L0 and L1 are unaffected and are "
            "available through .params."
        )

    def summary(self) -> str:
        """One block of text, leading with the provenance label (NFR-08)."""
        return self.spec.summary()

    def __repr__(self) -> str:
        return (
            f"Car(name={self.name!r}, "
            f"provenance={self.spec.provenance.label!r}, "
            f"mass={self.params.mass})"
        )


def reference_car_path() -> Path:
    """Where the reference 1/10-scale car directory is (NFR-10).

    Two locations, because there are two ways to have SlipX. An installed
    wheel carries the directory inside the package; a checkout has it at
    ``examples/cars``. It is the same car either way, installed from the
    repository copy rather than duplicated into the package sources, so the
    two cannot drift into disagreeing reference cars.

    The parameters are labelled ``provisional`` and describe no measured
    vehicle (NFR-08). It is a car that validates, not a car that is right.
    """
    installed = Path(__file__).resolve().parent / "examples" / "reference_1_10"
    if installed.is_dir():
        return installed

    in_tree = (
        Path(__file__).resolve().parents[4]
        / "examples" / "cars" / "reference_1_10"
    )
    if in_tree.is_dir():
        return in_tree

    raise FileNotFoundError(
        "the reference car is neither installed beside the package nor "
        "present in a source checkout. If this is an installed wheel, the "
        "package data is missing and the install is incomplete."
    )


def load_reference_car(strict: bool = False) -> Car:
    """Load the reference car. The shortest path to a steppable model."""
    return load_car(reference_car_path(), strict=strict)


def load_car(directory: str | Path, strict: bool = False) -> Car:
    """Load, validate and convert a car directory.

    Args:
        directory: Path to the directory holding ``car.yaml``.
        strict: Treat SCH-04 plausibility warnings as errors. A competition
            harness should pass True; a student exploring an unusual car
            should not, because a warning means "possible but unusual" and
            refusing those would make the loader wrong about unusual cars.

    Returns:
        A :class:`Car` whose ``params`` can be handed straight to
        :meth:`slipx.VehicleModel.create`.

    Raises:
        ValueError: the parameters describe no possible object.
        slipx_schema.ValidationError: one or more fields failed validation,
            with every failure reported together.
    """
    schema = _require_schema()
    return Car(schema.load_car(directory, strict=strict))
