# Copyright 2026 The SlipX Authors
# SPDX-License-Identifier: Apache-2.0

"""The types a parsed car directory becomes.

``VehicleParameters`` mirrors ``slipx::VehicleParams`` field for field, and
that correspondence is the point: the C++ core takes a plain struct and knows
nothing about files, schemas, YAML or versions (CORE-01). This package's whole
job is to turn a directory into that struct, and then get out of the way.

Nothing here imports the ``slipx`` package. The dependency runs slipx ->
slipx_schema -> slipx_core, never upward, and a convenience import in the
wrong direction is how that stops being true.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional


@dataclass(frozen=True)
class Provenance:
    """How a set of numbers was obtained (NFR-08).

    Travels with the parameters rather than beside them, so that tooling
    printing a result can print the label without having to go and look it up,
    and so that dropping the label takes a deliberate act.
    """

    label: str  # "provisional" | "identified" | "measured"
    source: str
    method: str
    date: str
    contributor: str = ""
    vehicle: str = ""
    residuals: Dict[str, Any] = field(default_factory=dict)
    validation_report: str = ""
    notes: str = ""

    @property
    def is_measured_or_identified(self) -> bool:
        return self.label in ("measured", "identified")

    def summary(self) -> str:
        """One line, for tooling output. Leads with the label, deliberately."""
        text = f"{self.label.upper()}: {self.source} ({self.date})"
        if self.label == "provisional":
            text += " -- not measured against any vehicle"
        return text


@dataclass(frozen=True)
class Tyre:
    """One resolved (compound, surface) pair."""

    compound: str
    surface: str
    c_alpha: float  # per tyre, at the nominal load          [N/rad]
    mu_y0: float  #                                              [-]
    mu_x0: float  #                                              [-]
    provenance: Provenance
    nominal_load: Optional[float] = None  #                      [N]
    k_mu: Optional[float] = None  #                              [-]
    mf_lite: Optional[Dict[str, float]] = None  # C, E (B derived), L2
    sigma: Optional[float] = None  # relaxation length, L2       [m]
    # Longitudinal slip stiffness per tyre, schema 0.2.0. None on a migrated
    # 0.1.0 file, and the loader refuses L2 by name rather than defaulting it
    # (ADR-0025, ADR-0030).
    c_kappa: Optional[float] = None  #             [N per unit slip]

    @property
    def reference(self) -> str:
        return f"{self.compound}/{self.surface}"


@dataclass(frozen=True)
class VehicleParameters:
    """Field-for-field mirror of ``slipx::VehicleParams``.

    Units and sign conventions are the core's (see ``slipx/conventions.hpp``);
    they are not restated here, because two statements of a convention are two
    things that can disagree.
    """

    mass: float
    izz: float
    ixx: float
    iyy: float
    lf: float
    lr: float
    track_front: float
    track_rear: float
    h_cog: float
    wheel_radius: float
    c_alpha_f: float
    c_alpha_r: float
    mu_clip: float
    accel_max: float
    decel_max: float
    v_max: float
    steer_max: float
    drag_coeff: float
    roll_resist: float
    provenance_label: str
    # None means the file did not specify it and the core's own default
    # stands. It is the one field that may be absent, because it is a
    # numerical mitigation rather than a property of the car; when it is
    # absent, Car.notes says so out loud.
    v_eps: Optional[float] = None
    # The whole-car longitudinal slip stiffness, per tyre. None when either
    # tyre file lacks the field, in which case L2 is refused by name rather
    # than defaulted. The core carries ONE value because the manoeuvre that
    # identifies it cannot separate the axles; when the two tyre files
    # disagree the loader takes the mean and Car.notes says so.
    c_kappa: Optional[float] = None

    # The drivetrain and actuator fields, consumed from L2 (ADR-0031). The
    # layout and differential are required by dynamics.schema.json so they are
    # always present; everything else is optional at the schema level
    # (ADR-0030) and None here when the file did not carry it, in which case
    # L2 is refused with the field named, never defaulted.
    layout: str = "2WD_rear"  # "2WD_rear" | "2WD_front" | "4WD"
    differential: str = "spool"  # "spool" | "open" | "lsd"
    lsd_preload: Optional[float] = None  # required when lsd         [N m]
    # limits.yaml `esc` block
    torque_stall: Optional[float] = None  #                          [N m]
    omega_free: Optional[float] = None  #                          [rad/s]
    torque_per_amp: Optional[float] = None  #                      [N m/A]
    drive_efficiency: Optional[float] = None  # esc.efficiency         [-]
    # limits.yaml `electrical` block
    pack_nominal_v: Optional[float] = None  #                          [V]
    pack_v_full: Optional[float] = None  #                             [V]
    pack_v_empty: Optional[float] = None  #                            [V]
    pack_capacity_ah: Optional[float] = None  #                      [A h]
    pack_internal_resistance: Optional[float] = None  #              [ohm]
    current_max: Optional[float] = None  #                             [A]
    regen_current_max: Optional[float] = None  #                       [A]
    # limits.yaml `steering` block beyond max_angle
    steer_rate_max: Optional[float] = None  # steering.max_rate    [rad/s]
    steer_bandwidth: Optional[float] = None  # steering.bandwidth  [rad/s]
    steer_damping: Optional[float] = None  # steering.damping          [-]

    @property
    def wheelbase(self) -> float:
        return self.lf + self.lr


@dataclass(frozen=True)
class Car:
    """A validated car directory.

    Attributes:
        notes: Everything that was decided rather than read. SCH-02 prohibits
            silent defaulting; this is what makes the remaining defaulting
            unsilent. A caller that wants to be strict can treat a non-empty
            notes list as a failure.
        warnings: SCH-04 plausibility warnings. The car is usable; somebody
            should look at it.
    """

    name: str
    schema_version: str
    directory: Path
    params: VehicleParameters
    provenance: Provenance
    tyre_front: Tyre
    tyre_rear: Tyre
    description: str = ""
    sensors: List[Dict[str, Any]] = field(default_factory=list)
    notes: List[str] = field(default_factory=list)
    warnings: List[str] = field(default_factory=list)
    raw: Dict[str, Any] = field(default_factory=dict)

    def summary(self) -> str:
        """What tooling prints when it loads a car.

        The provenance label goes on the second line, not in a footnote. NFR-08
        requires the label to appear in tooling output and not only in the
        documentation, because the documentation is not what somebody reads
        before they trust a number.
        """
        lines = [
            f"{self.name} (schema {self.schema_version})",
            f"  provenance: {self.provenance.summary()}",
            f"  mass {self.params.mass} kg, wheelbase {self.params.wheelbase:.3f} m, "
            f"izz {self.params.izz} kg m^2",
            f"  tyres: front {self.tyre_front.reference} "
            f"[{self.tyre_front.provenance.label}], "
            f"rear {self.tyre_rear.reference} "
            f"[{self.tyre_rear.provenance.label}]",
        ]
        for note in self.notes:
            lines.append(f"  note: {note}")
        for warning in self.warnings:
            lines.append(f"  warning: {warning}")
        return "\n".join(lines)
