# Copyright 2026 The SlipX Authors
# SPDX-License-Identifier: Apache-2.0

"""Reading a car directory (SCH-01 to SCH-05).

The reference parser. It reads a directory, checks it against the schemas and
the rules, resolves tyre references, and produces a ``Car`` whose ``params``
field is ready to hand to the core.

Order of operations, which matters:

  1. version gate, per file, before anything else is believed (SCH-01)
  2. migration forward to the current minor
  3. JSON Schema validation (SCH-02)
  4. rules JSON Schema cannot express (SCH-03, SCH-04)
  5. tyre resolution (SCH-05)
  6. assembly

Errors accumulate through steps 3 to 5 and are raised together at the end.
Somebody with four mistakes in a car file should need one run to find them all.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import yaml

from . import rules
from .errors import (
    CarDirectoryError,
    FieldError,
    Report,
    SchemaVersionError,
    Warning_,
)
from .migrate import migrate
from .model import Car, Provenance, Tyre, VehicleParameters
from .validate import validate_document
from .version import SCHEMA_VERSION, Version, compatibility

#: Where tyre files are looked for when car.yaml does not say. A path
#: convention, not a defaulted parameter; the loader records which directory it
#: used.
DEFAULT_TYRES_DIR = "tyres"


def _read_yaml(path: Path) -> Dict[str, Any]:
    if not path.exists():
        raise CarDirectoryError(f"{path} does not exist")
    with path.open(encoding="utf-8") as handle:
        # safe_load, never load: a car file is a data file, and one that can
        # construct arbitrary Python objects is a car file that can be a
        # supply-chain problem the first time somebody downloads one from the
        # registry.
        document = yaml.safe_load(handle)
    if document is None:
        raise CarDirectoryError(f"{path} is empty")
    if not isinstance(document, dict):
        raise CarDirectoryError(f"{path} must contain a mapping at the top level")
    return document


def _gate_version(document: Dict[str, Any], kind: str, path: Path) -> Dict[str, Any]:
    """SCH-01: check the declared version, then migrate to the current one."""
    declared = document.get("schema_version")
    if declared is None:
        raise SchemaVersionError(
            f"{path}: no schema_version. Every file carries one (SCH-01); "
            f"without it the parser cannot know what the fields mean. Add "
            f"schema_version: \"{SCHEMA_VERSION}\""
        )
    if not isinstance(declared, str):
        raise SchemaVersionError(
            f"{path}: schema_version must be a quoted string, not {type(declared).__name__}. "
            f"YAML reads 0.1 as a number and 0.1.0 as a string, which is exactly "
            f"the kind of difference that should not depend on how many dots "
            f"somebody typed"
        )

    try:
        version = Version.parse(declared)
    except ValueError as exc:
        raise SchemaVersionError(f"{path}: {exc}") from exc

    usable, reason = compatibility(version)
    if not usable:
        raise SchemaVersionError(f"{path}: {reason}")

    return migrate(kind, document, version)


def _provenance_from(document: Dict[str, Any]) -> Provenance:
    return Provenance(
        label=document["label"],
        source=document["source"],
        method=document["method"],
        date=document["date"],
        contributor=document.get("contributor", ""),
        vehicle=document.get("vehicle", ""),
        residuals=document.get("residuals", {}),
        validation_report=document.get("validation_report", ""),
        notes=document.get("notes", ""),
    )


def _resolve_tyre(
    reference: Dict[str, Any],
    tyres_dir: Path,
    axle: str,
    report: Report,
) -> Optional[Tyre]:
    """SCH-05: resolve a (compound, surface) pair against local files.

    Registry resolution arrives with the registry in P2. Until then the search
    path is the car's own tyres directory, and a reference that does not
    resolve is a refusal rather than a fallback: running a car on tyres nobody
    can name is how asphalt coefficients end up on a sports hall floor.
    """
    compound = reference.get("compound")
    surface = reference.get("surface")
    filename = f"{compound}_{surface}.yaml"
    path = tyres_dir / filename

    if not path.exists():
        report.errors.append(
            FieldError(
                path=f"tyres.{axle}",
                message=(
                    f"no tyre file for ({compound}, {surface}); looked for "
                    f"{path}. Tyres are referenced rather than embedded, so an "
                    f"unresolvable reference stops the run instead of "
                    f"substituting some other surface's coefficients"
                ),
                file="dynamics.yaml",
                requirement="SCH-05",
            )
        )
        return None

    document = _gate_version(_read_yaml(path), "tyre", path)
    errors = validate_document("tyre", document, file=filename)
    report.errors.extend(errors)
    if errors:
        return None

    if document["compound"] != compound or document["surface"] != surface:
        report.errors.append(
            FieldError(
                path=f"tyres.{axle}",
                message=(
                    f"{filename} declares ({document['compound']}, "
                    f"{document['surface']}) but was resolved for "
                    f"({compound}, {surface}); the file name and its contents "
                    f"disagree"
                ),
                file=filename,
                requirement="SCH-05",
            )
        )
        return None

    report.warnings.extend(rules.check_tyre_plausibility(document, filename))

    friction = document["friction"]
    return Tyre(
        compound=document["compound"],
        surface=document["surface"],
        c_alpha=float(document["linear"]["c_alpha"]),
        mu_y0=float(friction["mu_y0"]),
        mu_x0=float(friction["mu_x0"]),
        k_mu=friction.get("k_mu"),
        nominal_load=document.get("nominal_load"),
        mf_lite=document.get("mf_lite"),
        sigma=document.get("relaxation", {}).get("sigma"),
        c_kappa=document["linear"].get("c_kappa"),
        provenance=_provenance_from(document["provenance"]),
    )


def _weakest_label(labels: List[str]) -> str:
    """The weakest claim among several, which is the only honest one to make.

    A car whose chassis was measured and whose tyres were guessed is a car with
    guessed parameters (NFR-08).
    """
    for label in ("provisional", "identified", "measured"):
        if label in labels:
            return label
    return "provisional"


def load_car(directory: str | Path, strict: bool = False) -> Car:
    """Load and validate a car directory.

    Args:
        directory: Path to the directory holding ``car.yaml``.
        strict: Treat SCH-04 plausibility warnings as errors. Off by default,
            because a warning means "possible but unusual" and refusing those
            would make the parser wrong about unusual cars. A competition
            harness should turn it on.

    Returns:
        A validated :class:`~slipx_schema.model.Car`.

    Raises:
        CarDirectoryError: a file named by the manifest is missing.
        SchemaVersionError: a file declares a version this parser cannot read.
        ValidationError: one or more fields failed validation, all of them
            reported together.
    """
    root = Path(directory)
    if not root.is_dir():
        raise CarDirectoryError(f"{root} is not a directory")

    manifest_path = root / "car.yaml"
    manifest = _gate_version(_read_yaml(manifest_path), "car", manifest_path)

    report = Report()
    report.errors.extend(validate_document("car", manifest, file="car.yaml"))
    report.raise_if_failed(f"{manifest_path} is not a valid car manifest")

    notes: List[str] = []

    # ------------------------------------------------------------ dynamics
    dynamics_path = root / manifest["dynamics"]
    dynamics = _gate_version(_read_yaml(dynamics_path), "dynamics", dynamics_path)
    report.errors.extend(
        validate_document("dynamics", dynamics, file=manifest["dynamics"])
    )

    # -------------------------------------------------------------- limits
    limits_path = root / manifest["limits"]
    limits = _gate_version(_read_yaml(limits_path), "limits", limits_path)
    report.errors.extend(validate_document("limits", limits, file=manifest["limits"]))

    # ---------------------------------------------------------- provenance
    provenance_path = root / manifest["provenance"]
    provenance_doc = _gate_version(
        _read_yaml(provenance_path), "provenance", provenance_path
    )
    report.errors.extend(
        validate_document("provenance", provenance_doc, file=manifest["provenance"])
    )

    # ------------------------------------------------------------- sensors
    sensors: List[Dict[str, Any]] = []
    if "sensors" in manifest:
        sensors_path = root / manifest["sensors"]
        sensors_doc = _gate_version(_read_yaml(sensors_path), "sensors", sensors_path)
        report.errors.extend(
            validate_document("sensors", sensors_doc, file=manifest["sensors"])
        )
        sensors = sensors_doc.get("sensors", [])
    else:
        notes.append(
            "no sensors file; nothing consumes one until slipx_sense lands in P1"
        )

    # Structural failures make every check below meaningless, so stop here.
    report.raise_if_failed(f"{root} did not validate")

    # ------------------------------------------------------ rules (SCH-03/04)
    geometry = dynamics["geometry"]
    report.errors.extend(
        rules.check_dimensional_legality(geometry, file=manifest["dynamics"])
    )
    geo_errors, geo_warnings = rules.check_geometric_consistency(
        geometry, file=manifest["dynamics"]
    )
    report.errors.extend(geo_errors)
    report.warnings.extend(geo_warnings)

    inertia_errors, inertia_warnings = rules.check_inertia(
        dynamics["inertia"], dynamics["mass"], geometry, file=manifest["dynamics"]
    )
    report.errors.extend(inertia_errors)
    report.warnings.extend(inertia_warnings)

    # ------------------------------------------------------- tyres (SCH-05)
    tyres_dir = root / manifest.get("tyres_dir", DEFAULT_TYRES_DIR)
    if "tyres_dir" not in manifest:
        notes.append(f"tyre files resolved from '{DEFAULT_TYRES_DIR}/' by convention")

    tyre_front = _resolve_tyre(dynamics["tyres"]["front"], tyres_dir, "front", report)
    tyre_rear = _resolve_tyre(dynamics["tyres"]["rear"], tyres_dir, "rear", report)

    report.raise_if_failed(f"{root} did not validate")
    assert tyre_front is not None and tyre_rear is not None

    if strict and report.warnings:
        report.errors.extend(
            FieldError(
                path=w.path,
                message=w.message + " (strict mode: warnings are errors)",
                file=w.file,
                requirement=w.requirement,
            )
            for w in report.warnings
        )
        report.raise_if_failed(f"{root} did not validate in strict mode")

    # ------------------------------------------------------------- assembly
    provenance = _provenance_from(provenance_doc)

    # Cornering stiffness in a tyre file is per tyre; an axle has two of them,
    # and the single-track tiers want the axle value.
    c_alpha_f = 2.0 * tyre_front.c_alpha
    c_alpha_r = 2.0 * tyre_rear.c_alpha

    # L1 clips lateral force at a single mu (see l1_bicycle.cpp), so a car with
    # different compounds front and rear gets the weaker of the two. Said out
    # loud rather than averaged quietly. Per-axle friction arrives with MF-lite
    # at L2, where it belongs.
    mu_clip = min(tyre_front.mu_y0, tyre_rear.mu_y0)
    if tyre_front.mu_y0 != tyre_rear.mu_y0:
        notes.append(
            f"front and rear tyres differ in peak friction "
            f"({tyre_front.mu_y0} and {tyre_rear.mu_y0}); L1 has one friction "
            f"clip and takes the lower, {mu_clip}. Per-axle friction arrives "
            f"with L2"
        )

    v_eps = dynamics.get("numerics", {}).get("v_eps")
    if v_eps is None:
        notes.append(
            "no numerics.v_eps; the core's own slip-angle speed floor applies. "
            "It is a numerical mitigation rather than a property of the car, "
            "and it is recorded in the run manifest either way"
        )

    # The core carries ONE longitudinal slip stiffness, because the run that
    # identifies it is a straight-line acceleration and cannot separate the
    # axles. Two tyre files claiming different values are reduced to the mean,
    # out loud; either file lacking the field leaves None, and the L2 refusal
    # downstream names the file (ADR-0025, ADR-0030).
    c_kappa: Optional[float] = None
    if tyre_front.c_kappa is not None and tyre_rear.c_kappa is not None:
        c_kappa = 0.5 * (float(tyre_front.c_kappa) + float(tyre_rear.c_kappa))
        if tyre_front.c_kappa != tyre_rear.c_kappa:
            notes.append(
                f"front and rear tyres claim different longitudinal slip "
                f"stiffness ({tyre_front.c_kappa} and {tyre_rear.c_kappa} N "
                f"per unit slip); the core carries one value, because the "
                f"manoeuvre that identifies it cannot separate the axles, so "
                f"the mean {c_kappa} is used"
            )

    labels = [provenance.label, tyre_front.provenance.label, tyre_rear.provenance.label]
    effective_label = _weakest_label(labels)
    if effective_label != provenance.label:
        notes.append(
            f"the car is labelled '{provenance.label}' but its tyres are "
            f"'{_weakest_label(labels[1:])}', so the set as a whole is "
            f"'{effective_label}'"
        )

    params = VehicleParameters(
        mass=float(dynamics["mass"]),
        izz=float(dynamics["inertia"]["izz"]),
        ixx=float(dynamics["inertia"]["ixx"]),
        iyy=float(dynamics["inertia"]["iyy"]),
        lf=float(geometry["lf"]),
        lr=float(geometry["lr"]),
        track_front=float(geometry["track_front"]),
        track_rear=float(geometry["track_rear"]),
        h_cog=float(geometry["h_cog"]),
        wheel_radius=float(geometry["wheel_radius"]),
        c_alpha_f=c_alpha_f,
        c_alpha_r=c_alpha_r,
        mu_clip=mu_clip,
        accel_max=float(limits["drivetrain"]["accel_max"]),
        decel_max=float(limits["drivetrain"]["decel_max"]),
        v_max=float(limits["drivetrain"]["v_max"]),
        steer_max=float(limits["steering"]["max_angle"]),
        drag_coeff=float(dynamics["resistance"]["drag_coeff"]),
        roll_resist=float(dynamics["resistance"]["roll_resist"]),
        provenance_label=effective_label,
        v_eps=float(v_eps) if v_eps is not None else None,
        c_kappa=c_kappa,
    )

    return Car(
        name=manifest["name"],
        schema_version=manifest["schema_version"],
        directory=root,
        params=params,
        provenance=provenance,
        tyre_front=tyre_front,
        tyre_rear=tyre_rear,
        description=manifest.get("description", ""),
        sensors=sensors,
        notes=notes,
        warnings=[str(w) for w in report.warnings],
        raw={
            "car": manifest,
            "dynamics": dynamics,
            "limits": limits,
            "provenance": provenance_doc,
        },
    )


def validate_car(directory: str | Path) -> Report:
    """Validate without assembling, returning everything found.

    For a linting tool or a registry submission check, where the useful output
    is the complete list of problems rather than an exception on the first
    structural one.
    """
    report = Report()
    try:
        car = load_car(directory)
        report.warnings.extend(
            Warning_(path="", message=w) for w in car.warnings
        )
    except Exception as exc:  # noqa: BLE001 - reported, not swallowed
        from .errors import ValidationError

        if isinstance(exc, ValidationError):
            report.errors.extend(exc.errors)
        else:
            report.errors.append(FieldError(path="", message=str(exc)))
    return report
