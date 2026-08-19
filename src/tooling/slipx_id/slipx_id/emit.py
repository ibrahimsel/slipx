# Copyright 2026 The SlipX Authors
# SPDX-License-Identifier: Apache-2.0

"""From a fit outcome to a car directory (ADR-0040).

The output is a complete, loadable car directory: dynamics, two tyre files,
limits, manifest, and a provenance file whose residuals block carries every
fitted parameter with its one-sigma interval. Emission refuses, naming the
fields, when the provenance is not populated; parameters no stage
identified keep provisional values and are named in the notes rather than
silently blended in. Before reporting success the directory is loaded back
through ``slipx_schema``, so an emitted car that cannot load is an error
here and a plausibility warning is on the terminal now, not in a bug report
later.
"""

from __future__ import annotations

import hashlib
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import yaml

import slipx
import slipx_schema
from slipx_schema.version import SCHEMA_VERSION

from .optimise import FitReport
from .session import FitOutcome, identified_v_max

#: Units for the residuals block, keyed by emitted parameter name.
_UNITS = {
    "roll_resist": "-",
    "drag_coeff": "kg/m",
    "c_alpha_f": "N/rad",
    "c_alpha_r": "N/rad",
    "mu_y0": "-",
    "mu_x0": "-",
    "k_mu": "-",
    "shape_c": "-",
    "curvature_e": "-",
    "c_kappa": "N",
    "relax_length": "m",
    "steer_bandwidth": "rad/s",
    "steer_damping": "-",
    "torque_stall": "N m",
    "omega_free": "rad/s",
    "current_max": "A",
}

_PROVENANCE_REQUIRED = ("contributor", "source", "method", "date")


@dataclass(frozen=True)
class EmittedCar:
    directory: Path
    warnings: Tuple[str, ...]
    notes: Tuple[str, ...]


def _residual_entry(report: FitReport, name: str, scale: float = 1.0) -> Dict:
    value = report.value(name) * scale
    sigma = report.sigma(name)
    entry: Dict[str, object] = {
        "value": float(value),
        "stddev": float(abs(sigma * scale)) if sigma is not None else 0.0,
    }
    unit = _UNITS.get(name)
    if unit is not None:
        entry["unit"] = unit
    return entry


def _bag_digest(path: Path) -> str:
    """SHA-256 over the bag's files, sorted by name, so the provenance is
    tied to the bytes the fit actually read (ADR-0041)."""
    digest = hashlib.sha256()
    files = sorted(path.iterdir()) if path.is_dir() else [path]
    for file in files:
        if file.is_file():
            digest.update(file.name.encode("utf-8"))
            digest.update(file.read_bytes())
    return digest.hexdigest()


def _dump(document: Dict, path: Path, header: str) -> None:
    text = "".join(f"# {line}\n" for line in header.splitlines())
    text += yaml.safe_dump(document, sort_keys=False)
    path.write_text(text, encoding="utf-8")


def check_provenance(provenance) -> None:
    """The emission refusal, callable before the expensive part.

    The CLI runs this before fitting anything, because discovering after a
    minute of optimisation that nobody filled in the contributor is a
    refusal delivered at the least useful moment.
    """
    missing = [
        key
        for key in _PROVENANCE_REQUIRED
        if not str(provenance.get(key, "")).strip()
    ]
    if missing:
        raise ValueError(
            "refusing to emit a parameter set without a populated provenance "
            "block: " + ", ".join(missing) + " missing. An identified set "
            "that cannot say who identified it, from what and how, is a "
            "provisional set with a better title (ADR-0013)"
        )


def emit_car_directory(outcome: FitOutcome) -> EmittedCar:
    session = outcome.session
    provenance = session.provenance
    check_provenance(provenance)

    directory = session.output
    if directory.exists():
        raise FileExistsError(
            f"{directory} already exists; refusing to overwrite an emitted "
            f"car. Choose a fresh output directory"
        )

    notes: List[str] = list(outcome.notes)
    fixed = session.fixed
    bench = session.bench
    lateral = outcome.lateral
    transient = outcome.transient

    header = (
        f"Emitted by slipx-id for '{session.name}' on {provenance['date']}.\n"
        f"Identified parameters carry residuals in provenance.yaml; anything "
        f"not listed there was measured, read off a datasheet, or kept "
        f"provisional, and the notes say which."
    )

    # ------------------------------------------------------------ residuals
    residuals: Dict[str, Dict] = {}
    for name in ("roll_resist", "drag_coeff"):
        residuals[name] = _residual_entry(outcome.resistances, name)
    for name in lateral.report.names:
        residuals[name] = _residual_entry(lateral.report, name)
    if outcome.c_kappa is not None:
        residuals["c_kappa"] = _residual_entry(outcome.c_kappa, "c_kappa")
    if outcome.mu_x0 is not None:
        value, spread = outcome.mu_x0
        residuals["mu_x0"] = {
            "value": float(value),
            "stddev": float(spread),
            "unit": "-",
        }
    if transient is not None:
        for name in transient.names:
            residuals[name] = _residual_entry(transient, name)
    radius = bench.wheel_radius
    if outcome.esc is not None:
        residuals["torque_stall"] = _residual_entry(
            outcome.esc, "stall_force", radius
        )
        residuals["omega_free"] = _residual_entry(
            outcome.esc, "free_speed", 1.0 / radius
        )

    # --------------------------------------------------------------- values
    if transient is not None:
        shape_c = transient.value("shape_c")
        curvature_e = transient.value("curvature_e")
        relax = transient.value("relax_length")
    else:
        shape_c = lateral.shape_c
        curvature_e = lateral.curvature_e
        relax = slipx.TyreCoefficients().relax_length
        notes.append(
            "relaxation.sigma is provisional (no step steer was recorded)"
        )

    if outcome.mu_x0 is not None:
        mu_x0 = float(outcome.mu_x0[0])
    else:
        mu_x0 = slipx.TyreCoefficients().mu_x0
        notes.append(
            "friction.mu_x0 is provisional (no traction-limited window was "
            "declared)"
        )

    # The flat cap the launch identified is the current limit only when the
    # tyre was not what bound it. A launch with a declared traction window
    # identified the tyre instead, so the ESC configuration value is the
    # honest source for the current limit there.
    current_max: Optional[float] = None
    if outcome.esc is not None and outcome.mu_x0 is None:
        current_max = (
            outcome.esc.value("cap_force")
            * radius
            / float(fixed["torque_per_amp"])
        )
        residuals["current_max"] = _residual_entry(
            outcome.esc, "cap_force", radius / float(fixed["torque_per_amp"])
        )
    elif "current_max" in fixed:
        current_max = float(fixed["current_max"])
        if outcome.mu_x0 is not None:
            notes.append(
                "the launch's flat cap was the tyre (a traction window was "
                "declared), so electrical.current_max comes from the fixed "
                "block, not the fit"
            )
    else:
        notes.append(
            "electrical.current_max is absent: the launch was traction "
            "limited or missing, and the fixed block does not state the ESC "
            "configuration value. L2 will refuse this car by name until it "
            "is added"
        )

    v_max_result = identified_v_max(outcome)
    if v_max_result is not None:
        v_max, v_max_note = v_max_result
        notes.append(f"drivetrain.v_max {v_max:.2f} m/s: {v_max_note}")
        # A configured speed clip is not observable from a launch that never
        # reached it, and the fitted curve is identified at the session's
        # pack state, so the crossing can sit above the clip the team knows
        # they configured. The stated clip caps the extrapolation.
        if "v_max" in fixed and float(fixed["v_max"]) < v_max:
            notes.append(
                f"drivetrain.v_max capped at the configured {fixed['v_max']}"
                f" m/s, below the curve crossing; the clip is configuration,"
                f" not physics, and the run never reached it"
            )
            v_max = float(fixed["v_max"])
    elif "v_max" in fixed:
        v_max = float(fixed["v_max"])
    else:
        raise ValueError(
            "v_max is neither identified (no launch reached the curve) nor "
            "stated in the fixed block; nothing is defaulted (ADR-0025)"
        )

    # ---------------------------------------------------------------- files
    directory.mkdir(parents=True)
    (directory / "tyres").mkdir()

    provenance_common = {
        "schema_version": SCHEMA_VERSION,
        "label": "identified",
        "source": str(provenance["source"]),
        "method": str(provenance["method"]),
        "date": str(provenance["date"]),
        "contributor": str(provenance["contributor"]),
    }
    if provenance.get("vehicle"):
        provenance_common["vehicle"] = str(provenance["vehicle"])

    # One tyre file or two. The same physical tyre on both axles must show
    # stiffnesses related by the ADR-0039 load law; when the fit agrees with
    # that within tolerance, one file at the front's static load is the
    # honest emission and the loader restates the rear exactly. When the
    # axles disagree beyond it, the car is genuinely wearing two different
    # tyres and gets two files, each named for its axle.
    load_law_rear = lateral.c_alpha_f * (
        bench.static_rear_per_tyre / bench.static_front_per_tyre
    ) ** (1.0 - lateral.k_mu)
    sigma_f = lateral.report.sigma("c_alpha_f") or 0.0
    sigma_r = lateral.report.sigma("c_alpha_r") or 0.0
    tolerance = max(0.02 * load_law_rear, 3.0 * (sigma_f + sigma_r))
    one_tyre = abs(lateral.c_alpha_r - load_law_rear) <= tolerance

    if one_tyre:
        axle_files = [
            (session.compound, lateral.c_alpha_f, bench.static_front_per_tyre,
             "both axles; the rear is this same tyre restated at its own "
             "static load (ADR-0039)"),
        ]
        front_reference = rear_reference = session.compound
        notes.append(
            f"one tyre file: the fitted axle stiffnesses agree with a "
            f"single tyre under the load law "
            f"({lateral.c_alpha_r:.1f} against {load_law_rear:.1f} N/rad "
            f"predicted for the rear)"
        )
    else:
        axle_files = [
            (f"{session.compound}-front", lateral.c_alpha_f,
             bench.static_front_per_tyre, "the front axle"),
            (f"{session.compound}-rear", lateral.c_alpha_r,
             bench.static_rear_per_tyre, "the rear axle"),
        ]
        front_reference = f"{session.compound}-front"
        rear_reference = f"{session.compound}-rear"
        notes.append(
            f"two tyre files: the fitted rear stiffness "
            f"({lateral.c_alpha_r:.1f} N/rad) is not one tyre's load-law "
            f"restatement of the front ({load_law_rear:.1f} N/rad "
            f"predicted), so the axles are described separately"
        )

    for compound, c_alpha_axle, static, where in axle_files:
        linear: Dict[str, float] = {"c_alpha": 0.5 * c_alpha_axle}
        if outcome.c_kappa is not None:
            linear["c_kappa"] = outcome.c_kappa.value("c_kappa")

        # A tyre file is the shareable unit, so its own residuals travel in
        # it: per tyre where the file's value is per tyre.
        axle_name = "c_alpha_f" if "front" in where else "c_alpha_r"
        if one_tyre:
            axle_name = "c_alpha_f"
        tyre_residuals: Dict[str, Dict] = {
            "c_alpha": _residual_entry(lateral.report, axle_name, 0.5),
            "mu_y0": _residual_entry(lateral.report, "mu_y0"),
            "k_mu": _residual_entry(lateral.report, "k_mu"),
        }
        if transient is not None:
            for name in ("shape_c", "curvature_e", "relax_length"):
                tyre_residuals[name] = _residual_entry(transient, name)
        else:
            for name in ("shape_c", "curvature_e"):
                tyre_residuals[name] = _residual_entry(lateral.report, name)
        if outcome.c_kappa is not None:
            tyre_residuals["c_kappa"] = _residual_entry(
                outcome.c_kappa, "c_kappa"
            )
        if outcome.mu_x0 is not None:
            tyre_residuals["mu_x0"] = {
                "value": float(outcome.mu_x0[0]),
                "stddev": float(outcome.mu_x0[1]),
                "unit": "-",
            }

        tyre_doc = {
            "schema_version": SCHEMA_VERSION,
            "compound": compound,
            "surface": session.surface,
            "nominal_load": float(static),
            "linear": linear,
            "friction": {
                "mu_y0": lateral.mu_y0,
                "mu_x0": mu_x0,
                "k_mu": lateral.k_mu,
            },
            "mf_lite": {"C": float(shape_c), "E": float(curvature_e)},
            "relaxation": {"sigma": float(relax)},
            "provenance": dict(
                provenance_common,
                residuals=tyre_residuals,
                notes=(
                    f"Fitted for {where}. Stated at that axle's static "
                    f"per-tyre load. A value without a residual here is "
                    f"provisional, not identified."
                ),
            ),
        }
        _dump(
            tyre_doc,
            directory / "tyres" / f"{compound}_{session.surface}.yaml",
            header,
        )

    dynamics_doc = {
        "schema_version": SCHEMA_VERSION,
        "mass": bench.mass,
        "inertia": {
            "ixx": float(fixed["ixx"]),
            "iyy": float(fixed["iyy"]),
            "izz": bench.izz,
        },
        "geometry": {
            "lf": bench.lf,
            "lr": bench.lr,
            "track_front": bench.track_front,
            "track_rear": bench.track_rear,
            "h_cog": bench.h_cog,
            "wheel_radius": bench.wheel_radius,
            "length": session.length,
            "width": session.width,
        },
        "tyres": {
            "front": {
                "compound": front_reference,
                "surface": session.surface,
            },
            "rear": {"compound": rear_reference, "surface": session.surface},
        },
        "drivetrain": {
            "layout": str(fixed["layout"]),
            "differential": str(fixed["differential"]),
        },
        "resistance": {
            "drag_coeff": outcome.resistances.value("drag_coeff"),
            "roll_resist": outcome.resistances.value("roll_resist"),
        },
    }
    if str(fixed["differential"]) == "lsd":
        dynamics_doc["drivetrain"]["lsd_preload"] = float(fixed["lsd_preload"])
    _dump(dynamics_doc, directory / "dynamics.yaml", header)

    steering: Dict[str, float] = {
        "max_angle": float(fixed["steer_max"]),
        "max_rate": float(fixed["steer_rate_max"]),
    }
    if transient is not None:
        steering["bandwidth"] = transient.value("steer_bandwidth")
        steering["damping"] = transient.value("steer_damping")
    else:
        for key, name in (
            ("steer_bandwidth", "bandwidth"),
            ("steer_damping", "damping"),
        ):
            if key in fixed:
                steering[name] = float(fixed[key])

    limits_doc: Dict[str, object] = {
        "schema_version": SCHEMA_VERSION,
        "steering": steering,
        "drivetrain": {
            "accel_max": float(fixed["accel_max"]),
            "decel_max": float(fixed["decel_max"]),
            "v_max": float(v_max),
        },
    }
    if outcome.esc is not None:
        limits_doc["esc"] = {
            "torque_stall": outcome.esc.value("stall_force") * radius,
            "omega_free": outcome.esc.value("free_speed") / radius,
            "torque_per_amp": float(fixed["torque_per_amp"]),
            "efficiency": float(fixed["drive_efficiency"]),
        }
    electrical: Dict[str, float] = {
        "pack_nominal_v": float(fixed["pack_nominal_v"]),
        "pack_v_full": float(fixed["pack_v_full"]),
        "pack_v_empty": float(fixed["pack_v_empty"]),
        "pack_capacity_ah": float(fixed["pack_capacity_ah"]),
        "pack_internal_resistance": float(fixed["pack_internal_resistance"]),
        "regen_current_max": float(fixed["regen_current_max"]),
    }
    if current_max is not None:
        electrical["current_max"] = current_max
    limits_doc["electrical"] = electrical
    _dump(limits_doc, directory / "limits.yaml", header)

    provenance_doc = dict(
        provenance_common,
        residuals=residuals,
        # The report itself is written after emission (it replays through
        # the emitted car, so the car must exist first); naming it here is
        # the promise the CLI then fulfils in the same run.
        **(
            {"validation_report": "validation.svg"}
            if session.validation
            else {}
        ),
        data=[
            {
                "name": f"{manoeuvre.role}:{manoeuvre.bag.name}",
                "sha256": _bag_digest(manoeuvre.bag),
            }
            for manoeuvre in session.manoeuvres
        ],
        notes="\n".join(notes) if notes else "",
    )
    _dump(provenance_doc, directory / "provenance.yaml", header)

    car_doc = {
        "schema_version": SCHEMA_VERSION,
        "name": session.name,
        "description": (
            f"Identified by slipx-id from {len(session.manoeuvres)} "
            f"manoeuvre recordings on {provenance['date']}."
        ),
        "dynamics": "dynamics.yaml",
        "limits": "limits.yaml",
        "provenance": "provenance.yaml",
    }
    _dump(car_doc, directory / "car.yaml", header)

    # ------------------------------------------------------------- readback
    loaded = slipx_schema.load_car(directory)
    warnings = tuple(str(w) for w in loaded.warnings)
    return EmittedCar(
        directory=directory, warnings=warnings, notes=tuple(notes)
    )
