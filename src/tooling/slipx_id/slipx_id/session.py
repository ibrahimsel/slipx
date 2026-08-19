# Copyright 2026 The SlipX Authors
# SPDX-License-Identifier: Apache-2.0

"""An identification session: bags in, fitted parameters out.

A session file names the car's bench constants, the topics, the bags and
what each one recorded, and the provenance of the whole exercise. Running
it applies the staged fits in the manoeuvre library's order and returns a
:class:`FitOutcome` that :mod:`slipx_id.emit` turns into a car directory.

Everything the session does not identify comes from the file's ``fixed``
block (datasheet and configuration values), and everything the file does
not state is refused by name. The one exception is inside the transient
stage's replay: when no launch was recorded, the replay's ESC uses the
core's defaults to hold speed, which is recorded as a note because it
affects fidelity of the fit, not the emitted claim.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Mapping, Optional, Sequence, Tuple

import yaml

import slipx

from .optimise import FitReport
from .reconstruct import Bench
from .rosbag import DEFAULT_TOPIC_MAP, TopicMap, read_recording
from .stages import (
    LateralFit,
    fit_c_kappa,
    fit_coastdown,
    fit_esc,
    fit_lateral,
    fit_mu_x0,
    fit_transient,
)

ROLES = (
    "coastdown",
    "launch",
    "skidpad",
    "ramp_steer",
    "step_steer",
    "circle_to_slip",
)

#: What the fixed block must carry: the values identification does not
#: produce, with the manoeuvre library's coverage table saying why each one
#: is measured or read off a datasheet instead.
FIXED_FIELDS = (
    "ixx",
    "iyy",
    "steer_max",
    "steer_rate_max",
    "accel_max",
    "decel_max",
    "torque_per_amp",
    "drive_efficiency",
    "pack_nominal_v",
    "pack_v_full",
    "pack_v_empty",
    "pack_capacity_ah",
    "pack_internal_resistance",
    "regen_current_max",
    "layout",
    "differential",
)

BENCH_FIELDS = (
    "mass",
    "lf",
    "lr",
    "h_cog",
    "track_front",
    "track_rear",
    "wheel_radius",
    "izz",
    "length",
    "width",
)


@dataclass(frozen=True)
class SessionManoeuvre:
    bag: Path
    role: str
    ballast_mass: float = 0.0
    traction_window: Optional[Tuple[float, float]] = None


@dataclass(frozen=True)
class Session:
    name: str
    surface: str
    compound: str
    bench: Bench
    length: float
    width: float
    fixed: Mapping[str, object]
    topics: TopicMap
    manoeuvres: Tuple[SessionManoeuvre, ...]
    provenance: Mapping[str, str]
    output: Path


def _require(mapping: Mapping, key: str, where: str):
    if key not in mapping:
        raise ValueError(
            f"the session file's {where} block is missing '{key}'; nothing "
            f"is defaulted, because a parameter nobody stated is worse than "
            f"one that does not exist (ADR-0025)"
        )
    return mapping[key]


def load_session(path) -> Session:
    """Parse and check a session file, resolving paths against it."""
    source = Path(path)
    document = yaml.safe_load(source.read_text(encoding="utf-8"))
    if not isinstance(document, dict):
        raise ValueError(f"{source} does not contain a mapping")
    base = source.resolve().parent

    car = _require(document, "car", "top-level")
    name = _require(car, "name", "car")
    surface = _require(car, "surface", "car")
    compound = _require(car, "compound", "car")
    bench_block = _require(car, "bench", "car")
    for field_name in BENCH_FIELDS:
        _require(bench_block, field_name, "car.bench")
    bench = Bench(
        mass=float(bench_block["mass"]),
        lf=float(bench_block["lf"]),
        lr=float(bench_block["lr"]),
        h_cog=float(bench_block["h_cog"]),
        track_front=float(bench_block["track_front"]),
        track_rear=float(bench_block["track_rear"]),
        wheel_radius=float(bench_block["wheel_radius"]),
        izz=float(bench_block["izz"]),
    )

    fixed = dict(_require(document, "fixed", "top-level"))
    for field_name in FIXED_FIELDS:
        _require(fixed, field_name, "fixed")
    if fixed["differential"] == "lsd":
        _require(fixed, "lsd_preload", "fixed")

    topics_block = _require(document, "topics", "top-level")
    wheel_names = dict(
        _require(topics_block, "wheel_names", "topics")
    )
    for wheel in ("FL", "FR", "RL", "RR"):
        _require(wheel_names, wheel, "topics.wheel_names")
    topics = TopicMap(
        pose=_require(topics_block, "pose", "topics"),
        imu=_require(topics_block, "imu", "topics"),
        wheels=_require(topics_block, "wheels", "topics"),
        drive=_require(topics_block, "drive", "topics"),
        wheel_names=wheel_names,
    )

    manoeuvre_block = _require(document, "manoeuvres", "top-level")
    if not manoeuvre_block:
        raise ValueError("the session lists no manoeuvres; there is no fit")
    manoeuvres: List[SessionManoeuvre] = []
    for index, entry in enumerate(manoeuvre_block):
        role = _require(entry, "role", f"manoeuvres[{index}]")
        if role not in ROLES:
            raise ValueError(
                f"manoeuvres[{index}] role '{role}' is not one of the "
                f"manoeuvre library's six: {', '.join(ROLES)}"
            )
        bag = base / str(_require(entry, "bag", f"manoeuvres[{index}]"))
        window = entry.get("traction_window")
        manoeuvres.append(
            SessionManoeuvre(
                bag=bag,
                role=role,
                ballast_mass=float(entry.get("ballast_mass", 0.0)),
                traction_window=(
                    (float(window[0]), float(window[1]))
                    if window is not None
                    else None
                ),
            )
        )

    provenance = dict(document.get("provenance", {}))
    output = base / str(_require(document, "output", "top-level"))

    return Session(
        name=str(name),
        surface=str(surface),
        compound=str(compound),
        bench=bench,
        length=float(bench_block["length"]),
        width=float(bench_block["width"]),
        fixed=fixed,
        topics=topics,
        manoeuvres=tuple(manoeuvres),
        provenance=provenance,
        output=output,
    )


@dataclass(frozen=True)
class FitOutcome:
    """Everything a session produced, before it becomes files."""

    session: Session
    resistances: FitReport
    lateral: LateralFit
    esc: Optional[FitReport]
    c_kappa: Optional[FitReport]
    mu_x0: Optional[Tuple[float, float]]  # (value, top-decile spread)
    transient: Optional[FitReport]
    notes: Tuple[str, ...]

    def reports(self) -> Dict[str, FitReport]:
        out: Dict[str, FitReport] = {
            "coastdown": self.resistances,
            "lateral": self.lateral.report,
        }
        if self.esc is not None:
            out["esc"] = self.esc
        if self.c_kappa is not None:
            out["c_kappa"] = self.c_kappa
        if self.transient is not None:
            out["transient"] = self.transient
        return out


def _ballasted_bench(bench: Bench, extra: float) -> Bench:
    """Ballast fixed over the CoG changes the mass and nothing else; the
    circle-to-slip doc makes checking that the balance point stayed put part
    of the procedure."""
    return Bench(
        mass=bench.mass + extra,
        lf=bench.lf,
        lr=bench.lr,
        h_cog=bench.h_cog,
        track_front=bench.track_front,
        track_rear=bench.track_rear,
        wheel_radius=bench.wheel_radius,
        izz=bench.izz,
    )


def run_session(session: Session, *, sample_stride: int = 2) -> FitOutcome:
    """The staged fits, in the manoeuvre library's order."""
    notes: List[str] = []
    by_role: Dict[str, List] = {role: [] for role in ROLES}
    windows: List[Tuple[object, Tuple[float, float]]] = []
    for manoeuvre in session.manoeuvres:
        bench = (
            _ballasted_bench(session.bench, manoeuvre.ballast_mass)
            if manoeuvre.ballast_mass
            else session.bench
        )
        recording = read_recording(
            manoeuvre.bag,
            bench,
            topic_map=session.topics,
            name=f"{manoeuvre.role}:{manoeuvre.bag.name}",
        )
        by_role[manoeuvre.role].append(recording)
        if manoeuvre.traction_window is not None:
            windows.append((recording, manoeuvre.traction_window))

    if not by_role["coastdown"]:
        raise ValueError(
            "no coastdown was recorded, and every later stage consumes its "
            "resistances; it is the first manoeuvre for a reason"
        )
    resistances = fit_coastdown(by_role["coastdown"])

    lateral_recordings = (
        by_role["skidpad"] + by_role["ramp_steer"] + by_role["circle_to_slip"]
    )
    if not lateral_recordings:
        raise ValueError(
            "no skidpad, ramp steer or circle-to-slip was recorded; the "
            "lateral fit has nothing to fit"
        )
    lateral = fit_lateral(
        lateral_recordings, session.bench, sample_stride=sample_stride
    )

    esc: Optional[FitReport] = None
    c_kappa: Optional[FitReport] = None
    mu_x0: Optional[Tuple[float, float]] = None
    if by_role["launch"]:
        launch = by_role["launch"][0]
        esc = fit_esc(launch, resistances)
        c_kappa = fit_c_kappa(launch, resistances, lateral.k_mu)
        launch_windows = [w for r, w in windows if r in by_role["launch"]]
        if launch_windows:
            mu_x0 = fit_mu_x0(
                launch, resistances, lateral.k_mu, launch_windows[0]
            )
        else:
            notes.append(
                "no launch declared a traction-limited window, so mu_x0 was "
                "not identified and stays provisional; the straight-line "
                "procedure says whose judgement that window is"
            )
    else:
        notes.append(
            "no launch was recorded: the ESC curve, c_kappa and mu_x0 were "
            "not identified. Without c_kappa the emitted car loads for L0 "
            "and L1 and is refused for L2 by name (ADR-0025)"
        )

    transient: Optional[FitReport] = None
    if by_role["step_steer"]:
        base = _transient_base(session, resistances, lateral, esc, c_kappa, notes)
        transient = fit_transient(by_role["step_steer"], base)
    else:
        notes.append(
            "no step steer was recorded: the relaxation length and the "
            "servo pair were not identified and stay provisional"
        )

    return FitOutcome(
        session=session,
        resistances=resistances,
        lateral=lateral,
        esc=esc,
        c_kappa=c_kappa,
        mu_x0=mu_x0,
        transient=transient,
        notes=tuple(notes),
    )


def _transient_base(
    session: Session,
    resistances: FitReport,
    lateral: LateralFit,
    esc: Optional[FitReport],
    c_kappa: Optional[FitReport],
    notes: List[str],
) -> "slipx.VehicleParams":
    """The parameter set the transient replay holds fixed."""
    bench = session.bench
    fixed = session.fixed
    params = slipx.VehicleParams()
    params.mass = bench.mass
    params.izz = bench.izz
    params.ixx = float(fixed["ixx"])
    params.iyy = float(fixed["iyy"])
    params.lf = bench.lf
    params.lr = bench.lr
    params.track_front = bench.track_front
    params.track_rear = bench.track_rear
    params.h_cog = bench.h_cog
    params.wheel_radius = bench.wheel_radius
    params.steer_max = float(fixed["steer_max"])
    params.steer_rate_max = float(fixed["steer_rate_max"])
    params.accel_max = float(fixed["accel_max"])
    params.decel_max = float(fixed["decel_max"])
    params.roll_resist = resistances.value("roll_resist")
    params.drag_coeff = resistances.value("drag_coeff")
    params.c_alpha_f = lateral.c_alpha_f
    params.c_alpha_r = lateral.c_alpha_r
    params.mu_clip = lateral.mu_y0
    for tyre in (params.tyre_front, params.tyre_rear):
        tyre.mu_y0 = lateral.mu_y0
        tyre.k_mu = lateral.k_mu
        tyre.shape_c = lateral.shape_c
        tyre.curvature_e = lateral.curvature_e
    params.torque_per_amp = float(fixed["torque_per_amp"])
    params.drive_efficiency = float(fixed["drive_efficiency"])
    params.pack_nominal_v = float(fixed["pack_nominal_v"])
    params.pack_v_full = float(fixed["pack_v_full"])
    params.pack_v_empty = float(fixed["pack_v_empty"])
    params.pack_capacity_ah = float(fixed["pack_capacity_ah"])
    params.pack_internal_resistance = float(fixed["pack_internal_resistance"])
    params.regen_current_max = float(fixed["regen_current_max"])
    layout = {
        "2WD_rear": slipx.DriveLayout.RearWheelDrive,
        "2WD_front": slipx.DriveLayout.FrontWheelDrive,
        "4WD": slipx.DriveLayout.AllWheelDrive,
    }
    differential = {
        "open": slipx.Differential.Open,
        "spool": slipx.Differential.Spool,
        "lsd": slipx.Differential.Lsd,
    }
    params.layout = layout[str(fixed["layout"])]
    params.differential = differential[str(fixed["differential"])]
    if str(fixed["differential"]) == "lsd":
        params.lsd_preload = float(fixed["lsd_preload"])
    if c_kappa is not None:
        params.c_kappa = c_kappa.value("c_kappa")
    if esc is not None:
        radius = bench.wheel_radius
        params.torque_stall = esc.value("stall_force") * radius
        params.omega_free = esc.value("free_speed") / radius
        params.current_max = (
            esc.value("cap_force") * radius / float(fixed["torque_per_amp"])
        )
    else:
        notes.append(
            "the transient replay's ESC used the core's defaults to hold "
            "speed, because no launch identified a curve; this touches the "
            "fit's speed hold, not any emitted value"
        )
    return params


def identified_v_max(outcome: FitOutcome) -> Optional[Tuple[float, str]]:
    """Where the fitted drive curve meets the fitted resistances.

    Returns the speed and a note saying whether it was observed or
    extrapolated, or None when no ESC curve was identified.
    """
    if outcome.esc is None:
        return None
    stall = outcome.esc.value("stall_force")
    free = outcome.esc.value("free_speed")
    cap = outcome.esc.value("cap_force")
    rr = outcome.resistances.value("roll_resist")
    drag = outcome.resistances.value("drag_coeff")
    weight = outcome.session.bench.weight

    def net(v: float) -> float:
        curve = min(cap, stall * (1.0 - v / free))
        return curve - (rr * weight + drag * v * v)

    lo, hi = 0.0, free
    if net(lo) <= 0.0:
        return None
    for _ in range(200):
        mid = 0.5 * (lo + hi)
        if net(mid) > 0.0:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi), (
        "identified from the fitted torque curve against the fitted "
        "resistances"
    )
