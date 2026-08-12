# Copyright 2026 The SlipX Authors
# SPDX-License-Identifier: Apache-2.0

"""One recorded run, normalised once, for every sink to consume (SINK-01).

The core hands out a live ``VehicleState`` and a live ``StepDiagnostics`` that
are overwritten by the next step, so anything that wants to look at a run has
to copy the numbers out as it goes. Doing that once, here, is what makes a
sink a formatter rather than a second simulator: a sink receives a
:class:`Recording` and writes it, and every sink receives the same one
(ADR-0028).

**NaN means absent, everywhere in a Recording.** That is the single rule a sink
has to honour, and this module is where it is established. The core already
uses NaN for a diagnostic its tier cannot represent (ADR-0006), so those arrive
that way. The recorded state is the case that needs work: ``state.hpp`` parks a
field an unimplemented tier does not integrate at *zero* rather than NaN, and
says why, which is that the state is hashed and ``hash.hpp`` treats a NaN in a
trajectory as evidence the run is already broken. That reasoning is about the
hash. It does not extend to a plot, where a flat zero line labelled "front left
vertical load" for an L1 run is exactly the believable lie ADR-0006 exists to
prevent. So the zeros are converted to NaN once, here, against the table below,
and no sink has to know which tier represents what.

A Recording is for looking at a run, not for reproducing one. Reproduction is
``Simulation.replay`` over the input log (SIM-07), which is unaffected by
anything in this file.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Mapping, Optional, Tuple

# Wheel order, fixed by conventions.hpp: front-left, front-right, rear-left,
# rear-right. Named rather than indexed so that a topic in a viewer says FL and
# not 0.
WHEELS = ("FL", "FR", "RL", "RR")

_TIER_NAMES = {
    0: "L0_Kinematic",
    1: "L1_Bicycle",
    2: "L2_DoubleTrack",
    3: "L3_Extended",
}

_L0, _L1, _L2, _L3 = 0, 1, 2, 3

# The tier at which each recorded state field becomes a quantity somebody
# integrated, rather than a default nobody wrote. Below it the field is
# recorded as NaN, which every sink then carries through as absent.
#
# The rows are read off state.hpp's own per-field comments and off the tier
# sources, not guessed. Three of them are guesses about the future and are
# therefore written as L3, meaning "no tier that exists represents this":
#
#   steer_rate   needs the steering servo, CORE-10. l2_double_track.cpp sets
#                it to zero and says so. It becomes L2 when CORE-10 lands.
#   soc, pack_v  need the battery, CORE-09. L2 does not write them.
#
# omega_w is L2 and not L3 even though ADR-0027 gave L2 no wheel rotational
# state: L2 inverts the wheel speed quasi-statically from the delivered slip
# ratio, so the number is computed from the step rather than left over from
# construction. It is a derived wheel speed and not an integrated one, which is
# a limitation of the tier and not a reason to hide it.
#
# This table is the coupling ADR-0028 accepted when it multiplied the number of
# formats tracking the state layout: a new state field is added here, once, and
# the sinks consume what they understand.
_STATE_REPRESENTED_FROM: Dict[str, int] = {
    "pos.x": _L0,
    "pos.y": _L0,
    "pos.z": _L3,  # no vertical degree of freedom below L3
    "yaw": _L0,
    "pitch": _L3,
    "roll": _L3,
    "vel_body.x": _L0,
    "vel_body.y": _L0,  # geometric at L0, dynamic from L1
    "vel_body.z": _L3,
    "rates.x": _L3,
    "rates.y": _L3,
    "rates.z": _L0,
    "steer": _L0,
    "steer_rate": _L3,
    "soc": _L3,
    "pack_v": _L3,
    "omega_w": _L2,
    "Fz": _L2,
    "alpha_lag": _L2,
}

_NAN = float("nan")


def _state_columns() -> List[str]:
    """Every recorded state field, in a fixed order."""
    names = [
        "pos.x", "pos.y", "pos.z",
        "yaw", "pitch", "roll",
        "vel_body.x", "vel_body.y", "vel_body.z",
        "rates.x", "rates.y", "rates.z",
        "steer", "steer_rate",
        "soc", "pack_v",
    ]
    for base in ("omega_w", "Fz", "alpha_lag"):
        names.extend(f"{base}.{wheel}" for wheel in WHEELS)
    return names


def _diagnostic_columns() -> List[str]:
    names: List[str] = []
    for base in ("alpha", "kappa", "fx", "fy", "fz"):
        names.extend(f"{base}.{wheel}" for wheel in WHEELS)
    names.extend([
        "alpha_front", "alpha_rear",
        "fy_front", "fy_rear",
        "fz_front", "fz_rear",
        "ax", "ay",
        "load_transfer_long", "load_transfer_lat",
    ])
    return names


def _flag_columns() -> List[str]:
    # Booleans, kept apart from the floats because a bool has no NaN and so
    # cannot carry the absent rule. At the single-track tiers both wheels of an
    # axle carry that axle's flag, which state.hpp documents; the float arrays
    # stay NaN there rather than duplicating a value the tier did not compute.
    names = [f"tyre_saturated.{wheel}" for wheel in WHEELS]
    names.extend(["steer_saturated", "accel_saturated", "speed_saturated"])
    return names


STATE_COLUMNS = tuple(_state_columns())
DIAGNOSTIC_COLUMNS = tuple(_diagnostic_columns())
FLAG_COLUMNS = tuple(_flag_columns())


def represented(column: str, tier_index: int) -> bool:
    """Does ``tier_index`` actually write this state field?"""
    base = column.split(".")[0]
    minimum = _STATE_REPRESENTED_FROM.get(column)
    if minimum is None:
        minimum = _STATE_REPRESENTED_FROM[base]
    return tier_index >= minimum


@dataclass(frozen=True)
class AgentRecord:
    """One agent's run: who it was, and what it did, column by column.

    Column-major rather than a list of frames because that is what every
    consumer wants. A plot wants one column, a Rerun time series wants one
    column, and the frame a message-per-step format needs is one slice across
    the columns at a fixed index.
    """

    name: str
    index: int
    tier: str
    provenance: str
    params_digest: str
    seed: int
    trajectory_hash: str
    state: Mapping[str, Tuple[float, ...]]
    diagnostics: Mapping[str, Tuple[float, ...]]
    flags: Mapping[str, Tuple[bool, ...]] = field(default_factory=dict)

    def state_frame(self, i: int) -> Dict[str, float]:
        """One step of state, NaN columns dropped (SINK-05)."""
        return _frame(self.state, i)

    def diagnostics_frame(self, i: int) -> Dict[str, float]:
        """One step of diagnostics, NaN columns dropped (SINK-05)."""
        return _frame(self.diagnostics, i)

    def flags_frame(self, i: int) -> Dict[str, bool]:
        return {name: column[i] for name, column in self.flags.items()}


def _frame(columns: Mapping[str, Tuple[float, ...]], i: int) -> Dict[str, float]:
    # `value == value` is False for NaN and for nothing else, which is the one
    # test that does not need math.isnan on every field of every step.
    return {
        name: column[i]
        for name, column in columns.items()
        if column[i] == column[i]
    }


@dataclass(frozen=True)
class Recording:
    """A finished run, and everything a sink is allowed to write about it.

    Nothing here is synthesised: every column came out of a ``VehicleState`` or
    a ``StepDiagnostics``, and every scalar came out of the run manifest
    (SINK-05). A sink that wants to draw a track, interpolate a gap or infer a
    quantity is writing something this object does not contain, and that is the
    boundary rather than a matter of taste.
    """

    times: Tuple[float, ...]
    dt: float
    stride: int
    agents: Tuple[AgentRecord, ...]
    trajectory_hash: str
    manifest_json: str
    core_version: str
    schema_version: str
    integrator: str
    git_sha: str
    slipx_version: str

    def __len__(self) -> int:
        return len(self.times)

    @property
    def provenance(self) -> str:
        """The provenance of the parameters this run was produced from.

        One label if every agent agrees, otherwise all of them, because a run
        whose cars have different provenance is not entitled to the better one
        (NFR-08).
        """
        labels = sorted({agent.provenance for agent in self.agents})
        return labels[0] if len(labels) == 1 else ", ".join(labels)

    def provenance_line(self) -> str:
        """The one line every sink must put somewhere a reader will see it.

        NFR-08 is a claim-discipline requirement, not a formatting preference:
        no parameter set shipped with SlipX has been validated against a real
        car, and an artefact that does not say so is a stronger claim than the
        project is entitled to make.
        """
        return (
            f"SlipX {self.slipx_version} (core {self.core_version}) - "
            f"parameters: {self.provenance} - "
            f"trajectory {self.trajectory_hash}"
        )


def _vec(value) -> Tuple[float, float, float]:
    return (value.x, value.y, value.z)


def _read_state(state, tier_index: int) -> Dict[str, float]:
    values: Dict[str, float] = {}
    pos = _vec(state.pos)
    vel = _vec(state.vel_body)
    rates = _vec(state.rates)
    for axis, i in (("x", 0), ("y", 1), ("z", 2)):
        values[f"pos.{axis}"] = pos[i]
        values[f"vel_body.{axis}"] = vel[i]
        values[f"rates.{axis}"] = rates[i]
    values["yaw"] = state.yaw
    values["pitch"] = state.pitch
    values["roll"] = state.roll
    values["steer"] = state.steer
    values["steer_rate"] = state.steer_rate
    values["soc"] = state.soc
    values["pack_v"] = state.pack_v
    for base, array in (
        ("omega_w", state.omega_w),
        ("Fz", state.Fz),
        ("alpha_lag", state.alpha_lag),
    ):
        for i, wheel in enumerate(WHEELS):
            values[f"{base}.{wheel}"] = array[i]

    # The one place the zeros become absences. See the module docstring.
    return {
        name: (values[name] if represented(name, tier_index) else _NAN)
        for name in STATE_COLUMNS
    }


def _read_diagnostics(diagnostics) -> Tuple[Dict[str, float], Dict[str, bool]]:
    values: Dict[str, float] = {}
    for base, array in (
        ("alpha", diagnostics.alpha),
        ("kappa", diagnostics.kappa),
        ("fx", diagnostics.fx),
        ("fy", diagnostics.fy),
        ("fz", diagnostics.fz),
    ):
        for i, wheel in enumerate(WHEELS):
            values[f"{base}.{wheel}"] = array[i]
    for name in (
        "alpha_front", "alpha_rear", "fy_front", "fy_rear",
        "fz_front", "fz_rear", "ax", "ay",
        "load_transfer_long", "load_transfer_lat",
    ):
        values[name] = getattr(diagnostics, name)

    flags: Dict[str, bool] = {}
    for i, wheel in enumerate(WHEELS):
        flags[f"tyre_saturated.{wheel}"] = bool(diagnostics.tyre_saturated[i])
    for name in ("steer_saturated", "accel_saturated", "speed_saturated"):
        flags[name] = bool(getattr(diagnostics, name))
    return values, flags


def record_run(
    sim,
    steps: Optional[int] = None,
    *,
    duration: Optional[float] = None,
    stride: int = 1,
) -> Recording:
    """Step ``sim`` and copy out everything a sink is allowed to see.

    Frames are recorded *after* each step, so the first one is at ``dt`` and
    not at zero. That is deliberate: before the first step there are no
    diagnostics, only a default-constructed block whose tier field reads L0 and
    whose numbers are zeros, and recording that would put one frame of fiction
    at the front of every run.

    Args:
        sim: a :class:`slipx.Simulation` with its agents already added. It is
            stepped from wherever it currently is, so a run can be recorded in
            pieces.
        steps: how many steps to record. Exactly one of this and ``duration``.
        duration: seconds, rounded down to whole steps.
        stride: record every nth step. The run is stepped in full either way,
            so the trajectory does not depend on how much of it was kept.

    Returns:
        A :class:`Recording`. Recording does not perturb the run: the sim is
        stepped exactly as ``run`` would step it, nothing is fed back, and the
        trajectory hash is the hash of the same run recorded or not (NFR-02).
    """
    if (steps is None) == (duration is None):
        raise ValueError("give exactly one of steps and duration")
    if stride < 1:
        raise ValueError("stride must be at least 1")

    dt = sim.dt
    if steps is None:
        steps = int(duration / dt)
    if steps < 1:
        raise ValueError("a run of no steps records nothing")

    count = sim.agent_count
    if count == 0:
        raise ValueError("the simulation has no agents")

    tiers = [int(sim.model(i).tier) for i in range(count)]
    provenance = [
        sim.model(i).params.provenance.name.lower() for i in range(count)
    ]

    times: List[float] = []
    state_columns: List[Dict[str, List[float]]] = [
        {name: [] for name in STATE_COLUMNS} for _ in range(count)
    ]
    diagnostic_columns: List[Dict[str, List[float]]] = [
        {name: [] for name in DIAGNOSTIC_COLUMNS} for _ in range(count)
    ]
    flag_columns: List[Dict[str, List[bool]]] = [
        {name: [] for name in FLAG_COLUMNS} for _ in range(count)
    ]

    first = sim.step_count
    for step in range(1, steps + 1):
        sim.advance()
        if step % stride:
            continue
        times.append((first + step) * dt)
        for i in range(count):
            for name, value in _read_state(sim.state(i), tiers[i]).items():
                state_columns[i][name].append(value)
            values, flags = _read_diagnostics(sim.diagnostics(i))
            for name, value in values.items():
                diagnostic_columns[i][name].append(value)
            for name, flag in flags.items():
                flag_columns[i][name].append(flag)

    manifest = sim.manifest()
    agents = tuple(
        AgentRecord(
            name=manifest.agents[i].name,
            index=i,
            tier=_TIER_NAMES.get(tiers[i], str(tiers[i])),
            provenance=provenance[i],
            params_digest=manifest.agents[i].params_digest,
            seed=manifest.agents[i].seed,
            trajectory_hash=sim.agent_trajectory_hash(i),
            state={k: tuple(v) for k, v in state_columns[i].items()},
            diagnostics={k: tuple(v) for k, v in diagnostic_columns[i].items()},
            flags={k: tuple(v) for k, v in flag_columns[i].items()},
        )
        for i in range(count)
    )

    from ..version import __version__

    return Recording(
        times=tuple(times),
        dt=dt,
        stride=stride,
        agents=agents,
        trajectory_hash=manifest.trajectory_hash,
        manifest_json=manifest.to_json(),
        core_version=manifest.slipx_core_version,
        schema_version=manifest.schema_version,
        integrator=str(manifest.integrator),
        git_sha=manifest.git_sha,
        slipx_version=__version__,
    )
