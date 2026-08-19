# Copyright 2026 The SlipX Authors
# SPDX-License-Identifier: Apache-2.0

"""SlipX: vehicle dynamics for 1/10-scale autonomous racecars.

The Python API is the C++ API. Same names, same units, same ISO 8855 sign
conventions, same tiers; a tutorial written in one translates line by line into
the other. What this package adds is the two things a Python user needs and a
C++ embedder does not: loading a car directory, and a Gymnasium-shaped
interface (P5).

Quick start::

    import slipx

    car = slipx.load_car("examples/cars/reference_1_10")
    print(car.summary())                   # prints the provenance label

    model = slipx.VehicleModel.create(slipx.Tier.L1_Bicycle, car.params)
    state = slipx.VehicleState()
    state.vel_body.x = 5.0

    diagnostics = slipx.StepDiagnostics()
    model.step(state, slipx.DriveInput(steer_cmd=0.1), 1e-3, diagnostics)
    print(state.yaw_rate, diagnostics.alpha_front)

What is here: tiers L0, L1 and L2, all three reachable from a car directory;
the fixed-step orchestrator, seeded random streams, run manifests and
trajectory hashing; and ``slipx.sinks``, which records a run and writes it to a
file. L2 is the tier at which different cars actually behave differently: four
contact patches, load transfer, MF-lite tyres, a differential, an ESC, a
battery and a steering servo. L3 raises rather than silently giving you a
simpler model, and so does any tier a build does not implement.

Sensors and the ROS 2 layer arrive in P1.

No parameter set shipped with SlipX has been validated against a real car. The
honest phrasing for anything built on them is "physically structured and
identifiable", not "validated", and ``Car.summary()`` says so on every load.
"""

from __future__ import annotations

# The compiled core. Everything in it is generated from the same headers the
# C++ tests run against, so there is no second implementation to drift.
from ._slipx import (  # noqa: F401
    AgentManifest,
    AgentSpec,
    CombinedForce,
    ConformanceSpec,
    ContactParams,
    Differential,
    DnfCause,
    DnfEvent,
    DriveInput,
    DriveLayout,
    Integrator,
    MfLite,
    Provenance,
    Rng,
    RunManifest,
    Simulation,
    SimulationConfig,
    StepDiagnostics,
    StepSteerSpec,
    Tier,
    TyreCoefficients,
    TrajectoryHash,
    Vec3,
    VehicleModel,
    VehicleParams,
    VehicleState,
    cornering_stiffness_at_load,
    core_version,
    derive_seed,
    friction_ellipse,
    hash_text,
    hold_speed,
    make_conformance_run,
    make_mf_lite,
    mf_lite_fy,
    peak_lateral_force,
    peak_longitudinal_force,
    step_steer,
)
from .cars import (
    Car,
    load_car,
    load_reference_car,
    reference_car_path,
    to_vehicle_params,
)
from .version import __version__

# Recording a run and emitting it (SINK-01 to SINK-05, ADR-0028). Standard
# library only, and importing it imports no encoder: the MCAP and Rerun SDKs
# are optional extras, reached through `sinks.sink_for` and never imported
# because somebody typed `import slipx`.
from . import sinks  # noqa: E402  (after version, which record_run reports)

__all__ = [
    "AgentManifest",
    "AgentSpec",
    "Car",
    "CombinedForce",
    "ConformanceSpec",
    "ContactParams",
    "Differential",
    "DnfCause",
    "DnfEvent",
    "DriveInput",
    "DriveLayout",
    "Integrator",
    "MfLite",
    "Provenance",
    "Rng",
    "RunManifest",
    "Simulation",
    "SimulationConfig",
    "StepDiagnostics",
    "StepSteerSpec",
    "Tier",
    "TyreCoefficients",
    "TrajectoryHash",
    "Vec3",
    "VehicleModel",
    "VehicleParams",
    "VehicleState",
    "__version__",
    "cornering_stiffness_at_load",
    "core_version",
    "derive_seed",
    "friction_ellipse",
    "hash_text",
    "hold_speed",
    "load_car",
    "load_reference_car",
    "make_conformance_run",
    "make_mf_lite",
    "mf_lite_fy",
    "peak_lateral_force",
    "peak_longitudinal_force",
    "reference_car_path",
    "sinks",
    "step_steer",
    "to_vehicle_params",
]
