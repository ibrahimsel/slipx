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

What is here at P0: tiers L0 and L1, the fixed-step orchestrator, seeded random
streams, run manifests and trajectory hashing. L2 (the tier at which different
cars actually behave differently), sensors and the ROS 2 layer arrive in P1;
asking for an unimplemented tier raises rather than silently giving you L1.

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
    ConformanceSpec,
    DriveInput,
    Integrator,
    Provenance,
    Rng,
    RunManifest,
    Simulation,
    SimulationConfig,
    StepDiagnostics,
    StepSteerSpec,
    Tier,
    TrajectoryHash,
    Vec3,
    VehicleModel,
    VehicleParams,
    VehicleState,
    core_version,
    derive_seed,
    hash_text,
    hold_speed,
    make_conformance_run,
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
    "ConformanceSpec",
    "DriveInput",
    "Integrator",
    "Provenance",
    "Rng",
    "RunManifest",
    "Simulation",
    "SimulationConfig",
    "StepDiagnostics",
    "StepSteerSpec",
    "Tier",
    "TrajectoryHash",
    "Vec3",
    "VehicleModel",
    "VehicleParams",
    "VehicleState",
    "__version__",
    "core_version",
    "derive_seed",
    "hash_text",
    "hold_speed",
    "load_car",
    "load_reference_car",
    "make_conformance_run",
    "reference_car_path",
    "sinks",
    "step_steer",
    "to_vehicle_params",
]
