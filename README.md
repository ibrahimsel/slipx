# SlipX

<!-- Absolute URLs throughout this file, not repository-relative ones. This
     README is also the PyPI long description, and PyPI serves it from another
     origin where a relative path resolves to nothing. -->
![SlipX: vehicle dynamics for 1/10-scale racecars](https://raw.githubusercontent.com/ibrahimsel/slipx/main/docs/assets/slipx-banner.gif)

[![PyPI](https://img.shields.io/pypi/v/slipx)](https://pypi.org/project/slipx/)
[![Python](https://img.shields.io/pypi/pyversions/slipx)](https://pypi.org/project/slipx/)
[![Licence](https://img.shields.io/badge/licence-Apache--2.0-blue)](https://github.com/ibrahimsel/slipx/blob/main/LICENSE)
[![CI](https://github.com/ibrahimsel/slipx/actions/workflows/ci.yml/badge.svg)](https://github.com/ibrahimsel/slipx/actions/workflows/ci.yml)

Vehicle dynamics for 1/10-scale autonomous racecars, the RoboRacer and F1TENTH
class. C++17, CPU-only, headless, Apache-2.0.

Two things separate it from the physics in a game engine. It is a library
before it is a simulator: `slipx_core` depends on the C++ standard library and
nothing else, so it goes inside a stack you already run. And every tyre
parameter is identifiable from a manoeuvre driven in a car park with the
sensors already on a competition car, so there is nothing to guess.

**Early days.** Three of the four tiers are built, and a car directory
parameterises all three: schema 0.2.0 carries the longitudinal slip
stiffness, the ESC curve, the battery endpoints and the servo constants the
double-track tier consumes. The extended tier raises rather than handing
back a simpler model. No parameter set here has been fitted to a real
vehicle, and the tooling says so every time it loads one.

## Quickstart

```
pip install slipx
slipx-conformance
```

Wheels cover Linux, macOS and Windows on CPython 3.9 to 3.13, so the normal
case needs no compiler. `slipx-conformance` integrates the canonical step steer
and prints the run manifest and its trajectory hash. Releases are pre-releases
for now, which `pip` selects only while no final release exists; pin one with
`pip install slipx==0.1.0a1`.

```python
import slipx

car = slipx.load_reference_car()   # or load_car("examples/cars/reference_1_10")
print(car.summary())               # leads with the provenance label: PROVISIONAL

model = slipx.VehicleModel.create(slipx.Tier.L1_Bicycle, car.params)
state = slipx.VehicleState()
state.vel_body.x = 5.0

diagnostics = slipx.StepDiagnostics()
for _ in range(1000):
    model.step(state, slipx.DriveInput(steer_cmd=0.1), 1e-3, diagnostics)

print(state.yaw_rate, diagnostics.alpha_front, diagnostics.ay)
```

`StepDiagnostics` is optional so the hot path stays cheap: ask for it and you
get slip angles, slip ratios, per-tyre forces, load transfer and saturation
flags. Anything a tier cannot represent comes back as **NaN, never zero**, so a
plot of the kinematic tier's slip angles is empty rather than flat at a number
somebody would believe.

To look at a run, record it and write it out. MCAP is the default; Rerun and
Foxglove read it.

```python
sim = slipx.make_conformance_run(slipx.ConformanceSpec())
run = slipx.sinks.record_run(sim, duration=5.0, stride=10)
slipx.sinks.write(run, "lap")            # lap.mcap, needs slipx[mcap]
```

## Tiers

Fidelity levels selected at construction, behind one `VehicleModel` interface,
so moving between them is one argument rather than a rewrite.

| Tier | Model | States | Status |
|---|---|---|---|
| `L0_Kinematic` | Kinematic bicycle | 4 | built |
| `L1_Bicycle` | Dynamic bicycle, linear tyres | 6 | built |
| `L2_DoubleTrack` | Double-track, load transfer, MF-lite tyres, relaxation, drivetrain | 13 | built |
| `L3_Extended` | Adds thermal and suspension | | raises |

`L2_DoubleTrack` has four contact patches, per-corner vertical loads, MF-lite
with a real peak and falling branch, a combined-slip friction ellipse, a tyre
relaxation transient, and the drivetrain: an open, spool or preloaded-LSD
differential on a 2WD or 4WD layout, an ESC torque-speed curve with current
and regen limits, battery sag and state of charge, and a slew-limited
second-order steering servo, so commanded and achieved steer are separate
numbers. Braking is motor braking through the driven axle, capped by the
regen limit. It uses parallel steer rather than Ackermann, and it has no
wheel rotational state, so a locked or spinning wheel is not representable.
Each of those is named in the source.

Below `L2_DoubleTrack` nothing represents CoG height, weight distribution,
differential or tyre compound, so those parameters correctly have no effect.

## The tyre model

L1 uses a linear tyre, `Fy = -C_alpha * alpha`, clipped at `mu * Fz`. A clip has
no peak and no falling branch, so it cannot spin a car; `tyre_saturated` fires
the instant it engages.

L2 uses MF-lite, a reduced Magic Formula with load sensitivity and combined
slip. Every parameter earns its place by being identifiable:

| Symbol | Meaning | Identifiable from |
|---|---|---|
| `C_alpha0` | Cornering stiffness at nominal load | Skidpad, low-slip region |
| `C_kappa` | Longitudinal slip stiffness | Encoder slip ratio against IMU acceleration |
| `mu_y0`, `mu_x0` | Peak lateral / longitudinal friction | Circle-to-slip, straight-line accel |
| `k_mu` | Load sensitivity exponent | Skidpad at two ballast configurations |
| `C`, `E` | MF shape factors; `B` is derived, never asked for | Full slip sweep |
| `sigma` | Relaxation length | Step steer transient |

Full Pacejka is not the goal. A parameter nobody can identify is worse than one
that does not exist, because it gets guessed and then trusted.

Tyres are referenced as a `(compound, surface)` pair rather than embedded in the
car file. The same car on carpet and on polished concrete is two different
vehicles.

## Determinism

Fixed step, seeded per agent, lockstep barrier, headless. Every run writes a
manifest hashing the schema versions, parameter files, seeds, integrator, git
SHA, compiler ID and flags.

**On one build, the same manifest and inputs replay bit-identically.** That
needs `-ffp-contract=off`, no `-ffast-math`, no `-march=native`, a fixed
reduction order and no multithreading inside the integrator.

**Across builds, bit-identity is not promised.**
`conformance/reference_hashes.tsv` is keyed by architecture, compiler and build
type, and `tools/check_conformance.py` compares a run against the row matching
its own build. A mismatch there is a bug; a build with no row is a build about
which nothing was claimed.

So `slipx-conformance` printing a different hash on your machine is expected,
and published wheels carry no hash claim at all. The variable is libm, not the
compiler: one wheel hashes differently on glibc 2.28 and 2.39 while agreeing to
every printed digit, and every x86-64 reference row is identical across GCC 11,
GCC 13 and Clang 18.

## Building from source

```
cmake -S . -B build -DSLIPX_BUILD_PYTHON=ON
cmake --build build -j
ctest --test-dir build && python3 -m pytest
```

Needs CMake 3.20 and a C++17 compiler. GoogleTest is fetched if absent; the
Python parts want PyYAML, jsonschema and pybind11. None of that reaches
`slipx_core`, which builds and passes its full suite alone under
`-DSLIPX_CORE_ONLY=ON`.

Embedding it pulls in no transitive dependency:

```cmake
add_subdirectory(slipx)
target_link_libraries(your_simulator PRIVATE slipx::core)
```

`step` is `const` and stateless: no hidden state, no clock, no ambient RNG, no
allocation. N instances parallelise trivially and snapshot/restore is a
`memcpy`.

## The stack

Dependencies point downward only, and `tools/dep_lint.py` fails the build if
that stops being true.

```
slipx_registry   community parameter sets (data only, no code)          planned
slipx_id         system identification: manoeuvres, fitting, reports    planned
slipx_ros        ROS 2 wrapper: topics, TF, /clock, rosbag, launch      planned
slipx            Python package: bindings, car loader, run sinks        built
slipx_sim        orchestrator: N agents, fixed step, lockstep, replay   built
slipx_scene      track loading, BVH, contact, race control, events      planned
slipx_sense      sensor simulation: raycasting, noise, latency          planned
slipx_schema     JSON Schema definitions + reference parser             built
slipx_core       vehicle dynamics. The C++ standard library, nothing
                 else.                                                  built
```

The directories for the planned pieces exist and are empty. Nothing there
half-works.

A car is a versioned directory: `car.yaml` as manifest, a URDF/xacro owning
geometry, inertias and sensor mounts, then `dynamics.yaml`, `sensors.yaml`,
`limits.yaml` and `provenance.yaml`. Copy `examples/cars/reference_1_10` to
start one.

## Not in scope

No photorealistic rendering, no camera model: SlipX is LiDAR-first and runs on
CPU. That rules out a sensor, not a picture. A finished run renders to a
self-contained animated SVG, provenance label and trajectory hash drawn into
the image, with no dependency beyond the standard library and nothing drawn
that the run did not record: no track, no car body.

No interactive viewer. SlipX owns the encoder rather than the viewer, so a run
is emitted as MCAP and scrubbing a timeline is served by a tool built for it. A
Rerun sink is available too; both encoders are optional extras rather than
install requirements. The SVG sink needs no extra at all, and writes a file
rather than opening a window, like every sink here.

No full-scale road vehicles, traffic or urban scenarios. No autonomy stack; the
reference controllers exist to validate the sim, not to win with. No
aerodynamics at 1/10 scale, where it is negligible below roughly 15 m/s.
Collision physics will be plausible and deterministic, not fitted to data.

## What can honestly be claimed

Verification runs in four layers.

- **Analytical**: closed-form cases with known answers, which catches the sign
  and unit errors this kind of code actually has.
- **Invariant**: energy and momentum conservation, left/right mirror symmetry
  asserted bit for bit, monotonicity under property-based sampling.
- **Cross-tier**: the tiers must agree in the low lateral acceleration limit.
  For the reference car, L1 and L2 agree on path radius to within 1% below
  0.23 g, and the crossover is printed on every test run.
- **Empirical**: missing. Until an outside contributor supplies a fitted
  parameter set with a validation report, the honest phrasing is *physically
  structured and identifiable*, not *validated*.

The first three are in place: 240 C++ tests and 194 Python tests, including an
allocation counter proving `step` never touches the allocator. Every shipped
parameter set carries a `measured`, `identified` or `provisional` label, and
the tooling prints it rather than leaving it in the documentation.

## Learning the subject

If the vocabulary here is new, there is a tutorial series on the concepts
themselves, for somebody arriving from robotics or computer science rather than
vehicle dynamics:
[**Autonomous racing, from the ground up**](https://github.com/ibrahimsel/slipx/tree/main/docs/racing).
Slip angles, load transfer, understeer and oversteer, the racing line, the g-g
diagram, combined slip, differentials, and where a small electric car's
acceleration and braking actually come from. A guide to the subject, not a
manual for this library.

## Where the reasoning lives

[`docs/adr`](https://github.com/ibrahimsel/slipx/tree/main/docs/adr) has one
numbered record per architectural decision, with what it costs: why there is no
Eigen, why a missing tier raises, why diagnostics are NaN, why reference hashes
are keyed by build. Most of the tempting simplifications have a record
explaining what they break.

The component diagram is in
[`docs/architecture/slipx.md`](https://github.com/ibrahimsel/slipx/blob/main/docs/architecture/slipx.md)
and the release history in
[`CHANGELOG.md`](https://github.com/ibrahimsel/slipx/blob/main/CHANGELOG.md).

The banner is an illustration, not output: `docs/assets/make_banner.py` carries
its own model and predates the code it advertises. It will be regenerated from
`slipx_core`.

## Licence

Apache-2.0.
