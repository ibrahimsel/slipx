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

Two things make it different from the physics in a game engine. It is a library
before it is a simulator, so `slipx_core` depends on the C++ standard library
and nothing else and can go inside a stack you already run. And every tyre
parameter is one you can identify from a manoeuvre driven in a car park with the
sensors already bolted to a competition car: wheel encoders, an IMU, LiDAR pose.
No dyno, no tyre rig, nothing to guess.

**Early days, and the limits are worth reading before you start.** Two of the
four fidelity tiers are built: a kinematic bicycle and a dynamic bicycle with
linear tyres. The double-track tier, where two different cars start behaving
differently, is not implemented, and asking for it raises rather than quietly
handing back the simpler model. No parameter set here has been fitted to a real
vehicle, and the tooling says so every time it loads one.

## Quickstart

```
pip install slipx
slipx-conformance
```

Wheels cover Linux, macOS and Windows on CPython 3.9 to 3.13, so the normal case
needs no compiler. `slipx-conformance` integrates the canonical step steer and
prints the run manifest and its trajectory hash.

Releases are pre-releases for now (`0.1.0a1` and onwards), which `pip` selects
only while no final release exists. Pin one with `pip install slipx==0.1.0a1`.

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

That is one second of a step steer at 1 kHz. `StepDiagnostics` is optional so
the hot path stays cheap; ask for it and you get slip angles, slip ratios,
per-tyre forces, load transfer terms and actuator saturation flags, which
between them let a student plot exactly why the car spun.

Anything a tier cannot represent comes back as **NaN, never zero**. The
kinematic tier has no tyres, so its slip angles are NaN and a plot of them is
empty. Zero is a number somebody would plot and believe.

## Tiers

Fidelity levels selected at construction, all behind one `VehicleModel`
interface, so moving between them is one argument rather than a rewrite.

| Tier | Model | States | Status |
|---|---|---|---|
| `L0_Kinematic` | Kinematic bicycle | 4 | built |
| `L1_Bicycle` | Dynamic bicycle, linear tyres | 6 | built |
| `L2_DoubleTrack` | Double-track, load transfer, MF-lite tyres | ~15 | not implemented, raises |
| `L3_Extended` | Adds thermal and suspension | | not implemented, raises |

Below `L2_DoubleTrack` nothing represents CoG height, weight distribution,
differential or tyre compound, so those parameters correctly have no effect.
That is the teaching artefact rather than a bug: the tier where your change
stops mattering tells you what the model is actually made of.

## The tyre model

What ships today is a linear tyre, `Fy = -C_alpha * alpha`, clipped at
`mu * Fz`. A clip is not a Magic Formula: there is no peak, no falling branch
beyond it, and therefore no mechanism by which the car spins.
`StepDiagnostics` raises `tyre_saturated` the instant the clip engages, so the
point where the model stops being believable is a number you can plot rather
than a feeling you develop.

MF-lite, a reduced Magic Formula with load sensitivity and combined slip, comes
with the double-track tier. The schema already accepts and validates its full
parameter set, so a tyre file identified today will still be correct when that
tier lands. Every parameter earns its place by being identifiable:

| Symbol | Meaning | Identifiable from |
|---|---|---|
| `C_alpha0` | Cornering stiffness at nominal load | Skidpad, low-slip region |
| `mu_y0`, `mu_x0` | Peak lateral / longitudinal friction | Circle-to-slip, straight-line accel |
| `k_mu` | Load sensitivity exponent | Skidpad at two ballast configurations |
| `B`, `C`, `E` | MF shape factors | Full slip sweep |
| `sigma` | Relaxation length | Step steer transient |

Full Pacejka 5.2 is not the goal. A parameter nobody can identify is worse than
one that does not exist, because it gets guessed and then trusted.

Tyres are referenced as a `(compound, surface)` pair rather than embedded in the
car file. The same car on carpet and on polished concrete is two different
vehicles, and the schema should say so rather than let someone quietly carry
asphalt coefficients into a sports hall.

## Determinism

Fixed step, seeded per agent, lockstep barrier, headless. Every run writes a
manifest hashing the schema versions, parameter files, seeds, integrator, git
SHA, compiler ID and flags.

**On one build, the same manifest and inputs replay bit-identically.** That
needs `-ffp-contract=off`, no `-ffast-math`, no `-march=native`, a fixed
reduction order and no multithreading inside the integrator.

**Across builds, bit-identity is not promised.** `conformance/reference_hashes.tsv`
is keyed by architecture, compiler and build type. A run is compared against the
row matching its own build: a mismatch there is a bug, and a build with no row
is a build about which nothing was claimed.

```
$ python3 tools/check_conformance.py
  L0/rk4                       d74f90169a5951c2  matches reference
  L0/semi_implicit_euler       44b1d28010f293c4  matches reference
  L1/rk4                       d44a9a68616ec899  matches reference
  L1/semi_implicit_euler       9a2532ced2e1e06d  matches reference
```

So `slipx-conformance` printing a different hash on your machine is the expected
outcome, not a failure. Published wheels deliberately carry no hash claim: they
call `sin`, `cos` and `atan2`, resolve them against the host C library at run
time, and those are not correctly rounded. The same wheel on glibc 2.28 and on
glibc 2.39 agrees to every digit of the printed state and still hashes
differently. The compiler, by contrast, is visibly not the variable: every
x86-64 row in that file is identical across GCC 11, GCC 13 and Clang 18.

## Building from source

```
cmake -S . -B build -DSLIPX_BUILD_PYTHON=ON
cmake --build build -j
ctest --test-dir build
python3 -m pytest
```

Needs CMake 3.20 and a C++17 compiler. GoogleTest is fetched if absent; the
Python parts want PyYAML, jsonschema and pybind11. None of that reaches
`slipx_core`, which builds alone:

```
cmake -S . -B build-core -DSLIPX_CORE_ONLY=ON
cmake --build build-core -j && ctest --test-dir build-core
```

Embedding it in a C++ simulator is two lines and pulls in no transitive
dependency:

```cmake
add_subdirectory(slipx)
target_link_libraries(your_simulator PRIVATE slipx::core)
```

`step` is `const` and stateless: no hidden state, no clock, no ambient RNG, no
allocation. N instances parallelise trivially and snapshot/restore is a `memcpy`.

## The stack

Dependencies point downward only, and `tools/dep_lint.py` fails the build if
that stops being true.

```
slipx_registry   community parameter sets (data only, no code)          planned
slipx_id         system identification: manoeuvres, fitting, reports    planned
slipx_ros        ROS 2 wrapper: topics, TF, /clock, rosbag, launch      planned
slipx            Python package: pybind11 bindings + Gymnasium adapter  built
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
geometry, inertias and sensor mounts (the same TF tree that runs on the real
car), then `dynamics.yaml`, `sensors.yaml`, `limits.yaml` and `provenance.yaml`.
Copy `examples/cars/reference_1_10` to start one.

## Not in scope

No photorealistic rendering: SlipX is LiDAR-first and runs on CPU, and there is
no camera model. That rules out a sensor, not a picture: rendering a finished
run to a self-contained SVG, with the provenance label and the trajectory hash
drawn into the image, is in scope and lands with the double-track tier. It
writes a file and never opens a window, so it needs no display server and no
GPU. No full-scale road vehicles, traffic or urban scenarios. No
autonomy stack; the reference controllers exist to validate the sim, not to win
with. No aerodynamics at 1/10 scale, where it is negligible below roughly
15 m/s. Collision physics will be plausible and deterministic, not fitted to
data, and the docs will keep saying so.

## What can honestly be claimed

Verification runs in four layers, because unit tests and a demo video establish
nothing about a physics library. **Analytical**: closed-form cases with known
answers, which is what catches the sign and unit errors this kind of code
actually has. **Invariant**: energy and momentum conservation, left/right mirror
symmetry asserted bit for bit, monotonicity under property-based sampling.
**Cross-tier**: the tiers must agree in the low lateral acceleration limit, and
for the reference car the two built ones agree on path radius to within 5% below
0.23 g, with the crossover printed on every test run.

Those three are in place: 148 C++ tests and 87 Python tests, including an
allocation counter proving `step` never touches the allocator.

**Empirical** is the one that is missing. Until an outside contributor supplies
a fitted parameter set with a validation report, the honest phrasing
is *physically structured and identifiable*, not *validated*. Every shipped
parameter set carries a `measured`, `identified` or `provisional` label, and the
tooling prints it rather than leaving it in the documentation.

## Learning the subject

If the vocabulary in this README is new, there is a tutorial series on the
concepts themselves, written for somebody arriving from robotics or computer
science rather than from vehicle dynamics:
[**Autonomous racing, from the ground up**](https://github.com/ibrahimsel/slipx/tree/main/docs/racing).

It covers slip angles and the shape of the tyre curve, load transfer and why it
costs you grip, which vehicle model is allowed to answer which question,
understeer and oversteer, the racing line, and the g-g diagram. It is a guide to
the subject rather than a manual for this library, so it is worth reading even
if you never install anything.

## Where the reasoning lives

[`docs/adr`](https://github.com/ibrahimsel/slipx/tree/main/docs/adr) has one
numbered record per architectural decision, each stating what was considered and
what the decision costs: why there is no Eigen, why a missing tier raises rather
than falling back, why diagnostics are NaN, why reference hashes are keyed by
build. Read it
before proposing to change any of them, since most of the tempting
simplifications have a record explaining what they break.

The component diagram is in
[`docs/architecture/slipx.md`](https://github.com/ibrahimsel/slipx/blob/main/docs/architecture/slipx.md),
and the release history is in
[`CHANGELOG.md`](https://github.com/ibrahimsel/slipx/blob/main/CHANGELOG.md).

The banner at the top is an illustration, and it was not produced by this
library. `docs/assets/make_banner.py` carries its own single-track model and its
own Magic Formula tyre, and it predates the code it advertises. It shows a car
in a sustained drift, which is a manoeuvre the shipped tiers cannot reproduce,
because a clipped linear tyre has no falling branch and so cannot spin a car. It
will be regenerated from `slipx_core` once the double-track tier and MF-lite
land, and that regeneration is a fair test of both.

## Licence

Apache-2.0.
