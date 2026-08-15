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
class. C++17, CPU-only, headless, deterministic, Apache-2.0, with Python
bindings.

SlipX is a library before it is a simulator. `slipx_core` depends on the C++
standard library and nothing else, so it goes inside a stack you already run
instead of asking you to move into one. Every tyre parameter it asks for is
identifiable from a manoeuvre you can drive in a car park with the sensors
already bolted to a competition car.

> **Alpha.** The API is not stable before `1.0.0`, and a minor bump may break
> it. This page describes the current source tree: the track, sensor and
> orchestrator sections below are ahead of the published wheel, which is
> `0.2.0`. The parameter sets shipped here are labelled `provisional`: they are
> plausible for the class and have not been fitted to a vehicle. Every loader
> and every example prints that label.

## Install

```bash
pip install slipx
```

Wheels cover Linux, macOS and Windows on CPython 3.9 to 3.13, so the usual case
needs no compiler. Recording formats are optional extras:

```bash
pip install "slipx[mcap]"     # MCAP recording (also: rerun, or sinks for both)
```

Verify the install:

```bash
slipx-conformance
```

It integrates the canonical step steer and prints the run manifest with its
trajectory hash.

## Quickstart

```python
import slipx

car = slipx.load_reference_car()      # or load_car("path/to/your_car")
print(car.summary())

model = slipx.VehicleModel.create(slipx.Tier.L2_DoubleTrack, car.params)
state = slipx.VehicleState()
state.vel_body.x = 5.0

diagnostics = slipx.StepDiagnostics()
for _ in range(1000):
    model.step(state, slipx.DriveInput(steer_cmd=0.1), 1e-3, diagnostics)

print(state.yaw_rate, diagnostics.alpha_front, diagnostics.ay)
```

The Python API is the C++ API: same names, same units, same ISO 8855 sign
conventions, same tiers. A snippet written in one translates line by line into
the other.

Three runnable programs are in
[`examples/`](https://github.com/ibrahimsel/slipx/tree/main/examples), and the
test suite executes them rather than proofreading them.

## Fidelity tiers

Fidelity is selected at construction, behind one `VehicleModel` interface, so
moving between levels is one argument rather than a rewrite.

| Tier | Model | States | Status |
|---|---|---|---|
| `L0_Kinematic` | Kinematic bicycle | 4 | available |
| `L1_Bicycle` | Dynamic bicycle, linear tyres | 6 | available |
| `L2_DoubleTrack` | Double-track, load transfer, MF-lite tyres, relaxation, drivetrain | 13 | available |
| `L3_Extended` | Adds thermal and suspension effects | | raises |

`L2_DoubleTrack` is the tier at which different cars behave differently. It has
four contact patches with per-corner vertical loads, MF-lite tyres with a real
peak and falling branch, a combined-slip friction ellipse, a tyre relaxation
transient, and the drivetrain: an open, spool or preloaded-LSD differential on
a 2WD or 4WD layout, an ESC torque-speed curve with current and regen limits,
battery sag and state of charge, and a slew-limited second-order steering
servo, so commanded and achieved steer are separate numbers.

Its limits are named in the source: braking is motor braking through the driven
axle and capped by the regen limit, steering is parallel rather than Ackermann,
and there is no wheel rotational state, so a locked or spinning wheel is not
representable.

Below `L2_DoubleTrack`, nothing represents CoG height, weight distribution,
differential or tyre compound, so those parameters correctly have no effect on
the trajectory.

A tier that is not implemented raises. It never hands back a simpler model.

## Diagnostics

`StepDiagnostics` is optional so the hot path stays cheap. Ask for it and you
get slip angles, slip ratios, per-tyre forces, vertical loads, load transfer
and saturation flags for the step just taken.

Anything a tier cannot represent comes back as **NaN, never zero**. A plot of
the kinematic tier's slip angles is therefore empty, rather than flat at a
number somebody would believe. Sinks carry that through: a NaN is written as
absent, never as a plotted zero.

## Describing a car

A car is a versioned directory. Copy
[`examples/cars/reference_1_10`](https://github.com/ibrahimsel/slipx/tree/main/examples/cars/reference_1_10)
to start your own.

```
your_car/
  car.yaml          identity, geometry, mass, which tyres each axle runs
  dynamics.yaml     drivetrain, ESC, battery, steering servo
  limits.yaml       travel and command bounds
  sensors.yaml      sensor placement
  provenance.yaml   where every number came from
  tyres/
    sponge_carpet.yaml    one (compound, surface) pair
```

The loader refuses rather than defaults. A parameter it cannot fill produces an
error naming the field and where the number should come from, so a missing
value is never quietly replaced by a plausible one.

`car.yaml` may also point at a URDF or xacro owning frames, link geometry and
sensor mounts. It is optional today and becomes load-bearing once a TF tree is
needed.

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
| `mu_y0`, `mu_x0` | Peak lateral and longitudinal friction | Circle-to-slip, straight-line acceleration |
| `k_mu` | Load sensitivity exponent | Skidpad at two ballast configurations |
| `C`, `E` | MF shape factors; `B` is derived, never asked for | Full slip sweep |
| `sigma` | Relaxation length | Step steer transient |

Full Pacejka is not the goal. A parameter nobody can identify is worse than one
that does not exist, because it gets guessed and then trusted.

Tyres are referenced as a `(compound, surface)` pair rather than embedded in the
car file. The same chassis on carpet and on polished concrete is two different
vehicles, and only the pair changes.

The derivation, with worked numbers, is in
[`docs/reference/tyre-model.md`](https://github.com/ibrahimsel/slipx/blob/main/docs/reference/tyre-model.md).

## Tracks

A track is a four-column TUM-format centreline CSV plus a manifest beside it.
Arc length is derived rather than read, and the manifest declares a surface
identifier rather than a friction number, so grip keeps coming from the tyre
file somebody measured. Loading a car whose tyres do not cover the track's
surface is refused by name rather than approximated.

```python
import slipx_schema

track = slipx_schema.load_track("examples/tracks/paddock_stadium")
print(track.summary())
```

In C++, `slipx_scene` adds the parts a run needs: `Track` and `Centreline`,
`project` for arc length and signed lateral offset, `LapCounter` for lap
counting and track limits with a per-agent tolerance, and `Walls` for the
drivable corridor implied by the track widths.

Lap counting accumulates signed progress along the centreline rather than
watching for line crossings, so reversing over the line subtracts, a car
wobbling on it accumulates nothing, and direction falls out instead of being
special-cased. Track limits report a margin rather than a flag, and remember an
excursion after the car comes back.

**SlipX ships no third-party track geometry.** Every public centreline set in
this format is copyleft, and the F1TENTH maps of real venues carry no licence at
all, so neither can enter an Apache-2.0 tree.
[`tools/convert_track.py`](https://github.com/ibrahimsel/slipx/blob/main/tools/convert_track.py)
fetches one into a track directory on your own machine instead, recording the
source, its licence and the retrieval date in the manifest. One generated track,
`paddock_stadium`, ships and is what CI and the examples run on; it is labelled
as generated, with no real-world counterpart.

## Sensors

`slipx_sense` is the C++ sensor layer. It is not yet exposed through the Python
bindings.

**2D LiDAR.** Every ray is individually timestamped and cast from the emitter
pose interpolated at that ray's own timestamp, so motion distortion emerges from
the physics rather than being applied afterwards: a fast-spinning car produces a
measurably skewed scan and a stationary one produces none, with no special case
for either. Per-sensor latency (constant plus jitter) is configurable
independently of rate, alongside dropouts and range-dependent noise.

**IMU.** Three errors on three timescales: white noise stated as a density, so
sampling faster gives noisier samples and the same answer after averaging; a
bias random walk, which averaging does not remove; and a fixed scale error,
which is a property of the unit and identifiable from a manoeuvre with a known
total heading change.

**Wheel encoders.** Odometry derived from simulated counts, degrading under
wheel slip exactly as the tyre model says it should. It is noiseless on purpose:
an encoder has a quantisation rather than a noise floor, and its interesting
error is the slip, which is already in the wheel speeds.

The LiDAR receives the world as a function from a ray to a distance, so it never
learns what a track is, and the two layers are usable independently. A
[wall follower and a pure pursuit controller](https://github.com/ibrahimsel/slipx/tree/main/examples/cpp)
ship as examples of joining them up; they exist to exercise the simulator, not
to be competitive.

## Running agents

The orchestrator steps N agents in lockstep on a fixed step.

```python
import slipx

car = slipx.load_reference_car()

config = slipx.SimulationConfig()
config.dt = 1.0e-3
config.master_seed = 20260815
config.schema_version = car.spec.schema_version
sim = slipx.Simulation(config)

agent = slipx.AgentSpec()
agent.name = "alice"
agent.tier = slipx.Tier.L2_DoubleTrack
agent.params = car.params_for_tier(slipx.Tier.L2_DoubleTrack)
agent.initial_state.vel_body.x = 5.0
agent.policy = lambda state, time, rng: slipx.DriveInput(
    steer_cmd=0.1, accel_cmd=slipx.hold_speed(state, 5.0))
sim.add_agent(agent)

sim.run_for(5.0)
print(sim.trajectory_hash())
```

Every policy sees the world as it was at the start of the step, so a result
cannot depend on the order agents were added in. A policy is a callable of
`(state, time, rng)` and must be a pure function of those three for the run to
replay; anything else it reads is a way for the run to stop reproducing.

`sim.state(i)` and `sim.diagnostics(i)` return references that the next step
overwrites. Copy before keeping.

Two modes are available. Deterministic mode is the default. Validation mode
paces against a wall clock with latency and jitter enabled, and is soft in the
direction that matters: it sleeps when it is ahead and does not catch up when it
is behind, since catching up would mean stepping faster than real time. Full
simulation state snapshots and restores bit-identically in deterministic mode.

## Recording a run

Record once, then write. Recording does not perturb the run: the trajectory
hash is the same recorded or not, and `stride` decides how much is kept, never
how much is simulated.

```python
from slipx import sinks

run = sinks.record_run(sim, duration=5.0, stride=10)
sinks.write(run, "lap", format="svg")     # lap.svg, no extra needed
sinks.write(run, "lap")                   # lap.mcap, the default
```

| Format | Needs | Notes |
|---|---|---|
| `"mcap"` | `slipx[mcap]` | The default. Archival, and read by Rerun and Foxglove |
| `"rerun"` | `slipx[rerun]` | Writes an `.rrd` file |
| `"svg"` | nothing | One self-contained animated document, theme-aware, standard library only |

Every sink writes a file and never opens a window. Nothing is drawn that the run
did not record, so there is no invented track and no invented car body.

## Reproducibility

Fixed step, seeded per agent, lockstep barrier, headless. Every run writes a
manifest hashing the schema versions, parameter files, seeds, integrator, git
SHA, compiler ID and flags, and the C library.

**On one build, the same manifest and inputs replay bit-identically.** That
needs `-ffp-contract=off`, no `-ffast-math`, no `-march=native`, a fixed
reduction order and no threads inside the integrator.

**Across builds, bit-identity is not promised.**
[`conformance/reference_hashes.tsv`](https://github.com/ibrahimsel/slipx/blob/main/conformance/reference_hashes.tsv)
is keyed by architecture, compiler, C library and build type, and
`tools/check_conformance.py` compares a run against the row matching its own
build. A mismatch there is a bug. A build with no row is a build about which
nothing was claimed.

So `slipx-conformance` printing a different hash on your machine is expected,
and published wheels carry no hash claim at all. The variable is libm rather
than the compiler: one wheel hashes differently on glibc 2.28 and 2.39 while
agreeing to every printed digit, and every x86-64 reference row is identical
across GCC 11, GCC 13 and Clang 18. That is why the C library is in the key and
why the manifest records it.

Input logging and replay re-run from a recorded input sequence, ignoring the
policies, which is what a leaderboard appeal needs once the policies are gone.

## Performance

Measured on an AMD Ryzen 7 7800X3D under WSL2, GCC 13.3, RelWithDebInfo,
single-threaded. The LiDAR cases use 1080 rays at 40 Hz over a full circle with
a 10 m maximum range, a Hokuyo UST-10LX, on a track whose walls come to 696
segments.

| Case | Measured | P1 target |
|---|---|---|
| L2 single-agent step | 1.92 us | under 5 us |
| 1 agent, L2 and 2D LiDAR, headless | 179x real time | over 100x |
| 20 agents, L2 and 2D LiDAR, headless | 8.4x real time | over 10x, missed |

No number here should be compared against a number from a different machine.
The benchmark is a plain executable with no benchmark-library dependency, run by
CTest so a change that makes the simulator an order of magnitude slower fails
the build. The twenty-agent target is the one still missed, by 24 per cent, and
the arithmetic saying why is in
[`docs/reference/performance.md`](https://github.com/ibrahimsel/slipx/blob/main/docs/reference/performance.md)
along with the before-and-after for each figure.

The integrator has no threads in it by design, and the orchestrator steps agents
in a fixed order for the same reason. The figures above are per core: a machine
with sixteen of them runs sixteen independent simulations rather than one
simulation sixteen times faster.

## Embedding in C++

Embedding the core pulls in no transitive dependency:

```cmake
add_subdirectory(slipx)
target_link_libraries(your_simulator PRIVATE slipx::core)
```

`step` is `const` and stateless: no hidden state, no clock, no ambient RNG, no
allocation. N instances parallelise trivially, and snapshot and restore are a
`memcpy`.

Parameters arrive as a plain `VehicleParams` struct. Nothing in the core parses
a file, so if you already have your own parameter format you can keep it.

## Building from source

```bash
cmake -S . -B build -DSLIPX_BUILD_PYTHON=ON && cmake --build build -j
```

```bash
ctest --test-dir build && python3 -m pytest
```

Needs CMake 3.20 and a C++17 compiler. GoogleTest is fetched if absent; the
Python parts want PyYAML, jsonschema and pybind11. None of that reaches
`slipx_core`, which configures, builds and passes its full suite alone under
`-DSLIPX_CORE_ONLY=ON`. The
[`Makefile`](https://github.com/ibrahimsel/slipx/blob/main/Makefile) wraps these
and the CI checks; `make help` lists the targets.

Verification runs in three layers, all in the suite: analytical cases with
closed-form answers, invariants (energy and momentum conservation, left/right
mirror symmetry asserted bit for bit, monotonicity under property-based
sampling), and cross-tier agreement in the low lateral acceleration limit. For
the reference car on an open differential, L1 and L2 agree on path radius to
within 1% up to 0.82 g and part company by 0.85 g. The crossover is printed on
every test run,
[plotted from the library](https://github.com/ibrahimsel/slipx/blob/main/docs/racing/assets/cross-tier-crossover.svg)
and measured by
[an example](https://github.com/ibrahimsel/slipx/blob/main/examples/02_where_the_tiers_disagree.py).

## Components

Dependencies point downward only, and `tools/dep_lint.py` fails the build if
that stops being true.

```
slipx_registry   community parameter sets (data only, no code)          planned
slipx_id         system identification: manoeuvres, fitting, reports    planned
slipx_ros        ROS 2 wrapper: topics, TF, /clock, rosbag, launch      planned
slipx            Python package: bindings, car loader, run sinks        available
slipx_sim        orchestrator: N agents, fixed step, lockstep, replay   available
slipx_scene      track loading, projection, laps, limits, walls         available
slipx_sense      2D LiDAR, IMU, wheel encoders                          available
slipx_schema     JSON Schema definitions and reference parser           available
slipx_core       vehicle dynamics. The C++ standard library, nothing
                 else.                                                  available
```

`slipx_scene` and `slipx_sense` are siblings and neither includes the other. The
directories for the planned pieces exist and are empty; nothing there
half-works.

`slipx_core` and `slipx_schema` are versioned independently, because a schema
addition the core never sees should not force a core release, and a core change
should not invalidate every car file.

## Documentation

- [**Reference**](https://github.com/ibrahimsel/slipx/tree/main/docs/reference):
  sign conventions and units, every parameter with its unit and the tier it
  first affects, the state and diagnostics blocks, the Python API, the MF-lite
  derivation, and the performance numbers.
- [**Examples**](https://github.com/ibrahimsel/slipx/tree/main/examples): three
  Python programs and the C++ reference stack, all executed by the test suite.
- [**Autonomous racing, from the ground up**](https://github.com/ibrahimsel/slipx/tree/main/docs/racing):
  thirteen articles and a glossary on the concepts themselves, for somebody
  arriving from robotics or computer science rather than vehicle dynamics. Slip
  angles, load transfer, understeer and oversteer, the racing line, the g-g
  diagram, combined slip, differentials, actuator lag and why a laser scan taken
  while moving is not a shape of the world. A guide to the subject, not a manual
  for this library.
- [**Architecture**](https://github.com/ibrahimsel/slipx/blob/main/docs/architecture/slipx.md)
  and
  [**changelog**](https://github.com/ibrahimsel/slipx/blob/main/CHANGELOG.md).

The banner at the top is output, not an illustration:
[`docs/assets/make_banner.py`](https://github.com/ibrahimsel/slipx/blob/main/docs/assets/make_banner.py)
rolls the car out of `slipx_core` at the double-track tier, and it drifts
because its rear tyre is a lower-grip compound than its front. Every figure in
the tutorial series is generated the same way, and none of them carries a model
of its own.

## Scope

No photorealistic rendering and no camera model. SlipX is LiDAR-first and runs
on CPU. That rules out a sensor, not a picture: a finished run renders to a
self-contained animated SVG with the provenance label and trajectory hash drawn
into the image.

No interactive viewer. SlipX owns the encoder rather than the viewer, so a run
is emitted as MCAP and scrubbing a timeline is served by a tool built for it.

No full-scale road vehicles, traffic or urban scenarios. No autonomy stack; the
reference controllers exist to validate the simulator, not to win with. No
aerodynamics at 1/10 scale, where it is negligible below roughly 15 m/s.

## Licence

Apache-2.0. No copyleft dependency anywhere, including the optional extras,
which is what makes embedding the core in a project with its own licensing
constraints possible.
