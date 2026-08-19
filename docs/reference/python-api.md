# The Python API

The Python API **is** the C++ API: same names, same units, same ISO 8855 sign
conventions, same tiers. A tutorial written in one translates line by line
into the other, and there is no second implementation to drift, because the
bindings are generated from the headers the C++ tests run against.

What the Python package adds is the two things a Python user needs and a C++
embedder does not: loading a car directory, and recording a run to a file.

Runnable versions of everything below are in [`examples/`](../../examples/).

## Installing

```
pip install slipx                 # the library and the SVG sink
pip install "slipx[mcap]"         # + the MCAP encoder
pip install "slipx[rerun]"        # + the Rerun SDK
pip install "slipx[sinks]"        # both
```

SlipX installs, imports and passes its tests with neither SDK present, and
neither is imported by `import slipx`.

## Loading a car

```python
import slipx

car = slipx.load_reference_car()          # ships inside the package
car = slipx.load_car("path/to/my_car")    # a directory, not a file
print(car.summary())                      # leads with the provenance label
```

| Name | What it does |
|---|---|
| `load_car(directory, strict=False)` | Load a car directory. `strict=True` turns warnings into errors. |
| `load_reference_car(strict=False)` | The reference 1/10 car that ships with the package. |
| `reference_car_path()` | Where that directory lives, for copying. |
| `to_vehicle_params(parameters)` | Build a `VehicleParams` from an already-parsed mapping. |

A `Car` exposes `name`, `provenance`, `notes`, `warnings`, `summary()` and:

```python
params = car.params_for_tier(slipx.Tier.L2_DoubleTrack)
```

`params_for_tier` is where the loader's refusal lives. A parameter it cannot
fill produces an error **naming the field and where it should come from**,
rather than a plausible default. Asking for L2 from a schema 0.1.0 tyre file
is the canonical case: it names `c_kappa` and schema 0.2.0.

Loading needs `pyyaml` and `jsonschema`, which the distribution requires. The
core does not: `VehicleParams` is a struct you can fill in by hand, and
nothing below the loader knows what a file is.

## Stepping a model directly

```python
model = slipx.VehicleModel.create(slipx.Tier.L2_DoubleTrack, params)
state = slipx.VehicleState()
state.vel_body.x = 5.0
diagnostics = slipx.StepDiagnostics()

model.step(state, slipx.DriveInput(steer_cmd=0.1, accel_cmd=0.0),
           1.0e-3, diagnostics)
```

`step` is `const` and stateless: no hidden state, no wall clock, no ambient
random number generator. Everything that changes is in the state you passed
in, which is why a run replays.

Tiers are `Tier.L0_Kinematic`, `Tier.L1_Bicycle`, `Tier.L2_DoubleTrack` and
`Tier.L3_Extended`. **An unimplemented tier raises.** It never falls back to a
simpler one: a trajectory labelled L2 that is actually L1 is worse than no
trajectory.

Integrators are `Integrator.RK4` (the default) and
`Integrator.SemiImplicitEuler`.

## The orchestrator

For more than one car, or for a run you want a manifest and a hash from:

```python
config = slipx.SimulationConfig()
config.dt = 1.0e-3            # fixed, always
config.master_seed = 20260813
config.schema_version = car.spec.schema_version
sim = slipx.Simulation(config)

agent = slipx.AgentSpec()
agent.name = "alice"
agent.tier = slipx.Tier.L2_DoubleTrack
agent.params = params
agent.initial_state.vel_body.x = 5.0
agent.policy = lambda state, time, rng: slipx.DriveInput(
    steer_cmd=0.1, accel_cmd=slipx.hold_speed(state, 5.0))
sim.add_agent(agent)

sim.run_for(5.0)
print(sim.trajectory_hash())
```

Agents run in lockstep: every policy sees the world as it was at the start of
the step, so a result cannot depend on the order agents were added in.

| Member | Notes |
|---|---|
| `add_agent(spec)` | Returns the agent's index. No upper bound on the count. |
| `advance()` | One step for every agent. |
| `run(steps)`, `run_for(duration)` | `run_for` rounds to whole steps: a partial step would be a second step size. |
| `reset()` | Back to the initial states, clock, hashes and random streams. Does not rebuild the models, so a reset run is the same run. |
| `state(i)`, `diagnostics(i)` | **References the next step overwrites.** Copy before keeping. |
| `model(i)`, `rng(i)` | |
| `time`, `step_count`, `dt`, `agent_count` | `time` is `steps × dt`, never an accumulated sum. |
| `trajectory_hash()`, `agent_trajectory_hash(i)` | |
| `agent_running(i)`, `dnf(i)` | Rollover ends an agent's run; `dnf(i)` is the `DnfEvent` that ended it, or `None`. See below. |
| `manifest()` | See below. |
| `set_input_logging(True)`, `input_log()`, `replay(log)` | Re-run from a recorded input sequence, ignoring the policies: what a leaderboard appeal does when the policies are gone. |

A policy is a callable of `(state, time, rng)` and must be a pure function of
those three if the run is to replay. Anything else it reads, a wall clock, a
global counter or an unseeded random number generator, is a way for the run to
stop reproducing.

`slipx.hold_speed(state, target)` is a proportional speed controller, and
`slipx.step_steer(spec)` is the canonical step-steer policy.

### Contact

Agents that declare a collision footprint can hit each other: one planar
impulse with restitution and Coulomb friction per touching pair per step,
plausible and deterministic, fitted to nothing (ADR-0043). Declare the
footprint from the car file's own geometry block, and tune the constants on
the config if you must:

```python
geometry = car.spec.raw["dynamics"]["geometry"]
agent.footprint_length = geometry["length"]
agent.footprint_width = geometry["width"]
config.contact.restitution = 0.3        # plausible, not fitted
```

Both dimensions or neither: one without the other is refused. An agent with
no footprint (the default) touches nothing, which is why every scenario
written before contact existed still reproduces bit for bit. A DNF'd car
keeps its footprint and is immovable: the wreck stays on track and other
cars bounce off it.

### Rollover, the first discrete event

At L2, a step whose diagnostics show both wheels of one side at zero vertical
load ends that agent's run: the agent is DNF, its policy is never called
again, its pose freezes where the event found it and its velocities read
zero, so it stays in the world as a stationary obstacle. `dnf(i)` returns the
event, which carries the cause (`DnfCause.RolloverLeft` names the unloaded
side), the step and the time; the manifest records the outcome per agent.
There is no flight or landing simulation, and nothing un-freezes an agent
short of `reset()`. Below L2 the per-wheel loads are NaN and no car can roll,
which is a stated limitation of those tiers.

## Recording a run

```python
from slipx import sinks

run = sinks.record_run(sim, duration=8.0, stride=10)
sinks.write(run, "slalom", format="svg")     # slalom.svg
sinks.write(run, "slalom")                   # slalom.mcap, the default
```

Recording does not perturb the run: the sim is stepped exactly as `run_for`
would step it, nothing is fed back, and the trajectory hash is the same
recorded or not. `stride` decides how much is **kept**, never how much is
simulated.

| Format | Needs | Notes |
|---|---|---|
| `"mcap"` | `slipx[mcap]` | The default. Archival, Apache-2.0 down to the CLI, and the format P3's event stream will use. |
| `"rerun"` | `slipx[rerun]` | Writes an `.rrd` file. |
| `"svg"` | nothing | Standard library only. One self-contained animated document, theme-aware, with the provenance label and the trajectory hash drawn into the picture. |

Two rules every sink keeps:

- **No sink opens a window.** A file, always. Scrubbing a timeline is a solved
  problem and SlipX is not going to solve it a third time.
- **No sink draws anything the run did not contain.** A quantity the tier
  could not compute is absent (no line, no legend entry), never a flat trace
  at zero. There is no track in any picture, because nothing in SlipX has a
  track yet and a drawn kerb would assert one.

## The manifest

```python
m = sim.manifest()
m.to_json()                 # what a dispute is settled with
m.configuration_digest()    # was this the same setup?
m.trajectory_hash           # did it give the same answer?
```

Every run can emit one. It records the core and schema versions, the step size
and integrator, every seed, every agent's parameter digest, the git SHA, the
compiler and its flags, the platform, **the C library**, and the resulting
hashes.

The C library is in there because the trajectory hash tracks libm: the same
wheel produces a different hash on a different glibc, so that is the first
thing to check before treating a differing hash as a bug.

Bit-identity is promised for the same binary on the same C library, and
explicitly not across platforms.

## The tyre, on its own

The tyre model is reachable without a vehicle around it, which is what a tyre
plot and a check of a fitted set both need:

```python
tyre = slipx.make_mf_lite(coefficients, c_alpha_per_tyre, fz_nominal)
slipx.mf_lite_fy(tyre, alpha, fz)
slipx.peak_lateral_force(tyre, fz)
slipx.peak_longitudinal_force(tyre, fz)
slipx.cornering_stiffness_at_load(tyre, fz)
slipx.friction_ellipse(fx, fy, fx_max, fy_max)
```

`B` is derived at construction and is never read from a parameter set, exactly
as in the core. [The derivation](tyre-model.md) explains why.

## Determinism helpers

| Name | Notes |
|---|---|
| `Rng(seed)` | SplitMix64. Per-agent streams come from `derive_seed(master, index)`, mixed rather than added so adjacent agents do not get correlated streams. |
| `TrajectoryHash`, `hash_text(s)` | FNV-1a, with published test vectors. |
| `make_conformance_run(spec)` | The canonical run the published reference hashes describe. |
| `core_version`, `__version__` | The core's version and the distribution's. They are the same number today and are not required to be. |
