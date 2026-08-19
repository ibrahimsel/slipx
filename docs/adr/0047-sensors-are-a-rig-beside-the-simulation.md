# ADR-0047: sensors are a rig beside the simulation, configured per agent

- **Status:** Proposed
- **Date recorded:** 2026-08-19
- **Requirements:** SENSE-01, SENSE-02, SENSE-03, SENSE-04, SIM-02, NFR-02
- **Related:** ADR-0037 (sensing never sees the scene), ADR-0042 (DNF),
  ADR-0044 (reproducibility from the input log), ADR-0045 (the agent
  overlay), ADR-0006 (NaN for what a tier cannot represent)

## Context

The sensor models exist and are tested, but nothing owns the job of running
them against a simulation. Every consumer so far wires them by hand: the
reference stack test freezes the car's pose across a scan and says so, the
benchmark carries its own scheduling loop, and `sensors.yaml` is validated
and then read by nothing. Racing makes the gap concrete: a 20-agent race
wants the ego car fully sensed and the opponents cheap or blind, which is a
per-agent configuration question, and the ROS bridge will need one place
that turns "this car's sensor file" into scans, IMU samples and odometry
arriving on their own schedules.

The decision that had to be taken is where this machinery lives, and in
particular whether sensors go inside `Simulation`. Inside, they would be
recorded in the manifest, folded into the configuration digest, and stepped
by `advance()`; the alternative is a separate object that observes a
simulation it cannot write to. A third option, leaving the wiring to each
caller forever, is what the reference stack already shows: every caller
re-implements scheduling, nobody implements motion distortion honestly, and
the per-agent case never gets built.

## Decision

Sensors are a `SensorRig` in `slipx_sim`, a separate object holding a
`const` reference to the simulation it observes, configured per agent with
plain spec structs, and they can never perturb a trajectory.

The reasoning, in the order the constraints bind:

- **Observation is free, structurally.** The rig reads states and
  diagnostics through the simulation's const surface and holds its own
  random streams, so an agent with a full sensor suite drives bit for bit
  the trajectory of its bare twin. That is why sensors stay out of the
  manifest and the configuration digest: the digest names what changes the
  trajectory, and sensors do not. A closed-loop run whose policy steers on
  scans depends on the rig the way it depends on any other policy internals,
  and is replayed the same way: the commands are in the input log, and
  `replay` needs no rig (ADR-0044's reasoning).
- **The rig is where the layers meet.** ADR-0037 forbade sensing from
  knowing the scene and named `slipx_sim` as the place above both where
  they meet. The rig honours the same seam: the world arrives as a function
  from an agent index, a pose and a bearing to a hit, so the rig includes
  sensor headers and never scene headers. The agent index is in the
  signature because a car must not see itself: the caller closes over
  whatever answers rays (walls, the agent overlay of ADR-0045, or both) and
  applies the self-skip, which only the caller can, since only it knows
  which box belongs to which agent.
- **Schedules are exact; truth is step-resolution.** Each sensor samples at
  (k + phase) / rate, computed from k rather than accumulated, matching the
  simulation's own steps-times-dt discipline. A sample instant that falls
  between step boundaries is served by the first step at or after it and
  stamped with its exact scheduled instant, so the served truth is at most
  one step late and the schedule never drifts. The LiDAR is the exception
  where it matters: the emitter pose is interpolated per ray from the pose
  history the rig records at each `collect`, so motion distortion emerges
  as SENSE-01 requires, while OTHER agents are seen at step resolution,
  because their poses between steps are not defined by the simulation.
  Both halves of that sentence are the honest statement, and the second is
  a stated approximation.
- **Latency is a delivery time, drawn once per message.** A sample taken at
  its instant becomes visible only once simulation time reaches instant
  plus constant plus a uniform jitter, mirroring the LiDAR's own latency
  semantics (uniform because a bus delay is bounded, jitter bounded by the
  constant so nothing is stamped before it was measured). The rig owns this
  for the IMU and the encoder, whose models deliberately do not model
  transport.
- **Randomness derives per sensor.** Each sensor instance draws from
  `derive_seed(derive_seed(rig_seed, agent), instance)` with instances
  numbered in declaration order, so adding a sensor to one agent cannot
  change another agent's observations, and the draw order within a sensor
  (sample first, then jitter) is fixed.
- **A DNF'd agent stops sensing.** Its pending deliveries still arrive, but
  no new sample is taken: the simulation promised a wreck that is
  stationary and collidable, and a live IMU on a frozen car would have to
  invent readings from diagnostics the model is no longer producing.

Configuration arrives as structs (`AgentSensors`: lidars, IMUs, encoders,
each with rate, phase and latency), because parsing lives above the C++
layers; the mapping from `sensors.yaml` to these structs is the Python
loader's job and a schema revision, recorded separately.

## Consequences

- Every pre-existing trajectory and reference hash is untouched, by
  construction rather than by care: the rig cannot write to the simulation.
- The rig does not participate in snapshot and restore, and `collect`
  refuses a simulation whose clock moved backwards: after a reset or a
  restore, the caller builds a new rig and replays from the start. If a
  use for mid-run sensor state capture appears, it is its own decision.
- Consumers must call `collect` after every step for full pose-history
  fidelity; the rig serves missed intervals from the poses it saw, and a
  caller that collects rarely gets correspondingly coarse interpolation.
  That is a documented cost of keeping the rig outside `advance()`, paid
  so that observation stays structurally free.
- A sensored policy closes the loop outside the simulation (read the rig,
  post a command), so in-process sensored driving is a caller pattern
  rather than a `Policy` signature change. If that pattern proves common
  enough to deserve sugar, the sugar can be added without moving the rig.
- Sensor configuration is invisible to the manifest, so a run that wants
  to record what its stack could see must record it at the layer that
  configured the sensors (the bridge, the harness); the manifest keeps
  promising exactly what it can verify.
