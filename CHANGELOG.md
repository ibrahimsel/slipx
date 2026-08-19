# Changelog

Semantic versioning (NFR-09), with the pre-1.0 caveat that a minor bump may
break the API. `tools/version_check.py` holds the four places a version is
written in agreement; the release workflow refuses a tag that disagrees with
the tree.

Two numbers are versioned here and they move independently. The distribution
and `slipx_core` share one; `slipx_schema` has its own. They moved apart at
schema 0.2.0, which shipped without a core release, and a release that bumps
one does not touch the other.

**A changed reference hash is a release event.** If a version changes any row
in `conformance/reference_hashes.tsv`, it is recorded here with the reason,
because every result anybody has previously compared against that row becomes
incomparable.

## Unreleased

No reference hash moves in this section so far. `slipx_scene` sits above the
core and the core's numerical paths are untouched; the eighteen rows were
re-checked, not re-measured.

- **The race event stream is MCAP.** Every race-control outcome travels as
  a JSON message on `/race/events` in an MCAP file the race layer writes
  itself (a two-hundred-line hand-rolled encoder, for the manifest
  writer's reason), with the pinned ruleset and the full mechanised
  configuration in a metadata record: the file answers "who won, under
  what rules" by itself, and a leaderboard, report or CI job is meant to
  consume it and nothing else. A full race replays from the stream alone,
  asserted by a reader that holds only the file; the reference `mcap`
  library reads a stream the C++ demo (`slipx_race_demo`) writes, which
  is what holds "one format, not two" against the same library that reads
  the run sinks. Identical races write identical bytes.
- **Race control** (ADR-0046). A new component, `slipx_race`, above the
  sim and the scene: the referee that the pinned RoboRacer ruleset
  (`github.com/f1tenth/roboracer_rules` @ `202c377`, 2025-10-13, stated by
  the build and printed by `race::ruleset_statement()`) turns into code.
  Time trials with the two-category scoring of rule 2.4.5, the obstacle
  avoidance test of 2.5.1.6 (passing means not touching it and not coming
  to a complete stop), and head-to-head rounds and matches per 2.5:
  side-by-side grid starts one car width apart, first to the lap count,
  light contact recorded and never penalised, crashes attributed from
  relative geometry and closing speed (the car contributing more approach
  speed is at fault), restarts with the at-fault car set back two metres
  plus one when the victim still runs, three warnings a disqualification,
  and the border enforced by rule rather than by physics, because nothing
  in SlipX collides a car with a wall. Every judgment the rulebook leaves
  to referees is a named `RaceConfig` field, labelled as a mechanisation.
  Every outcome is a flat timestamped `RaceEvent` for the event stream to
  encode. The sim now reports per-step `ContactEvent`s (pair, impulse,
  and each car's approach contribution) for race control to interpret.
- **The racing broadphase, and a renegotiated performance target**
  (ADR-0045). `slipx/scene/broadphase.hpp`: a prebuilt `SceneBvh` over the
  wall segments (fully specified build, ordered pruned traversal,
  thread-safe query) and an `AgentOverlay` of per-step refit oriented
  boxes answering rays with a self-skip and conservative pair queries;
  both are asserted bit-for-bit against brute-force definitions. Measured
  on the workload that matters, the BVH loses the wall rays to the grid
  by a factor of three (95 against 280 ns per ray), so the grid stays and
  the benchmark now prints both costs per commit, plus the agent-overlay
  cost of cars seeing cars. With the acceleration-structure route
  measured shut, the 20-agent target is renegotiated from 10x to over 7x
  per the standing decision, with both sessions' numbers and the
  alternation evidence recorded in `docs/reference/performance.md` (the
  racing-phase code added nothing measurable; the drift between sessions
  is the machine).
- **The barrier and its timeout policies** (ADR-0044). Commands can arrive
  asynchronously through a step-tagged `CommandMailbox`, the one
  synchronised doorway into the otherwise single-threaded simulation; the
  tag is the acknowledgement, and a missing entry is answered by the
  agent's `TimeoutPolicy`: wait (strict lockstep, timing cannot change the
  trajectory), freeze (a pause that resumes where it stopped), coast, or
  DNF (`DnfCause.Timeout`, ADR-0042's machinery). One hung agent can no
  longer hang a race unless waiting is what was configured. A live run
  with non-wait mailbox agents is decided partly by a wall clock, and its
  manifest says plainly that bit-identity then holds only under replay
  from the input log, where a missed step is recorded as a NaN-tagged
  slot; NaN commands are refused at every door, which is what makes the
  marker sound. The Python stepping calls release the GIL so a poster
  thread can feed a waiting barrier. Also fixed in passing:
  `sim.replay(sim.input_log())` used to clear the log it was replaying.
- **Agent-to-agent contact** (ADR-0043). One planar impulse with
  restitution and Coulomb friction per touching pair per step, computed by
  a pure function in `slipx_core` (`slipx/contact.hpp`) and applied by the
  orchestrator between steps. Contact exists between agents that declare a
  rectangular footprint (`AgentSpec.footprint_length/width`, the car
  file's own `geometry.length/width`); an agent that declares none touches
  nothing, so every pre-contact scenario reproduces bit for bit, asserted
  in the suite and re-checked against the published rows. The constants
  are plausible for foam bumpers and fitted to nothing, and every document
  that touches them says so; what is promised instead is determinism,
  momentum conservation, the friction cone and mirror symmetry, all held
  by the invariant tests. A DNF'd car keeps its footprint and is
  immovable. The manifest records footprints and contact constants in the
  configuration digest. Tutorial article 17 explains the model.
- **Rollover is a discrete event that ends an agent's run** (ADR-0042). The
  orchestrator reads each step's diagnostics, and both wheels of one side at
  zero vertical load is a DNF: the policy is never called again, the pose
  freezes where the event found it, the velocities read zero, and the car
  stays in the world as a stationary obstacle. The event carries its cause
  (which side unloaded), survives snapshot and restore, reproduces under
  replay, and the manifest reports each agent's outcome. There is no flight
  or landing simulation. Below L2 the loads are NaN and nothing can roll,
  which is a stated limitation of those tiers. The core is untouched: no
  reference hash moved, re-checked under the conformance script.
- **The registry is live.** The staged `registry/` content left this tree
  for https://github.com/ibrahimsel/slipx_registry: the contribution flow
  (a by-product of running `slipx-id`, not a separate act), the
  acceptance bar in prose pointing at the bar in code, and a CI runner
  over every entry on every pull request. Until the 0.4.0 release is on
  PyPI its CI installs SlipX from git source, and says so. The self-test
  that exercised the staged runner now exercises the same bar directly.
- **The validation report.** `validation:` bags in a session file replay
  through the emitted car and produce `validation.svg` beside it: measured
  against replayed yaw rate, lateral acceleration and encoder speed, with
  the headline being the worst channel of the worst run. The emitted
  provenance names the report, which is what the registry's acceptance bar
  requires; the report itself states that a set which validates on these
  runs has validated on these runs and on nothing else.
- **`slipx-id`, one command from bags to a car directory** (ADR-0040). The
  fitter reads rosbag2 recordings directly, without ROS: sqlite3 storage
  with the standard library, MCAP through the existing extra, CDR decoded by
  hand for exactly the message set the manoeuvre library records, and a
  topic whose type it does not speak refused by name. A session file names
  the car's bench constants, the topics, the bags and the provenance; the
  emitted car directory carries per-parameter residuals, the SHA-256 of
  every bag consumed, and loads straight back through `slipx.load_car`.
  Emission refuses without a populated provenance block, and anything no
  stage identified stays provisional with a note instead of becoming a
  quiet number.
- **Schema 0.4.0** (ADR-0041): the tyre compound becomes a community
  vocabulary like surfaces already were (an identification tool fits tyres
  a two-word enum never anticipated); provenance gains an optional `data`
  block naming and digesting the recordings a fit consumed; and the
  registry's acceptance bar is now code,
  `slipx_schema.rules.check_registry_submission`. Every 0.3.0 document is
  already a valid 0.4.0 document and the migrations are identities.
- **`slipx_id`, the identification package** (ADR-0038): the staged fits of
  the manoeuvre library, a deterministic Levenberg-Marquardt on the standard
  library alone, and the synthetic self-test that round-trips every MF-lite
  parameter through the forward model before any real data exists. The
  Magic Formula shape pair is identified as a curve rather than as
  coordinates, and the fitter reports the C-E entanglement by name instead
  of printing two confident numbers; the ADR carries the reasoning.
  `VehicleParams` gained `copy()`.
- **Fixed: a tyre file loaded under a different car silently kept the wrong
  load reference** (ADR-0039). A tyre file states its coefficients at its own
  `nominal_load`; the core states every tyre at the static per-tyre load of
  the car wearing it, and the loader never bridged the two, which
  misreferenced every load-dependent coefficient for any pairing other than
  the file's original car. The loader now restates `mu_y0`, `mu_x0`,
  `c_alpha` and `c_kappa` at the car's static load, exactly (MF-lite's load
  laws are power laws, so the restatement is lossless and the derived `B` is
  invariant), records a note naming the factors, and warns when the ratio is
  extreme enough to look like a units error. The reference pairing is
  unaffected: its tyre file now states the exact static load, so every
  number ADR-0032 chose still loads to the bit.
- **Fixed: a LiDAR ray through a grid cell corner could miss the wall it
  hits, on Windows.** The raycast index's grid walk never steps diagonally;
  through an exact corner it visits one of the two cells sharing it, and
  which one depends on the last bit of the C library's `cos` and `sin`.
  MSVC's UCRT rounds `cos(-π/4)` and `-sin(-π/4)` to the same double, the
  walk tied, stepped around the cell holding the wall, and reported open
  space where brute force reports a wall at exactly the corner; glibc rounds
  the pair one ulp apart, which is the only reason no Linux test ever saw
  it. The walk now tests both corner-adjacent cells whenever a crossing
  lands within a nanometre of a cell corner, which can only add hits the
  intersection test already approves.
- **`slipx_scene`, the track.** A centreline loads from the four-column form
  of the TUM racetrack database, read unextended, with arc length derived
  rather than read. Everything about a track that is not geometry lives in a
  manifest beside it, and the manifest declares a surface identifier rather
  than a friction coefficient, so grip still comes from the
  `(compound, surface)` tyre file that somebody measured (ADR-0034).
- **Schema 0.3.0** adds one document kind, the track manifest, and changes no
  field of any kind that already existed (ADR-0036). Every migration step is
  the identity; car directories written at 0.1.0 and 0.2.0 keep loading. The
  schema version and the distribution version are different numbers again,
  which is the normal state.
- **SlipX ships no third-party track geometry, and now says why** (ADR-0035).
  Every public centreline set in this format is copyleft, and the F1TENTH
  maps of real venues carry no licence at all. `tools/convert_track.py`
  fetches one into a track directory on your own machine instead, recording
  the source, its licence and the date in the manifest, and it refuses to
  write geometry we may not redistribute into this repository.
- **One generated track ships**, `examples/tracks/paddock_stadium`: two 8 m
  straights and two 3 m ends, 34.85 m a lap, labelled provisional and
  described in its own manifest as not being a real place. It is what CI and
  the examples run on.
- **The L2 step is 2.6 times faster and computes exactly what it did before.**
  Measured on the machine named in `docs/reference/performance.md`, 4.98 us to
  1.92 us, with a single agent and a 2D LiDAR going from 93x real time to 179x
  and twenty agents from 4.0x to 8.4x. Every reference hash is unchanged, which
  is the point: the tier calls `pow` forty times a step instead of two hundred
  and eighty, because the tyre's load sensitivity splits into a half that
  depends on the tyre and a half that depends on the load, and it evaluates
  each wheel's slip angle and Magic Formula shape term once per derivative
  instead of once per load pass, because neither depends on a vertical load.
  The wall traversal tests each grid cell as it reaches it and stops at the
  first hit it can prove is nearest, instead of gathering every candidate along
  the ray. Of the three P1 performance targets, the single-agent LiDAR one is
  now met and the twenty-agent one is still missed, by 24 per cent; the
  arithmetic saying why, and what it would take, is on that page.
- **Two holes the speed work found in the test suite are now closed.** No case
  distinguished the front tyre from the rear one in the friction budget,
  because the reference car has its CoG mid-wheelbase and the same compound at
  both ends, so a model reading one tyre for all four wheels produced every
  published trajectory unchanged. And no track under test had segments uneven
  enough in length for a wall's bounding box to reach cells the wall crosses
  nowhere near, which is the case that separates "nearest wall" from "first
  wall found".

## 0.2.0

**The first final release**, and the one where the double-track tier is
complete, reachable from a car file and visible.

- **L2 is built and is the tier at which different cars behave differently.**
  Thirteen states: four contact patches with per-corner loads, MF-lite tyres
  with a real peak and falling branch, a combined-slip friction ellipse, the
  tyre relaxation transient, an open, spool or preloaded-LSD differential on a
  2WD or 4WD layout, an ESC torque-speed curve with current and regen limits,
  battery sag and state of charge, and a slew-limited second-order steering
  servo.
- **Schema 0.2.0** adds the longitudinal slip stiffness and the actuator
  fields, so a car directory parameterises all three built tiers.
- **`slipx.sinks`**: a finished run is recorded once and written to a file.
  MCAP by default, Rerun as an extra, and an SVG sink that needs nothing
  installed at all.
- **Every figure and the README banner are generated from `slipx_core`**
  rather than from a model that lived beside it.
- **Reference documentation** (`docs/reference/`), the MF-lite derivation and
  three runnable examples.

**`0.1.0` final will never be published.** ADR-0017 planned to cut it from the
`0.1.0a1` tree once the pre-release had proven the packaging path; it has, and
that proof is the whole of what a content-identical final would have added.
Reasoning in
[ADR-0029](docs/adr/0029-no-0-1-0-final-first-final-is-0-2-0.md). `pip install
slipx` now resolves to this release rather than to the pre-release.

**Every reference hash in this release differs from `0.1.0a1`'s**, in three
separate deliberate movements recorded below. A result compared against a
number published in `0.1.0a1` is not comparable with one produced here.

### L2, in the order it was built

The pieces below landed before any tier consumed them; each is usable on its
own.

- New public header `slipx/load_transfer.hpp`: quasi-static longitudinal and
  lateral load transfer from mass, CoG height, wheelbase and track (CORE-05),
  plus the static rollover threshold `g t / (2 h)`. Header-only and pure.
  Reasoning in [ADR-0022](docs/adr/0022-load-transfer-is-quasi-static.md).
- New public header `slipx/tyre.hpp`: MF-lite, a reduced Magic Formula with
  load sensitivity and a combined-slip friction ellipse (CORE-06). The
  stiffness factor `B` is derived from the cornering stiffness rather than read
  from a parameter set, so MF-lite reproduces L1's linear tyre exactly at small
  slip and contributors are never asked for a number they cannot measure.
  Reasoning in
  [ADR-0023](docs/adr/0023-mf-lite-derives-b-from-cornering-stiffness.md).
  The pure-slip longitudinal law is absent: it needs a slip stiffness that
  `tyre.schema.json` 0.1.0 does not carry. `friction_ellipse` is independent of
  that and is here. See
  [ADR-0025](docs/adr/0025-c-kappa-enters-the-core-ahead-of-the-schema.md).
- New public header `slipx/relaxation.hpp`: tyre relaxation length, a
  first-order lag in distance rolled rather than in time, so the time constant
  is `sigma / speed` (CORE-07). It lags the slip angle rather than the lateral
  force, because a lagged force is not bounded by the friction budget it was
  produced under and would report grip on a lifted wheel. Reasoning in
  [ADR-0026](docs/adr/0026-relaxation-lags-the-slip-angle-not-the-force.md).
- `VehicleParams` gains `relax_length` [m]. `VehicleState` gains a per-wheel
  `alpha_lag` [rad].

- **Tier L2, the double-track model, is implemented** (CORE-02, CORE-12).
  `VehicleModel::create` no longer throws for it. Ten states: the six of L1
  plus a lagged slip angle per wheel. Four contact patches with per-corner
  vertical loads, MF-lite with a real peak and falling branch, a combined-slip
  friction ellipse, and the tyre transient. `StepDiagnostics` per-wheel slip
  angles, slip ratios, forces, loads and saturation flags are numbers at L2
  where they are NaN below it.

  It is the MINIMAL double-track and the gaps are deliberate: no differential
  or drive layout (CORE-11), no ESC (CORE-08), no battery (CORE-09), no
  steering servo (CORE-10), parallel steer rather than Ackermann, and no wheel
  rotational state, so a locked or spinning wheel is not representable.
  `l2_double_track.cpp`'s header lists every one of them and
  [ADR-0027](docs/adr/0027-l2-closes-its-algebraic-loops-without-iterating.md)
  gives the reasoning for the last.

- `VehicleParams` gains `tyre_front` and `tyre_rear` (MF-lite coefficients per
  axle, including the relaxation length, which is a property of the tyre) and
  `c_kappa`. The `relax_length` field added earlier in this cycle moved into
  `TyreCoefficients`; nothing released ever carried it at the top level.

- **L2 cannot be built from a tyre file at schema 0.1.0.**
  `Car.params_for_tier` raises, naming the missing longitudinal slip stiffness
  and the schema version that adds it, rather than defaulting it. That is a
  refusal and not the tier substitution ADR-0005 forbids: nothing is returned
  at all. Building L2 from a `VehicleParams` constructed in C++ or Python works
  today. See
  [ADR-0025](docs/adr/0025-c-kappa-enters-the-core-ahead-of-the-schema.md).

- New reference rows for L2 under both integrators. They are additions, so no
  previously published number became incomparable by their arrival.

### Emitting a run

- **New subpackage `slipx.sinks`** (SINK-01 to SINK-05). A finished run is
  recorded once into a `Recording` and handed to a sink, and a sink writes one
  file. `slipx.sinks.write(recording, path)` is the whole API;
  `record_run(sim, duration=..., stride=...)` builds the recording. SlipX owns
  the encoder and not the viewer, and the reasoning, the rejected alternatives
  and the licence and archival arguments are in
  [ADR-0028](docs/adr/0028-runs-are-emitted-to-sinks-viewers-are-external.md).

- **MCAP is the default sink** and needs the `slipx[mcap]` extra. JSON messages
  under a JSON Schema per channel, two channels per agent, the provenance label
  and the whole run manifest as metadata records.

- **A Rerun sink** is available under the `slipx[rerun]` extra: one scalar time
  series per recorded column, under an entity tree that mirrors the state
  layout.

- **Both are extras and never install requirements.** SlipX installs, imports
  and passes its tests with both absent, and neither SDK is imported when
  `slipx` is. `slipx[sinks]` installs both.

- **A quantity a tier cannot represent arrives absent, never as zero.** MCAP
  leaves the key out of the JSON object; the Rerun sink does not send the
  column. `record_run` is where that is decided, once, for every format: the
  zeros `VehicleState` parks in fields an unimplemented tier does not integrate
  become NaN on the way out, because the reason they are zeros is that the
  state is hashed and that reason does not extend to a plot
  ([ADR-0006](docs/adr/0006-diagnostics-report-nan-not-zero.md)).

- **No sink opens a window**, even where the SDK offers to. `tools/licence_scan.py`
  now cross-checks the extras `pyproject.toml` offers against a table carrying
  their licences, and CI gains a job that installs both and runs the suite with
  them present.

No reference hash moves: nothing here touches a numerical path, and recording a
run does not perturb it.

### Every reference hash moves

**All twelve rows of `conformance/reference_hashes.tsv` are rerecorded.**
`alpha_lag` is part of `VehicleState`, the trajectory hash covers the state
layout in a fixed field order, and four new fields per step change the hash for
every tier including L0 and L1, neither of which has a tyre transient at all.

A result compared against the numbers published in `0.1.0a1` is **not
comparable** with one produced from this tree. The old and new values, for
x86-64 `RelWithDebInfo`, identical across GCC 11, GCC 13 and Clang 18 as
before and measured separately under each:

| Case | `0.1.0a1` | Now |
|---|---|---|
| L0 / rk4 | `d74f90169a5951c2` | `cf6aba9e280a24b9` |
| L0 / semi_implicit_euler | `44b1d28010f293c4` | `4cb3269ec5ba7ac3` |
| L1 / rk4 | `d44a9a68616ec899` | `f4da160a691289eb` |
| L1 / semi_implicit_euler | `9a2532ced2e1e06d` | `2e2fb5a549ad190c` |

This is a change to the state layout the hash covers, which
[ADR-0008](docs/adr/0008-reference-hashes-are-keyed-by-build.md) lists as a
legitimate reason for a hash to move. It was taken once, deliberately, rather
than accumulated across the rest of the tier.

### Schema 0.2.0: L2 is reachable from a car file

- **`slipx_schema` moves to 0.2.0**, one bump covering both this milestone
  and the coming drivetrain slice; the audit and the field list are
  [ADR-0030](docs/adr/0030-schema-0-2-0-adds-c-kappa-and-the-actuator-fields.md).
  `tyre.schema.json` gains `linear.c_kappa`, the longitudinal slip stiffness
  per tyre. `limits.schema.json` gains an `esc` block (`torque_stall`,
  `omega_free`, `torque_per_amp`, `efficiency`) and the pack voltage
  endpoints `pack_v_full` and `pack_v_empty`, consumed when the drivetrain
  slice lands. The 0.1.0 to 0.2.0 migration is the identity for every kind:
  every new field is optional, so a migrated file gains nothing it did not
  carry.
- **`Car.params_for_tier(Tier.L2_DoubleTrack)` returns parameters** when the
  tyre files carry `c_kappa`, so L2 is reachable from a car file. The
  ADR-0025 refusal stays for files that lack the field, naming each gap and
  saying why migrating the version cannot supply the measurement.
- `mf_lite.B` is optional and never consumed, which
  [ADR-0023](docs/adr/0023-mf-lite-derives-b-from-cornering-stiffness.md)
  already said and the schema now agrees with. A stated `B` is cross-checked
  against the derived value; a disagreement is a named warning.
- New plausibility warning on the `C`/`E` pair: each bound can pass alone
  while the pair puts the tyre's peak at a slip angle no car reaches, so the
  loader checks where the pair puts the peak and warns above 4 times the
  linear saturation angle.
- The reference tyre file carries `c_kappa: 120.0`, provisional like
  everything else in it. Its `mf_lite` block now matches the core's default
  shape (`C` 1.68, `E` 0.42, peak at about 2.7 times the linear saturation
  angle) and drops `B`: the previous pair put the peak at 8.8 times, and its
  stated `B` disagreed with the value the model derives, both of which the
  new warnings would have flagged on every load.

No reference hash moves: nothing here touches a numerical path, and
`tools/check_conformance.py` was verified unchanged against the recorded
rows.

### Drivetrain and actuators: L2 is complete

The four deliberate gaps in minimal L2, closed in one slice. Design and costs
in
[ADR-0031](docs/adr/0031-drivetrain-and-actuators-are-quasi-static-except-the-servo.md).

- **Differentials and drive layouts** (CORE-11). Open, spool and
  preloaded-LSD torque splits on 2WD front, 2WD rear or 4WD (locked centre,
  50/50), replacing the equal four-wheel split, all in closed form inside
  ADR-0027's two-pass structure with no wheel rotational state. The open
  diff reproduces the no-yaw-moment behaviour the equal split was chosen
  for, and its defining failure: a lifted inside wheel starves the whole
  axle. The spool drives the inner wheel harder, scrubs while coasting and
  pushes wide. **Braking goes through the driven axle only**: a 1/10-scale
  car brakes through its motor, so there are no friction brakes and no brake
  bias field, and the previous equal four-wheel braking split is gone.
- **ESC** (CORE-08). A torque-speed curve stated at the wheels, scaled by
  actual pack voltage, capped by `torque_per_amp * current_max`; braking
  torque capped by the regen limit, which on the provisional numbers is
  about 0.23 g and is the only brake the car has. `accel_max` and
  `decel_max` remain command bounds. New diagnostics `drive_torque`,
  `pack_current` and `esc_saturated` (NaN or false below L2).
- **Battery** (CORE-09). Open-circuit voltage linear in state of charge
  between `pack_v_empty` and `pack_v_full`, internal-resistance sag closed
  in a fixed two passes, state of charge integrated from the terminal
  current. `soc` becomes integrator state at L2; `pack_v` is written
  algebraically each step. An ideal supply (equal endpoints, zero
  resistance) reproduces the bare curve exactly, and the tests hold it to
  exact equality.
- **Steering servo** (CORE-10). `steer` and `steer_rate` become integrated
  state: a slew-limited second-order lag with a hard travel stop, so
  commanded and achieved steer are different numbers and both are visible.
  An underdamped servo overshoots by up to
  `exp(-pi zeta / sqrt(1 - zeta^2))`, and the invariant suite bounds it
  rather than denying it.
- `VehicleParams` gains the `layout` and `differential` enums,
  `lsd_preload`, the ESC block (`torque_stall`, `omega_free`,
  `torque_per_amp`, `drive_efficiency`, `current_max`,
  `regen_current_max`), the battery block (`pack_nominal_v`, `pack_v_full`,
  `pack_v_empty`, `pack_capacity_ah`, `pack_internal_resistance`) and the
  servo constants (`steer_rate_max`, `steer_bandwidth`, `steer_damping`),
  all with units and sign conventions in the bindings. The struct default
  differential is **open** on the rear axle; the reference car's file stays
  `spool`, because it describes a real class of car.
- `Car.params_for_tier(L2)` fills the new fields from `limits.yaml` and
  `dynamics.yaml` and refuses by name when a block is absent, including the
  ADR-0030 case of current limits stated in amperes with no
  `torque_per_amp` to turn them into torque. The reference car's
  `limits.yaml` gains its `esc` block and pack endpoints, provisional like
  everything else in it.
- Recordings carry `steer_rate`, `soc` and `pack_v` from L2 (previously
  reserved rows), plus the new diagnostics columns; below L2 they still
  arrive absent, never as a plausible zero.
- The manifest's parameter digest now covers the tyre blocks and `c_kappa`,
  which it had missed, and every new field.
- Three articles the tutorial series owed:
  [differentials](docs/racing/10-differentials.md),
  [the motor, the ESC and the battery](docs/racing/11-motor-esc-and-battery.md)
  and [actuator lag](docs/racing/12-actuator-lag.md), with three new figures.
  Concepts, not SlipX, as the series brief requires.

### The six L2 reference rows move

The L2 state vector grew from 10 to 13 and the drivetrain changed the
forces, so every L2 hash moves, once, at the end of the slice. L0 and L1
were re-measured under all three compilers and are byte-identical to before,
as they must be: no code either tier executes changed. A previous L2 result
is **not comparable** with one produced from this tree. Old and new values
for x86-64 `RelWithDebInfo`, identical across GCC 11, GCC 13 and Clang 18
and measured separately under each:

| Case | Before | Now |
|---|---|---|
| L2 / rk4 | `5735026d574b9b59` | `1e6c4659f6d3193a` |
| L2 / semi_implicit_euler | `81a36eefe4447697` | `bbb29ca838cb97ba` |

### An SVG sink, with no dependency at all

- **New format `svg`**, registered in the same table as the other two and
  special-cased nowhere: `sinks.write(run, "step_steer", format="svg")`.
  `sinks.formats()` now returns `("mcap", "rerun", "svg")`.
- **No extra, and no import outside the standard library.** A test parses the
  module's own source and asserts its absolute imports are exactly `math`,
  `pathlib`, `typing` and `__future__`, so the property is checked rather than
  intended.
- One self-contained animated document: the trajectory as a path, a marker
  animated along it from the recorded position and yaw, and one time-series
  panel per quantity the tier actually produced, each with a moving cursor.
  SMIL, so there is no script in the file and nothing to fetch.
- **The provenance label and the trajectory hash are drawn into the image**,
  the hash a second time on its own line in a monospace face, because a
  rendered run gets pasted into a slide with everything around it discarded.
- **Nothing is drawn that the run did not record.** No track, and no car body:
  an outline needs a length and a width, and the recording carries a digest of
  the parameters rather than the parameters. What moves along the path is a
  marker sized in screen pixels. A test asserts the drawn-element inventory
  against the recording's own column names.
- **A quantity the tier cannot represent is absent, never a plotted zero.** A
  panel whose columns are all NaN is not drawn and the document is shorter:
  ten panels at L2, five at L1, four at L0. A NaN inside a column breaks the
  trace rather than bridging it.
- **Byte-identical run to run**, which the SDK-backed sinks cannot promise.
- Theme-aware through an embedded `prefers-color-scheme` stylesheet and an
  opaque background card, so one render is legible on a light and a dark page.

No reference hash moves: encoding a run does not perturb it, and a test
asserts the trajectory hash is unchanged by writing.

### The tyre model is reachable from Python, and the figures come out of it

- **New bindings**: `slipx.MfLite`, `slipx.TyreCoefficients`,
  `slipx.CombinedForce`, `make_mf_lite`, `mf_lite_fy`, `peak_lateral_force`,
  `peak_longitudinal_force`, `cornering_stiffness_at_load` and
  `friction_ellipse`. A vehicle model answers what a car did; these answer
  what a tyre does, which is what a tyre plot and a check of a fitted set both
  need. `B` is derived at construction and is never read from a parameter set,
  exactly as in the core.
- **The tutorial series' figures are generated from `slipx_core`.**
  `docs/racing/assets/make_figures.py` no longer carries its own Magic
  Formula: every tyre curve in the series is evaluated by the library at the
  reference car's parameters. The two had drifted, and the drift is what
  ADR-0032 found.
- **The README banner is a rollout of the double-track tier.**
  `docs/assets/make_banner.py` no longer carries its own single-track model
  either. The car drifts because its rear tyre is a lower-grip compound than
  its front, which is a parameter choice rather than a special case in the
  drawing.
- **New figure `cross-tier-crossover.svg`**, the released artefact of the L1
  versus L2 comparison, generated from the library and referenced from
  [article 3](docs/racing/03-vehicle-models.md). On an open differential the
  two tiers agree on path radius to within 1% up to 0.82 g and cross that band
  by 0.85 g; on the reference
  car's spool they disagree by 15% from the first degree of steering, because
  a single-track model has no differential to represent. The README's
  "within 1% below 0.23 g" was measured before the tyre was corrected and has
  been replaced.
- Three tutorial articles moved worked numbers with the tyre: the skidpad
  understeer-gradient example in article 7, its peak-location figure reading,
  and article 9's combined-slip arithmetic, which also picks up the reference
  car's `mu_x0` of 1.05 in place of a value the script had invented.

### The reference tyre stops being made of sponge, and the L1 and L2 rows move again

The reference car's cornering stiffness was 60 N/rad per tyre against a static
per-tyre load of 8.6 N. With `mu_y0 = 1.1` that put the tyre's force peak at
**24 degrees of slip angle**, which no car reaches, so the reference car could
not get to its own tyre peak and MF-lite's falling branch was unreachable in
every figure drawn from it. It is now 210 N/rad per tyre (420 and 455 N/rad on
the front and rear axles, keeping the old ratio exactly), which puts the peak
at **6.9 degrees**. Both the old and the new numbers are `provisional`;
the difference is that the new one is plausible. Reasoning, including why
`mu_y0` and `c_kappa` were left alone, in
[ADR-0032](docs/adr/0032-the-reference-tyre-peaks-where-a-tyre-peaks.md).

This is the **second** hash movement in this unreleased cycle. The two are
independent and both are listed, because a release note that merges them would
be describing a change nobody made. Nothing has been published since
`0.1.0a1`, whose rows were already incomparable with either.

All twelve L1 and L2 rows move. **The six L0 rows do not**, and were
re-measured under all three compilers to confirm it: the kinematic tier has no
tyre. Old and new values for x86-64 `RelWithDebInfo`, identical across GCC 11,
GCC 13 and Clang 18 and measured separately under each:

| Case | Before | Now |
|---|---|---|
| L1 / rk4 | `f4da160a691289eb` | `0d8f69a1e3b58038` |
| L1 / semi_implicit_euler | `2e2fb5a549ad190c` | `2649495a464deb0b` |
| L2 / rk4 | `1e6c4659f6d3193a` | `1bbccc8a6af12ae0` |
| L2 / semi_implicit_euler | `bbb29ca838cb97ba` | `893d57a524b12892` |
| L0 / rk4 | `cf6aba9e280a24b9` | unchanged |
| L0 / semi_implicit_euler | `4cb3269ec5ba7ac3` | unchanged |

### The C library joins the build key, and no hash moves

`conformance/reference_hashes.tsv` gains a `libc` column between
`compiler_major` and `build_type`. **No hash changes.** The eighteen existing
rows are re-keyed, not re-measured: each gains the `glibc-2.39` it was in fact
recorded under, so nothing anybody has compared against becomes incomparable.

The reason is a measured one. The trajectory hash tracks libm, not the
compiler, and libm is not correctly rounded: one wheel, byte for byte the same
binary, produced two different trajectory hashes on glibc 2.28 and glibc 2.39.
Every column of the old key was identical across those two runs, so the run on
2.28 matched a published row it was never entitled to match and
`tools/check_conformance.py` reported it as a determinism bug in code that was
behaving exactly as designed. It now gets the honest outcome instead: no
reference row for this build, exit 0, and the row printed for whoever wants to
publish it. Reasoning in
[ADR-0033](docs/adr/0033-the-c-library-is-part-of-the-build-key.md), which
supersedes ADR-0008.

- **The run manifest gains `libc_id` and `libc_version`** in its `build` block,
  read at run time rather than captured when CMake configures: for a
  redistributed wheel the binary is fixed at build time and the C library is
  not chosen until it is installed. Both feed the configuration digest, so two
  runs on different C libraries are now different setups rather than one setup
  that failed to reproduce. Exposed on `slipx.RunManifest`.
- **The manifest's determinism note is narrower and truer.** `"within_build":
  "bit-identical"` is now `"bit-identical for the same binary on the same C
  library"`, and the strings no longer cite requirement IDs that a reader of
  this repository cannot look up.
- `tools/exit_gate.py` prints the build it ran on, C library included, and its
  failure message says to check that line first.

## 0.1.0a1

First publication. Pre-release: the version number is the one part of a release
that can never be withdrawn, so the first artefacts under this name are spent
proving the packaging path against a live index rather than being pinned.

- `slipx_core` with tiers L0 (kinematic bicycle, 4 states) and L1 (dynamic
  bicycle, linear tyres, 6 states) behind one `VehicleModel` interface. RK4 and
  semi-implicit Euler. No dependency outside the C++ standard library.
- L2 and L3 are not implemented. `VehicleModel::create` throws for them rather
  than falling back to L1, because a trajectory labelled L2 that is actually L1
  is worse than no trajectory.
- `slipx_sim`: fixed-step lockstep orchestrator, per-agent seeded RNG, run
  manifests, trajectory hashing, replay from an input log.
- `slipx_schema` 0.1.0: six JSON Schema documents and a reference parser with
  version gating and migration machinery.
- Python bindings via pybind11, the car loader, and the `slipx-conformance`
  console script. The reference car ships inside the wheel.
- Reference trajectory hashes published for x86-64 under GCC 11, GCC 13 and
  Clang 18 at `RelWithDebInfo`.

No parameter set here has been validated against a real car. Every shipped set
is labelled `provisional` and the tooling prints the label (NFR-08).
