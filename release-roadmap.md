# SlipX release roadmap: 0.1.0a1 to 1.0.0

Last updated: 2026-08-12. Published version: `0.1.0a1` (PyPI, tag `v0.1.0a1`,
commit `644cb12`). Current tree: L2 minimal double-track and the sink layer are
implemented and unreleased.

This file is the working plan from the current state to a `1.0.0` release of
the `slipx` distribution. It is written to be executed and updated by agents
working in this checkout, one task at a time, in order unless a task says
otherwise.

## How to use this file

- Every task has a stable ID (`M2.4`), a checkbox and a "Done when" line.
  Mark a task `[x]` only when its "Done when" holds and the verification
  commands in the last section pass.
- Update the `Status:` line of a milestone when it changes
  (`not-started | in-progress | blocked | done`), and the `Last updated:` line
  at the top of the file whenever you edit it.
- Never delete a task. A task that becomes wrong gets struck through with a
  one-line reason and, if needed, a replacement task with a new ID appended to
  the same milestone.
- Tasks marked **DECISION** need the user (or an ADR) to settle something
  before implementation. Do not implement past an unsettled DECISION that the
  task depends on.
- `docs/adr/` is authoritative for decisions and their reasoning. Read
  `docs/adr/README.md` before overriding anything here. Where this file and an
  ADR disagree, the ADR wins. `docs/spec/SRS.md` (local-only, gitignored)
  carries the requirement IDs cited here; every task below restates the
  requirement in words so the file stands alone.

## What 1.0.0 means (proposed, to be ratified)

`1.0.0` of the distribution is a semantic-versioning commitment on the public
API, and the point at which the project's central claim has been made true by
external facts, not internal milestones. Proposed criteria, each tracked as a
task in M8:

1. The P1, P2 and P3 exit gates are met (an external team's tuning transfers
   to hardware; three externally contributed, validation-reported parameter
   sets exist; a course or competition has run an evaluation on SlipX).
2. The `slipx_core` public API is audited, documented and frozen; from 1.0.0 a
   breaking change means a major bump.
3. Performance targets are measured and published; the doc coverage and
   cross-architecture tolerance gates run in CI.
4. Every claim in the README and in tooling output is true, with "validated"
   used only where a validation report backs it.

P4 (3D sensing) and P5 (ecosystem) are directions, not commitments, and are
**not** part of 1.0.0 unless the re-plan at the P3 gate pulls something in.
Ratifying this definition is itself a task (M8.1) and wants an ADR.

## Version ladder (proposed, not yet decided)

The distribution and `slipx_core` share one version; `slipx_schema` has its
own and the two are never coupled (`tools/version_check.py` refuses to compare
them). Numbers below are a proposal to make the milestones concrete; the
actual number is decided at each release.

| Milestone | Distribution | Schema | Content |
|---|---|---|---|
| M0 | (0.1.0 decision) | 0.1.0 | Housekeeping, loose ends |
| M1 | unreleased | 0.2.0 | `c_kappa`, L2 reachable from a car file |
| M2 | unreleased | 0.2.0 | Drivetrain and actuators (ESC, battery, servo, differential) |
| M3 | unreleased | 0.2.0 | `slipx.viz`, the SVG sink; regenerated figures |
| M4 | **0.2.0** | 0.2.0 | Release: L2 complete and visible |
| M5 | **0.3.0** | as needed | Sensors, track, ROS 2, reference stack; P1 gate |
| M6 | **0.4.0** | +provenance | System identification and the registry; P2 gate |
| M7 | **0.5.0** | as needed | Racing: contact, race control, events; P3 gate |
| M8 | **1.0.0** | independent | API freeze, gates verified, release |

---

## M0. Housekeeping and loose ends

Status: done. Size: small (hours). Hash impact: none.

- [x] **M0.1** Commit `docs/adr/0028-runs-are-emitted-to-sinks-viewers-are-external.md`
  (currently untracked) together with the unstaged index row in
  `docs/adr/README.md`. Five existing commits cite a record that is not in the
  repository.
  Done when: the record is tracked and the ADR index lists it.
- [x] **M0.2** **DECISION**: `uv.lock` is untracked and nobody has decided
  whether it belongs in the tree. Either commit it or add it to `.gitignore`,
  deliberately.
  Done when: `git status` is clean of `uv.lock` one way or the other.
  Decided 2026-08-12 by the user: committed.
- [x] **M0.3** Fix the stale `Tier` enum docstrings in
  `src/bindings/slipx/src/bindings.cpp` (around line 108): `L2_DoubleTrack` is
  still documented as "Not implemented yet", wrong since 2026-08-03. This is
  what `help(slipx.Tier)` prints. Do not cite requirement IDs in the new text
  (code comments must stand alone for readers without the spec).
  Done when: `help(slipx.Tier)` describes L2 as built and minimal, and names
  what it lacks; the L3 text still says it raises.
- [x] **M0.4** **DECISION**: whether to publish `0.1.0` final. The plan of
  record (ADR-0017) cuts it from the same tree as `0.1.0a1`, i.e. from commit
  `644cb12`, now that the pre-release has rendered and installed correctly.
  The current tree cannot be `0.1.0`: its reference hashes are incompatible
  with the published `a1` numbers. Note the pip behaviour: while no final
  release exists, plain `pip install slipx` selects the pre-release; the first
  final changes that. Options: (a) tag and publish `0.1.0` from `644cb12`,
  (b) skip straight to `0.2.0` at M4 and record why. Either is fine; deciding
  by default is not.
  Done when: decided and recorded (CHANGELOG note or ADR if it changes the
  release policy).
  Decided 2026-08-12: option (b), recorded as ADR-0029 with a CHANGELOG note;
  `0.1.0` is skipped and never published, the first final is `0.2.0` at M4.

## M1. Schema 0.2.0: the longitudinal slip stiffness

Status: done. Size: small (days). Hash impact: **none** (no numerical
path changes; verified against the recorded rows).

Goal: L2 becomes reachable from a car file. Today the loader
(`Car.params_for_tier`) deliberately raises for L2, naming the missing
longitudinal slip stiffness `c_kappa` and schema 0.2.0 (ADR-0025). This
milestone makes that error message come true.

- [x] **M1.1** Audit what M2 (ESC, battery, servo, differential) will need
  from the schema before cutting 0.2.0, so the minor bump happens once.
  `dynamics.yaml` was designed to carry drivetrain, ESC and servo sections;
  verify the 0.1.0 schema actually has every field M2's `VehicleParams`
  additions will be filled from, and put any missing ones into the same 0.2.0
  bump. Silent defaulting is prohibited (SCH-02), so a field the loader cannot
  fill must either exist in the schema or produce a refusal that names it.
  Done when: a written list (in the M1 commit message or an ADR) of every
  field 0.2.0 adds, cross-checked against M2's parameter needs.
- [x] **M1.2** Add `c_kappa` to `tyre.schema.json` as schema 0.2.0, with the
  0.1.0 to 0.2.0 migration. The migration cannot invent a value: a migrated
  0.1.0 file still has no `c_kappa`, still loads for L0 and L1, and still gets
  the naming refusal for L2. Bounds should reflect what is identifiable from
  encoder slip ratio against IMU acceleration.
  Done when: schema version gating, migration and validation tests cover both
  file versions; `tools/version_check.py` reports schema 0.2.0 and still
  compares it to nothing.
- [x] **M1.3** `Car.params_for_tier` fills `c_kappa` from a 0.2.0 tyre file
  and stops refusing L2; the refusal path stays, verbatim in spirit, for files
  that lack the field.
  Done when: pytest covers both paths; building L2 from
  `load_reference_car()` works once M1.6 lands.
- [x] **M1.4** **DECISION**: the `C`/`E` consistency rule. `C` and `E` are
  bounded independently in the schema but jointly decide how far out the tyre
  peak sits relative to the linear-tyre reference angle; legal 0.1.0 values
  can put the peak at a slip angle no car reaches (multiple of about 21, where
  a real tyre is 1.5 to 3). Options: an SCH-04-style consistency check on the
  peak-slip multiple that warns (recommended) or rejects above about 4, or
  leave it to the fitter in P2. Documented in `tyre.hpp` and asserted in the
  C++ tests either way.
  Done when: decided, recorded, and if a rule is added it has tests either
  side of the threshold.
  Decided 2026-08-12: a warning above a peak-slip multiple of 4, in
  `rules.check_tyre_plausibility`, with tests either side (about 3.8 quiet,
  about 4.4 warns); recorded in ADR-0030. The C++ tests already pin the
  reference pair inside 1.5 to 3.
- [x] **M1.5** Make `mf_lite.B` derived-and-checked at the loader, as
  ADR-0023 promised: `B` is computed from cornering stiffness in
  `make_mf_lite` and never consumed from the file, so a file whose `B`
  disagrees with the derived value must be reported, not silently ignored.
  Verify what the L2 assembly slice already did here and implement whatever is
  missing.
  Done when: a tyre file with an inconsistent `B` produces a named warning or
  error, with a test.
- [x] **M1.6** Give the reference car's tyre file a `c_kappa` value,
  labelled `provisional` like everything else it carries, and keep the loader
  printing the label.
  Done when: `load_reference_car().params_for_tier(Tier.L2_DoubleTrack)`
  returns params and `summary()` still leads with PROVISIONAL.
- [x] **M1.7** Update the README (the "Early days" paragraph and anywhere
  else claiming L2 cannot be built from a tyre file) and the CHANGELOG.
  Done when: no stale claim remains; README snippets still run as written.

## M2. Drivetrain and actuators (slice 6)

Status: done. Size: large (weeks). Hash impact: **every L2 row moves;
L0 and L1 rows must not** (the state layout already carries `omega_w`,
`steer`, `steer_rate`, `soc`, `pack_v`; only L2's values change). Record the
L2 rows **once, at the end of the slice**, not per feature.

Goal: the four deliberate gaps in minimal L2, in one slice. ESC with
torque-speed curve, current limit and regen with its own limit (no brake bias
in the 1/10 schema); battery internal-resistance sag and state-of-charge decay
affecting available torque; steering servo with slew-rate limit and
second-order lag, exposing commanded versus achieved angle; spool, open and
preloaded-LSD differentials with 2WD and 4WD, replacing L2's equal drive
split.

- [x] **M2.1** Design pass first, and it needs an ADR: how the differential
  models coexist with ADR-0027 (exactly two load/force passes, no convergence
  loop, no wheel rotational state; slip ratio inverted quasi-statically from
  delivered force). A spool constrains left and right wheel speeds, an LSD
  transfers torque on a speed difference, and neither speed exists as state.
  Either express the torque-split rules quasi-statically within the two-pass
  structure, or supersede ADR-0027 explicitly. Do not add an iterative solver;
  nondeterminism hides in convergence loops and the pass count is in the
  trajectory.
  Done when: an ADR records the chosen representation and its costs.
- [x] **M2.2** Steering servo (slew limit, second-order lag). `steer` and
  `steer_rate` become integrated state at L2; diagnostics expose commanded
  versus achieved.
  Done when: analytical cases (step response rise, slew saturation) and an
  invariant case (achieved never leads commanded) pass.
- [x] **M2.3** ESC: torque-speed curve, current limit, reverse/regen braking
  with its own limit.
  Done when: analytical cases against the curve, saturation flags in
  diagnostics, and a conventions case for sign of regen torque pass.
- [x] **M2.4** Battery: internal-resistance sag and state-of-charge decay
  reducing available torque. `soc` and `pack_v` become integrated at L2.
  Done when: sag under load and monotone SoC decay are asserted; a full-SoC
  run reproduces the no-battery torque within tolerance.
- [x] **M2.5** Differential and drive layout (spool, open, preloaded LSD;
  2WD, 4WD), replacing the equal drive split. Keep the measured insight that
  motivated equal split: a load-proportional split puts extra thrust on the
  loaded outer wheels, which is a spurious yaw moment turning the car in
  (2 percent of steady-state radius at 0.36 g on the reference car). The open
  differential should reproduce the behaviour that measurement demanded;
  assert L1/L2 radius agreement at low lateral acceleration survives.
  Done when: per-type analytical cases pass and the cross-tier low-ay
  agreement holds for the open differential default.
- [x] **M2.6** Update `_STATE_REPRESENTED_FROM` in
  `src/bindings/slipx/slipx/sinks/recording.py`: the `steer_rate`, `soc` and
  `pack_v` rows currently read "L3" meaning "no tier that exists", and a
  test asserts they arrive absent from sink output. Landing this slice makes
  that test fail **by design**; move the rows to L2 and update the test
  deliberately, so no battery reading ships that nobody modelled.
  Done when: sink tests assert those quantities now arrive present at L2 and
  absent below it.
- [x] **M2.7** Schema and loader wiring for every new parameter (fields cut
  in M1.1's 0.2.0 audit), bindings for the new `VehicleParams` members, units
  and sign conventions in every doc comment. No silent defaulting: a file
  missing a needed field gets a refusal that names it.
  Done when: L2 with each drivetrain type is buildable from a car file and
  the dependency lint, licence scan and version check pass.
- [x] **M2.8** Rerecord the L2 reference rows once, at slice end, measured
  separately under GCC 11, GCC 13 and Clang 18 (which must agree). CHANGELOG
  section with old and new values side by side; reason in the commit message;
  TSV header note. Assert L0 and L1 rows are byte-identical to before.
  Done when: `tools/check_conformance.py` passes and the CHANGELOG carries
  the table.
- [x] **M2.9** Tutorial articles the index already owes: differentials, ESC
  and battery behaviour, actuator lag. Follow the brief in full (concepts, not
  SlipX; worked numbers; British spelling; no em dashes; figures via
  `make_figures.py` and render-checked).
  Done when: articles exist, are indexed, and figures render correctly in
  headless Chrome.
- [x] **M2.10** Mutation pass over the slice: try targeted mutations against
  the new code (sign flips, dropped limits, swapped axles); every escape
  produces a new test before the slice is called done.
  Done when: the mutations tried and their outcomes are listed in the commit
  message or ADR, per project convention.

## M3. `slipx.viz`: the SVG sink

Status: in-progress. Size: medium (days to a week). Hash impact: none, unless
M3.1 decides to change the reference car (then **all rows move, once**).

Goal: the run viewer as one more implementation of the sink protocol
(ADR-0024, ADR-0028). A self-contained animated SVG, Python standard library
only, no display server, no GPU, no window, written to a file. Not
special-cased in the sink code.

- [x] **M3.1** **DECISION**, before regenerating any figure: the reference
  car's provisional `c_alpha` describes a very soft tyre (peak near 24 degrees
  of slip angle where the tutorial figures show about 7). Regenerating figures
  from `slipx_core` makes the disagreement visible. Either change
  `reference_params()` once, deliberately, with a full 12-row hash rerecord
  and CHANGELOG table, or keep the numbers and label the disagreement. Both
  parameter sets are provisional, so neither is wrong; shipping the
  inconsistency silently is.
  Done when: decided and recorded; if changed, the rerecord follows M2.8's
  discipline.
  Decided 2026-08-12 by the user: change it. `c_alpha` is multiplied by 3.5
  (420 front, 455 rear per axle; 210 per tyre in the car file), which puts the
  tyre peak at 6.9 degrees where it was 24.3. Recorded as ADR-0032. The twelve
  L1 and L2 rows were rerecorded under all three compilers; the six L0 rows do
  not move and were re-measured to confirm it. `mu_y0`, the shape factors and
  `c_kappa` are untouched.
- [ ] **M3.2** The SVG writer as a `RunSink` implementation over the shared
  `Recording`: trajectory, per-wheel traces from diagnostics, SMIL animation.
  A file, never a window.
  Done when: it registers like the MCAP and Rerun sinks, byte-identical
  output run to run, and imports nothing outside the standard library.
- [ ] **M3.3** Draw the provenance label and the trajectory hash **into** the
  image, not beside it; a rendered run gets pasted into slides and must carry
  its own label.
  Done when: a test parses the SVG and finds both.
- [ ] **M3.4** Draw nothing that is not in the recorded state, diagnostics or
  manifest. In particular no track geometry until `slipx_scene` exists; a
  drawn kerb asserts a track that does not exist.
  Done when: a review of the drawn elements against the recording columns is
  asserted in a test (element inventory versus recorded fields).
- [ ] **M3.5** The NaN rule, same as every sink: a quantity the tier cannot
  represent arrives absent (no line, no legend entry), never plotted as zero.
  Done when: the sink has the same absence test the MCAP and Rerun sinks
  carry, run at L0/L1 where per-wheel fields are NaN.
- [ ] **M3.6** Theme awareness: embed a `prefers-color-scheme` stylesheet and
  a background card, the same pattern `docs/racing/assets/make_figures.py`
  uses, so the file is legible on light and dark pages from one render.
  Done when: both themes render legibly in headless Chrome screenshots.
- [ ] **M3.7** Render the cross-tier crossover (L1 versus L2 agreement
  falling off with lateral acceleration) as a released artefact generated from
  `slipx_core`, not from a local model.
  Done when: the artefact is generated by a checked-in script and referenced
  from the docs.
- [ ] **M3.8** Regenerate `docs/racing/assets` and the README banner from
  `slipx_core`, and delete the local illustrative tyre models in
  `make_figures.py` and `docs/assets/make_banner.py` when doing so. Check
  every figure by rendering it, not by reading the SVG. Mind the two paid-for
  traps: Unicode has no Latin subscript y or z (use the `sub()` helper), and
  geometry in a diagram should be asserted in code.
  Done when: no local tyre model remains, figures render correctly, and the
  banner regenerates from the library.

## M4. Release 0.2.0

Status: not-started. Size: small (days). Blocked by: M1, M2, M3.

Goal: the first release where the double-track tier is complete, reachable
from a car file and visible. Everything below is release engineering.

- [ ] **M4.1** The libm ADR, before the next release: the reference-hash key
  (architecture, compiler, build type, tier, integrator) has no C library
  column, and `"within_build": "bit-identical"` in the manifest is imprecise
  for a redistributed dynamically linked wheel, where libm varies by host
  (measured: one wheel, two hashes on glibc 2.28 versus 2.39). Fixing the key
  or the wording is an architectural change and wants an ADR superseding
  ADR-0008, not a quiet edit.
  Done when: the ADR is accepted and whatever it decides is implemented.
- [ ] **M4.2** The per-minor-release documentation set: reference
  documentation, a written tyre-model derivation, and three runnable examples.
  Audit what exists (the racing series is a tutorial, not the derivation) and
  fill the gaps. Examples must be executed, not proofread.
  Done when: all three exist and the examples run from a clean venv install.
- [ ] **M4.3** README audit: every claim replaced rather than appended
  (tier table status, the "not yet built" SVG line, test counts, the
  tyre-file reachability claim). Both Python snippets executed as written.
  Absolute URLs only; the README is the PyPI long description.
  Done when: every claim in the README is currently true.
- [ ] **M4.4** Version bump in the four coupled places (`pyproject.toml`,
  `CMakeLists.txt`, `slipx/version.hpp` twice over, `slipx/version.py`);
  CMake carries the numeric part only. CHANGELOG cut for the release,
  including the hash-movement tables from M2.8 (and M3.1 if taken).
  Done when: `tools/version_check.py` passes.
- [ ] **M4.5** Release rehearsal then release: dispatch the workflow to
  TestPyPI, verify the render and a cold install; then tag, and dispatch to
  PyPI with publish enabled, from the tag (the workflow refuses otherwise).
  Trusted Publishing, no tokens. Expect the full wheel matrix (five platforms,
  CPython 3.9 to 3.13) plus the sdist. A published version is never reused;
  a broken upload means the next number.
  Done when: `pip install slipx==<version>` works from a clean venv and the
  PyPI page renders correctly.

## M5. The rest of P1: sensing, track, ROS 2, reference stack

Status: not-started. Size: extra large (many weeks). Release: 0.3.0 at the
end. Hash impact: none expected in `slipx_core`; sensors and scene live above
it. Determinism constraints extend to every new layer: seeded per-agent RNG
only, no wall clock, fixed iteration order.

Goal: the remaining P1 deliverables, so a RoboRacer team can run their
existing stack against SlipX. New C++ components (`slipx_scene`,
`slipx_sense`, `slipx_ros`) go into the currently empty placeholder
directories and must respect the downward dependency order
(`tools/dep_lint.py` enforces it).

- [ ] **M5.1** `slipx_scene`, first slice: load a centreline CSV
  (`s, x, y, w_left, w_right, banking, mu`), TUM racetrack database
  compatible, with per-segment surface identifier resolving the tyre
  `(compound, surface)` pair. Refuse to run when the track's declared surface
  has no matching tyre entry.
  Done when: a real track loads, and surface mismatch produces a named
  refusal.
- [ ] **M5.2** **DECISION**: the first track. Prefer one with a published
  real-world counterpart so P2 fits can be cross-checked; Porto or an
  equivalent from the public F1TENTH map set.
  Done when: one track ships with provenance for its geometry.
- [ ] **M5.3** Lap counting and track-limit detection with configurable
  tolerance, per agent.
  Done when: analytical cases (crossing geometry) and property cases
  (direction, multiple laps) pass.
- [ ] **M5.4** `slipx_sense`, 2D LiDAR: every ray individually timestamped,
  emitter pose interpolated at that ray's timestamp so motion distortion
  emerges from the physics rather than being bolted on; per-sensor latency
  (constant plus jitter distribution) configurable independently of rate;
  dropouts and range-dependent noise, with material-dependent dropout
  probability behind a parameter.
  Done when: a fast-spinning agent produces a measurably distorted scan and a
  stationary one does not; latency and noise are seeded and reproducible.
- [ ] **M5.5** IMU model (bias random walk, scale error, noise density) and
  wheel-encoder odometry derived from simulated encoder counts, degrading
  correctly under wheel slip (this consumes L2's slip ratios).
  Done when: encoder odometry diverges from ground truth exactly when slip is
  present, asserted against the diagnostics.
- [ ] **M5.6** `slipx_sim` additions: validation mode (soft real-time with
  latency and jitter enabled) alongside deterministic mode; snapshot and
  restore of full simulation state (the state is already a memcpy by design;
  expose and test it).
  Done when: a snapshot taken mid-run restores bit-identically in
  deterministic mode.
- [ ] **M5.7** `slipx_ros`: the per-agent topic set under `/car_N/`
  (`drive` in, `scan`, `imu`, `odom`, ground truth out), matching F1TENTH
  topic names and QoS where conventions exist; `/clock` published and correct
  under `use_sim_time`; ground truth in a clearly namespaced subtree,
  disableable at launch, with the disabled state recorded in the run manifest;
  target ROS 2 Jazzy plus the current rolling LTS and publish a support
  matrix.
  Done when: an existing F1TENTH stack connects with a remap file and no code
  change.
- [ ] **M5.8** Reference stack: wall-follower plus pure pursuit, as examples
  that validate the simulator, explicitly not a competitive stack.
  Done when: both run a lap on the shipped track headlessly in CI.
- [ ] **M5.9** Performance benchmarks, measured and published: L2
  single-agent step under 5 microseconds per core; one agent with L2 and 2D
  LiDAR at 100 times real time or better headless; 20 agents with L2 and 2D
  LiDAR at 10 times real time or better. Tracked per commit, not measured
  once.
  Done when: a benchmark suite runs in CI and the numbers are published in
  the docs with the hardware named.
- [ ] **M5.10** P1 exit gate (external fact): a RoboRacer team runs their
  existing stack against SlipX with a one-file topic remap and reports that a
  tuning change made in sim held on their car. Track the outreach as work:
  identify teams, offer support, collect the report.
  Done when: the report exists and can be cited.
- [ ] **M5.11** Release 0.3.0, following the M4 release checklist (docs set,
  README audit, version bump, TestPyPI rehearsal, tag, publish).
  Done when: published and installable.

## M6. P2: identification and the registry

Status: not-started. Size: extra large (many weeks). Release: 0.4.0 at the
end. This phase is the differentiator; the SRS is explicit that it must not
slip behind the racing features, because parameter sets compound and racing
features do not.

- [ ] **M6.1** The manoeuvre library: skidpad, step steer, ramp steer,
  straight-line acceleration, coastdown, circle-to-slip. Each with a written
  procedure, a space requirement and a safety note, and each executable with
  only the sensors on a stock competition car (encoders, IMU, LiDAR pose).
  No dyno, no tyre rig, no force platform, ever.
  Done when: the six procedures ship as documentation with worked parameter
  coverage (which manoeuvre identifies which parameter).
- [ ] **M6.2** The synthetic self-test **first**: generate data from known
  parameters through the forward model, fit, and assert recovery within
  tolerance. It is the only way to test the fitter without hardware, so it is
  the first thing built, not the last.
  Done when: round-trip recovery of every MF-lite parameter is asserted in
  CI.
- [ ] **M6.3** `slipx_id`, the fitter: ingest rosbag2, emit `dynamics.yaml`
  with per-parameter residuals and confidence intervals. Refuse to emit a
  parameter set without a populated provenance block; warn when identified
  parameters fall outside physically plausible bounds for the declared scale.
  Done when: the synthetic self-test passes end to end through the real bag
  path, and both refusal and warning have tests.
- [ ] **M6.4** The validation report: replay measured control inputs through
  the fitted model and plot divergence in yaw rate, lateral acceleration and
  speed, with a single headline fit metric. Reuses the recording and sink
  machinery from M3 rather than growing its own plotting.
  Done when: a report generates from the synthetic self-test's data and reads
  correctly.
- [ ] **M6.5** Schema: the provenance block (source, method, date,
  contributor, residuals) required on any parameter set submitted to the
  registry. A schema minor bump, versioned independently of the core as
  always.
  Done when: schema validation enforces it with tests.
- [ ] **M6.6** **DECISION**, then build: registry hosting and curation. A git
  repository with PR review is the cheapest credible option; decide the
  acceptance bar before the first submission arrives. Contribution must be a
  by-product of using `slipx_id`, one command from bag to submitted PR, not a
  separate act of altruism.
  Done when: `slipx_registry` exists with a contribution workflow and the
  acceptance bar written down.
- [ ] **M6.7** **DECISION**: buy a chassis. Materially changes the honesty of
  the launch claim; the SRS recommends yes. The first credible in-house
  parameter set is worth more than its cost.
  Done when: decided; if yes, one fitted set with a validation report exists
  and is labelled `identified`, not `provisional`.
- [ ] **M6.8** P2 exit gate (external fact): three parameter sets in the
  registry contributed by people who are not the maintainer, each with an
  attached validation report. Seed by co-authoring with two friendly teams.
  After this gate, and only after it, the word "validated" becomes available
  for those sets; the project-wide claim stays "physically structured and
  identifiable" and the labels do the talking.
  Done when: three external sets with reports are merged.
- [ ] **M6.9** Tutorial articles as the work surfaces them (system
  identification, residuals and confidence, the validation report), per the
  standing brief.
- [ ] **M6.10** Release 0.4.0, per the release checklist.

## M7. P3: racing

Status: not-started. Size: extra large (many weeks). Release: 0.5.0 at the
end. Hash impact: contact and rollover enter `slipx_core` numerical paths;
expect deliberate hash movements, recorded per the standing discipline.

- [ ] **M7.1** Rollover detection as a discrete event that halts the agent
  (DNF); no flight or landing simulation. The static rollover threshold
  already exists in `load_transfer.hpp`; this makes it an event.
  Done when: a CoG sweep above the threshold produces the event
  deterministically and the event carries the cause.
- [ ] **M7.2** Agent-to-agent contact as a planar impulse with restitution,
  Coulomb friction and resulting yaw moment. Plausible and deterministic, not
  fitted to data, and the docs keep saying so.
  Done when: momentum conservation and mirror symmetry hold in the invariant
  suite; determinism holds across runs.
- [ ] **M7.3** Lockstep barrier protocol with per-agent acknowledgement and a
  configurable timeout policy (freeze, coast or DNF), so one hung agent cannot
  hang a race.
  Done when: a deliberately hung agent exercises each policy in a test.
- [ ] **M7.4** Prebuilt static scene BVH with a per-step dynamic agent
  overlay refit only.
  Done when: broadphase results match a brute-force reference in tests.
- [ ] **M7.5** Race control per the published RoboRacer ruleset: time trial,
  obstacle avoidance test, head-to-head, grid and rolling starts; contact
  attribution from relative geometry and closing velocity with the ruleset's
  penalty logic; the ruleset repository tracked as a versioned dependency with
  the implemented revision stated.
  Done when: scenario tests cover each procedure and the build states its
  ruleset revision.
- [ ] **M7.6** The structured event stream: every race-control outcome as
  timestamped, machine-readable events, encoded as MCAP so the event stream
  and the run sinks are one format, not two. Any leaderboard, report or CI job
  consumes the event stream and nothing else.
  Done when: a full race replays from its event stream alone.
- [ ] **M7.7** Per-agent sensor configuration (cheap opponents run 2D or no
  sensors); `race_sync` client library implementing the barrier, linkable
  into a student control node with under ten lines of change; RMW default
  benchmarked and decided (`rmw_zenoh` versus Fast-DDS discovery server, at 6
  and 20 agents) with the multicast failure mode documented; multi-host agents
  with the simulator as sync authority.
  Done when: each has tests or, for the RMW decision, a recorded benchmark.
- [ ] **M7.8** CI leaderboard harness with seeded scenario batches.
  Done when: a leaderboard run is reproducible from its manifest and seeds.
- [ ] **M7.9** **DECISION**: governance. If a competition adopts SlipX,
  ownership of the ruleset implementation becomes contested; decide before,
  not after.
  Done when: recorded (ADR or governance doc).
- [ ] **M7.10** P3 exit gate (external fact): one course or one competition
  runs an evaluation on SlipX.
  Done when: it has happened and can be cited.
- [ ] **M7.11** Release 0.5.0, per the release checklist.

## M8. 1.0.0

Status: not-started. Size: medium. Blocked by: M5.10, M6.8, M7.10 (the three
external gates).

- [ ] **M8.1** Ratify the definition of 1.0.0 at the top of this file as an
  ADR (it is a release-policy decision, and this roadmap only proposes it).
  Re-plan P4 and P5 at the same time, as the SRS requires at the P3 gate;
  they are directions, not commitments.
  Done when: the ADR is accepted.
- [ ] **M8.2** Public API audit and freeze on `slipx_core`: enumerate the
  public surface (headers, `VehicleParams`, `VehicleState`,
  `StepDiagnostics`, the tier and integrator enums), remove or regularise
  anything not meant to be public, and document what a breaking change means
  from 1.0.0 on. `slipx_schema` keeps its own independent version; 1.0.0 of
  the distribution asserts nothing about the schema version.
  Done when: the API reference is complete and the semver policy is written
  down.
- [ ] **M8.3** The doc coverage gate: a doc build that fails on any
  undocumented public parameter (every one carries units and a sign
  convention). Verify whether it exists in CI; build it if not.
  Done when: CI fails on an undocumented public parameter, demonstrated by
  mutation.
- [ ] **M8.4** The cross-architecture tolerance suite: agreement within a
  stated tolerance across x86-64 and aarch64, asserted in a CI matrix, with
  the limitation documented rather than glossed (bit-identity across
  platforms is explicitly not promised).
  Done when: the cross-arch job runs in CI with the tolerance stated.
- [ ] **M8.5** Claim audit across README, docs and tooling output: every
  parameter set labelled `measured`, `identified` or `provisional` with the
  label printed by tooling; "validated" appears only where a validation
  report backs it; the four-layer verification story updated with the
  empirical layer now real.
  Done when: a full read of README, CHANGELOG and tool output finds no claim
  ahead of the evidence.
- [ ] **M8.6** Release engineering: the per-minor docs set (reference docs,
  tyre derivation, three runnable examples) current; migration notes from
  0.x; deprecation sweep; version bump in the four places; TestPyPI
  rehearsal; tag; publish to PyPI via Trusted Publishing with the approval
  gate on the `pypi` environment.
  Done when: `pip install slipx` resolves to 1.0.0 from a clean venv, the
  PyPI page renders, and the conformance script runs out of the wheel.

---

## Standing obligations (every milestone)

- **ADRs are part of the change, not a follow-up.** Any architectural
  decision lands with a numbered record from `docs/adr/template.md`. Records
  are immutable: supersede, never edit.
- **Hash discipline.** A changed reference hash is a release event: CHANGELOG
  section with old and new values side by side, reason in the commit message,
  note in the TSV header, measured separately under GCC 11, GCC 13 and
  Clang 18. Move rows once per slice, not per feature. Rows that should not
  move get asserted, not assumed.
- **Mutation passes.** After each slice, try targeted mutations against the
  new code; every escape produces a new test before the slice is done, and
  the mutations tried are listed in the commit message or ADR.
- **The tutorial series is a standing task.** When development surfaces a
  concept a newcomer would need, write the article without being asked,
  following the brief in `docs/racing` (concepts not SlipX, worked numbers,
  one concept per document, British spelling, no em dashes, figures
  render-checked in headless Chrome).
- **Claim discipline.** "Physically structured and identifiable", never
  "validated", until a validation report exists for the specific set.
  Provenance labels are printed by tooling, not just documented.
- **Git flow.** Conventional Commits, short messages, no co-author line,
  no pushing (the user pushes). Each slice lands as its own commit with its
  tests.
- **Code comments stand alone.** Do not cite `SRS.md` or requirement IDs
  (`CORE-05`, `SINK-01`) in code comments; the spec is not published, so a
  reader of the code cannot follow the reference. Put the reasoning in the
  comment itself or in an ADR.
- **No em dashes anywhere.** British spelling in all prose.
- **Two paid-for traps in the Python layer**: `sim.state(i)` and
  `sim.diagnostics(i)` return references the next step overwrites (copy
  before keeping), and a test that mutates a module and restores it must
  clear `__pycache__`.
- **matplotlib is unusable in this environment** (broken numpy); every figure
  is hand-rolled SVG from `docs/racing/assets/make_figures.py`.

## Guardrails: the decisions most likely to be broken by a reasonable change

The ADRs are the argument; this is only the index. Read the record before
working around any of these.

| Rule | Record |
|---|---|
| `slipx_core` depends on the C++ standard library and nothing else | ADR-0002 |
| Dependencies point strictly downward; bindings sit above what they bind | ADR-0003 |
| `step` is `const` and stateless; no unordered iteration in numerics | ADR-0004 |
| An unimplemented tier throws; never fall back to a simpler tier | ADR-0005 |
| Diagnostics use NaN, never zero, for what a tier cannot represent | ADR-0006 |
| Reference hashes are keyed by build; pinned runners | ADR-0008 |
| No `if (scale == ...)` branching, ever | ADR-0012 |
| Core and schema versions are independent and never compared | ADR-0015 |
| A released wheel asserts nothing about its own trajectory hash | ADR-0020 |
| The viewer writes a file, never opens a window, draws no invented track | ADR-0024 |
| The loader refuses rather than defaulting a missing parameter | ADR-0025 |
| Relaxation lags the slip angle, not the force | ADR-0026 |
| L2: two passes, no convergence loop, no wheel rotational state | ADR-0027 |
| Runs go to sinks; viewers are external; NaN arrives absent, never zero | ADR-0028 |

## Verification commands

Run before marking any task done:

```
cmake -S . -B build -DSLIPX_BUILD_PYTHON=ON
cmake --build build -j
ctest --test-dir build --output-on-failure
python3 -m pytest
```

The sink tests skip without extras; the interesting pytest run installs
`mcap rerun-sdk pyyaml jsonschema pytest` into a fresh 3.12 venv and runs from
the repository root.

The five CI checks a test suite does not cover:

```
python3 tools/dep_lint.py               # dependency direction
python3 tools/licence_scan.py           # Apache-2.0, no copyleft, extras table
python3 tools/version_check.py          # the version, in four places
python3 tools/check_conformance.py      # trajectory hashes per build
cmake -S . -B build-core -DSLIPX_CORE_ONLY=ON   # the core builds alone
```
