# SlipX release roadmap: 0.1.0a1 to 1.0.0

Last updated: 2026-08-20. The session that started at M7.8 finished every
buildable task in M7: the leaderboard harness's tests and mutation pass
(M7.8), the sensor rig (ADR-0047), sensors.yaml wired through schema
0.5.0 (ADR-0048), the racing world composed in slipx_sim (ADR-0049), the
ROS 2 bridge (ADR-0050, M5.7 built with its external exit condition
still open), lockstep racing through race_sync (ADR-0051), and the RMW
benchmark with the race-day default decided (ADR-0052), which ticked
M7.7. What remains in M5 through M7 is external or deliberate: the three
exit gates (M5.10, M6.8, M7.10, the user's outreach), M6.7 ("not now"),
M7.9 (deferred until adoption), and the 0.3.0/0.4.0/0.5.0 releases, cut
together with the user per the 2026-08-19 decision. M8 is next and is
blocked by the gates. In the session before, M7.1 through M7.6 were built (rollover
events, contact, the barrier, the broadphase with the 20-agent target
renegotiated to over 7x, race control on the pinned ruleset, the MCAP
event stream; ADR-0042 to ADR-0046) and M5.9 and M5.12 closed.
Earlier the same day, four user decisions
were recorded: the 20-agent performance target waits for M7.4's broadphase
before any renegotiation (since executed, see M5.12); the WSL environment
gets ROS 2 and the extra compilers installed by the user; the 0.3.0, 0.4.0
and 0.5.0 releases are cut together after M7 rather than at each milestone
end; the registry is a separate repository whose content is staged in this
tree. Published version: `0.2.0` (PyPI, tag `v0.2.0`).
Current tree: L2 is complete (drivetrain, battery, servo and differentials),
the sink layer writes MCAP, Rerun and SVG, and every figure is generated from
`slipx_core`. M4 shipped that as `0.2.0`; M5 is in progress and is the rest of
P1.

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

Status: done. Size: medium (days to a week). Hash impact: none, unless
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
- [x] **M3.2** The SVG writer as a `RunSink` implementation over the shared
  `Recording`: trajectory, per-wheel traces from diagnostics, SMIL animation.
  A file, never a window.
  Done when: it registers like the MCAP and Rerun sinks, byte-identical
  output run to run, and imports nothing outside the standard library.
- [x] **M3.3** Draw the provenance label and the trajectory hash **into** the
  image, not beside it; a rendered run gets pasted into slides and must carry
  its own label.
  Done when: a test parses the SVG and finds both.
- [x] **M3.4** Draw nothing that is not in the recorded state, diagnostics or
  manifest. In particular no track geometry until `slipx_scene` exists; a
  drawn kerb asserts a track that does not exist.
  Done when: a review of the drawn elements against the recording columns is
  asserted in a test (element inventory versus recorded fields).
- [x] **M3.5** The NaN rule, same as every sink: a quantity the tier cannot
  represent arrives absent (no line, no legend entry), never plotted as zero.
  Done when: the sink has the same absence test the MCAP and Rerun sinks
  carry, run at L0/L1 where per-wheel fields are NaN.
- [x] **M3.6** Theme awareness: embed a `prefers-color-scheme` stylesheet and
  a background card, the same pattern `docs/racing/assets/make_figures.py`
  uses, so the file is legible on light and dark pages from one render.
  Done when: both themes render legibly in headless Chrome screenshots.
- [x] **M3.7** Render the cross-tier crossover (L1 versus L2 agreement
  falling off with lateral acceleration) as a released artefact generated from
  `slipx_core`, not from a local model.
  Done when: the artefact is generated by a checked-in script and referenced
  from the docs.
- [x] **M3.8** Regenerate `docs/racing/assets` and the README banner from
  `slipx_core`, and delete the local illustrative tyre models in
  `make_figures.py` and `docs/assets/make_banner.py` when doing so. Check
  every figure by rendering it, not by reading the SVG. Mind the two paid-for
  traps: Unicode has no Latin subscript y or z (use the `sub()` helper), and
  geometry in a diagram should be asserted in code.
  Done when: no local tyre model remains, figures render correctly, and the
  banner regenerates from the library.

## M4. Release 0.2.0

Status: done. Size: small (days). Blocked by: M1, M2, M3.

Goal: the first release where the double-track tier is complete, reachable
from a car file and visible. Everything below is release engineering.

- [x] **M4.1** The libm ADR, before the next release: the reference-hash key
  (architecture, compiler, build type, tier, integrator) has no C library
  column, and `"within_build": "bit-identical"` in the manifest is imprecise
  for a redistributed dynamically linked wheel, where libm varies by host
  (measured: one wheel, two hashes on glibc 2.28 versus 2.39). Fixing the key
  or the wording is an architectural change and wants an ADR superseding
  ADR-0008, not a quiet edit.
  Done when: the ADR is accepted and whatever it decides is implemented.
  Decided 2026-08-13 by the user: key on it, recorded as ADR-0033 superseding
  ADR-0008. The TSV gains a `libc` column, the manifest gains `libc_id` and
  `libc_version` read at run time and folded into the configuration digest,
  and `within_build` narrows to "the same binary on the same C library". The
  eighteen rows were re-keyed to `glibc-2.39`, not re-measured: **no hash
  moved**, re-verified under GCC 11, GCC 13 and Clang 18.
- [x] **M4.2** The per-minor-release documentation set: reference
  documentation, a written tyre-model derivation, and three runnable examples.
  Audit what exists (the racing series is a tutorial, not the derivation) and
  fill the gaps. Examples must be executed, not proofread.
  Done when: all three exist and the examples run from a clean venv install.
  Done 2026-08-13: `docs/reference/` (conventions, `VehicleParams`, state and
  diagnostics, the Python API) and `docs/reference/tyre-model.md`, the MF-lite
  derivation with worked reference-car numbers. Three examples in `examples/`,
  executed by `test_examples.py` in the tree and by hand from a clean venv
  install of the wheel, run outside the checkout with `PYTHONPATH` cleared.
- [x] **M4.3** README audit: every claim replaced rather than appended
  (tier table status, the "not yet built" SVG line, test counts, the
  tyre-file reachability claim). Both Python snippets executed as written.
  Absolute URLs only; the README is the PyPI long description.
  Done when: every claim in the README is currently true.
  Done 2026-08-13: the build key gained its C library column, the cross-tier
  number was wrong by one sample of the figure's sweep (within 1% to 0.82 g,
  crossing by 0.85 g, and the figure's steer grid was refined so the figure,
  the CHANGELOG, article 3 and the README agree), test counts, the inverted
  banner claim, the URDF the reference car does not carry, and links to
  `docs/reference` and `examples`. Both snippets executed from a clean venv
  install of the wheel, outside the checkout.
- [x] **M4.4** Version bump in the four coupled places (`pyproject.toml`,
  `CMakeLists.txt`, `slipx/version.hpp` twice over, `slipx/version.py`);
  CMake carries the numeric part only. CHANGELOG cut for the release,
  including the hash-movement tables from M2.8 (and M3.1 if taken).
  Done when: `tools/version_check.py` passes.
  Done 2026-08-13: `0.2.0` in all four places, CMake carrying `0.2.0` as the
  numeric part; the CHANGELOG cut keeps all three hash-movement tables and
  says at the top that every hash differs from `0.1.0a1`'s.
- [x] **M4.5** Release rehearsal then release: dispatch the workflow to
  TestPyPI, verify the render and a cold install; then tag, and dispatch to
  PyPI with publish enabled, from the tag (the workflow refuses otherwise).
  Trusted Publishing, no tokens. Expect the full wheel matrix (five platforms,
  CPython 3.9 to 3.13) plus the sdist. A published version is never reused;
  a broken upload means the next number.
  Done when: `pip install slipx==<version>` works from a clean venv and the
  PyPI page renders correctly.
  Ready 2026-08-13, awaiting the user: this is the one M4 task an agent cannot
  do, because it dispatches workflows, tags and publishes. Rehearsed locally
  as far as it goes: the wheel builds from this tree through
  scikit-build-core, installs into a clean venv, and outside the checkout with
  `PYTHONPATH` cleared the exit gate passes against the pinned
  `0d8f69a1e3b58038` and `slipx-conformance` prints the same hash.
  `tools/version_check.py --expect v0.2.0` passes, so the tag the workflow
  wants is `v0.2.0`.

## M5. The rest of P1: sensing, track, ROS 2, reference stack

Status: in-progress; M5.1 to M5.6, M5.8, M5.9 and M5.12 done (the 20-agent
performance target renegotiated to over 7x on 2026-08-19 after M7.4's
broadphase was measured, per the standing decision), M5.7 unblocked late
on 2026-08-19 (the ROS 2 install completed) and now buildable, M5.10 the
external gate, M5.11 deferred to the end of M7. Size:
extra large (many weeks). Release: 0.3.0 at the end. Hash impact: none
expected in `slipx_core`, and none observed; sensors and scene live above
it. Determinism constraints extend to every new layer: seeded per-agent RNG
only, no wall clock, fixed iteration order.

Goal: the remaining P1 deliverables, so a RoboRacer team can run their
existing stack against SlipX. New C++ components (`slipx_scene`,
`slipx_sense`, `slipx_ros`) go into the currently empty placeholder
directories and must respect the downward dependency order
(`tools/dep_lint.py` enforces it).

- [x] **M5.1** `slipx_scene`, first slice: load a centreline CSV
  (`s, x, y, w_left, w_right, banking, mu`), TUM racetrack database
  compatible, with per-segment surface identifier resolving the tyre
  `(compound, surface)` pair. Refuse to run when the track's declared surface
  has no matching tyre entry.
  Done when: a real track loads, and surface mismatch produces a named
  refusal.
  Design settled 2026-08-15 as ADR-0034 (proposed): a track is a four-column
  TUM centreline CSV plus a manifest beside it, the manifest declares a
  surface identifier and never a friction number, arc length is derived and
  banking is refused rather than defaulted, and the surface-to-tyre refusal
  lives in `slipx_scene` against a caller-supplied list of `(compound,
  surface)` pairs so the component stays below `slipx_schema`.
  Built 2026-08-15: `src/world/slipx_scene` with `Centreline` (the parser,
  derived arc length) and `Track` (the manifest checks and the surface to
  tyre pairing), 30 tests, all 272 in the suite green, the five CI checks
  clean and no conformance hash moved. Mutation pass: 21 tried, 19 caught.
  Two escapes, both recorded rather than papered over. Deleting the per-field
  finiteness check escapes because libstdc++'s number grammar rejects `nan`
  and `inf` before the check can see them, so the branch is a guard for
  libc++ that no test on this library can reach; probing it found a real hole
  and closed it, since fields can all be finite while the derived arc length
  overflows, and that case now has a test. Replacing `sqrt` of the sum of
  squares with `hypot` escapes because the two agree within one build, which
  is the whole point: the reason for `sqrt` is that it is correctly rounded
  and `hypot` is not, so the difference only appears across C libraries
  (ADR-0033) and no single-build test can see it.
  Completed 2026-08-15 with the Python half: `track.schema.json` as schema
  0.3.0 (ADR-0036), `slipx_schema.load_track` and `check_surface`, the
  generated track `examples/tracks/paddock_stadium` with its generator, and
  `tools/convert_track.py`. A real track loads through the converter and the
  shipped one is asserted by both suites. Second mutation pass over the
  Python half: 12 tried, 12 caught.
- [x] **M5.2** **DECISION**: the first track. Prefer one with a published
  real-world counterpart so P2 fits can be cross-checked; Porto or an
  equivalent from the public F1TENTH map set.
  Done when: one track ships with provenance for its geometry.
  Licences established 2026-08-15, and they rule out shipping the obvious
  candidates: `f1tenth/f1tenth_racetracks`, the set that carries centrelines
  in this format, is GPL-3.0; its upstream `TUMFTM/racetrack-database` is
  LGPL-3.0; the underlying centrelines are OpenStreetMap, so ODbL and
  share-alike as a derived database. Porto itself lives in
  `f1tenth/f1tenth_simulator` as an occupancy grid with **no licence at all**,
  as does `CPS-TUWien/f1tenth_maps`, so neither is redistributable.
  `f1tenth/f1tenth_gym` is MIT but ships no Porto and no centreline. The
  copyleft ones put copyleft licence text in an Apache-2.0 repository, which
  `tools/licence_scan.py` fails on by design; the unlicensed ones grant
  nothing at all, which is the harder stop of the two.
  The choice is therefore not "which track" but "how a track reaches the
  user".
  Decided 2026-08-15 by the user, recorded as ADR-0035 (proposed): SlipX ships
  an Apache-2.0 fetch-and-convert tool that writes the source, its licence and
  the retrieval date into the track manifest, so Porto stays one command away
  and no third-party geometry enters the tree or a wheel; one track we
  generate ships alongside it, labelled as generated with no real-world
  counterpart, and it is what CI and the examples run.
- [x] **M5.3** Lap counting and track-limit detection with configurable
  tolerance, per agent.
  Done when: analytical cases (crossing geometry) and property cases
  (direction, multiple laps) pass.
  Done 2026-08-15. `projection.hpp` turns a world position into arc length
  and a signed lateral offset, closest to a segment rather than to a sample
  (up to 5 cm apart on a track sampled at 0.1 m, which is a third of a car's
  width). `LapCounter` counts progress rather than line crossings: each
  update adds the signed distance moved along the centreline, so reversing
  over the line subtracts and a car wobbling on it accumulates nothing, and
  direction falls out instead of being special-cased. Track limits carry a
  per-agent tolerance that is required rather than defaulted, report a margin
  rather than a flag, and remember an excursion after the car comes back.
  Mutation pass: 15 tried, 14 caught first time. The escape was real and was
  mine: the open-track test drove 20 m of a 30 m track, so removing the guard
  that stops an open track reporting laps changed nothing. Driving it end to
  end, where the distance equals the track length exactly, catches it.
- [x] **M5.4** `slipx_sense`, 2D LiDAR: every ray individually timestamped,
  emitter pose interpolated at that ray's timestamp so motion distortion
  emerges from the physics rather than being bolted on; per-sensor latency
  (constant plus jitter distribution) configurable independently of rate;
  dropouts and range-dependent noise, with material-dependent dropout
  probability behind a parameter.
  Done when: a fast-spinning agent produces a measurably distorted scan and a
  stationary one does not; latency and noise are seeded and reproducible.
  Done 2026-08-15. The layering had to be settled first and it wanted a
  record: `slipx_sense` and `slipx_scene` are siblings and neither includes
  the other (ADR-0037), so the LiDAR receives the world as a function from a
  ray to a distance and never learns what a track is. The seeded generator
  moved from `slipx/sim/rng.hpp` to `slipx/sense/rng.hpp`, the lowest C++
  layer above the core that needs it; no hash moved, which was checked rather
  than assumed. `tools/dep_lint.py` now enforces the sibling rule.
  Distortion is not modelled, it emerges: every ray is cast from the pose at
  its own timestamp, so nothing multiplies anything by a speed and a
  stationary car is undistorted without a special case. `slipx_scene` gained
  `Walls`, which offsets the centreline by its widths (with the mitre, or
  corners cut in by 29 per cent on a square) and intersects rays against it.
  Mutation pass: 15 tried, 12 caught first time. Three escapes, all weak
  tests rather than weak code, now covered: a measurement noise pushed past
  the maximum range, a return from inside the blind zone that noise could
  push out of it, and a closing wall on an open track, which needed a
  three-point fixture because on two points that segment is the same one
  reversed.
- [x] **M5.5** IMU model (bias random walk, scale error, noise density) and
  wheel-encoder odometry derived from simulated encoder counts, degrading
  correctly under wheel slip (this consumes L2's slip ratios).
  Done when: encoder odometry diverges from ground truth exactly when slip is
  present, asserted against the diagnostics.
  Done 2026-08-15. The IMU carries the three errors that live on different
  timescales: white noise stated as a density so sampling faster gives
  noisier samples and the same answer after averaging, a bias random walk
  that averaging does not remove, and a fixed scale error that is a parameter
  rather than a draw, because it is a property of the unit and identifiable
  from a manoeuvre with a known total heading change. Encoder odometry is
  noiseless on purpose: an encoder has a quantisation rather than a noise
  floor, and its interesting error is the slip, which is already in the wheel
  speeds. The odometry cases run a real L2 car, so the tyre model decides
  when it slips and the assertion is against the reported slip ratios rather
  than against invented wheel speeds.
  Mutation pass: 11 tried, 9 caught. One escape was a missing test for the
  accelerometer scale error. The other found a real bug: counts were floored,
  so a wheel turned backwards through 6.37 counts reported seven edges
  crossed when it had crossed six. Truncation towards zero is right on both
  sides, and the two agree for a wheel that only ever goes forwards, which is
  how it would have shipped.
- [x] **M5.6** `slipx_sim` additions: validation mode (soft real-time with
  latency and jitter enabled) alongside deterministic mode; snapshot and
  restore of full simulation state (the state is already a memcpy by design;
  expose and test it).
  Done when: a snapshot taken mid-run restores bit-identically in
  deterministic mode.
  Done 2026-08-15. The state was already a memcpy; what a snapshot has to
  carry beyond it is the bookkeeping. Each agent's running trajectory hash is
  a fold and has to be resumed with the states, or a resumed run agrees about
  every state and disagrees about the run. Each agent's generator has to
  carry its Box-Muller spare as well as its engine word, because `normal()`
  produces two values and keeps one: restoring the word alone gives a
  simulation that resumes correctly until something asks for a normal, and
  then diverges for a reason that looks like anything except a missing bool.
  Validation mode paces against a wall clock and is soft in the direction
  that matters: it sleeps when ahead and does not catch up when behind, since
  catching up means stepping faster than real time, which is the one thing a
  latency test must not do. The manifest records the mode, the mode is part
  of the configuration digest, and a validation manifest says NOT
  REPRODUCIBLE in the field where a deterministic one promises bit-identity.
  Mutation pass: 9 tried, 9 caught.
- [ ] **M5.7** `slipx_ros`: the per-agent topic set under `/car_N/`
  (`drive` in, `scan`, `imu`, `odom`, ground truth out), matching F1TENTH
  topic names and QoS where conventions exist; `/clock` published and correct
  under `use_sim_time`; ground truth in a clearly namespaced subtree,
  disableable at launch, with the disabled state recorded in the run manifest;
  target ROS 2 Jazzy plus the current rolling LTS and publish a support
  matrix.
  Done when: an existing F1TENTH stack connects with a remap file and no code
  change.
  **Blocked** 2026-08-15, on the environment rather than on the design. No
  ROS 2 is installed in this checkout's WSL distribution, its apt repository
  is not configured, and adding one needs a password `sudo` prompts for and
  an agent cannot answer. The exit condition is external anyway: it names an
  existing F1TENTH stack connecting, which is somebody else's stack. Unblocks
  with a ROS 2 Jazzy installation; the design work behind it does not depend
  on the rest of M5 and can be done at any point.
  Unblocking agreed 2026-08-19: the user runs the ROS 2 Jazzy install in the
  (since reset) WSL distribution; the bridge is built once it lands. The WSL
  reset also removed pip, venv support and the Python headers, so the
  bindings cannot build in WSL until the same install completes.
  Unblocked later on 2026-08-19: the install has completed and was
  verified (ackermann_msgs under /opt/ros/jazzy, colcon, rmw-zenoh-cpp,
  g++-11, clang-18, ninja, python3 pip/venv/headers). The bridge can now
  be built; the exit condition stays external (somebody else's stack).
  BUILT later the same day (ADR-0050): `slipx_ros`, an rclpy package
  above the bindings, deliberately outside the wheel. One node, one
  validation-mode simulation paced against the wall clock, N agents
  under /car_N/ speaking F1TENTH: drive in (held like a servo, speed
  through a named proportional gain mechanising a VESC's loop), scan,
  imu and odom out at sensor-data QoS with NaN never zero on the wire,
  ground truth declineable at launch with the choice recorded in the
  bridge manifest, /clock as steps times dt. odom is the encoder's own
  belief, dead-reckoned from its speed and the commanded steer through
  the kinematic bicycle, drifting by exactly the slip; its first test
  run failed by the driven wheels' spin-up under acceleration, the
  modelled effect, and the test now settles to steady state instead of
  widening a tolerance. The input log is always on and a test replays a
  live run bit for bit from it (ADR-0044's promise, kept). Nine rclpy
  tests run in the WSL ROS environment and skip cleanly where rclpy is
  absent, so the Windows suite stays green. Mutation pass over the
  bridge: 14 tried, 14 caught (speed loop sign, commands not held, NaN
  as zero, clock not sim time, ground truth ignoring the launch switch
  and broadcasting one car's truth, the reckoner dropping the steer or
  inverting the wheelbase, scan stamps laundered from the publishing
  step, IMU axes crossed, frame ids not the mount, the input log off,
  the grid unspaced, the mode not validation). Not tried: hardcoding
  real_time_factor, which changes only how long the wall clock takes to
  agree with the same trajectory. The support matrix the task names is
  published in docs/reference/ros-bridge.md (Jazzy tested; rolling
  expected, untested), alongside the transport recommendation of
  ADR-0052. TF added 2026-08-20 (ADR-0053): REP 105 shaped, identity
  mounts on tf_static (the sensor models cast from the vehicle origin),
  odom to base_link from the dead reckoner, and a map to odom correction
  composing the chain to the true pose, gated by the same switch as
  ground truth; --no-tf removes the broadcast, and frame ids lost the
  leading slash tf2 refuses to look up. Mutation pass over the TF code:
  12 tried, 11 caught first time; the escape (a non-identity mount
  rotation laundered back to identity by tf2's quaternion normalisation
  on lookup) became a wire-level test on /tf_static, then 12/12.
  The tick still waits for the external
  exit condition: an existing F1TENTH stack connecting with a remap
  file, which is the outreach the user runs; the two-machine lockstep
  measurement rides with it.
- [x] **M5.8** Reference stack: wall-follower plus pure pursuit, as examples
  that validate the simulator, explicitly not a competitive stack.
  Done when: both run a lap on the shipped track headlessly in CI.
  Done 2026-08-15, in `examples/cpp`, header-only and outside every
  component, because nothing in the library should depend on them. The pair
  is chosen so the two consume different halves of P1: the wall follower sees
  nothing but a scan, so its lap says the sensor chain is coherent, and pure
  pursuit takes ground truth against the centreline, so its lap says the
  geometry and the vehicle model agree about where the car is. When one lap
  fails and the other passes, the failure is already half localised. This is
  also the only place in the tree that links scene and sense together, which
  is what ADR-0037 intended: something above both hands the raycast to the
  sensor as a plain function.
  Extended 2026-08-15, at the user's request rather than as a task of its own:
  `examples/cpp/ghost_race.hpp` runs twenty of them at once under the
  orchestrator, each with its own controller and lap counter, and
  `examples/ghost_race_figure.py` draws the recording as an animated SVG. It
  is a demonstration and is labelled one everywhere it appears: with no
  contact model and no race control the cars cannot interact, so it is twenty
  time trials sharing a track and not a race. The figure is deliberately not
  the SVG sink and says so; the sink's one-car-per-file rule is a decision
  (ADR-0028) and is untouched. Occasioned by drawing the twenty-agent
  benchmark and finding that its cars hold a constant steering angle and
  leave the track inside two seconds, which `docs/reference/performance.md`
  now states.
  Mutation pass: 14 tried, 12 caught. Two escapes, both non-defects and both
  recorded rather than papered over. Taking the lap time as `now` rather than
  `now + dt` shifts every lap by one step, identically for every car, so it
  changes no comparison the demo makes, and pinning it would assert the exact
  step at which a floating-point projection crosses the line, which is a
  stricter claim than the lap counter itself makes. Leaving `omega_w` unseeded
  on the grid escapes because L2 has no wheel rotational state (ADR-0027): the
  wheel speeds are reported rather than integrated, and the one thing the
  model reads them for is a single step of ESC curve lookup that ADR-0031
  already records as expected. Closing the first round of escapes also found
  that a CI-time trim had opened one: three tests had been cut to a single
  lap, and a lap time that is really an elapsed clock is invisible until the
  second lap.
- [x] **M5.9** Performance benchmarks, measured and published: L2
  single-agent step under 5 microseconds per core; one agent with L2 and 2D
  LiDAR at 100 times real time or better headless; 20 agents with L2 and 2D
  LiDAR at 10 times real time or better. Tracked per commit, not measured
  once.
  Done when: a benchmark suite runs in CI and the numbers are published in
  the docs with the hardware named.
  Partly done 2026-08-15, and left unticked deliberately. The suite exists
  (`benchmarks/slipx_bench.cpp`, run by CTest as `Benchmarks.Run`) and the
  numbers are published with the machine named in
  `docs/reference/performance.md`, so the "Done when" line holds. Two of the
  three targets in the task itself do not, and ticking it would be a claim
  ahead of the evidence.
  Measured on a Ryzen 7 7800X3D under WSL2, GCC 13.3, RelWithDebInfo: L2 step
  4.84 us against a target of 5 (met, with no margin); one agent with a
  1080-ray 40 Hz LiDAR at 90x against a target of 100; twenty agents at 3.8x
  against a target of 10.
  Measuring it paid for itself immediately. The first raycast tested every
  wall segment for every ray, which put one agent at 16x; a uniform grid with
  a DDA traversal took it to 90x, and the grid is asserted against a
  brute-force reference over thousands of rays rather than trusted. Two bugs
  in the traversal were caught that way: sampling the ray every half cell
  misses cells it clips at a corner, and nudging the start past the entry
  point skips the origin's own cell for a ray that begins on a boundary,
  which is what a ray from a wall corner does.
  The remaining gap and the three candidate fixes are written up in
  `docs/reference/performance.md`; closing it is M5.12.
  Superseded by M5.12's numbers on 2026-08-15: the step is 1.92 us, one agent
  179x and twenty agents 8.4x, measured against a rebuild of this commit in
  the same session. The figures above are what this commit measured and are
  left as they were recorded.
  Ticked 2026-08-19 with M5.12's closure: two targets met as set, the third
  renegotiated to over 7x with the reason recorded (see M5.12 and
  ADR-0045), so every target now standing is met and published with the
  hardware named.
- [x] **M5.12** Close the performance gap M5.9 measured: one agent from 90x
  to over 100x, twenty agents from 3.8x to over 10x. The candidates, in the
  order they look worth trying, are early exit in the wall traversal, a
  projection that starts from the segment the same agent used last time, and
  reusing the previous ray's answer as a first guess within a scan. None
  changes what the simulator computes. Each lands with a before-and-after
  measurement recorded in `docs/reference/performance.md`.
  Note the arithmetic that constrains this: at 1 kHz the vehicle model alone
  costs one agent 4.84 ms per simulated second, so twenty agents cap at about
  10x before a single ray is cast. The 20-agent target needs the step under
  5 us and the sensing close to free, and may need the step itself revisited.
  Done when: both figures meet their targets on a named machine, or the
  targets are renegotiated deliberately with the reason recorded.
  Half done 2026-08-15, and left unticked for the same reason M5.9 is: one
  target is met and one is not, and ticking it would be a claim ahead of the
  evidence. Measured by alternating the old and new binaries in one session,
  best of ten each: the step 4.98 to 1.92 us, one agent 93x to 179x (**met**),
  twenty agents 4.0x to 8.4x (**missed**, by 24 per cent). No reference hash
  moved and none was allowed to; that was the constraint the whole slice was
  written under, and it is asserted after every change rather than hoped for.
  The step was revisited, as the note above anticipated, and it turned out to
  be the easy half. It was calling `pow` 280 times per millisecond of car:
  `Fz_nom^k_mu` does not depend on the load being asked about and both peak
  branches share `Fz^(1-k_mu)`, so tyre.hpp now splits the peak force law into
  a tyre-only half and a load-only half with the grouping written out, and the
  first load pass runs at loads that are a per-step constant. Separately, the
  two load passes were each recomputing every wheel's slip angle and Magic
  Formula shape term, which is four arctangents and a sine per wheel that no
  vertical load can change; those moved to once per derivative.
  The wall traversal now tests each cell as it reaches it and stops at the
  first hit it can prove is nearest, rather than gathering every candidate to
  the maximum range and testing the lot. The proof is that a segment's
  intersection lies inside its own bounding box, so an untested segment's hit
  is in a cell the walk has not entered and is no nearer than the current
  cell's far edge. About half the cost of a ray.
  The third candidate is deliberately not done and should be struck rather
  than carried: a projection that starts from the last segment is not on the
  path either benchmark measures, since neither runs a lap counter, and making
  the search local changes the answer `scene::project` returns, which is the
  one thing this slice refused to do. The second candidate, a first guess
  within a scan, was assessed and rejected: it cannot let the walk skip a
  cell, so it saves a few segment tests and nothing else.
  What is left is 24 per cent, all of it in the sensing, and the grid's cell
  size was swept over a factor of twenty to confirm the shipped choice sits at
  the bottom of a broad minimum. Closing it means a different acceleration
  structure, which is an ADR and the racing phase's broadphase, or a
  deliberate renegotiation of the target. Both are the user's call.
  Decided 2026-08-19 by the user: broadphase first. M7.4's BVH is in scope
  anyway, so the 20-agent figure is re-measured when it lands; only if it
  still misses 10x is the target renegotiated to the measured number, with
  the reason recorded in `docs/reference/performance.md`. M5.9 and M5.12 stay
  unticked until that re-measurement.
  Closed 2026-08-19, later the same day, by that re-measurement. The BVH
  landed (M7.4, ADR-0045) and lost to the grid on the workload by a factor
  of three (95 against 280 ns per wall ray, alternated in one session), so
  the acceleration-structure route is measured shut and the renegotiation
  clause applies. Re-measured with pre- and post-racing-phase binaries
  alternating in one session: 7.3x best, and the old binary measures the
  same, so the change from the published 8.4x is the machine's mood between
  sessions, not the racing code. Target renegotiated to over 7x, the number
  every session clears, recorded in `docs/reference/performance.md`; what
  would actually move the figure now is multi-core stepping, which is a
  determinism decision and would be its own ADR.
  Mutation pass: 28 tried, 23 caught. Two escapes were real holes and are now
  tests. No case distinguished the front tyre from the rear one in the
  friction budget, because the reference car has its CoG mid-wheelbase and the
  same compound at both ends, so reading one tyre for all four wheels produced
  every published trajectory unchanged. And no track under test had segment
  lengths uneven enough for a wall's bounding box to reach cells it crosses
  nowhere near, which is exactly the case separating "the nearest wall" from
  "the first wall found"; a fixture with 10 m straights and 0.1 m ends kills
  both that and an exit taken one cell too eagerly. The other five escapes are
  recorded rather than papered over, and all five are non-defects: three
  weaken the early-exit test in the safe direction, so they cost cells and
  change no answer; one loosens the grid clip the same way; and one changes
  which of two exactly equidistant segments wins, which is unobservable
  because both walls have strictly positive widths and so never coincide.
- [ ] **M5.10** P1 exit gate (external fact): a RoboRacer team runs their
  existing stack against SlipX with a one-file topic remap and reports that a
  tuning change made in sim held on their car. Track the outreach as work:
  identify teams, offer support, collect the report.
  Done when: the report exists and can be cited.
- [ ] **M5.11** Release 0.3.0, following the M4 release checklist (docs set,
  README audit, version bump, TestPyPI rehearsal, tag, publish).
  Done when: published and installable.
  Decided 2026-08-19 by the user: feature work through M7 lands first, and
  the 0.3.0, 0.4.0 and 0.5.0 releases are cut together with the user at the
  end, each from the commit that completed its milestone or as one combined
  release, to be settled then. The same decision applies to M6.10 and M7.11.

## M6. P2: identification and the registry

Status: in-progress; M6.1 to M6.6 and M6.9 done (the registry is live at
github.com/ibrahimsel/slipx_registry). What remains needs the outside
world: the chassis decision is "not now" (M6.7), the exit gate needs three
external contributions (M6.8), and the release is deferred to the end of M7
by the 2026-08-19 decision (M6.10). Size: extra large
(many weeks). Release: 0.4.0 at the end. This phase is the differentiator;
the SRS is explicit that it must not slip behind the racing features,
because parameter sets compound and racing features do not.

- [x] **M6.1** The manoeuvre library: skidpad, step steer, ramp steer,
  straight-line acceleration, coastdown, circle-to-slip. Each with a written
  procedure, a space requirement and a safety note, and each executable with
  only the sensors on a stock competition car (encoders, IMU, LiDAR pose).
  No dyno, no tyre rig, no force platform, ever.
  Done when: the six procedures ship as documentation with worked parameter
  coverage (which manoeuvre identifies which parameter).
  Done 2026-08-19: `docs/identification/`, six procedures plus an index that
  carries the bench-measurement list (including the bifilar pendulum for
  `izz`, timed by the car's own gyro) and the full coverage table: every
  `VehicleParams` field is sourced from a bench measurement, a manoeuvre, a
  datasheet, or stays `provisional` with the reason stated. Worked numbers
  throughout are the reference car's. Two honesty notes worth keeping: the
  understeer gradient alone identifies only the difference of the axle
  stiffnesses (the LiDAR pose is what makes them separately identifiable),
  and `k_mu`'s ballast signal is a 1.4 per cent change in limit speed, so it
  carries the widest confidence interval in the file by design. The C++
  suite and the four scriptable CI checks pass; pytest is blocked on the
  reset WSL environment and these files are prose only.
- [x] **M6.2** The synthetic self-test **first**: generate data from known
  parameters through the forward model, fit, and assert recovery within
  tolerance. It is the only way to test the fitter without hardware, so it is
  the first thing built, not the last.
  Done when: round-trip recovery of every MF-lite parameter is asserted in
  CI.
  Done 2026-08-19. `slipx_id` exists (ADR-0038): a deterministic
  Levenberg-Marquardt on the standard library, channels, the bag-level
  reconstruction, the synthetic manoeuvre library, and four staged fits.
  Recovery on noiseless synthetic data: resistances to 0.01 per cent, axle
  stiffnesses to 0.4 per cent, mu_y0 to 0.9 per cent, c_kappa and mu_x0 to
  3 per cent, k_mu to 4 per cent, the delays to 3 to 9 per cent. The shape
  pair (C, E) round-trips as a curve (within 3 per cent through the working
  range and under load transfer), not as coordinates: steady driving cannot
  sit on the falling branch, so the pair is degenerate there and the fitter
  reports the correlation by name instead of printing two confident numbers.
  The ADR records the three traps the self-test surfaced (triangular ramps,
  the steered wheels' induced-drag couple in the moment balance, and
  mirroring ADR-0027's two-pass load evaluation).
  Mutation pass: 6 tried, 5 caught (lift clamp deleted, slip-angle steer
  sign flipped, the induced-drag couple deleted, correlation threshold
  raised past the reported entanglement, ballast restatement dropped). One
  escape, recorded as a non-defect in the safe direction: bypassing the
  two-pass mirror and feeding measured ay into the load law now passes the
  tolerances, because the triangular slow ramps shrank that discrepancy
  below them; the mirror is kept because it is the model's actual structure
  and costs nothing.
- [x] **M6.3** `slipx_id`, the fitter: ingest rosbag2, emit `dynamics.yaml`
  with per-parameter residuals and confidence intervals. Refuse to emit a
  parameter set without a populated provenance block; warn when identified
  parameters fall outside physically plausible bounds for the declared scale.
  Done when: the synthetic self-test passes end to end through the real bag
  path, and both refusal and warning have tests.
  Done 2026-08-19, as `slipx-id` (ADR-0040): rosbag2 read without ROS
  (sqlite3 via the standard library, MCAP via the extra, CDR by hand for the
  five message types the manoeuvre library records, unknown types refused by
  name), a session file naming bags, bench constants, topics and provenance,
  and emission of a complete car directory rather than a lone dynamics.yaml,
  because a file the loader cannot open is not a deliverable. The end-to-end
  test writes real bags, fits off them, and loads the emitted car back
  through `slipx.load_car`; the refusal (empty provenance, checked before
  the expensive part) and the plausibility warning (an implausible mu_y0
  surfaces through the schema read-back) both have tests. Honesty paths
  exercised deliberately: the test car's launch is current limited, so
  mu_x0 stays provisional with a note, the flat cap identifies the ESC's
  current limit, and the configured v_max caps the curve extrapolated from
  a full pack. Mutation pass: 7 tried, 6 caught (CDR alignment without the
  header offset, drive fields swapped, tyre residuals dropped, the
  one-file-or-two decision inverted, the migration step deleted, plus the
  registry-check refusals); the escape (the ballast bench keeping the
  unballasted mass, diluted across nine recordings) became a direct test
  before the slice closed.
- [x] **M6.4** The validation report: replay measured control inputs through
  the fitted model and plot divergence in yaw rate, lateral acceleration and
  speed, with a single headline fit metric. Reuses the recording and sink
  machinery from M3 rather than growing its own plotting.
  Done when: a report generates from the synthetic self-test's data and reads
  correctly.
  Done 2026-08-19: `slipx_id.report`, one theme-aware SVG built on the SVG
  sink's own helpers, measured solid against replayed dashed for the three
  channels, per-panel divergence and a headline that is the worst channel of
  the worst run, so adding easy runs cannot improve it. Speed is the encoder
  speed on both sides (like against like), and its divergence is scaled by
  the mean speed because a held speed barely varies about it. The session
  file's `validation:` bags drive it from the CLI; the emitted provenance
  names `validation.svg`, which is what the registry's acceptance bar
  checks. The report says plainly that a set which validates on these runs
  has validated on these runs and on nothing else. The end-to-end test
  closes the loop: the car fitted from bags replays a slalom the fit never
  saw at under 6 per cent worst-channel divergence, and a deliberately
  wrong model is visibly wrong (the metric is tested to measure something).
  Mutation pass: 3 tried, 3 caught (headline as mean, divergence ignoring
  the replay, the speed scale swapped); command playback by interpolation
  rather than hold is unreachable with sine commands and is pinned by the
  channel-level hold test instead.
- [x] **M6.5** Schema: the provenance block (source, method, date,
  contributor, residuals) required on any parameter set submitted to the
  registry. A schema minor bump, versioned independently of the core as
  always.
  Done when: schema validation enforces it with tests.
  Done 2026-08-19, pulled forward into the M6.3 slice because emission
  needed it: schema 0.4.0 (ADR-0041). The provenance fields the task names
  had existed since 0.2.0; what the registry actually lacked was the
  compound vocabulary (the enum could not name a fitted tyre), the `data`
  block tying a fit to the SHA-256 of its recordings, and the acceptance
  bar as code: `rules.check_registry_submission` refuses by name a
  submission without the identified label, a contributor, residuals, a
  validation report and the data digests. Enforced with tests either side,
  and every 0.3.0 document remains valid.
- [x] **M6.6** **DECISION**, then build: registry hosting and curation. A git
  repository with PR review is the cheapest credible option; decide the
  acceptance bar before the first submission arrives. Contribution must be a
  by-product of using `slipx_id`, one command from bag to submitted PR, not a
  separate act of altruism.
  Done when: `slipx_registry` exists with a contribution workflow and the
  acceptance bar written down.
  Decided 2026-08-19 by the user: a separate repository with PR review. Its
  full content (layout, README, schema-validating CI, the acceptance bar) is
  prepared in this tree under a staging directory; the user creates the
  GitHub repository and pushes it.
  Staged 2026-08-19 under `registry/`: the README (what an entry is, the
  contribution flow as a by-product of `slipx-id`, the acceptance bar in
  prose pointing at the bar in code), `tools/check_submission.py` (a thin
  runner over `rules.check_registry_submission` plus the
  validation-report-file check), the CI workflow that runs it over every
  entry on every pull request, the licence, and an empty `cars/`. The
  self-test proves the flow: the end-to-end suite copies its own emitted
  car in as an entry, the check accepts it, and deleting the validation
  report gets it refused by name. Remains before the box is ticked: the
  user creates the `slipx_registry` repository and pushes this directory,
  which then leaves this tree.
  Ticked 2026-08-19: at the user's instruction the repository was created
  and pushed via the gh CLI as
  https://github.com/ibrahimsel/slipx_registry (public, Apache-2.0, one
  initial commit). Two edits were made as the content left the tree: the
  staging note came out of the README, and both the README and the CI
  workflow gained the honest version note, since the published 0.2.0
  predates `slipx-id` and the acceptance-bar code: CI installs SlipX from
  git source until the 0.4.0 release is on PyPI, then switches to
  `pip install slipx` (the workflow comment says so at the line). The
  in-tree end-to-end test that ran the staged runner now exercises the
  same bar directly (`check_registry_submission` plus the report-file
  rule), because the runner lives in the registry's own CI now.
  One loose end, honest and expected: the registry's first CI run is RED,
  because installing SlipX from git gets the public main, which does not
  yet carry the acceptance-bar code; every commit since `c1bd582` is
  local. It goes green when the user pushes `slipx` main and re-runs the
  workflow (`gh run rerun -R ibrahimsel/slipx_registry <run-id>` or just
  the next push there).
- [ ] **M6.7** **DECISION**: buy a chassis. Materially changes the honesty of
  the launch claim; the SRS recommends yes. The first credible in-house
  parameter set is worth more than its cost.
  Done when: decided; if yes, one fitted set with a validation report exists
  and is labelled `identified`, not `provisional`.
  Decided 2026-08-19 by the user: not now. The P2 exit gate rests on
  external contributors' cars; the identification pipeline is built and
  synthetically proven, so a chassis bought later is exercised immediately.
  Left unticked because a "yes" would still change what exists; revisit at
  the 0.4.0 release at the latest.
- [ ] **M6.8** P2 exit gate (external fact): three parameter sets in the
  registry contributed by people who are not the maintainer, each with an
  attached validation report. Seed by co-authoring with two friendly teams.
  After this gate, and only after it, the word "validated" becomes available
  for those sets; the project-wide claim stays "physically structured and
  identifiable" and the labels do the talking.
  Done when: three external sets with reports are merged.
- [x] **M6.9** Tutorial articles as the work surfaces them (system
  identification, residuals and confidence, the validation report), per the
  standing brief.
  Done 2026-08-19: articles 14 (system identification, with the coastdown
  two-parameter fit worked by hand), 15 (residuals and confidence, with
  entanglement as the failure an interval alone cannot show) and 16
  (validation by replay, with the open-loop argument and what a validation
  is worth). One argument in three parts, indexed as such; concepts only,
  SlipX in asides; no figures, because none earned its place. Proper
  outside sources cited (Ljung; Numerical Recipes ch. 15; Oberkampf and
  Roy).
- [ ] **M6.10** Release 0.4.0, per the release checklist.

## M7. P3: racing

Status: in-progress; M7.1 to M7.8 done, M7.9 decided (deferred); what
remains is the external gate (M7.10) and the release (M7.11, cut with the
user per the 2026-08-19 decision).
Size: extra large (many weeks). Release:
0.5.0 at the end. Hash impact: contact and rollover enter `slipx_core`
numerical paths; expect deliberate hash movements, recorded per the standing
discipline. (M7.1 moved none: detection reads diagnostics the core already
computed, and the conformance script re-checked all six rows.)

- [x] **M7.1** Rollover detection as a discrete event that halts the agent
  (DNF); no flight or landing simulation. The static rollover threshold
  already exists in `load_transfer.hpp`; this makes it an event.
  Done when: a CoG sweep above the threshold produces the event
  deterministically and the event carries the cause.
  Done 2026-08-19, recorded as ADR-0042. Detection lives in `slipx_sim`,
  after each step, from that step's own diagnostics: both wheels of one
  side at zero vertical load, which the core's clamp writes as a literal
  zero. A single lifted wheel is deliberately not the signal (that is
  three-wheeling, routine near the limit); NaN below L2 means those tiers
  cannot roll, stated as their limitation. The DNF freezes the pose and
  zeroes the velocities, so the car reads as the stationary obstacle M7.2
  will collide with; the event carries cause, step and time, survives
  snapshot/restore, reproduces under replay, and the manifest reports the
  outcome per agent outside the configuration digest. The CoG sweep is
  asserted against the closed form: the event fires within 15 per cent of
  g t / 2h, monotonically earlier with height. Probing the scenario paid
  for itself: on the default tyre the car slides rather than rolls (as the
  docs promise), drive thrust props up the rear inner wheel so a powered
  car three-wheels indefinitely, and at car-park speed the steering runs
  out before the threshold, so the rollable scenario is a sticky compound,
  coasting, from 6 m/s. Mutation pass: 13 tried, 13 caught (cause side
  swap, single-wheel signal, diagonal pairing, freeze dropped, frozen agent
  stepping, policy still called, snapshot and restore each dropping the
  event, step off by one, exact zero never firing, replay bypassing
  detection, manifest not told, JSON dnf fields for running agents).
- [x] **M7.2** Agent-to-agent contact as a planar impulse with restitution,
  Coulomb friction and resulting yaw moment. Plausible and deterministic, not
  fitted to data, and the docs keep saying so.
  Done when: momentum conservation and mirror symmetry hold in the invariant
  suite; determinism holds across runs.
  Done 2026-08-19, recorded as ADR-0043. The mathematics is a pure
  header-only function pair in `slipx_core` (`contact.hpp`: SAT geometry
  with a clipped-incident-edge contact point, then one impulse with
  restitution, a Coulomb cone and a positional projection); the orchestrator
  applies it between steps, one pass per touching pair in ascending index
  order, no convergence loop. Contact exists between agents that declare a
  footprint (the car file's `geometry.length/width`); declaring none means
  touching nothing, which is what keeps every pre-contact trajectory
  bit-identical, asserted in-suite (footprinted single agent and distant
  pair against their bare twins) and re-checked against the published rows.
  Restitution and friction live in `SimulationConfig.contact`, labelled
  plausible everywhere they appear, folded into the configuration digest
  along with per-agent footprints. A DNF'd car enters the impulse with zero
  inverse mass: the wreck is immovable and collidable, closing the loop
  ADR-0042 opened. Momentum (linear and angular), the friction cone and
  bit-exact mirror symmetry hold in the invariant tests at both the pure
  and the run level; a below-threshold closing speed does not bounce (an
  anti-jitter device, documented as one). Article 17 and its render-checked
  figure explain the model; the ghost race's wording moved from "no contact
  model" to "no footprints declared".
  Mutation pass: 18 tried, 18 caught, one of them only after the test was
  sharpened rather than the mutant excused. Applying the (exactly zero)
  deltas to a frozen car escaped at first: the difference is confined to
  signed-zero bit patterns from round-tripping a zero velocity through the
  world frame, and the original scenario's geometry laundered them back to
  +0.0. Working out where a -0.0 survives (a broadside hit on a wreck whose
  yaw has negative cosine) turned the escape into a catch; the frozen-car
  test now rams the wreck side-on in that quadrant on purpose and says so.
  The other seventeen: normal never flipped, restitution dropped, friction
  cone unclamped, angular effective-mass term dropped (caught by the
  eccentric restitution law, not by momentum, which survives it), projection
  sign flipped, projection overshooting, kissing counted as touching,
  restitution threshold dropped, contact point as endpoint, centre offset
  ignored, incident edge most-parallel, contact pass dropped from advance
  and from replay, frozen car given inverse mass, self-contact, second
  agent's footprint not required, footprints left out of the digest.
- [x] **M7.3** Lockstep barrier protocol with per-agent acknowledgement and a
  configurable timeout policy (freeze, coast or DNF), so one hung agent cannot
  hang a race.
  Done when: a deliberately hung agent exercises each policy in a test.
  Done 2026-08-19, recorded as ADR-0044, and deliberately below the
  transport: the protocol lives in `slipx_sim` so the ROS bridge and the
  multi-host race inherit it instead of each reinventing it. Commands
  arrive through a `CommandMailbox`, a thread-safe queue of step-tagged
  entries and the one synchronised doorway into the still single-threaded
  simulation; the tag is the acknowledgement (post = command, ack = alive,
  hold my last one; tags strictly increase; stale entries are discarded at
  the barrier). A miss is answered per agent: wait (strict lockstep, and a
  test proves a dawdling poster thread cannot change one bit of the
  trajectory), freeze (a pause that resumes where it stopped), coast, or
  DNF through ADR-0042's machinery with its own cause. The wall-clock
  `barrier_timeout` decides when a miss is ruled, so a live run with
  non-wait mailbox agents is reproducible from its input log rather than
  by re-running, and the manifest says which promise applies instead of
  repeating one that no longer holds. A missed step is logged as a
  NaN-tagged slot; replay answers it with the agent's own policy, and NaN
  is refused at every command door, which is what makes the marker sound.
  The Python stepping calls release the GIL so a poster thread can feed a
  waiting barrier; a pytest proves it. Found and fixed in passing:
  `sim.replay(sim.input_log())` handed replay a reference to the member
  that reset() clears, and quietly replayed an empty log.
  Mutation pass: 17 tried, 17 caught, after two test gaps the mutation
  list itself exposed were closed first (a stale entry poisoning the queue
  for later fresh entries, and the timeout budget never being exercised):
  stale discard removed, miss silently holding the last command, posts
  never updating the hold, NaN accepted at post, non-monotonic tags
  accepted, freeze becoming coast, dnf never disqualifying, timeout-DNF
  leaving the car moving, wrong cause, marked step integrated anyway,
  replay ignoring the marker, replay accepting an impossible marker, NaN
  policy command integrated, manifest never admitting timing dependence,
  replay clearing the log it was handed, wait polling instead of waiting,
  timeout budget ignored.
- [x] **M7.4** Prebuilt static scene BVH with a per-step dynamic agent
  overlay refit only.
  Done when: broadphase results match a brute-force reference in tests.
  Done 2026-08-19, recorded as ADR-0045: `slipx/scene/broadphase.hpp`, a
  `SceneBvh` over the wall segments (fully specified sorted-median build,
  ordered pruned traversal, no scratch stamps so the query is thread-safe,
  every box fattened a nanometre for the corner-ulp reason the grid already
  learnt) and an `AgentOverlay` of refit oriented boxes answering rays with
  a self-skip and conservative sort-and-sweep pairs. Both match their
  brute-force definitions bit for bit on the same sweeps the grid is held
  to; the exact ray-segment arithmetic moved to one shared header so the
  two accelerators cannot drift. The measurement that mattered: on short
  corridor rays the BVH costs 280 ns against the grid's 95, so the grid
  keeps the wall rays, the benchmark prints both costs per commit, and the
  BVH's job is the broadphase it was named for. The 20-agent re-measure and
  the renegotiation are recorded under M5.12.
  Mutation pass: 12 tried, 12 caught (padding removed, right child assumed
  adjacent, node box shrinking, leaf skipping its last segment, prefilter
  inverted, self-skip ignored, inactive box casting, box-frame rotation
  sign, extents axis-swapped, sweep without its y check, pairs in sweep
  order, inactive boxes sweeping). Answer-preserving weakenings of the
  traversal pruning were not tried: the leaf test is exact and the boxes
  conservative, so those mutants cost only time by construction.
- [x] **M7.5** Race control per the published RoboRacer ruleset: time trial,
  obstacle avoidance test, head-to-head, grid and rolling starts; contact
  attribution from relative geometry and closing velocity with the ruleset's
  penalty logic; the ruleset repository tracked as a versioned dependency with
  the implemented revision stated.
  Done when: scenario tests cover each procedure and the build states its
  ruleset revision.
  Done 2026-08-19, recorded as ADR-0046: `slipx_race`, a new component
  above the sim and the scene (the dependency lint gained the layer in the
  same change), implementing `f1tenth/roboracer_rules` at revision
  `202c3771465b1690c0e28618271cca91d5c842c9` (2025-10-13), stated by the
  build via `ruleset_statement()` and cited rule by rule at each
  implementation site. Time trial with 2.4.5's two-category scoring and
  tie-by-laps; the obstacle test of 2.5.1.6 (an ordinary footprinted agent
  is the obstacle, contact is the sim's own model, "complete stop" is an
  operationalised floor that arms only once the car moves); head-to-head
  rounds and best-of-three matches per 2.5, with grid starts one car width
  apart, side swap and a seeded round-three coin flip, light contact
  recorded once per touch episode and never penalised, fault from approach
  contributions with ties against the car behind, crash restarts at 2 m
  plus the 1 m recovery bonus, three warnings a disqualification that ends
  the match, DNFs handing the round to the survivor, and the border
  enforced by rule (a wall crash places the car at rest where it left)
  because nothing in SlipX collides a car with a wall. Rolling starts ship
  as a labelled non-ruleset extension. The sim gained per-step
  ContactEvents (pair, impulse, per-car approach) and the core's impulse
  reports the approach split. Every referee judgment is a named RaceConfig
  field labelled as a mechanisation. Python bindings for race control are
  deferred to M7.8's harness decision. Scenario tests cover every
  procedure; a match replayed is the same match.
  Mutation pass: 20 tried, 20 caught, after three test gaps the drafting
  itself exposed were closed first (an exact streak assertion, a
  wrap-around obstacle placement, a bound on light-contact episodes):
  fault inverted, light threshold dropped, level restart, bonus dropped,
  warnings never disqualifying, DQ ending only the round, sides never
  swapping, simultaneous finish to the car behind, border crash leaving
  the car rolling, streak surviving a crash, stop check firing on a
  standing start, obstacle contact ignored, pass distance unwrapped,
  streak ranking inverted, ties ignoring laps, grid on the centreline,
  pose never wrapping, DNF rewarding the DNF'd car, light contact every
  step, rolling start at rest.
- [x] **M7.6** The structured event stream: every race-control outcome as
  timestamped, machine-readable events, encoded as MCAP so the event stream
  and the run sinks are one format, not two. Any leaderboard, report or CI job
  consumes the event stream and nothing else.
  Done when: a full race replays from its event stream alone.
  Done 2026-08-19. `slipx/race/event_stream.hpp`: a hand-rolled MCAP
  encoder (unchunked, uncompressed, no summary, CRC honestly zeroed as
  "not computed"), one JSON channel `/race/events` in exactly the Python
  sink's dialect (json messages, jsonschema schemas, absence over
  sentinels: an event with no second car has no `other` key), and a
  metadata record carrying the pinned ruleset, every RaceConfig field and
  the caller's run identifiers, so the file answers "who won, under what
  rules" alone. The reading half parses exactly what the writer emits and
  refuses anything else by name, strict down to the log time agreeing
  with the JSON time; the "replays from its stream alone" test
  reconstructs a crash-and-disqualification match, winner, warnings and
  all, from a consumer holding only the file, field for field bit-equal
  (17 significant digits round-trip). `slipx_race_demo` runs a canned
  deterministic match and writes its stream; pytest runs the binary and
  reads the file with the reference `mcap` library, which is what holds
  "one format, not two" against the same library that reads the run
  sinks. Identical races write identical bytes.
  Mutation pass: 9 tried, 9 caught (magic bytes wrong, doubles at nine
  digits, the absent agent written as a sentinel, log time in seconds,
  metadata record losing its name, messages claiming an undeclared
  channel, the ruleset revision left out, the type round-trip missing the
  last type, the reader trusting any record length). One test needed
  sharpening first: the schema JSON legitimately names "other", so the
  absence scan looks for the sentinel form, not the key.
- [x] **M7.7** Per-agent sensor configuration (cheap opponents run 2D or no
  sensors); `race_sync` client library implementing the barrier, linkable
  into a student control node with under ten lines of change; RMW default
  benchmarked and decided (`rmw_zenoh` versus Fast-DDS discovery server, at 6
  and 20 agents) with the multicast failure mode documented; multi-host agents
  with the simulator as sync authority.
  Done when: each has tests or, for the RMW decision, a recorded benchmark.
  Ticked 2026-08-20, with the last two halves. The RMW benchmark
  (`python -m slipx_ros.rmw_bench`, ADR-0052) runs the same fully
  sensored lockstep race over as-shipped Fast-DDS, Fast-DDS with a
  discovery server, and rmw_zenoh, at 6 and 20 agents, one client
  process per agent; measured on the 7800X3D under WSL2 and repeatable
  within one per cent, all three carry 20 agents at about 200 barrier
  turns per second, zenoh costs 5 to 12 per cent against multicast
  Fast-DDS while removing the multicast dependence, and the discovery
  server repeatably costs more (42 per cent at 6 agents) besides its
  super-client tooling wart, so the documented race-day default is
  rmw_zenoh and the multicast failure mode is documented from its
  mechanism in docs/reference/ros-bridge.md (loopback cannot exhibit
  it, and the numbers say they are loopback numbers). Multi-host needed
  no new mechanism (ADR-0051: the announcement and the stamped commands
  are ordinary topics and only the simulator advances); the
  router-mediated cross-process benchmark row runs as a test, which is
  the exact code path a second host adds a physical wire to, and the
  measured steps field is the simulator's own count so a short loop
  cannot report a clean row. Benchmark-harness mutation pass: 3 tried,
  3 caught (the loop stepping short, the rate inverted, the clients in
  the wrong namespace). The genuinely two-machine measurement rides
  with M5.7's external exit condition and is noted there, not here.
  In progress. The sensors half's C++ machinery landed 2026-08-19 as the
  sensor rig (ADR-0047): `slipx/sim/sensor_rig.hpp`, per-agent LiDAR, IMU
  and encoder instances observing a simulation they structurally cannot
  perturb (const reference, own seed, out of the manifest and the digest,
  so no reference hash can move and none did). Exact schedules with
  step-resolution truth, latency as a delivery time with the LiDAR's
  uniform-jitter semantics, per-ray pose interpolation from recorded
  history (shortest arc through the yaw seam, asserted analytically on a
  kinematic car), per-agent and per-instance stream derivation, DNF stops
  sampling while pending messages still deliver, and a wheel encoder below
  L2 refused by name because those tiers never write wheel speeds and the
  encoder would report a moving car as stationary. Building it surfaced
  and fixed a partial-attach defect (an attach that refuses one sensor now
  leaves the rig untouched). Mutation pass: 25 tried, 25 caught, two only
  after tests were sharpened rather than mutants excused (the round-robin
  lesson repeated: boundary-aligned schedules cannot tell a scheduled
  instant from the step that served it, so the suite now carries
  off-boundary sensors; and the ordering test's tiebreak initially agreed
  with the alphabet). Not tried, as answer-preserving by construction:
  removing the pending sort or the latest-wins comparison (deliveries
  already arrive in stamp order under the flush rule) and over-retaining
  pose history (memory, not answers). Remaining in this half: sensors.yaml
  wired to these structs through the loader (schema 0.5.0, its own ADR)
  and the Python surface; then the ROS halves, unblocked by the completed
  environment install.
  The sensors half COMPLETED later on 2026-08-19 with that wiring
  (ADR-0048): schema 0.5.0 gives each sensor entry a typed block carrying
  its model's full parameter set (strict when present, refused by name at
  build when absent, per the M1 pattern), renames latency.jitter_stddev to
  latency.jitter (the uniform half-width the models implement) with a
  variance-preserving migration, and fails a free-form noise object loudly
  rather than guessing at undefined keys. `slipx.sensors_for` builds
  AgentSensors from a loaded car, refusing lidar_3d outright until P4; the
  rig, the sensor specs and their samples are bound to Python with the
  world as a callable, and the reference sensors.yaml is rewritten in
  full. Tests: 10 schema-side, 12 binding-side including trajectory-hash
  equality between a sensored car and its bare twin and bit-identical
  streams from identically seeded rigs. Mutation pass over the wiring: 15
  tried, 15 caught (defaulted requirements, defaulted blocks, the jitter
  bound dropped, lidar_3d silently substituting lidar_2d, phase and
  latency defaulted, latency and jitter swapped, wheels_used dropped, IMU
  densities crossed, LiDAR rate and dropout not wired, the sqrt-3
  conversion dropped, the noise refusal dropped, the conversion skipped,
  an empty noise kept). The harness truncating migrate.py mid-pass and
  the file being restored from git plus the conversation is why the
  scratchpad harness now writes with an explicit encoding.
  Still open in M7.7: the race_sync client, the RMW benchmark and
  multi-host agents, all ROS work and now unblocked.
  The race_sync half landed later the same day (ADR-0051), on the bridge
  of ADR-0050: the simulator announces each step on a latched topic and
  advances only when every agent answered through ADR-0044's mailboxes,
  with the tag travelling as the header stamp of the stack's own
  AckermannDriveStamped (exact both ways at a millisecond step, no custom
  message package). Joining costs a student three lines
  (`RaceSyncClient(node, ns)`; `sync.publish(msg)`), with a strict
  `on_step=` callback mode for stacks that want the computation itself
  step-synchronous. Asserted rather than promised: two lockstep runs hash
  identically, a client dawdling every fiftieth step changes not one bit
  of trajectory or final position, the simulator never outruns a client
  (each step seen exactly once, in order), a wrapping-mode command lands
  one step later and holds like a servo, and a silent agent coasts or is
  ruled out by its policy rather than hanging the race. Mutation pass
  over the client and the bridge's lockstep half: 8 tried, 7 caught
  outright (a skipped tag, a dropped acknowledgement path on either
  side, the announcement off by one, and the mailbox policy ignored all
  hang the barrier, which the harness counts as caught because a hung
  barrier is the failure the policies exist to prevent; the stamp
  encoding off by a thousand fails the round-trip; the speed loop fed a
  zero velocity fails the brake phase). One escape recorded as a
  non-defect: dropping the client's duplicate-announcement guard is
  unobservable in-suite because announcements are only ever delivered
  once per step by construction, and the mailbox's own strictly
  increasing tags refuse the residue anyway; the guard stays as defence
  in depth for a client rejoining under latched announcements.
  Groundwork for the ROS bridge landed the same day (ADR-0049): the
  TrackWorld in slipx_sim composes the walls and the simulation's own
  footprints into the rig's world function (nearer-of-wall-and-car, asker
  skipped, boxes exactly the contact pass's, refit once per step, a world
  missing a car refuses to answer), with slipx_sim gaining the
  slipx_scene dependency ADR-0037 anticipated. Bound natively to Python
  (`slipx.TrackWorld`, `slipx.load_scene_track`; a SensorRig built from
  one keeps every ray out of the interpreter, which is what makes the
  bridge's 20-agent sensing viable). Mutation pass: 13 tried, 13 caught
  first time (comparison flipped, either branch dropped, self-skip
  dropped, centre offset dropped, negated and axis-swapped, the wreck's
  box deactivated, footprints never gathered, refit frozen after the
  first step, the missing-car guard dropped, either cast unbounded).
  Not tried, answer-preserving by construction: refitting more often
  than once per step (states cannot change between the rays of one
  collect).
- [x] **M7.8** CI leaderboard harness with seeded scenario batches.
  Done when: a leaderboard run is reproducible from its manifest and seeds.
  IN PROGRESS, staged 2026-08-19 as a session handoff; the code compiles
  and runs but its tests and mutation pass have NOT landed, so this stays
  unticked and the next session finishes it. What exists:
  `slipx/race/leaderboard.hpp` and `src/leaderboard.cpp` (Entrant with a
  policy factory, BatchConfig, `standings()` computing rows FROM PARSED
  EVENT STREAMS ONLY with a total ordering, and `run_round_robin()` which
  seeds each scenario as derive_seed(master, index), alternates the left
  slot per repetition, writes one stream per match plus
  `batch_manifest.json` (master seed, per-scenario seeds and pairings,
  ruleset, budgets) and `leaderboard.json`, and computes standings by
  reading its own files back from disk rather than trusting memory), and
  `tools/leaderboard_main.cpp` (`slipx_leaderboard <dir> [seed]`: three
  canned entrants whose steering carries a seeded jitter so the seed
  genuinely reaches the racing; smoke-run on the shipped track: 6 matches,
  sensible standings). Remaining, in order: (1) tests/test_leaderboard.cpp
  per the design already settled: the same batch twice writes identical
  stream bytes and identical leaderboard.json and equal rows; standings
  recomputed independently from result.stream_paths equal result.rows; a
  different master seed changes at least one stream's bytes (the jittered
  entrants make this hold); an undecided scenario (coasting entrants,
  tiny budget) counts as abandoned with no match win; register the test
  and a `Leaderboard.Run` ctest case for the tool (mirror Benchmarks.Run).
  (2) Mutation pass over leaderboard.cpp; candidates already identified:
  standings reading kMatchWon.agent unmapped, sort orders flipped or
  dropped, abandoned counted as wins, seed derivation ignoring the index
  (all scenarios one seed), left slot never alternating, manifest missing
  the seeds, standings computed from memory instead of the files (delete
  the read-back), an unwritable directory not throwing. (3) CHANGELOG
  entry and the M7.8 tick with the mutation list. No design questions are
  open; ADR-0046 already covers the layer and the stream doctrine covers
  the consumption rule, so no new ADR is expected.
  Done 2026-08-19, the next session, exactly per the staged plan and with
  no design change. `tests/test_leaderboard.cpp` (8 cases) plus a
  `Leaderboard.Run` ctest case mirroring `Benchmarks.Run`, so the tool
  itself runs in CI. The four settled cases hold; the seed case asserts
  on the parsed events rather than the bytes, because the recorded seed
  metadata would make the bytes differ even if the seed never reached
  the racing. Beyond them: the counting and ordering rules are pinned
  against synthetic parsed streams (wins mapped through the stream's own
  entrant names, with the winner as agent 1 on purpose; the full ordering
  including both tiebreaks; a stream with a round won but no match won
  still abandoned), the manifest and every stream carry the independently
  recomputed derive_seed(master, index) with the alternating slot
  reaching the round-start events, and an unwritable directory throws
  with the write's own message, so the refusal cannot be laundered
  through a later failure.
  Mutation pass: 19 tried, 17 caught, one of them only after the test was
  sharpened rather than the mutant excused. Dropping the round-wins
  tiebreak escaped because the engineered tiebreak agreed with the
  alphabetical fallback; the fixture now makes the round-wins order
  oppose the name order and says so. Two escapes, both recorded as
  non-defects. Dropping the read-back's could-not-read throw is a guard
  no in-process test can reach (a file the process just wrote is readable
  by the same process; the guard defends against out-of-process
  interference, the M5.1 escape's reasoning). Computing standings from
  memory instead of the read-back files is observationally equivalent
  exactly because the suite pins result.rows against standings recomputed
  from the files: the mutant survives only while memory and files agree,
  and any divergence fails that test whichever side the harness used.
- [x] **M7.9** **DECISION**: governance. If a competition adopts SlipX,
  ownership of the ruleset implementation becomes contested; decide before,
  not after.
  Done when: recorded (ADR or governance doc).
  Decided 2026-08-19 by the user, and recorded here as the governance
  position: deferred, deliberately. Deciding governance for an adoption
  that has not happened is getting ahead of ourselves; the decision is
  taken when and if someone adopts, and the tripwire is the first adoption
  enquiry. Until then the implementation states its ruleset revision
  (M7.5) and nothing more is promised.
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
| The C library is part of that build key; the hash tracks libm | ADR-0033 |
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
