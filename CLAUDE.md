# CLAUDE.md

Guidance for AI agents working in this repository. This file is deliberately
timeless: it carries only what should stay true for the whole development
process. Current state, plans and task tracking live in `release-roadmap.md`
and nowhere else; decisions and their reasoning live in `docs/adr/`. Do not
record progress here.

## What SlipX is

A vehicle dynamics library for 1/10-scale autonomous racecars (RoboRacer /
F1TENTH class): C++17, CPU-only, headless, deterministic, Apache-2.0, with
Python bindings. The library is the product, not the simulator: the goal is
`slipx_core` embedded in projects we do not maintain, and design questions are
resolved in favour of embeddability.

## Where truth lives

- `docs/adr/` is authoritative for decisions and their reasoning. Records are
  immutable: supersede with a new record, never edit an accepted one. Writing
  the ADR is part of making an architectural change, not a follow-up
  (`docs/adr/template.md`). Read `docs/adr/README.md` before working around
  any rule below.
- `release-roadmap.md` is the working plan and the only progress tracker.
  Update it as work lands; its "Standing obligations" section is the per-slice
  process checklist.
- `docs/spec/SRS.md` is the requirement spec, local-only and gitignored: it
  does not exist in a fresh clone. Never cite it or its requirement IDs
  (`CORE-05`, `SINK-01`) in code comments or public documents; a reader of the
  published repository cannot follow them, so the comment carries the
  reasoning itself or points at an ADR.
- `README.md` is the public thesis and the PyPI long description, so every
  link in it must be absolute. Every claim in it is load-bearing: landing work
  should replace a line, not add a paragraph, and its code snippets are held
  to the standard of having been executed as written.

## Layout

| Path | Contents |
|---|---|
| `src/core/slipx_core` | C++ core: fidelity tiers behind one `VehicleModel`, integrators, public headers |
| `src/core/slipx_schema` | JSON Schemas and the Python reference parser; independently versioned |
| `src/bindings/slipx` | pybind11 package: bindings, car loader, `slipx.sinks`, conformance console script |
| `src/orchestration/slipx_sim` | fixed-step lockstep orchestrator, manifests, replay, conformance binary |
| `src/bindings/slipx_c`, `src/world/*`, `src/integration/slipx_ros`, `src/tooling/*` | placeholders for later phases |
| `conformance/reference_hashes.tsv` | published trajectory hashes, keyed by build |
| `docs/racing/` | the tutorial series (its own brief below) |
| `tools/` | the CI check scripts |

## Commands

The Makefile wraps the canonical commands; `make help` lists targets.

- `make build`: CMake configure and build with the Python bindings. The
  extension is built in place next to the package sources and
  `pyproject.toml` puts both package directories on pytest's path; there is
  no install step.
- `make test` / `make pytest`: ctest and pytest. The sink tests skip without
  the `mcap` and `rerun` extras; the interesting pytest run installs
  `mcap rerun-sdk pyyaml jsonschema pytest` into a fresh venv and runs from
  the repository root.
- `make check`: the five CI checks a test suite does not cover (dependency
  direction, licence scan, version check, conformance hashes, and the core
  configuring and building alone).

## Architecture rules

Each rule has a record; the record is the argument and this is only the index.

- **Dependencies point strictly downward** (ADR-0003; `tools/dep_lint.py` is
  the operative statement of the stack order). `slipx_core` depends on the
  C++ standard library and nothing else (ADR-0002): no Eigen, no ROS, no
  threads, no I/O, no logging, no allocation inside `step`. It must build and
  pass its tests with `slipx_schema` absent; parameters arrive as a plain
  `VehicleParams` struct and parsing lives above. This is the most
  load-bearing decision in the design; a change that violates it is wrong
  even if it compiles.
- **`step` is `const` and stateless** (ADR-0004): no hidden state, wall clock
  or ambient RNG, no unordered iteration in a numerical path, and a fixed
  reduction order (floating-point addition does not associate; the
  mirror-symmetry invariant catches a wrong order).
- **Determinism is a hard requirement.** Fixed step, seeded per-agent RNG,
  lockstep, headless. `-ffp-contract=off`, never `-ffast-math` or
  `-march=native`, no threads in the integrator. Bit-identity is promised
  within one build only, never across platforms. Every run writes a manifest.
- **An unimplemented tier throws** (ADR-0005). Never fall back to a simpler
  tier, and never make a partially built tier constructible: that is the same
  fallback wearing a different hat.
- **NaN, never zero, for anything a tier cannot represent** (ADR-0006), and a
  sink must deliver that NaN as *absent*, never as a plotted zero (ADR-0028).
  Sinks and the viewer write files and never spawn a window or a viewer
  process (ADR-0024, ADR-0028), and nothing is drawn that is not in the
  recorded state, in particular no invented track.
- **The loader refuses rather than defaults** a parameter it cannot fill
  (ADR-0025); the refusal names the parameter and where it will come from.
- **No `if (scale == ...)` branching, ever** (ADR-0012). A scale branch means
  a parameter is missing.
- **Apache-2.0 throughout; no copyleft dependency anywhere** (it would defeat
  the embedding strategy). `tools/licence_scan.py` also cross-checks optional
  extras against their recorded licences.
- **Core and schema versions are independent and never compared** (ADR-0015).
  The distribution version lives in four coupled places (`pyproject.toml`,
  `CMakeLists.txt`, `slipx/version.hpp` twice over, `slipx/version.py`);
  `tools/version_check.py` enforces it and CMake carries the numeric release
  part only.
- **Hash discipline** (ADR-0008, ADR-0020). Reference hashes are keyed by
  build, and a changed hash is a deliberate release event: CHANGELOG table
  with old and new values side by side, reason in the commit message, note in
  the TSV header, measured separately under GCC 11, GCC 13 and Clang 18, rows
  moved once per slice. Rows that should not move are asserted, not assumed.
  A released wheel asserts nothing about its own hash, and the hash tracks
  libm, not the compiler: the same wheel prints different hashes on different
  glibc versions, so check the C library before debugging a "wrong" hash.
- **Releases are deliberate.** Nothing publishes on a push; the release
  workflow is dispatch-only with PyPI Trusted Publishing (no tokens), and a
  version published on PyPI is never reused. A broken upload means the next
  number.

## Domain concepts

- **Tiers L0 to L3** are fidelity levels selected at `VehicleModel`
  construction; roadmap phases P0 to P5 are a different axis. L0 kinematic
  bicycle, L1 dynamic bicycle with linear tyres, L2 double-track with load
  transfer and MF-lite (the default, and the tier system identification fits
  against), L3 adds thermal and suspension effects. Below L2, CoG height,
  weight distribution, differential and tyre compound deliberately have no
  effect: that is the teaching artefact, not a bug.
- **MF-lite** is a reduced Magic Formula with load sensitivity and combined
  slip. Every parameter must be identifiable from a manoeuvre drivable in a
  car park with onboard sensors only (wheel encoders, IMU, LiDAR pose); an
  unidentifiable parameter is worse than an absent one, and full Pacejka is
  explicitly not the target. Tyres are referenced as a `(compound, surface)`
  pair, never embedded in the car file.
- **ISO 8855 sign conventions and SI units** throughout, asserted in tests.
  Every public parameter documents its units and sign convention.
- **Claim discipline.** Parameter sets are labelled `measured`, `identified`
  or `provisional`, and tooling prints the label rather than leaving it to
  the docs. "Validated" is available only for a set with a validation report;
  the honest project-wide phrasing is "physically structured and
  identifiable".

## Conventions

- British spelling in all prose ("tyre", "manoeuvre", "licence"); em dashes
  are strictly forbidden; limitations are stated plainly, not hedged.
- Conventional Commits, short messages, no co-author line, never push (the
  user pushes).
- A slice lands as one commit with its tests, followed by a mutation pass:
  targeted mutations tried against the new code, every escape becomes a new
  test before the slice is done, and the mutations tried are listed in the
  commit message or ADR.

## Paid-for lessons

- `sim.state(i)` and `sim.diagnostics(i)` return references the next step
  overwrites; copy before keeping.
- A test that mutates a module and restores it must clear `__pycache__`, or
  the next run loads the mutant.
- Figures are hand-rolled SVG from standard-library-only scripts
  (`docs/racing/assets/make_figures.py`, `docs/assets/make_banner.py`);
  matplotlib is not an option. Unicode has no Latin subscript `y` or `z`, so
  use the scripts' `sub()` helper rather than pasting codepoints; assert a
  diagram's geometry in code; and check every figure by rendering it, never
  by reading the SVG:

  ```
  google-chrome --headless --disable-gpu --no-sandbox --hide-scrollbars \
    --screenshot=/tmp/f.png --window-size=W,H file://$PWD/path/to/figure.svg
  ```

- A physics test can be wrong before its assertion. Check the operating point
  is physically reachable (not past `v_max`, not past the friction limit, not
  below the CoG height at which lifting is possible at all), and check the
  sign of the effect: longitudinal load transfer under braking *raises* the
  front axle's lateral budget, which is why trail braking helps turn-in.

## The tutorial series (`docs/racing`)

A standing task, not a finished one: when development surfaces a concept a
newcomer would need explained, write the article without being asked and add
it to the index. The brief, which is easy to drift away from:

- It explains the concepts, never how to use SlipX. SlipX appears only in
  optional asides marked `> **In SlipX.** ...`; if an explanation needs the
  library to make sense, the explanation is wrong.
- One concept per document. Worked numbers rather than assertions, named
  assumptions, limitations stated plainly, no padding; plausible-sounding
  text salad is the specific failure mode to avoid.
- Link a good outside source for a real prerequisite rather than explaining
  it badly in passing, and cite properly.
- Diagrams only where they genuinely help, generated by `make_figures.py`,
  theme-aware (each SVG embeds a `prefers-color-scheme` stylesheet and a
  background card), and render-checked as above.
- The directory is self-contained with relative links only (it becomes a
  separate documentation site later). Articles are numbered by writing order,
  not index order.
