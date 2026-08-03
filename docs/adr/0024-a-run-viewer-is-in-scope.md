# ADR-0024: A run viewer is in scope, and it lives in the Python package

- **Status:** Accepted
- **Date recorded:** 2026-08-03 (decision taken during P1)
- **Requirements:** VIZ-01 to VIZ-05, NFR-04, NFR-08
- **Related:** [ADR-0003](0003-dependencies-point-downward.md),
  [ADR-0013](0013-provenance-labels-are-printed.md),
  [ADR-0007](0007-determinism-is-scoped-to-a-build.md)

## Context

Nothing in SlipX renders anything. The data is all there and all bound to
Python: `Simulation` exposes `state(i)`, `diagnostics(i)`, `input_log()` and
`replay(log)`, and SIM-07's replay is bit-identical and tested. The gap is
"cannot look at it", not "cannot reproduce it".

Keep the two senses of "replay" apart, because they get conflated. SIM-07 is
numerical reproduction of a trajectory and is done. Watching a race back is a
viewer and did not exist.

On paper the subject had no owner. There was no `VIZ-` requirement family, no
phase and no component. The README's "Not in scope" ruled out photorealistic
rendering, which reads as covering this and does not: that sentence is about a
*sensor*, a camera model an autonomy stack consumes, and a diagnostic picture
drawn for a human is a different artefact.

Against that, the project had already promised rendered output three times.
SRS §7 says the cross-tier crossover "should be plotted and tracked as a
released artefact". ID-04 requires a validation report "plotting divergence in
yaw rate, lateral acceleration and speed". The README twice sells plotting as
the teaching surface, including the sentence about a student plotting exactly
why the car spun and the argument that NaN slip angles produce an empty plot
where zero would produce a believable lie. Three public commitments with no
component behind them is the state that gets resolved by accident.

Three options were considered.

**A separate `slipx-viz` consuming the P3 structured event stream.** Keeps the
distribution and the release surface unchanged, and is the right shape if the
viewer ever grows a timeline, interaction or multi-agent comparison. It was
rejected on timing and on testability. The event stream is P3, so this leaves
two phases with nothing to look at. A separate repository needs its own CI,
licence scan, version policy and release path to draw a trajectory. And it
cannot be tested against the core's state layout in the same commit, so a
change like CORE-07's added state field breaks it silently and nobody finds out
until somebody renders something.

**Out of scope, stated in the README beside photorealistic rendering.** Cheap
and defensible for a library whose product is the core. Rejected because it
contradicts three existing commitments rather than clarifying them: SRS §7 and
ID-04 would both have to be amended, and the README's teaching claim would come
to rest entirely on the reader writing their own plotter against `state(i)`.

**In this repository, in the Python package.** Chosen.

## Decision

Rendering a completed run is in scope, and it lives in the `slipx` Python
package as `slipx.viz`, above the core and above `slipx_sim`, where
`tools/dep_lint.py` already enforces the direction (ADR-0003). It is a
diagnostic artefact and not a sensor; nothing about it is visible to
`slipx_core`, which continues to depend on the C++ standard library and nothing
else.

Output is a self-contained file, never a window. NFR-04 is "headless on a
five-year-old laptop, integrated graphics, no GPU, no display server", and a
file written to disk satisfies it by construction rather than by care. An
animated SVG using SMIL, standard library only, is the implementation that fits:
it needs no display server to produce, no third-party dependency, and no
decoder to view. Theme awareness comes from the same `prefers-color-scheme`
stylesheet `docs/racing/assets/make_figures.py` already embeds, so the file
stays legible on a light or a dark page. matplotlib is not an option in this
environment and would be a dependency in any case.

The provenance label and the trajectory hash are drawn INTO the image rather
than printed beside it. ADR-0013's argument is that a label in the documentation
is read once by a subset of readers, and the same argument applies with more
force to a picture: a picture is the artefact that gets pasted into a slide, and
it outlives the console line that was printed next to it. A rendered run that
does not say PROVISIONAL on its face is the failure NFR-08 exists to prevent.

Two boundaries, both hard.

**Nothing is drawn that is not in the state.** In particular no track. Track
geometry is `slipx_scene` in P3, and inventing a track before that component
exists is the error the project refuses everywhere else. A viewer that draws a
plausible kerb is asserting something about a track that does not exist.

**The output is a file, never a window.** No display server, no GPU, no event
loop, no interactive backend. This is what keeps NFR-04 true without a test for
it.

Timing: after the L2 tier is assembled, not before. The first thing the viewer
renders should be a real L2 run rather than an L1 one, and regenerating
`docs/racing/assets` from `slipx_core`, already a standing task and already
blocked on L2, wants the same axis and SVG-writing code. Building the renderer
first means writing that code twice or writing it against the wrong tier.

## Consequences

The distribution grows a rendering surface, and a rendering bug becomes
release-blocking in a way it would not be in a separate repository. That is the
price of the testability that made the separate repository unattractive, and it
is paid on purpose.

Every change to the state layout now touches the viewer. CORE-07 is exactly such
a change and lands before the viewer does; every later one has to keep it
compiling. This is the coupling that was wanted: it means the viewer cannot
quietly rot against a core that moved.

The README's "Not in scope" paragraph gains a sentence distinguishing a
diagnostic picture from a camera model, because on its own the photorealism
sentence reads as covering both. SRS §1.5 gains the same distinction and §5.8
gains the component.

`slipx.viz` is Python-only and therefore invisible to a C++ embedder, who is the
primary customer under the "library is the product" thesis. That embedder gets
`state(i)` and no picture. This is accepted: rendering in the core would need
I/O, which ADR-0003 forbids outright, so the alternative is not a C++ viewer but
no viewer.

Reversing this means either deleting the module or promoting it to its own
repository. The second is a real possibility rather than a hedge: if the viewer
acquires interaction, a timeline or a comparison mode, it has outgrown the
package, and the P3 event stream is the interface it should consume when it
goes. That would need a new record superseding this one, and it should say what
changed about the size of the thing.
