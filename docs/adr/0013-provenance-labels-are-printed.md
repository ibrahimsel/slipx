# ADR-0013: Every parameter set carries a provenance label and the tooling prints it

- **Status:** Accepted
- **Date recorded:** 2026-08-02 (decision taken during P0)
- **Requirements:** NFR-08
- **Related:** [ADR-0009](0009-mf-lite-over-full-pacejka.md),
  [ADR-0005](0005-tiers-throw-rather-than-fall-back.md)

## Context

The project's argument is that its parameters are measurable, in contrast to
game engine tyre curves that are not. That argument creates an obligation, and
it is easy to fail without noticing.

No parameter set shipped today has been validated against a real car. The
reference car's geometry and mass are typical of a 1/10-scale competition
chassis and its inertias are computed from a uniform box, scaled. Nothing about
it was measured on a vehicle.

A file like that, sitting in `examples/` and loading without complaint, becomes
a starting point. It gets copied, edited slightly, and cited. Within a few
months somebody is comparing a result against it as though it described
something.

Documenting the limitation in the README is necessary and insufficient. The
README is read once, by a subset of users, before they start.

## Decision

Every shipped parameter set is labelled `measured`, `identified` or
`provisional`, and the label is a required field rather than an optional
annotation. The weakest claim, `provisional`, is the default.

Tooling output prints the label. Not the documentation: the output. `car.summary()`
leads with the provenance line, and the exit gate prints it on every run,
including in CI.

Until an outside contributor supplies a fitted set with a validation report, the
honest phrasing everywhere is "physically structured and identifiable", not
"validated". Collision physics is described as plausible and deterministic, not
fitted to data, and the docs keep saying so.

## Consequences

Every demo, screenshot and CI log carries the word PROVISIONAL. That is
unattractive and it is the point: the label is on the artefact somebody would
otherwise quote.

Promoting a parameter set to `identified` requires the fitter and the validation
report from P2, so the label cannot be upgraded by opinion. P2's exit condition
is three parameter sets in the registry from people who are not us, each with a
report attached, which is the same claim from the other end.

The marketing register of the whole project is constrained by this. "Validated
vehicle dynamics" is a sentence that cannot be written until it is true, and it
would be the easiest sentence in the world to write.
