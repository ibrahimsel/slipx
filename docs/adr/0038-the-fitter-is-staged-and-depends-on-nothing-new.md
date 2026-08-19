# ADR-0038: The fitter is staged, fits what each manoeuvre identifies, and depends on nothing new

- **Status:** Proposed
- **Date recorded:** 2026-08-19 (decision taken at the start of P2)
- **Requirements:** ID-01 to ID-04 in spirit; `docs/spec` is not present in
  this checkout, so no ID is cited as authority.
- **Related:** [ADR-0003](0003-dependencies-point-downward.md),
  [ADR-0009](0009-mf-lite-over-full-pacejka.md),
  [ADR-0013](0013-provenance-labels-are-printed.md),
  [ADR-0016](0016-one-distribution-two-packages.md)

## Context

P2 needs `slipx_id`: something that turns a recording of the manoeuvre
library (`docs/identification/`) into a parameter set with residuals,
confidence intervals and a provenance block. Three questions had to be
settled before the first line, because each one is expensive to reverse:
where the fitter lives, what it may depend on, and whether it is one joint
optimisation or a sequence of small ones.

**Where it lives.** The options were a submodule of the `slipx` package, a
separate PyPI distribution, or a third package inside the existing
distribution. A submodule would let bindings code grow fitting logic and
vice versa with no boundary to stop it. A separate distribution would mean
`pip install slipx` does not include the tool whose output the registry
depends on, and the contribution flow is supposed to be a by-product of
having SlipX installed, not a second install.

**Dependencies.** The obvious fitter dependency is scipy. A hard scipy (and
therefore numpy) dependency would have to resolve on every wheel of the
ADR-0018 matrix, would put a large numerical stack under a tool whose whole
job is producing four dozen floats, and would contradict the thesis of a
library whose core depends on nothing. The problems here are small: never
more than eight simultaneous parameters, thousands of residuals, dense
matrices that fit on a page.

**One fit or many.** A single joint optimisation over every parameter against
every recording is the academically tidy option, and it is exactly what the
manoeuvre library was designed to avoid: at low speed the resistive,
drivetrain and tyre forces are the same order of magnitude, and a joint fit
trades them off against one another freely. The library's ordering exists
because each manoeuvre pins parameters the next one then consumes as known.

## Decision

**`slipx_id` is a third importable package in the `slipx` distribution**, at
`src/tooling/slipx_id`, below nothing and depending downward on `slipx` and
`slipx_schema` only. `tools/dep_lint.py` gains it at the top of the order.
Nothing in the library may import it: the fitter is a consumer of the public
API, and anything it needs that the API cannot give is an API gap to fix, not
a private hook to add.

**The optimiser is written here, in plain Python, on the standard library.**
Levenberg-Marquardt with finite-difference Jacobians, normal equations by
Cholesky, fixed iteration order, fixed relative step sizes, no randomness
anywhere: the same inputs produce the same fit to the last bit, which is a
property scipy does not promise across versions and this project will not
give up. Linear sub-problems (coastdown, slip stiffness) are solved in closed
form rather than iterated at all. Confidence comes from the Gauss-Newton
covariance at the optimum, and parameter correlations above a reporting
threshold are named in the output, because "these two numbers cannot be told
apart from this data" is a finding, not a failure.

**The fit is staged in the manoeuvre library's order**, each stage consuming
earlier results as constants: coastdown gives the resistances; straight-line
acceleration gives the ESC curve, `c_kappa` and `mu_x0`; skidpad and ramp
steer give the lateral curve through algebraic reconstruction (steady-state
force balance for axle forces, pose-derived slip angles, candidate tyres
evaluated through `make_mf_lite` at the measured loads); the step-steer
family is fitted by replaying the recorded commands through the forward
model, using the orchestrator's input-log playback and the recording
machinery rather than a second integration path.

**The synthetic self-test comes first.** Data generated from known parameters
through the forward model must round-trip: fit and recover every MF-lite
parameter within stated tolerances, in CI, before any bag parsing exists. It
is the only test of the fitter that needs no hardware, so it is the
foundation, not the afterthought.

**The Magic Formula shape pair is identified as a curve, not as
coordinates.** Building the self-test established this rather than assuming
it: steady-state driving cannot sit on the falling branch of the tyre curve
(a car balanced past the peak departs), and MF-lite's falling branch is
gentle, so whole families of (C, E) describe nearly the same force over the
reachable range and no car-park data can pick between them. The lateral
stage therefore reports the C-E correlation by name, the transient stage
carries the pair into its replay (a step past the limit rides the falling
branch for the few tenths of a second the replay can read), and what the
self-test asserts about the pair is that the fitted force curve tracks the
true one through the working range and under load transfer. Three
implementation facts the self-test forced, recorded so nobody rediscovers
them: the ramp sweep must be triangular (wind in and out), because the
tyre's relaxation lag shifts a one-way sweep's whole curve sideways and the
two branches cancel it; the reconstruction's yaw-moment balance must carry
the couple from the steered wheels' induced-drag asymmetry, which is
exactly the size of the shape-factor signal; and the lateral prediction
mirrors the model's fixed two-pass load evaluation (ADR-0027) rather than
feeding the measured acceleration into the load law, because the loads the
model used came from its first pass.

## Consequences

- A hand-rolled optimiser is code this project must now maintain, and it will
  be slower than scipy's. Both costs are accepted: the problems are small,
  and the optimiser's determinism is inspectable in a way a dependency's is
  not. If a future fit outgrows it, the escape hatch is an optional extra,
  never a hard dependency.
- Staged fitting inherits the manoeuvre library's blind spots. A parameter no
  stage identifies stays `provisional`, and the fitter says so rather than
  letting a joint fit smear a number into it. `k_mu` carries the widest
  interval by design (the ballast signal is a 1.4 per cent effect), and the
  output owns that.
- The distribution gains a package and a console script, which is a public
  API surface to keep stable from 1.0.0.
- Algebraic reconstruction in the lateral stage assumes steady state; data
  that is not steady (a rushed ramp, a wobbling skidpad) degrades the fit.
  The stage reports per-band residuals so that shows up as numbers rather
  than as a silently worse tyre.
