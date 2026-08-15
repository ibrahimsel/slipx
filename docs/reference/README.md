# Reference

What each thing is, what its units are and which tier can fill it in. This is
the lookup material; it does not teach the subject.

- **Learning vehicle dynamics**: the [racing series](../racing/README.md),
  twelve articles that explain the concepts and mention SlipX only in asides.
- **Running code**: the [examples](../../examples/), three programs that are
  executed by the test suite rather than proofread.
- **Why something is the way it is**: the
  [decision records](../adr/README.md). They are the argument; this directory
  is only the description.

| Page | Contents |
|---|---|
| [Sign conventions and units](conventions.md) | ISO 8855 axes, the sign of a slip angle, wheel ordering, SI throughout. Normative, and asserted in the tests. |
| [`VehicleParams`](vehicle-params.md) | Every parameter, its unit, its sign convention, its default and the tier from which it has any effect. |
| [`VehicleState` and diagnostics](state-and-diagnostics.md) | What a step reads and writes, and the NaN rule for what a tier cannot represent. |
| [The tyre model](tyre-model.md) | The MF-lite derivation: the shape function, load sensitivity, where `B` comes from, the friction ellipse, and what was dropped and why. |
| [The Python API](python-api.md) | Loading a car, stepping a model, the orchestrator, recording a run, the manifest. |
| [Performance](performance.md) | Measured step cost and real-time factors, the machine they were measured on, and which of the three targets are currently missed. |

## Three things that are true everywhere in this directory

**Tiers are chosen, never inferred, and an unimplemented tier raises.** L0 is
a kinematic bicycle, L1 a dynamic bicycle with linear tyres, L2 the
double-track model with load transfer and MF-lite tyres, L3 adds thermal and
suspension effects and is not built. Below L2, CoG height, weight
distribution, differential and tyre compound deliberately have **no** effect
on the trajectory. That is the teaching artefact, not a bug.

**A quantity a tier cannot represent is NaN, never zero.** Zero is a plausible
slip angle and would be believed. A sink delivers that NaN as absent, never as
a plotted zero.

**No parameter set shipped with SlipX has been validated against a real car.**
The honest phrasing for anything built on them is "physically structured and
identifiable", not "validated", and the tooling prints the provenance label
rather than leaving it to the documentation.
