# slipx_registry

Community-contributed, provenance-carrying parameter sets for
[SlipX](https://github.com/ibrahimsel/slipx). An entry here is a claim other
people will tune against, so every entry says who fitted it, from what data,
with what residuals, and how it validated. Nothing in this repository is a
guess with paperwork: guesses are refused at the door, by CI, by name.

> **Staging note.** This directory is the future `slipx_registry`
> repository's content, staged inside the SlipX tree until it is pushed to
> its own repository (the decision and its date are recorded in SlipX's
> release roadmap). Paths below read as if it stands alone.

## What an entry is

One directory under `cars/`, exactly as `slipx-id` emitted it:

```
cars/<team>__<car>__<surface>/
  car.yaml            the manifest
  dynamics.yaml       mass, geometry, tyre references, drivetrain
  limits.yaml         servo, ESC, battery, bounds
  tyres/*.yaml        the fitted (compound, surface) pairs
  provenance.yaml     who, what data (SHA-256 digested), residuals
  validation.svg      the replay report the acceptance bar requires
```

The recordings themselves stay with the contributor (they are large); the
provenance's `data` block names and digests every bag the fit consumed, so
an entry is tied to its data rather than to a story about it. Add a `url`
per bag when you can host them: a reviewer who can re-run your fit is a
reviewer who can trust it faster.

## Contributing

Contribution is a by-product of fitting, not a separate act:

1. Drive the six manoeuvres of the
   [manoeuvre library](https://github.com/ibrahimsel/slipx/blob/main/docs/identification/README.md),
   recording each as a rosbag2 bag, plus at least one validation run the
   fit will not consume (a slalom works well).
2. Write a session file and run the fitter, which needs no ROS on the
   analysis machine:

   ```
   pip install slipx
   slipx-id session.yaml
   ```

3. Copy the emitted directory into `cars/`, named
   `<team>__<car>__<surface>`, and open a pull request.

CI runs the same checks a reviewer starts from; anything it refuses, it
refuses by name.

## The acceptance bar

Checked by `tools/check_submission.py`, which calls
`slipx_schema.rules.check_registry_submission`, so the bar is code in the
SlipX distribution rather than prose here. An entry must:

- carry the `identified` label with a named contributor. Provisional sets
  are refused outright: the reference car already demonstrates plausible
  numbers, and the registry exists for fitted ones;
- carry per-parameter residuals and confidence intervals. An identified
  parameter without a residual is a provisional parameter with a better
  title;
- attach a validation report (`validation.svg`), the replay of a run the
  fit never consumed, with its divergence stated;
- digest its data: the `data` block naming each bag and its SHA-256;
- load cleanly through `slipx.load_car`. Plausibility warnings do not
  refuse an entry (unusual cars exist), but the reviewer sees them and may
  ask.

Review is by a human with the check's output in front of them. The checks
establish that the story is complete; the reviewer judges whether it holds
together, and convergence of compound and surface naming is review work,
on purpose (SlipX schema 0.4.0 made both a community vocabulary).

## What "validated" means here

An entry's set is validated **on the runs its report shows, and on nothing
else**. SlipX's own claim discipline applies verbatim: the label is
printed by tooling, the report travels with the entry, and nobody
extrapolates a slalom's agreement into a promise about a race.

## Licence

Apache-2.0, the same as SlipX, and for the same reason: entries are meant
to be embedded in other people's work without friction.
