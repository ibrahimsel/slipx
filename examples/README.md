# Examples

Three programs, in the order they are worth reading. Each one is executed by
the test suite rather than proofread, so what they print is what they print.

None of them needs an extra installed, and none needs this checkout: the
reference car ships inside the package, so an installed wheel is enough.

```
pip install slipx
python3 examples/01_load_a_car_and_drive_it.py
```

| File | What it is for |
|---|---|
| [`01_load_a_car_and_drive_it.py`](01_load_a_car_and_drive_it.py) | Load a car directory, step a model, read the state and the diagnostics. The shortest complete thing SlipX does. |
| [`02_where_the_tiers_disagree.py`](02_where_the_tiers_disagree.py) | The same manoeuvre at L0, L1 and L2, and a measurement of how far up the lateral-acceleration range the cheap model still answers for the expensive one. |
| [`03_record_a_run.py`](03_record_a_run.py) | Run a slalom under the orchestrator, record it, and write a self-contained animated SVG. Writes a file, never opens a window. |

The car they all load is `cars/reference_1_10`, whose parameters are
**provisional**: plausible for a 1/10-scale competition chassis, measured
against no vehicle. Every example prints that label rather than leaving it to
this file.

## The car directory

```
cars/reference_1_10/
  car.yaml          identity, geometry, mass, which tyres each axle runs
  dynamics.yaml     drivetrain, ESC, battery, steering servo
  limits.yaml       travel and command bounds
  sensors.yaml      sensor placement, for the layers that arrive in P1
  provenance.yaml   where every number came from, and what it is not
  tyres/
    sponge_carpet.yaml    one (compound, surface) pair
```

A tyre is referenced as a `(compound, surface)` pair and never embedded in the
car file, because the same chassis on carpet and on asphalt is the same car.

Copy the directory to describe your own car. The loader refuses rather than
defaults: a parameter it cannot fill produces an error naming the field and
where it should come from, so a missing number is never quietly replaced by a
plausible one.
