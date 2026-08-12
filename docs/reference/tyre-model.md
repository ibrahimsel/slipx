# The MF-lite tyre model: derivation

What SlipX's tyre is, where every term comes from, and what was dropped. This
is the derivation, not a tutorial: for the concepts behind it, read the racing
series ([tyres and grip](../racing/01-tyres-and-grip.md), [fitting a tyre
model](../racing/07-fitting-a-tyre-model.md), [combined
slip](../racing/09-combined-slip.md)). Worked numbers throughout are the
reference car's, whose parameters are **provisional**.

The implementation is `slipx/tyre.hpp`, header-only and pure. Every equation
below appears in it.

## 1. What the model has to do, and what it is allowed to cost

The design constraint is not accuracy. It is this: **every parameter must be
identifiable from a manoeuvre drivable in a car park, with the sensors already
on a competition car** (wheel encoders, an IMU, LiDAR-derived pose). A
parameter nobody can measure is worse than an absent one, because it gets
guessed, and a guessed parameter is a fitted parameter with the error hidden.

The full Magic Formula (Pacejka, *Tyre and Vehicle Dynamics*, 3rd edition,
Butterworth-Heinemann 2012) has upwards of fifty coefficients per tyre for
pure and combined slip, and identifying them properly needs a tyre test rig
with a flat track and a load cell. There is no such rig in a university
corridor, and there never will be for a 1/10-scale foam tyre.

So MF-lite keeps the parts of the Magic Formula that decide whether the car
makes the corner, and drops the parts that need a rig.

| Kept | Why |
|---|---|
| The shape function, with a genuine peak and a falling branch | The falling branch is the mechanism by which a car spins. A model without one recovers instantly from any slide, which real cars do not do. |
| Load sensitivity of the friction coefficient | It is why load transfer costs grip rather than merely moving it about, and it is identifiable from a skidpad at two ballast settings. |
| A friction ellipse for combined slip | It bounds the total force correctly, which is what decides whether the corner is made. |
| Relaxation length | It is the difference between a step steer that looks right and one that responds a tenth of a second too early. |

| Dropped | Why |
|---|---|
| Camber, turn slip, ply steer, conicity | Not identifiable in a car park. |
| Self-aligning moment | Needs a steering-torque measurement no 1/10 car has. |
| Pacejka's scaling factors | They exist to adapt a rig-fitted set to a new condition; there is no rig-fitted set to adapt. |
| Full combined-slip weighting functions | They move the peak lateral force to a different slip angle under braking. That is real, and it needs data nobody here can take. The ellipse gets the bound right and says so. |

## 2. The shape function

The Magic Formula's shape term, dimensionless:

```
s(x) = sin(C · atan(B·x − E·(B·x − atan(B·x))))
```

Three properties matter, and the C++ tests assert all three rather than
trusting this page:

- **`s` is odd.** The tyre behaves identically left and right. A vehicle model
  built on it has an exact mirror symmetry, and that symmetry is a test: a
  left-hand and a right-hand version of a manoeuvre must mirror to the last
  bit, which catches a wrong reduction order in the force summation.
- **`|s| ≤ 1`**, so the force never exceeds `μ·Fz`. This is a bound the curve
  approaches smoothly and reaches exactly once, not a clip applied afterwards.
- **`s` has a peak at finite slip and falls beyond it**, provided `C > 1` and
  `E < 1`.

`B` scales the input, `C` decides how far the curve turns over, `E` bends the
region around the peak.

## 3. The lateral force law and its sign

```
Fy = −μ_y(Fz) · Fz · s(α)
```

The minus sign is ISO 8855, and it is the whole sign convention: a positive
slip angle means the wheel's velocity vector lies to the **left** of the wheel
plane, and the tyre force opposes it. `conventions.hpp` is normative and
`test_conventions.cpp` asserts it. Getting this sign wrong produces a car that
diverges instead of settling, which is at least loud.

## 4. Load sensitivity

A tyre's friction coefficient falls as it is pressed harder into the road:

```
μ(Fz) = μ_0 · (Fz / Fz_nom)^(−k_μ)
```

`k_μ` is identifiable: drive a skidpad at two ballast configurations and the
difference in peak lateral acceleration gives it. That is the whole admission
argument for the parameter.

The peak force is better written grouped:

```
μ(Fz)·Fz = μ_0 · Fz_nom^k_μ · Fz^(1−k_μ)
```

because `μ(Fz)` diverges as `Fz → 0` while the product does not, and a lifted
wheel has exactly zero load. At L2 a wheel at the rollover threshold is
clamped to zero load, so this is not a hypothetical: written the first way it
would produce `∞ × 0 = NaN` and poison the run.

**Worked, on the reference car's front tyre** (`μ_y0 = 1.10`, `k_μ = 0.15`,
static per-tyre load 8.58 N):

| Load | Peak lateral force | Effective μ |
|---|---|---|
| 8.58 N (nominal) | 9.44 N | 1.100 |
| 17.17 N (double) | 17.02 N | 0.991 |

Doubling the load buys 80% more grip, not 100%. That 20% shortfall is the
entire reason load transfer is expensive: the outer tyre gains less than the
inner one loses.

**Worked, on the car in a corner.** At 6.13 m/s² (0.62 g) the reference car
transfers 2.68 N per side across a 0.24 m track:

| | Load | Peak lateral force |
|---|---|---|
| inner | 5.90 N | 6.87 N |
| outer | 11.26 N | 11.90 N |
| **pair** | 17.17 N | **18.76 N** |
| no transfer | 8.58 N each | 18.88 N |

The axle has lost 0.6% of its lateral capability by leaning on one tyre. A
single-track model cannot see that loss at all, which is precisely where L1
and L2 begin to disagree; `examples/02_where_the_tiers_disagree.py` measures
the crossing.

## 5. Where B comes from, and why it is not in the parameter file

`B` is derived at construction, never read from a file:

```
B = C_α / (C · μ_y0 · Fz_nom)
```

The argument is the identifiability rule applied to itself. The cornering
stiffness `C_α` is measurable: it is the low-slip slope of a skidpad. `B` on
its own is not, because only the product `B·C·μ·Fz` has an observable meaning.
Asking a contributor for both `B` and `C_α` is asking them to measure the same
quantity twice and then reconcile the two answers.

Deriving it buys a second thing, and this one is load-bearing for the tier
system. Expanding the shape function for small `x` gives `s(x) ≈ C·B·x`, so

```
Fy ≈ −C_α · α
```

which is exactly L1's linear tyre. **L2 agrees with L1 in the low-slip limit
by construction rather than by luck.** The cross-tier convergence test
therefore measures a discretisation error and nothing else; if the two tiers
disagreed at small slip, there would be no way to tell a modelling difference
from a parameter mismatch.

**Worked** (reference front tyre, `C_α = 210 N/rad` per tyre, `C = 1.68`,
`μ_y0 = 1.10`, `Fz_nom = 8.58 N`):

```
B = 210 / (1.68 × 1.10 × 8.58) = 13.24
```

and the two laws agree as they should near the origin:

| α [rad] | MF-lite Fy [N] | Linear −C_α·α [N] | Error |
|---|---|---|---|
| 0.001 | −0.2100 | −0.2100 | 0.02% |
| 0.005 | −1.0457 | −1.0500 | 0.41% |
| 0.010 | −2.0660 | −2.1000 | 1.62% |
| 0.020 | −3.9440 | −4.2000 | 6.09% |

`B` is held fixed as the load changes, so the cornering stiffness inherits the
load sensitivity of `μ` for free:

```
C_α(Fz) = B·C·μ(Fz)·Fz = C_α(Fz_nom) · (Fz / Fz_nom)^(1−k_μ)
```

At double load that is 378.5 N/rad rather than the 420 N/rad a linear scaling
would give. This degressive behaviour is what a real tyre does, and it arrives
without a second load-sensitivity parameter to identify.

### The nominal load is the car's static load

`Fz_nom` is the **static per-tyre load of the car wearing the tyre**, computed
from mass and weight distribution, rather than a `nominal_load` field from the
tyre file. Two reasons: the core needs no extra parameter for it, and the
static case is what every other result is compared against, so the cross-tier
comparison starts from exact agreement rather than nearly-exact agreement.

## 6. Where the peak lands: the trap in C and E

Deriving `B` fixes the slope at the origin. `μ` fixes the height of the peak.
So `C` and `E` are left deciding exactly one thing between them: **how far out
the peak sits**.

The natural scale is the slip angle at which the linear tyre would have
reached the peak:

```
α_lin = μ_y · Fz / C_α
```

The actual peak is a multiple of `α_lin` that depends on `C` and `E` alone,
not on `B`, `μ` or the load. For a real tyre that multiple is between about
1.5 and 3.

The trap: the multiple is extremely sensitive to the pair, and the schema
bounds them independently. `C = 1.05` with `E = 0.87` is legal and gives a
multiple near 21, which is a curve whose peak sits at a slip angle no car
reaches. The `C > 1` bound was meant to exclude "a curve with no peak", and on
its own it does not exclude "a curve whose peak is unreachable". The loader
therefore warns above a multiple of 4.

**Worked** (reference front tyre):

```
α_lin = 1.10 × 8.58 / 210 = 0.0450 rad = 2.58°
peak at 0.1211 rad = 6.94°,  multiple 2.69
```

which is a tyre that reaches its limit where a tyre should. It was not always:
before the reference car's cornering stiffness was corrected, the same
arithmetic put the peak at 24.3°, and the falling branch was unreachable in
every figure the project had drawn. The correction moved twelve published
trajectory hashes, and that cost is why this section exists.

## 7. Longitudinal force and the slip ratio

The longitudinal branch uses the same shape function with a slip stiffness
`C_κ` in place of `C_α` and a slip ratio in place of a slip angle. One value
serves all four tyres rather than a front and a rear, deliberately: the
manoeuvre that identifies it is a straight-line acceleration run, which
measures the whole car's longitudinal response and cannot separate the axles.
A parameter split finer than the measurement that produces it is exactly the
error this design is arranged to avoid.

`C_κ` is identifiable in a car park from encoder slip ratio against IMU
longitudinal acceleration. It entered the core before the schema had a field
for it, so a tyre file at schema 0.1.0 has no `c_kappa`; the loader **refuses
to build L2 from such a file** and names the missing field rather than
defaulting it. Schema 0.2.0 adds it.

## 8. Combined slip: the friction ellipse

A tyre has one contact patch and one friction budget, so longitudinal and
lateral force compete for it. The budget is an ellipse rather than a circle
because `μ_x` and `μ_y` differ:

```
d = sqrt((Fx / Fx_max)² + (Fy / Fy_max)²)
d ≤ 1:  the tyre delivers what was asked
d > 1:  both components are scaled by 1/d
```

Scaling **both** by the same factor makes this a projection along the demand
direction rather than a clip of each axis separately. The ratio `Fx : Fy` is
preserved, so a driver braking and steering together loses grip in both at
once, instead of losing all the steering first, which is what an axis-wise
clip does and is the reason this is one function rather than two clamps at the
call sites.

`Fx_max` and `Fy_max` are the peak force magnitudes at that wheel's **current**
load, so the ellipse shrinks and grows with load transfer within the step.

**The limitation, plainly:** an ellipse is a friction budget, not a
combined-slip tyre model. A full Magic Formula derives the interaction from
weighting functions of both slips, which reproduces the way peak lateral force
migrates to a different slip angle under braking. This does not. What it gets
right is the bound, and the bound is what decides whether the car makes the
corner.

## 9. Relaxation

A tyre does not develop lateral force instantly: the carcass has to deform,
and that takes distance rolled, not time. The lag is first-order in distance,
so the time constant is `σ / v` and a fast car responds sooner than a slow
one, in seconds, for the same tyre.

SlipX lags the **slip angle**, not the force. Lagging the force would produce
a force that was not bounded by the friction budget it was generated under,
which means a lifted wheel could still be reporting grip a moment after it
lifted. Lagging the slip angle keeps every force the output of the current
budget. `slipx/relaxation.hpp` carries the derivation.

`σ` is identifiable from the rise time of yaw rate in a step steer driven at
two speeds: one speed cannot separate the tyre lag from the vehicle's own yaw
dynamics, and two speeds can, because only the tyre term scales with `1/v`.

## 10. What identifies what

Every parameter, and the manoeuvre that produces it. This table is the
admission test: a row with no manoeuvre does not get a parameter.

| Parameter | Symbol | Identified by |
|---|---|---|
| Cornering stiffness | `C_α` | Low-slip slope of a skidpad, per axle |
| Peak lateral friction | `μ_y0` | Skidpad at the limit |
| Peak longitudinal friction | `μ_x0` | Straight-line acceleration and braking at the limit |
| Load sensitivity | `k_μ` | Skidpad at two ballast configurations |
| Shape and curvature | `C`, `E` | Where the peak sits in a ramp-steer sweep |
| Slip stiffness | `C_κ` | Encoder slip ratio against IMU acceleration |
| Relaxation length | `σ` | Yaw-rate rise time in a step steer, at two speeds |

None of these needs a dyno, a tyre rig or a force platform, and none ever
will: that constraint is what makes a contributed parameter set possible.

## Sources

- H. B. Pacejka, *Tyre and Vehicle Dynamics*, 3rd edition,
  Butterworth-Heinemann, 2012. Chapter 4 is the Magic Formula; the notation
  here follows it.
- E. Bakker, L. Nyborg, H. B. Pacejka, "Tyre Modelling for Use in Vehicle
  Dynamics Studies", SAE 870421, 1987. The original formulation.
- W. F. Milliken and D. L. Milliken, *Race Car Vehicle Dynamics*, SAE, 1995.
  Chapter 2 for load sensitivity, chapter 14 for the friction ellipse as a
  design tool.

## The decisions behind this

- MF-lite over full Pacejka, and the identifiability rule:
  [ADR-0009](../adr/0009-mf-lite-over-full-pacejka.md)
- Tyres as `(compound, surface)` pairs:
  [ADR-0010](../adr/0010-tyres-are-compound-surface-pairs.md)
- Deriving `B` rather than reading it:
  [ADR-0023](../adr/0023-mf-lite-derives-b-from-cornering-stiffness.md)
- `C_κ` entering the core ahead of the schema:
  [ADR-0025](../adr/0025-c-kappa-enters-the-core-ahead-of-the-schema.md)
- Lagging the slip angle rather than the force:
  [ADR-0026](../adr/0026-relaxation-lags-the-slip-angle-not-the-force.md)
- Why the reference tyre's stiffness was corrected:
  [ADR-0032](../adr/0032-the-reference-tyre-peaks-where-a-tyre-peaks.md)
