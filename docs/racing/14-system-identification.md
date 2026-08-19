# System identification: numbers from behaviour

Every model in this series has parameters: a mass, a stiffness, a friction
coefficient. Some of them you can put on a scale. Most of them you cannot,
and the only way to get a number is to drive the car and work backwards
from what it did. That working-backwards is called system identification,
and it has rules, because it fails in quiet ways when they are ignored.

## The idea, in one fit

A coasting car obeys a model with two unknowns, rolling resistance and
aerodynamic drag:

```
m dv/dt = −(c_rr m g + c_d v²)
```

Coast a 3.5 kg car through a measuring zone and read the deceleration at
two speeds: 0.55 m/s² at 10 m/s and 0.16 m/s² at 2 m/s. Two equations,
two unknowns:

```
0.55 · 3.5 = c_rr · 3.5 · 9.81 + c_d · 100
0.16 · 3.5 = c_rr · 3.5 · 9.81 + c_d · 4
```

Subtracting: 96 c_d = 1.365, so c_d ≈ 0.0142 kg/m, and back-substituting,
c_rr ≈ 0.0147. Two runs, two numbers, no equipment beyond the sensors the
car carries. Real fits use hundreds of samples and least squares instead
of two equations, but nothing about the idea changes: the model says what
behaviour the parameters imply, and the fit inverts that.

## Excitation, or why the manoeuvre matters

The coastdown works because the two forces separate: rolling resistance
is the same at every speed and drag grows as v², so fast samples measure
mostly drag and slow samples mostly rolling resistance. Coast only at one
speed and the two equations above collapse into one; any pair of numbers
along a line fits it equally well, and a fitting program will happily
print one such pair with great precision.

That is the central discipline of identification: **a parameter is only
identifiable if the data contains behaviour that changes when the
parameter changes**. Nobody learns a tyre's peak grip from gentle
driving, because gentle driving never visits the peak; nobody separates a
motor's torque curve from its current limit without crossing the speed
where one stops binding and the other starts. Test procedures are
designed backwards from this: each manoeuvre exists to excite the
parameters it is meant to pin down, which is why identification uses a
library of specific exercises rather than "drive around for a while".

## Staged fits, or why not one big one

Given six manoeuvres and twenty parameters, the tempting approach is one
grand optimisation over everything at once. It usually fits well and
identifies badly: at low speed the resistive, drivetrain and tyre forces
are all a few newtons, and a joint fit trades them off against one
another freely, arriving at a set of numbers whose sum is right and whose
parts are wrong. Nothing in the residual warns you.

The alternative is to fit in stages, in an order where each stage pins
down parameters the next stage then treats as known. Resistances first,
from coasting, where no tyre force operates. Then the drivetrain, from a
straight line, where the resistances are already known and subtracted.
Then the tyres, from cornering, with everything longitudinal accounted
for. Each fit is small, each fit's data actually excites its parameters,
and an entangled pair has nowhere to hide.

The cost is inherited error: a wrong resistance propagates into every
later stage. That trade is taken with open eyes, because inherited error
is visible (refit the first stage and the rest moves) where joint-fit
smearing is not.

## What cannot be identified

Some parameters resist every manoeuvre. A quantity that only matters in a
regime the car cannot safely reach, or that trades off exactly against
another over the whole reachable envelope, will come back from any honest
fit with a shrug. The professional response is not a cleverer optimiser;
it is to say so: report the parameter as unidentified, keep whatever
provisional value was there, and label it. A guessed number with a fit's
authority behind it is strictly worse than a labelled guess, because
somebody will tune against it in good faith.

> **In SlipX.** `slipx-id` implements exactly this structure over the
> manoeuvre library in `docs/identification/`: staged fits in the
> library's order, refusal to emit unlabelled numbers, and parameters no
> stage identified stay `provisional` with a note saying why. The
> coastdown fit above is the first stage, solved in closed form.

## Further reading

Ljung, *System Identification: Theory for the User*, 2nd ed., Prentice
Hall, 1999, is the standard reference, and its first chapter covers
everything this article says with full rigour. For vehicles specifically,
the identification chapters of Rajamani (see the series index) show the
bicycle-model versions of these fits.
