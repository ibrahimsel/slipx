# Autonomous racing, from the ground up

A tutorial series on the concepts behind autonomous racing at 1/10 scale: what
a slip angle is, why load transfer costs you grip, what people mean by
understeer, and why the fastest way round a corner is not the shortest.

**This is not a manual for SlipX.** It is a guide to the subject SlipX happens
to be a library for. You should be able to read the whole series, learn
something useful, and never install anything. Where a concept connects to
something concrete in the library there is a short aside marked like this:

> **In SlipX.** Asides look like this. They are optional and they are never
> load-bearing for the explanation around them.

## Who it is for

Somebody who can program, is comfortable with school mechanics and a bit of
trigonometry, and has now been handed a small autonomous racecar and a lot of
vocabulary nobody defined. That covers most people arriving at RoboRacer or
F1TENTH from a computer science or robotics background, which is most people
who arrive at all.

It assumes no vehicle dynamics. It does assume you would rather have the
reason than the recipe, so formulae come with their derivation sketched and
their assumptions named.

## Reading it

The first four articles are one argument in four parts and are best read in
order. The rest stand alone.

**Part one: the car**

| | Article | What it answers |
|---|---|---|
| 1 | [Tyres and grip](01-tyres-and-grip.md) | Where does cornering force actually come from? |
| 2 | [Load transfer](02-load-transfer.md) | Why does the car's grip change while you drive it? |
| 3 | [Vehicle models](03-vehicle-models.md) | Which model is allowed to answer which question? |
| 4 | [Understeer and oversteer](04-understeer-and-oversteer.md) | What do people mean by "balance"? |
| 9 | [Combined slip](09-combined-slip.md) | What happens when you brake and turn at once? |
| 10 | [Differentials](10-differentials.md) | Why does a driven axle need one, and what does the choice cost? |
| 11 | [The motor, the ESC and the battery](11-motor-esc-and-battery.md) | Where does acceleration actually come from? |

Articles 9 to 11 follow straight on from articles 1 and 2 and can be read there
instead of here; they are numbered by when they were written, not by where they
sit.

**Part two: the racing**

| | Article | What it answers |
|---|---|---|
| 5 | [The racing line](05-the-racing-line.md) | Why is the shortest path not the fastest? |
| 6 | [Speed and the g-g diagram](06-speed-and-the-gg-diagram.md) | How fast can I go, and where do I brake? |
| 17 | [A collision is an impulse](17-a-collision-is-an-impulse.md) | What does touching another car actually do? |
| 18 | [The same race twice](18-the-same-race-twice.md) | Why does a simulation give a different answer each run, and what does it take not to? |
| 19 | [Why simulated cars fall through walls](19-why-simulated-cars-fall-through-walls.md) | What does hitting a wall do, and what stops a car passing through one? |

**Part three: getting the numbers**

| | Article | What it answers |
|---|---|---|
| 7 | [Fitting a tyre model](07-fitting-a-tyre-model.md) | Where do the coefficients in a tyre model come from? |
| 14 | [System identification](14-system-identification.md) | How does driving a car turn into numbers for its model? |
| 15 | [Residuals and confidence](15-residuals-and-confidence.md) | How much should I believe a fitted number? |
| 16 | [Validation by replay](16-validation-by-replay.md) | What test does a fitted model have to pass? |

Articles 14 to 16 are one argument in three parts: what a fit is, how it
reports its own reliability, and the exam it has to pass afterwards.

**Part four: beyond steady state**

Articles 1 to 6 describe a car in a settled condition. These are about what
happens while it is still settling, which is most of a lap.

| | Article | What it answers |
|---|---|---|
| 8 | [Tyre relaxation](08-tyre-relaxation.md) | Why does grip arrive late, and why does that change with speed? |
| 12 | [Actuator lag](12-actuator-lag.md) | Why is the steering angle not the one you commanded? |
| 13 | [A scan is not a snapshot](13-a-scan-is-not-a-snapshot.md) | Why does a laser scan taken while moving show a shape the world does not have? |

Articles 8 and 12 are a pair: one delay that shrinks as the car speeds up and
one that does not. Article 13 is the third of the same kind, and the one that
bites hardest: a sensor whose measurement is smeared over time rather than
merely late.

**Reference**

- [Glossary](glossary.md), for when a word turns up before its article does.

**Planned, not yet written.** Listed so the shape of the series is visible and
so nobody writes the same article twice: the autonomy pipeline; localisation and
mapping; global and local planning; path tracking, from pure pursuit to MPC;
inertial and odometric sensor errors, which article 13 only touches; and the
sim-to-real gap. Contributions welcome, and the articles above are the register to
match.

## Conventions

Everything here follows the same conventions as the library, which are the
international standard ones rather than house style.

- **Axes and signs: ISO 8855.** x forward, **y to the left**, z up. Yaw is
  positive counter-clockwise seen from above, so a left turn increases it, and
  a positive steering angle is a left turn.
- **Units: SI, without prefixes.** Metres, kilograms, seconds, radians,
  newtons. Not degrees, not millimetres, not km/h. Slip angles are quoted in
  degrees in a few places because that is how people talk about them, and it
  is said explicitly each time.
- If a sign in a textbook disagrees with one here, check whether the book uses
  SAE. SAE puts y to the right and z down, which flips the sign of the slip
  angle and of the lateral force together. Both describe the same car.

## About the figures

Every diagram is generated by [`assets/make_figures.py`](assets/make_figures.py).
Run it to regenerate them all.

**Every tyre curve here is evaluated by SlipX itself**, at the parameters of
the reference car that ships with it, so a figure and a run of the library
cannot disagree about what a tyre does. They used to: the script carried its
own Magic Formula, and the two drifted apart without either being obviously
wrong.

That does not make the numbers measurements. The reference car's parameters are
**plausible for a 1/10-scale car and have not been measured on one**, so the
figures illustrate shapes and relationships and no number read off them means
anything about a real vehicle. The schematic figures are geometry and carry no
model at all, and the drivetrain and actuator plots are closed-form expressions
written out in the script, because those quantities are not tyre curves. The
distinction is marked in the script.

## Further reading, in general

Three books cover most of what the first four articles touch, in increasing
order of specialism:

- Rajamani, *Vehicle Dynamics and Control*, 2nd ed., Springer, 2012. The most
  accessible if you come from control. Chapters 2 and 13 are the bicycle model
  and the tyre model.
- Milliken and Milliken, *Race Car Vehicle Dynamics*, SAE International, 1995.
  The standard reference for the racing side. Long, discursive, and worth
  owning.
- Pacejka, *Tyre and Vehicle Dynamics*, 3rd ed., Butterworth-Heinemann, 2012.
  The Magic Formula from the person it is named after. Reach for it when you
  need the real thing rather than a reduced version.

For the autonomous racing field specifically, start with the survey: Betz,
Zheng, Liniger, Rosolia, Karle, Behl, Krovi and Mangharam, "Autonomous Vehicles
on the Edge: A Survey on Autonomous Vehicle Racing", *IEEE Open Journal of
Intelligent Transportation Systems*, 2022. It is open access and its reference
list is the map of the territory.

For the platform, [f1tenth.org](https://f1tenth.org) hosts build
instructions and a full lecture course, and the
[f1tenth_gym](https://github.com/f1tenth/f1tenth_gym) repository is the
simulator most groups start from.
