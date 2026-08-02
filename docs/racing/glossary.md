# Glossary

Terms as they are used in this series and in autonomous racing generally.
Where an article covers a term properly, it is linked; where a term is common
enough to meet before its article exists, the definition here stands alone.

All signs follow ISO 8855: x forward, **y to the left**, z up, yaw positive
counter-clockwise. Sources using SAE put y to the right and z down, which flips
the sign of slip angle and lateral force together.

---

**Ackermann angle**. The steering angle a corner needs from geometry alone,
`L / R`, ignoring tyre slip. The starting point of the
[steady-state cornering equation](04-understeer-and-oversteer.md).

**Ackermann steering geometry**. The linkage arrangement that makes the inner
front wheel steer more than the outer, so both point at the same turn centre.
Separate idea from the Ackermann angle, confusingly.

**Apex**. The point where a racing line comes closest to the inside of a
corner. A *geometric* apex sits at the middle of the corner; a *late* apex sits
further round. See [the racing line](05-the-racing-line.md).

**Body slip angle** (also **sideslip angle**). The angle between where the car
is pointing and where its centre of gravity is actually travelling,
`atan2(v_y, v_x)`. Distinct from a tyre's slip angle, and usually smaller.

**Camber**. The tilt of a wheel from vertical, seen from the front. Generates
a small lateral force of its own. Ignored throughout this series and in most
1/10-scale work.

**Characteristic speed**. For an understeering car, `sqrt(L/K)`: the speed at
which it needs twice the Ackermann angle. A way of quoting how strongly a car
understeers. See [understeer and oversteer](04-understeer-and-oversteer.md).

**Contact patch**. The area of tyre actually touching the road. About a
thumbnail on a 1/10-scale car. Everything the car does happens through four of
these.

**Cornering stiffness** (`C_alpha`). The slope of the lateral force curve at
zero slip angle, in newtons per radian, always quoted positive. The single most
useful number about a tyre. See [tyres and grip](01-tyres-and-grip.md).

**Critical speed**. For an oversteering car, `sqrt(-L/K)`: the speed above
which the car is **unstable** and must be actively stabilised. Has no analogue
for an understeering car.

**Double track model**. A model with all four wheels represented separately,
so load transfer and per-corner tyre behaviour exist. See
[vehicle models](03-vehicle-models.md).

**Downforce**. Aerodynamic load pressing the car onto the road, increasing
grip with speed. Negligible at 1/10 scale below roughly 15 m/s, and the main
reason full-size racing intuitions do not transfer directly.

**Friction ellipse** (loosely, **friction circle**). The closed curve bounding
the combined longitudinal and lateral force one tyre can make. Braking and
cornering spend the same budget. See [tyres and grip](01-tyres-and-grip.md).

**g-g diagram**. The friction ellipse at vehicle level: every combination of
longitudinal and lateral acceleration the car can reach. See
[speed and the g-g diagram](06-speed-and-the-gg-diagram.md).

**Kinematic bicycle model**. The four-state model that merges each axle into
one wheel and assumes no tyre slip at all. Cheap, has no friction limit, and is
the standard internal model for MPC.

**Load sensitivity**. The fall of peak friction coefficient as vertical load
rises, usually written `mu = mu_0 (Fz/Fz_nom)^(-k_mu)`. The reason
[load transfer](02-load-transfer.md) costs a car grip rather than merely moving
it around.

**Load transfer**. The redistribution of vertical load between tyres under
acceleration, caused by inertia acting at the centre of gravity while the
reaction acts at the road. Not caused by the springs.

**Magic Formula**. Pacejka's empirical tyre model, `Fy = D sin(C atan(B a - E
(B a - atan(B a))))`. Called magic because it is a fit with no derivation. See
[tyres and grip](01-tyres-and-grip.md).

**MPC** (model predictive control). Control by repeatedly solving a short-horizon
optimisation over a vehicle model. Popular in racing because constraints such as
the friction ellipse and the track boundary can be stated directly.

**Neutral steer**. `K = 0`: the car needs the Ackermann angle at every speed.

**Oversteer**. `K < 0`: the car needs *less* steering as it corners harder, and
above the critical speed it is unstable. At the limit, the rear axle saturating
first.

**Pure pursuit**. A path-tracking controller that steers towards a point a
fixed distance ahead on the path. Simple, robust, needs almost no model, and is
where most F1TENTH teams start. Introduced in Coulter, "Implementation of the
Pure Pursuit Path Tracking Algorithm", Carnegie Mellon Robotics Institute
technical report, 1992.

**Relaxation length** (`sigma`). The rolling distance a tyre needs for lateral
force to reach about 63 per cent of its steady value. A few centimetres, giving
a lag of `sigma/v`.

**Rollover threshold**. The lateral acceleration at which the inside wheels
reach zero load, `g t / (2h)` for a rigid car. Depends only on track and CoG
height: not on mass, not on weight distribution.

**Roll stiffness distribution**. How the total lateral load transfer divides
between the front and rear axles on a car with suspension. The main tool for
adjusting limit balance on a full-size car, and largely absent at 1/10 scale
where suspension travel is short or nonexistent.

**Slip angle** (`alpha`). The angle between where a wheel points and where it
is travelling. Force is a consequence of this, never of steering angle on its
own. See [tyres and grip](01-tyres-and-grip.md).

**Slip ratio** (`kappa`). The longitudinal equivalent, the fractional
difference between wheel surface speed and road speed. Positive under drive,
negative under braking.

**Stanley controller**. A path tracker steering on cross-track error and
heading error at the front axle, from the Stanford DARPA Grand Challenge entry.
See Thrun et al., "Stanley: The robot that won the DARPA Grand Challenge",
*Journal of Field Robotics*, 2006.

**Static stability factor**. `t / 2h`, the rollover threshold in units of g.
Quoted for road cars in safety ratings.

**Trail braking**. Carrying braking into the corner and releasing it as
steering is added, so the combined demand stays on the g-g boundary rather than
tracing a cross through the middle.

**Understeer**. `K > 0`: the car needs *more* steering as it corners harder.
Stable, and what road cars are deliberately built to do. At the limit, the front
axle saturating first.

**Understeer gradient** (`K`). The coefficient in `delta = L/R + K a_y`, in
radians per m/s². A property of the car, not of the corner or the speed. See
[understeer and oversteer](04-understeer-and-oversteer.md).

**Unsprung mass**. Wheels, tyres and hubs: the mass not carried by the
suspension. Matters for ride and for load transfer through the wheels rather
than the body.

**Yaw rate** (`r`). How fast the car is rotating about its vertical axis, in
rad/s. Positive counter-clockwise, so positive in a left turn.

---

[Series index](README.md)
