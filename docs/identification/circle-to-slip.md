# Circle-to-slip

**Identifies:** `mu_y0` definitively; `k_mu` with a ballast run; which end
of the car lets go first, which cross-checks the whole lateral fit.
**Needs first:** everything the skidpad and ramp steer produced, and a
ballast weight of about 10 per cent of the car's mass with a way to fix it
over the CoG.
**Space:** 10 m × 10 m for a 3 m circle with runoff on the outside. The car
will leave the circle at breakaway; that is the point.
**Risk:** the highest slide risk in the library, taken deliberately. The
slide is short and low-energy if the ramp is slow; it is long if the ramp is
impatient. Nobody stands outside the circle.

## The idea

Hold the radius fixed and raise the speed so slowly that the car is always
in steady state, and the lateral acceleration walks up the tyre curve until
the curve runs out. The last steady lap before breakaway *is* the limit:

```
a_y,max = v²_breakaway / R
```

On the reference car on a 3 m circle the naive prediction is
v = √(1.1 · 9.81 · 3) = 5.7 m/s. Lateral load transfer and load sensitivity
shave about two per cent off, so breakaway arrives just above 5.6 m/s. The
gap between the naive number and the measured one is exactly the load
sensitivity at work, which is the door `k_mu` comes in through.

## The ballast run, and an honest warning

`k_mu` says how much grip a tyre loses as its load rises. Adding 10 per
cent ballast raises every static load 10 per cent, which lowers the
limit acceleration by about `k_mu` × 10 per cent × the curve's local slope:
on the reference numbers, a 1.4 per cent drop, which at 3 m radius is a
breakaway speed lower by 0.04 m/s.

That is a small signal. It is measurable, because the limit acceleration is
an average over the last several settled laps rather than a single reading,
but `k_mu` will honestly carry the widest confidence interval in the file,
and the fitter says so rather than rounding the doubt away. An
unidentifiable parameter is worse than an absent one; `k_mu` sits at the
edge, and the numbers above are why.

Fix the ballast directly over the CoG, and confirm with the balance-rod
check from the [bench list](README.md) that the balance point has not
moved: ballast that shifts the weight split changes `lf` and `lr` mid-fit,
which is two lies for the price of one.

## Procedure

1. Enter the marked circle at a speed well below the expected limit and
   settle for two laps.
2. Raise the speed by no more than 0.05 m/s per lap. Slower is better; the
   whole run to breakaway takes a few minutes and the last ten laps are the
   data that matter.
3. At breakaway (the car runs wide, or the rear steps out and the yaw rate
   spikes), cut the throttle, let the car straighten, and stop.
4. Three runs each direction. Note which end let go: front first appears
   as the radius growing while yaw rate falls behind `v/R`; rear first as
   yaw rate rising past it.
5. Repeat the whole set with the ballast fitted.

## What to record

The standard bag. The fit takes, per lap: mean speed, mean yaw rate, the
realised radius from the pose, and flags the first lap where the steady
relation `r = v/R` breaks. IMU lateral acceleration averaged over the
settled laps is the limit measurement itself.

## What good data looks like

- The measured radius holding within a few centimetres of the marked circle
  until the last two or three laps.
- Breakaway arriving within a few per cent of the ramp-steer prediction; a
  large disagreement means the surface changed (dust, temperature, a
  different patch of tarmac) between sessions, and friction moved. The
  tyre file records a `(compound, surface)` pair for exactly this reason.
- The ballasted breakaway measurably below the unballasted one, in the
  ratio the fitted `k_mu` then reproduces.

## Failure modes

- **Ramping too fast.** The car carries yaw momentum through the limit and
  breakaway measures the transient, not the curve. If the last stable lap
  and the first broken one differ by more than one speed step, the ramp was
  too coarse.
- **The surface is the variable.** Morning damp, afternoon grit: two runs
  an hour apart can disagree by more than the ballast signal. Do the
  ballast comparison back to back, same hour, same patch.
- **Ballast moved the CoG.** The two runs then differ in balance as well as
  load, and the `k_mu` fit absorbs the difference as a wrong number with a
  confident face. The balance-rod check before and after is not optional.
