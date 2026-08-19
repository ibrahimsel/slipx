# Coastdown

**Identifies:** `roll_resist`, `drag_coeff`.
**Needs first:** `mass` (bench).
**Space:** a 40 m straight, flat and swept.
**Risk:** low. The car is decelerating and steering straight.

## Why this one is first

Everything that accelerates or brakes the car in later manoeuvres does so
against rolling resistance and drag. Fit those two first and the drivetrain
fit never has to estimate them, which matters because at low speed the three
forces are the same order of magnitude and a joint fit trades them off
against each other freely.

## The physics being fitted

A coasting car obeys

```
m · dv/dt = −(roll_resist · m · g + drag_coeff · v²)
```

with exactly the two unknowns. On the reference car (`mass` 3.5 kg,
`roll_resist` 0.015, `drag_coeff` 0.015 kg/m) the rolling term is a constant
0.51 N and the drag term passes it at 5.9 m/s; below about 3 m/s drag is
nearly invisible and above about 8 m/s it dominates. That is the whole design
of the procedure: the two parameters live at opposite ends of the speed
range, so the runs must cover both ends.

## Procedure

1. Mark a measuring zone in the middle of the straight, clear of the
   acceleration and stopping areas.
2. Accelerate gently to the target entry speed, then command exactly zero
   drive torque before the zone. Zero torque, not brake: regen braking must
   not fire, so check the ESC is configured to coast on neutral. Hold the
   steering centred.
3. Coast through the zone. Stop the car beyond it.
4. Repeat at entry speeds stepping down from the fastest the straight allows
   to a walking pace, and drive every speed **in both directions**, so a
   slope or a headwind cancels in the average instead of appearing as
   rolling resistance.

Do not try to coast from top speed to rest in one run. From 10 m/s the
reference car needs well over 100 m to stop; nobody has that car park. Each
pass samples the deceleration in one speed band, and the fit wants the
collection of bands, not one long coast.

## What to record

The standard bag (encoders, IMU, pose, commands). The fit uses wheel speed
for `v` and its slope for `dv/dt`; the IMU's longitudinal channel
cross-checks it and catches a sloping site.

## What good data looks like

- Deceleration around 0.55 m/s² at 10 m/s falling to about 0.16 m/s² at
  2 m/s (reference-car numbers; yours will differ, the ratio high to low
  speed should still be roughly three).
- The two directions of the same speed band disagreeing by a constant
  offset means slope, and the average is still right.
- The commands channel showing exactly zero drive through the zone. If it
  does not, the run is not a coastdown; discard it.

## Failure modes

- **Regen fired.** Deceleration is several times too large and has a step at
  the moment of the neutral command. Reconfigure the ESC and rerun.
- **Only one end of the speed range.** The fit will return a confident-looking
  pair of numbers that trade off against each other; the residuals per speed
  band expose it. Cover both ends.
- **Wind.** Gusts show as run-to-run scatter at the high-speed end that
  direction-averaging does not remove. Wait for a calm day; at 3.5 kg the car
  is a sail.
