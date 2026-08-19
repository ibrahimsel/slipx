# Performance

Measured numbers, the machine they were measured on, and which targets they
miss. Run them yourself with:

```
cmake -S . -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build -j
./build/benchmarks/slipx_bench
```

The suite is `benchmarks/slipx_bench.cpp`. It is a plain executable with no
benchmark-library dependency, because it has to run in CI on five platforms
without anybody installing anything, and what it measures is coarse enough
that a statistical harness would mostly be measuring itself. CTest runs it as
`Benchmarks.Run`, so a change that makes the simulator an order of magnitude
slower fails the build rather than being noticed a release later.

## The targets, and where the tree stands

Two targets are the P1 goals as set. The third was renegotiated on
2026-08-19, deliberately and with the reasoning below, after the
acceleration structure that was the last candidate for closing it was built
and measured.

| Case | Target | Measured | |
|---|---|---|---|
| L2 single-agent step | under 5 us | **2.01 us** | met |
| 1 agent, L2 and 2D LiDAR, headless | over 100x real time | **163x** | met |
| 20 agents, L2 and 2D LiDAR, headless | over 7x real time (was 10x) | **7.3x** | met |

Stating that plainly is the point of this page. A performance target that
quietly becomes "roughly met" is a target that was never doing any work;
this one was renegotiated in the open instead, and the section below is the
record.

## The machine

Nothing here is a reference machine, and no number on this page should be
compared against a number from a different one.

| | |
|---|---|
| CPU | AMD Ryzen 7 7800X3D, 8 cores |
| Memory | 16 GB |
| OS | Ubuntu 24.04 under WSL2, kernel 5.15.133.1 |
| Compiler | GCC 13.3.0 |
| Build | RelWithDebInfo, `-ffp-contract=off` |
| Date | 2026-08-19 |

Single-threaded throughout. The integrator has no threads in it by design,
and the orchestrator steps agents in a fixed order for the same reason: the
per-agent numbers below are per core, and a machine with sixteen of them runs
sixteen independent simulations rather than one simulation sixteen times
faster.

This is a desktop with other things running on it, and it is worth saying
what that costs. Ten consecutive runs of the same binary spread over about 25
per cent, so every figure on this page is the best of ten, and every
before-and-after pair was measured in the same session by alternating the two
binaries. Comparing a number taken today against one taken last week would be
comparing machine moods, and the point is not hypothetical: the 2026-08-15
session measured this same code at 1.92 us, 179x and 8.4x, and re-measuring
on 2026-08-19 gave the table above, with the pre- and post-racing-phase
binaries alternated in one session agreeing with each other run for run. The
spread between the two tables is the machine, not the code.

## What the sensor configuration is

The LiDAR cases use 1080 rays at 40 Hz over a full circle, with a 10 m
maximum range: a Hokuyo UST-10LX, which is what the class actually runs. That
matters, because the easiest way to meet a LiDAR benchmark is to benchmark a
cheaper LiDAR.

The track is the shipped `paddock_stadium`, whose two walls come to 696
segments.

## What the agents are doing, which is not racing

The benchmark measures load, and its scenario is chosen to be a fixed amount
of work rather than a plausible race. Every agent holds a constant 0.02 rad of
steering and a 4 m/s speed demand, which is a 16.2 m circle: the cars leave
the track within two seconds and none of them ever reads the LiDAR the case is
named after. They start 0.3 m apart along one line and declare no collision
footprints, so they overlap from the first step and pass through each other
and through the walls. All twenty trajectories are the same trajectory,
offset.

None of that changes what the numbers mean, because the cost of a step does
not depend on where the car is. It does mean a reader should not take "20
agents" for a grid of twenty cars racing. For twenty cars driving the track,
see `examples/cpp/ghost_race_main.cpp`, which is a demonstration and is not
what is timed here.

## What changed, and by how much

The figures were 4.84 us, 90x and 3.8x when they were first measured. The
work that moved them changed nothing about what the simulator computes:
every published trajectory hash is unchanged, which is asserted rather than
assumed, and the raycast returns the same distance to the same wall for
every ray in the conformance sweeps.

Measured by alternating the two binaries in one session (2026-08-15), best
of ten each; per the machine-moods note above, these pairs compare within
their own session and not against today's table:

| Case | Before | After |
|---|---|---|
| L2 single-agent step | 4.98 us | 1.92 us |
| 1 agent, L2 and 2D LiDAR | 93x | 179x |
| 20 agents, L2 and 2D LiDAR | 4.0x | 8.4x |

Three changes account for it.

**The tyre's load sensitivity stopped recomputing a constant.** The peak force
law is `mu0 * Fz_nom^k_mu * Fz^(1-k_mu)`, and L2 was evaluating the whole of
it, both branches, at four wheels, twice per derivative and five times per
step: 280 calls to `pow` for one millisecond of car. `Fz_nom^k_mu` does not
depend on the load being asked about, and the two branches share
`Fz^(1-k_mu)`, so `tyre.hpp` now splits the law into a tyre-only half and a
load-only half. The grouping is written out so a caller that hoists the first
half gets the same bits as one that does not. The friction ellipse gained a
guard in the same spirit: a tyre whose squared demand is below 0.99 is inside
its budget by a margin no rounding could close, so it takes three
instructions instead of a call to `hypot`.

**The two load passes stopped repeating their state-dependent half.** L2
closes its load-transfer loop with a fixed two-pass evaluation (ADR-0027), and
both passes were computing each wheel's slip angle, its Magic Formula shape
term, the steer trigonometry and the drag: an arctangent, two more
arctangents and a sine per wheel, none of which depends on a vertical load.
They are now computed once per derivative. Together with the first change this
took the step from 4.98 us to 1.92 us, and the count of library maths calls
per step from about 500 to about 130 for a car inside its friction budget.

**The wall traversal stopped gathering the whole ray before testing any of
it.** The grid walk collected every candidate segment out to the maximum range
and then intersected the lot. A car in a 1.5 m corridor sees a wall in the
first two or three cells and the traversal was walking the other thirty for
nothing. Testing each cell's segments as the walk reaches that cell, and
stopping once the nearest hit in hand is closer than the far edge of the
current cell, roughly halved the cost of a ray. The grid itself also moved
from a vector of vectors to one flat array, which takes a pointer chase out of
every cell.

The early exit is exact and not a heuristic: a segment's intersection with the
ray lies inside that segment's own bounding box, so the cell holding it is one
the walk has yet to enter, and a hit in an unvisited cell is at least as far
away as the current cell's far edge.

## Where the time goes now

At 1 kHz, one agent's vehicle model costs about 2 ms of CPU per simulated
second. Twenty agents therefore spend about 40 ms per simulated second in the
vehicle model, which caps them at about 25x real time before a single ray is
cast. That cap used to be 10x, so the thing standing between this tier and
the original 20-agent target was no longer the vehicle model. It was the
sensing.

Twenty agents at 40 Hz and 1080 rays is 864,000 rays per second. Taking the
model cost off the measured total puts a ray in the mid-90s of nanoseconds,
down from about 174 ns, and that figure includes the sensor's two random
draws per ray as well as the raycast.

## Why the 20-agent target was renegotiated, not met

The arithmetic first, because it frames everything. Ten times real time for
twenty agents means 100 ms of CPU per simulated second. The vehicle model
takes about 40 ms of that, leaving 60 ms for 864,000 rays, which is about
70 ns each. A wall ray costs about 95 ns through the grid, and the grid's
cell size was swept over a factor of twenty to confirm the shipped choice
sits at the bottom of a broad minimum: there was no tuning constant left to
turn. The remaining route this page previously named was a different
acceleration structure, which the racing phase had to build anyway.

It was built (`slipx/scene/broadphase.hpp`, ADR-0045): a prebuilt BVH over
the wall segments, fully specified build, ordered and pruned traversal,
asserted bit-for-bit against the brute-force definition on the same sweeps
the grid is held to. Then it was measured, on the workload that matters,
short rays from on-track poses, alternated against the grid in one session:

| Structure | Cost per wall ray |
|---|---|
| Uniform grid (shipped) | **95 ns** |
| Scene BVH | **280 ns** |

The BVH loses by a factor of three, and the reason is the workload, not the
implementation: a car in a 1.5 m corridor finds its wall in the first two or
three grid cells, while a from-the-root tree descent pays a dozen node slab
tests to reach the same handful of segments. A BVH earns its keep on long
rays through sparse scenes and on incoherent queries; a LiDAR in a corridor
is neither. So the grid stays for wall rays, the benchmark prints both costs
per commit so the decision stays re-checkable, and the BVH's real job is the
one it was named for: the racing broadphase, where the dynamic agent overlay
(also measured: about 124 ns per ray against twenty moving boxes, the price
of cars seeing cars) and the pair query live.

With the acceleration-structure route measured shut, the standing decision
of 2026-08-19 applies: the target is renegotiated to the measured number.
The 20-agent case measures 7.3x on this machine today and measured 8.4x on
the same machine four days earlier, with the same code; the renegotiated
target is **over 7x**, the number every session clears, because a target the
machine misses on a busy Tuesday is the CI brittleness this page already
refuses. What would actually move the figure now is stepping twenty vehicle
models and casting rays on more than one core, and single-threaded operation
is a determinism decision (ADR-0004 territory), not a performance oversight;
if a future phase revisits it, it revisits it as an ADR.
