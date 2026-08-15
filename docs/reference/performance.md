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

The three targets are the P1 goals. One of them is still not met.

| Case | Target | Measured | |
|---|---|---|---|
| L2 single-agent step | under 5 us | **1.92 us** | met |
| 1 agent, L2 and 2D LiDAR, headless | over 100x real time | **179x** | met |
| 20 agents, L2 and 2D LiDAR, headless | over 10x real time | **8.4x** | missed |

Stating that plainly is the point of this page. A performance target that
quietly becomes "roughly met" is a target that was never doing any work.

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
| Date | 2026-08-15 |

Single-threaded throughout. The integrator has no threads in it by design,
and the orchestrator steps agents in a fixed order for the same reason: the
per-agent numbers below are per core, and a machine with sixteen of them runs
sixteen independent simulations rather than one simulation sixteen times
faster.

This is a desktop with other things running on it, and it is worth saying what
that costs. Ten consecutive runs of the same binary spread over about 25 per
cent, so every figure on this page is the best of ten, and every before-and-
after pair was measured in the same session by alternating the two binaries.
Comparing a number taken today against one taken last week would be comparing
machine moods.

## What the sensor configuration is

The LiDAR cases use 1080 rays at 40 Hz over a full circle, with a 10 m
maximum range: a Hokuyo UST-10LX, which is what the class actually runs. That
matters, because the easiest way to meet a LiDAR benchmark is to benchmark a
cheaper LiDAR.

The track is the shipped `paddock_stadium`, whose two walls come to 696
segments.

## What changed, and by how much

The three figures above were 4.84 us, 90x and 3.8x when they were first
measured. The work that moved them changed nothing about what the simulator
computes: every published trajectory hash is unchanged, which is asserted
rather than assumed, and the raycast returns the same distance to the same
wall for every ray in the conformance sweeps.

Measured by alternating the two binaries in one session, best of ten each:

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

At 1 kHz, one agent's vehicle model costs 1.92 ms of CPU per simulated second.
Twenty agents therefore spend 38 ms per simulated second in the vehicle model,
which caps them at about 26x real time before a single ray is cast. That cap
used to be 10x, so the thing standing between this tier and the 20-agent
target is no longer the vehicle model. It is the sensing.

Twenty agents at 40 Hz and 1080 rays is 864,000 rays per second. Taking the
model cost off the measured total puts a ray at about 93 ns, down from about
174 ns, and that figure includes the sensor's two random draws per ray as well
as the raycast.

## Why the 20-agent target is still missed

The arithmetic is short and it is worth writing down rather than promising
another round of work.

Ten times real time for twenty agents means 100 ms of CPU per simulated
second. The vehicle model takes 38 ms of that, leaving 62 ms for 864,000 rays,
which is 71 ns each. A ray costs 93 ns. The gap is 24 per cent, and it is
entirely in the sensing.

Where a ray's 93 ns goes, on this track: about 6.6 grid cells are walked and
about 8.2 segments intersected per ray, plus a sine and a cosine for the ray
direction, plus the sensor's uniform and normal draws. The grid's cell size
was swept over a factor of twenty and the shipped choice, four mean segment
lengths, sits at the bottom of a broad minimum: halving it walks twice as many
cells for half the segments and comes out 7 per cent worse, doubling it comes
out 28 per cent worse. There is no tuning constant left to turn.

Closing 24 per cent from here means a different acceleration structure rather
than a better uniform grid, which is a design change with an ADR attached and
a broadphase the racing phase has to build anyway. Two of the three candidates
this page previously listed have been done; the third, a projection that
starts from the segment the same agent used last time, is deliberately not
done, for two reasons. It is not on the path either benchmark measures, since
neither runs a lap counter, and making the search local changes the answer
`scene::project` returns for a car that has been somewhere else, which is
precisely what the rest of this work refused to do.

So the honest position is that the target needs either that structure or a
deliberate renegotiation, and neither is a thing to slip into a performance
commit.
