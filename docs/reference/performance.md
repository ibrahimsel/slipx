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

The three targets are the P1 goals. Two of them are not met.

| Case | Target | Measured | |
|---|---|---|---|
| L2 single-agent step | under 5 us | **4.84 us** | met, with no margin |
| 1 agent, L2 and 2D LiDAR, headless | over 100x real time | **90x** | missed |
| 20 agents, L2 and 2D LiDAR, headless | over 10x real time | **3.8x** | missed |

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

## What the sensor configuration is

The LiDAR cases use 1080 rays at 40 Hz over a full circle, with a 10 m
maximum range: a Hokuyo UST-10LX, which is what the class actually runs. That
matters, because the easiest way to meet a LiDAR benchmark is to benchmark a
cheaper LiDAR.

The track is the shipped `paddock_stadium`, whose two walls come to 696
segments.

## Where the time goes

At 1 kHz, one agent's vehicle model alone costs 4.84 ms of CPU per simulated
second, so the step time caps a single agent at about 200x real time and
twenty agents at about 10x before a single ray is cast. The 20-agent target
is therefore a statement about the vehicle model as much as about the
sensors, and meeting it needs the step under about 5 us **and** the sensing
close to free.

The sensing is not close to free. Twenty agents at 40 Hz and 1080 rays is
864,000 rays per second, and the raycast currently costs of the order of
190 ns per ray.

That figure was 12 times worse before this measurement existed. The first
implementation tested every wall segment for every ray: 1080 rays against 696
segments is three quarters of a million intersection tests per scan, and a
single car with a LiDAR ran at 16x. A uniform grid with a proper DDA
traversal took it to 90x. The grid is the least clever structure that fixes
the problem, and it is worth being explicit that it was built against a
measurement rather than against an intuition, because the comment it replaced
said exactly that a spatial index should wait for one.

## What would close the gap

Not attempted yet, in rough order of how much they would buy against how much
they would cost:

- **Early exit in the traversal.** The DDA gathers every candidate along the
  ray and then tests them all. Testing per cell and stopping at the first hit
  closer than the next cell boundary would cut the work on the many rays that
  hit a wall a metre away, which on a 1.5 m corridor is most of them.
- **A cheaper projection.** `scene::project` searches every centreline
  segment on every call, which the lap counter does once per agent per step.
  Starting from the segment the same agent used last time makes it local, at
  the cost of an answer that depends on where the car has been.
- **Coherent scan batching.** Consecutive rays in a scan overwhelmingly hit
  the same wall segment. Reusing the previous ray's answer as a first guess
  would skip most of the traversal.

None of these changes what the simulator computes, and each should be landed
against a before-and-after measurement recorded here.
