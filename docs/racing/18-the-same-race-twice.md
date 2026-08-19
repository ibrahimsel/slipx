# The same race twice

Run your stack against a simulator, watch it clip the third apex, fix the
code, run again, and the car sails through. Was that the fix? Run it a third
time, unchanged, and it clips the apex again. Nothing you wrote is
nondeterministic. Where does the difference come from, and what would it
take for a simulated race to be the same race every time you run it?

## Floating point is not the culprit

The first suspect everyone names is floating-point arithmetic, and it is
innocent of this particular crime. Floating-point operations are exactly
specified: the same operations on the same values in the same order produce
the same bits, every time, on the same machine. `0.1 + 0.2` does not equal
`0.3` in a double, but it equals `0.30000000000000004` reliably, run after
run. A wrong answer delivered identically forever is not a source of
run-to-run difference.

What floating point does do is make the *order* of operations part of the
computation. Addition does not associate:

```
(0.1 + 0.2) + 0.3  =  0.6000000000000001
0.1 + (0.2 + 0.3)  =  0.6
```

Both are honest roundings, one unit apart in the last place. So a sum taken
over a hash table, whose iteration order is allowed to vary, is a different
number on a different run even though every element was identical. A
simulation that wants to be reproducible must therefore fix its reduction
orders: not because one order is more correct, but because an unfixed order
is a hidden input. Goldberg's classic survey is the proper treatment of why
these roundings behave the way they do
([What Every Computer Scientist Should Know About Floating-Point
Arithmetic](https://docs.oracle.com/cd/E19957-01/806-3568/ncg_goldberg.html)).

The genuine sources of run-to-run difference are more mundane: iteration
over unordered containers, uninitialised memory, a random generator seeded
from the clock, and, the big one for robotics, timing.

## Timing is an input nobody declared

A control stack and a simulator are separate programs exchanging messages.
The stack computes a steering command from a scan; the command travels back
and is applied when it arrives. *When it arrives* depends on the operating
system's scheduler, the network, and whatever else the machine was doing,
none of which appears in anyone's source code. The wall clock has become an
input to the physics.

How much can a millisecond matter? Take a car at 5 m/s commanding a modest
steering angle of 0.05 rad on a 0.32 m wheelbase. The kinematic curvature is

```
kappa = tan(0.05) / 0.32 ≈ 0.156 per metre
```

which at 5 m/s is a lateral acceleration of `v² · kappa ≈ 3.9 m/s²`. If
that command lands one millisecond later on run B than on run A, then for
one millisecond run B turns at the old rate while run A turns at the new
one: a lateral velocity difference of about 4 mm/s, and a position
difference of a few micrometres. Trivial, apparently.

But the stack is a feedback loop. On the next scan, run B's car is
somewhere fractionally different, so its next command differs fractionally,
and lands at its own fractionally different time. Near the limit of grip
the loop amplifies rather than forgives: two runs that differed by
micrometres at the first corner are taking visibly different lines a few
corners later, and by the apex that matters, one clips and one does not.
Neither run is wrong. The physics was deterministic in both; the schedule
was not, and the schedule was an input. That is the property to object to:
not that the answer is bad, but that the *experiment is unrepeatable*, so
you cannot attribute a change in outcome to a change in code.

## Two honest fixes, and their prices

**Replay.** Record every command actually applied, tagged with the step it
was applied at. Feeding that log back reproduces the trajectory exactly,
because the log has replaced the schedule: arrival times no longer matter,
only the recorded step indices. The price is what replay can answer. It
reproduces what the car *did*, which settles protests, enables bisection of
physics changes, and lets a crash be re-examined frame by frame. It cannot
re-run the *stack*: the decisions are baked into the log, so "would my fix
have avoided that crash" is exactly the question replay cannot ask.

**Lockstep.** Make the schedule itself deterministic: the simulator
announces "computing step 1000, commands please", waits until every
participant has answered for that step, and only then integrates. A command
is now a function of a step index rather than of a wall clock, so a slow
scheduler tick delays the *experiment* without touching its *result*. The
price is pace: a lockstep race runs exactly as fast as its slowest
participant thinks, and a hung participant would stall it forever, which is
why every lockstep design needs a stated policy for what a missed answer
means: wait, coast on, or forfeit.

The two compose. A lockstep race is reproducible while it runs; its log
makes it re-examinable afterwards; and a live, wall-clock race, which is
the more realistic test of a stack that must survive latency, is at least
replayable even though it is not repeatable.

## What "the same" can honestly mean

Bit-for-bit identity is achievable, but only within a stated boundary: one
binary, one machine, one C library. Across compilers, or across the
mathematical libraries that ship with different systems, `sin` and `exp`
are allowed to differ in the last bit, and those last bits feed the same
amplification as everything else. So a claim of reproducibility should
name its scope: *identical here, close over there*, with "close" checked
against a tolerance rather than asserted. A benchmark that publishes a
number without the machine, the build and the scope of "the same" attached
has published an anecdote.

> **In SlipX.** Every run writes a manifest that states which promise
> applies: a deterministic run claims bit-identity for the same binary on
> the same C library, a wall-clock ("validation") run says plainly that it
> is NOT REPRODUCIBLE beyond replay, and the input log is always recorded,
> so `replay(log)` is available either way. Lockstep racing over ROS 2
> announces each step and waits on per-agent mailboxes, with the missed
> answer policies named above as configuration; the test suite asserts
> that a deliberately slow client changes not one bit of the trajectory.

## The limits, stated plainly

Lockstep changes what is being tested. A stack that only wins under
lockstep, given all the thinking time it likes per step, has not shown it
can drive at real-time rates; the wall-clock mode exists precisely to keep
that question honest. And reproducibility of the simulation says nothing
about fidelity to reality: a perfectly repeatable wrong answer is still
wrong. Repeatability is not truth. It is the precondition for finding
truth, because without it no comparison, between two code versions, two
parameter sets or two teams, means anything at all.
