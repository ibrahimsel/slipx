# Overtaking

Two cars cannot occupy the same piece of track, so a race is a queue unless
somebody does something about it. This article is about what that something
costs. The claim to keep hold of: **a pass is bought with a speed
differential, and it has to be spent somewhere on the lap that has room for
it**. Everything else, brake markers, defensive lines, switchbacks, is the
question of where and how to spend.

It builds on [the racing line](05-the-racing-line.md) and
[the speed profile](06-speed-and-the-gg-diagram.md), and what happens when a
move ends in contact is [article 17](17-a-collision-is-an-impulse.md). The
worked numbers below take `mu = 0.6`, plausible for sponge tyres on carpet,
so the grip budget `mu g` is about 6 m/s². Named assumption, as always:
your surface will differ, the arithmetic will not.

## The follower's problem

Following is easy; that is the problem. A follower matching the leader's
speed sits at a constant gap indefinitely, and a follower whose controller
treats everything in front as an obstacle will do exactly that, braking
when the leader brakes, at a respectful distance, for the rest of the race.
The queue is stable. To pass, the follower must at some point be *faster
than the leader while alongside them*, and "alongside" needs about a car
length of relative travel plus lateral room that the leader is not using.

That relative travel is the entire currency. Whatever the source of the
speed difference, the pass consumes metres of it.

## Buying a pass with top speed

The cheapest pass looks free: be faster down the straight. Take a leader
running 4.0 m/s and a follower capable of 4.4 m/s, ten per cent more, on a
26 m straight, which is generous at this scale.

Closing speed is 0.4 m/s. From 2 m behind, pulling up to half a metre takes

```
t = 1.5 / 0.4 = 3.75 s,   during which the follower covers 3.75 · 4.4 ≈ 16.5 m
```

Getting from "half a metre behind" to "alongside" is roughly another car
length, 0.55 m, of relative travel: another 1.4 s and 6 m of straight. The
whole move needs about 22 m, most of the longest straight a sports hall
holds, and it needed a ten per cent speed advantage to exist at all. Two
conclusions fall out. First, pure straight-line passes are rare below a
large performance difference, which is why a field of near-equal cars forms
a train. Second, there is no help from the air at this scale: at 4.5 m/s
the drag on a 1/10 car is a fraction of a newton against a 3.5 kg mass, so
slipstreaming, the classic subsidy for straight-line passes, is
[a full-scale phenomenon](https://en.wikipedia.org/wiki/Slipstream) that
does not survive the shrink.

## Buying a pass with brake timing

The braking zone prices differently. Braking for a chicane taken at
1.8 m/s from an approach at 4.5 m/s, at the 6 m/s² budget, takes

```
d = (v² - u²) / 2a = (4.5² - 1.8²) / 12 ≈ 1.42 m
```

The entire braking zone is a metre and a half. Now shift a follower's
whole braking profile 0.3 m later than the leader's: the follower covers
that 0.3 m at the 4.5 m/s approach speed while the leader is already slow,
and pays it back at the 1.8 m/s corner speed, so the overlap gained is

```
gain = Δd · (1 - v_corner / v_approach) = 0.3 · (1 - 1.8/4.5) = 0.18 m
```

A third of a car length, in one corner, with no top-speed advantage at
all; the ten per cent advantage above needed about two metres of straight
to buy the same overlap. This is why late braking is the pass of choice at
every scale, and why at 1/10 scale it is decided by tenths of a metre.

The catch is that "shift the profile later" assumes the extra 0.3 m of
braking still fits before the corner. Brake 0.3 m late without extending
the zone and the equation charges you instead: the car arrives still
carrying

```
v_entry = sqrt(1.8² + 2 · 6 · 0.3) ≈ 2.6 m/s
```

against the 1.8 m/s the radius permits: forty-five per cent hot, which the
line must absorb as extra radius or the move ends in the wall or in
somebody's sidepod. Room to run wide at corner entry is what makes an
overtaking spot; a braking zone into a corner with no spare width is a
queue with extra steps. This is a geometric fact about tracks, not about
drivers: passes happen where the track is wide *before* it is slow.

## Buying a pass with the line, and paying it back

The attacker on the inside of a corner takes a tighter radius. From
`v = sqrt(mu g R)`: inside at `R = 1.2 m` gives 2.7 m/s where the wider
entry at `R = 2.0 m` gives 3.5 m/s. The inside car gains position while
cornering slower, and exits slower, which the defender can answer with the
**switchback**: concede the inside, keep the wider arc, and use the exit
speed difference down the following straight to take the place straight
back. An inside move into a corner that leads onto a long straight is
therefore only half a pass; the move has to stick through the exit, which
means the attacker must claw back to the racing line before the defender's
better exit matures. Where the next corner arrives quickly, the switchback
has no straight to mature on, and the inside move is simply a pass.

## Defence, greed, and what actually sorts a field

Put the three prices together and a field of autonomous cars sorts itself
by a single quantity: **how small a margin each car will accept**. The
defensive car spends its clearance on safety, braking early and yielding
the line the moment its forward gap shrinks; it finishes intact, at the
back. The greedy car spends clearance on pace, braking at the number the
physics gives rather than the number that leaves slack for surprises, and
leaning out of the line to keep its speed when the car ahead lifts. Between
equals, the greedy car passes the defensive one and nothing passes it back:
the margin *is* the sorting variable. The limit of greed is
[article 17](17-a-collision-is-an-impulse.md): a margin of zero is a
collision strategy, and most rulesets put the responsibility for a clean
pass on the overtaking car. Where exactly to sit on that spectrum is a
planning problem with its own literature; the survey by
[Betz et al.](https://arxiv.org/abs/2202.07008) maps it under "local
planning" and "opponent handling".

None of this needed the cars to be autonomous. What autonomy changes is
that the margin stops being a nerve and becomes a parameter: a number in a
file, dealt to a controller, doing to a robot field exactly what
temperament does to a human one.

> **In SlipX.** The bridge's demo grid deals each car its numbers from a
> seeded spread: a top speed for every card, plus a lookahead and an
> aggression for the racers, and aggression is literally the margin above:
> it scales the brake distance the racer accepts and how far it leans out
> of the line when blocked. The shipped `paddock_gp` circuit is drawn from
> this article's arithmetic: its widest point is the braking zone for the
> hairpin, so the late-braking pass has somewhere to happen, and its
> chicane narrows to 1.4 m, the tightest point of the lap, so a defended
> line through it is genuinely worth holding.

## What this article left out

Slipstream, as measured: negligible at this scale and stated so above, not
modelled. Tyre and brake temperature, which at full scale make late-race
passes a different economy. Blue flags, damage, and every rule that turns a
geometric overlap into a completed position change; a ruleset decides when
a pass *counts*, and this article only says what one *costs*.
