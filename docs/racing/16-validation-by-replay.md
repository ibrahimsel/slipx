# Validation by replay: the test a fit has to pass

A fitted model agrees with the data it was fitted to. That sentence
contains no information: the fit made it true by construction, the way a
student who saw the exam answers passes the exam. Validation is the other
exam, the one with questions the fit never saw, and for vehicle models it
has a specific, honest form: replay.

## The replay

Record a run the fit did not consume: the driver's commands and the car's
response. Then feed the same commands to the fitted model, starting from
the same initial condition, and let it drive itself. No corrections, no
nudging the model back towards the measurement; the commands alone. Then
lay the model's response over the car's, channel by channel: yaw rate,
lateral acceleration, speed.

Open loop matters. A model compared under closed-loop control is
flattered by the controller, which spends its whole life cancelling
exactly the errors you are trying to see; a wrong model with a good
controller tracks the path anyway. Replay removes the flattery. Whatever
divergence appears is the model's own.

## Measuring the divergence

Eyeballing overlaid traces invites optimism, so the divergence is a
number. A standard choice per channel: the RMS of the error between
replayed and measured, divided by a scale that says how much the channel
actually moved. Worked numbers, for a yaw-rate trace that swings ±1.4
rad/s with an RMS variation of 1.0 rad/s: if the replayed trace misses by
an RMS of 0.03 rad/s, the divergence is three per cent. The same 0.03
against a channel that barely moved would be a large fraction of nothing,
which is why the scale term is there.

Two disciplines keep the number honest. Take the **worst** channel of the
worst run as the headline, not the average, so a report cannot be
improved by padding it with easy runs. And normalise a near-constant
channel by its level rather than its variation: a speed held at 3 m/s
varies by centimetres per second, and dividing by that hair's breadth
would make an excellent model look broken.

## What a validation is worth

Exactly what it exercised, and nothing more. A model that replays a
slalom at three metres per second within two per cent has demonstrated
its slalom at three metres per second; it has claimed nothing about
launches, nothing about the friction limit, nothing about a surface it
never touched. The report should say which runs it contains, and the
reader should check that the runs cover the regime they care about
before believing anything.

This sounds restrictive because it is; it is also the entire difference
between a validated model and a marketed one. The phrase "validated
model", uncredentialled, means "somebody once compared it with
something". A validation report means: these runs, this divergence,
judge for yourself.

## Why parameters are not the test

It is tempting to validate by comparing fitted parameters against known
ones: the fitted stiffness is within two per cent of the bench value, so
the model is good. But parameters are only as meaningful as their
identifiability, and the previous article showed that entangled
parameters can be individually wrong while jointly right. The behaviour
is the product; the parameters are an encoding of it. Replay tests the
behaviour, which is why it is the exam that counts. When the parameters
also agree, that is evidence about the identification method, not extra
evidence about this model.

> **In SlipX.** A session can list validation runs the fit never
> consumes; the tool replays them through the emitted parameter set and
> writes a report with per-channel divergences and the worst-channel
> headline, and the parameter set's provenance names the report. The
> community registry refuses a submission without one.

## Further reading

Oberkampf and Roy, *Verification and Validation in Scientific
Computing*, Cambridge University Press, 2010, is the full treatment of
what validation can and cannot establish; its distinction between
verification (solving the equations right) and validation (solving the
right equations) is the frame this article sits inside.
