# Residuals and confidence: how much to believe a fitted number

A fit always produces a number. That is its failure mode: it produces one
whether or not the data justified it, with the same number of decimal
places either way. Residuals and confidence intervals are how a fit
reports not just the answer but the grounds for it, and reading them is a
skill worth exactly as much as the fit itself.

## The residual

Fit a line through measured points and the residuals are what is left:
measurement minus model, point by point. They carry two messages.

Their **size** bounds how well the model explains the data. Fit the
coastdown of the previous article and find residuals of 0.01 m/s² against
decelerations of 0.5 m/s²: the model accounts for 98 per cent of what was
measured, and the remaining 2 per cent is sensor noise, wind, or physics
the model does not carry.

Their **shape** is the sharper diagnostic. Residuals should look like
noise: centred on zero, patternless. A residual that grows with speed
says the model is missing a speed-dependent term; a residual that flips
sign between the first and second half of a session says something
drifted, a tyre warming or a battery sagging. A fit with small residuals
in a suspicious pattern is worse than a fit with honest scatter, because
the pattern is a systematic error wearing noise's clothing.

## The confidence interval

Refit the same car on a different day and the numbers move a little; the
confidence interval is the fit's own estimate of how much. Mechanically
it comes from the residuals and the sensitivity: a parameter the data
constrains tightly (change it a little and the residuals grow a lot) gets
a narrow interval; a parameter the data barely feels gets a wide one.

Worked example. Suppose a cornering stiffness fits as 420 N/rad with a
one-sigma interval of ±4 N/rad, and a load sensitivity exponent fits as
0.15 ± 0.05. The first number is known to one per cent; the second to
thirty. Both are honest outputs from the same session, because the
manoeuvres excited the stiffness strongly (every corner bends on it) and
the load sensitivity weakly (its observable effect was a few per cent
change in the limit). Quoting both as bare numbers hides exactly the
distinction that matters when a lap time depends on one of them.

## Entanglement, the failure the interval cannot show alone

Two parameters can be individually uncertain and jointly certain. Fit
`y = (a + b) t` to data and the fit knows `a + b` to high precision while
knowing neither `a` nor `b` at all: any pair summing to the right value
fits identically. Each parameter's interval is enormous, but the real
information is the **correlation**: the fit's report that these two trade
off one for one.

This is not a contrived case. Model parameters that act through the same
mechanism over the reachable operating range are entangled in exactly
this way, and the honest fit output names the pairs: "these two numbers
cannot be told apart from this data". A reader who sees two confident
central values without the correlation attached has been misled by
omission. The practical responses, in order of preference: find a
manoeuvre that separates the pair; fix one of them from another source
and fit the other; or carry the pair as a pair and never quote one
without the other.

> **In SlipX.** Every fit report carries per-parameter one-sigma
> intervals and names any pair whose correlation magnitude exceeds 0.95.
> The Magic Formula shape factors are the canonical entangled pair:
> steady-state driving cannot sit on the falling branch of the tyre
> curve, so families of (C, E) draw the same reachable curve, and the
> fitter recovers the curve while flagging the coordinates.

## Further reading

Press, Teukolsky, Vetterling and Flannery, *Numerical Recipes*, 3rd ed.,
Cambridge University Press, 2007, chapter 15, is the most readable
treatment of least squares, covariance and confidence that exists, and
its warnings about goodness of fit are the ones this article compresses.
