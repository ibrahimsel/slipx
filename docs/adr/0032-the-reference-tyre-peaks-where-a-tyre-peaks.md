# ADR-0032: The reference car's cornering stiffness is raised so its tyre peaks near 7 degrees

- **Status:** Accepted
- **Date recorded:** 2026-08-12
- **Requirements:** CORE-06, NFR-02, NFR-08, ID-06
- **Related:** [ADR-0008](0008-reference-hashes-are-keyed-by-build.md),
  [ADR-0009](0009-every-parameter-must-be-identifiable.md),
  [ADR-0023](0023-mf-lite-derives-b-from-cornering-stiffness.md),
  [ADR-0030](0030-schema-0-2-0-adds-c-kappa-and-the-actuator-fields.md),
  [ADR-0031](0031-drivetrain-and-actuators-are-quasi-static-except-the-servo.md)

## Context

The reference car and the `VehicleParams` struct defaults carried a cornering
stiffness of 120 N/rad on the front axle and 130 on the rear: 60 and 65 N/rad
per tyre against a static per-tyre load of 8.58 N. Normalised, that is about 7
per radian, where a full-size passenger tyre is 14 to 21 and anything
resembling a racing tyre is higher.

The consequence is easiest to see through the tyre's own peak. With
`mu_y0 = 1.1` the linear extrapolation reaches the friction limit at 9.0
degrees of slip angle, and the MF-lite shape factors put the true peak at 2.69
times that: **24.3 degrees**. No 1/10-scale car reaches 24 degrees of slip
angle in anything that is still driving, so the reference car could never get
to its own tyre's peak. The falling branch that CORE-06 exists to provide was
unreachable, the whole car behaved as a soft linear tyre with a distant
saturation, and every plot drawn from the reference parameters showed a curve
that keeps rising through the region a real tyre has already given up in.

Nothing caught it, and it is worth saying why. `rules.check_tyre_plausibility`
does check the saturation angle, but its band is 0.02 to 0.35 rad, which is 1.1
to 20 degrees, and 9.0 sits comfortably inside. It also checks the peak
multiple, and 2.69 is comfortably inside 1.5 to 3. Both individual checks pass;
what is implausible is their product, and no rule looked at that.

The trigger was M3.8, which regenerates the tutorial series' figures from
`slipx_core` and deletes the local illustrative tyre model that
`make_figures.py` has been carrying. Those figures show a peak near 7 degrees,
because that is where a tyre peaks. Regenerating them from the library would
have replaced correct figures with wrong ones.

Alternatives considered:

**Keep the numbers and label the disagreement in the regenerated figures.**
Free, no hash movement, and honest in the narrow sense that both parameter sets
are labelled provisional so neither is a claim. Rejected because it ships a
figure that is physically wrong and says so in a caption, which teaches the
wrong shape to every reader who looks at the picture and not the caption.
`make_figures.py`'s own docstring already states the standard: a tutorial whose
tyre curve disagrees with the library beside it teaches the wrong thing twice.

**Change `mu_y0` instead.** The saturation angle is `mu_y0 * Fz / c_alpha`, so
lowering the friction coefficient moves the peak inwards too. Rejected: 1.1 for
sponge on carpet is the more defensible of the two numbers, and reducing it to
0.31 to fix the angle would be plainly wrong about grip in order to be right
about stiffness.

**Change the MF-lite shape factors.** They set the multiple, not the linear
angle, and 2.69 is already where it should be. Moving the peak to 7 degrees
this way needs a multiple below 1, which is not a tyre.

**Defer to the release milestone so the hash table is written once.** Rejected
as the wrong ordering: M3's figures are generated from these numbers, so
deferring the change means either generating figures twice or blocking M3 on
M4.

## Decision

**The cornering stiffness of the reference car and of the `VehicleParams`
struct defaults is multiplied by 3.5**, giving 420 N/rad front and 455 N/rad
rear per axle, and 210 N/rad per tyre in
`examples/cars/reference_1_10/tyres/sponge_carpet.yaml`. The tyre's force peak
lands at 6.9 degrees of slip angle.

Three things about the shape of the change.

**A uniform scale, not a re-derivation.** The 120:130 front-to-rear ratio is
preserved exactly, so the default car still understeers and its understeer
gradient keeps its sign; only the magnitude changes. The gradient falls by the
same factor of 3.5, which is correct and is the point: a stiffer tyre needs
less slip angle to hold the same corner.

**`c_alpha` only.** `mu_y0`, `mu_x0`, `k_mu`, the MF-lite shape factors and
`c_kappa` are unchanged. `c_kappa` is documented as roughly twice the per-tyre
cornering stiffness and that ratio is now broken, which is a real loose end;
it is left alone deliberately, because `c_kappa` is identified by a different
manoeuvre (straight-line acceleration) than `c_alpha` (a skidpad), and moving
a parameter because a different parameter moved is exactly the coupling
ADR-0009 exists to prevent. The ratio was a plausibility argument, not a
physical law, and it is now marked as one.

**Still provisional.** Nothing here was measured. 210 N/rad per tyre is 24.5
per radian normalised, which is a plausible number for a small soft tyre rather
than a correct one. The change replaces an implausible provisional value with a
plausible one; it does not make the parameter set anything other than
provisional, and NFR-08's labelling is unaffected.

## Consequences

**Twelve of the eighteen reference rows move, and this is the second movement
in one unreleased cycle.** Every L1 and L2 row is rerecorded, under GCC 11, GCC
13 and Clang 18 separately, which agree. The six L0 rows do **not** move and
were re-measured to confirm it: the kinematic tier has no tyre and no cornering
stiffness, so a change to `c_alpha` that moved an L0 hash would be evidence of
a different change. That asymmetry is the cheapest available proof that the
edit did what it says.

The release notes for `0.2.0` therefore carry two hash tables, one for
ADR-0031's drivetrain slice and one for this. That is allowed and worth saying
out loud: nothing has been published since `0.1.0a1`, whose rows were already
incomparable with both, so the second movement costs no external user anything
that the first had not already cost them. It would not be allowed between two
published releases.

**Five test operating points moved, and each is now self-checking.** A stiffer
tyre reaches its limits at different speeds and steer angles, so the cases that
were tuned against the soft tyre no longer sat where they meant to: an L1
convention case saturated `mu_clip` instead of testing the linear law, an L2
braking case moved into a front-limited regime where trail braking raises
lateral acceleration rather than lowering it, and two lifting cases stopped
lifting because half a radian of steer now understeers and scrubs rather than
loading the outside. Where a case depends on being in a particular regime, that
is now an `ASSERT` rather than a comment, so the next parameter change fails
loudly instead of quietly testing something else.

**A test that hard-codes a number from the reference car is a liability.**
Three schema tests wrote a `B` consistent with the old `c_alpha`, and after the
change they asserted "no warning" about a value that did in fact warn, or
performed a string replacement that silently matched nothing. They now derive
the value from the file under test. Any future test that needs a number from
the reference car should do the same.

**The plausibility rule still does not catch this class of error.** The rule
that would have is one on the product: the slip angle at which the tyre's force
actually peaks, `mu_y0 * Fz / c_alpha` times the MF-lite multiple, which should
sit between roughly 3 and 12 degrees. It is not added here, because a warning
threshold is a claim about real tyres and this record is already making one
change to the numbers; adding it belongs with the identification work in P2,
where there will be measured sets to calibrate the band against. Until then the
gap is recorded rather than closed.

**Reversal.** Putting the old numbers back is a one-line change and another
twelve-row rerecord. What it would not restore is the five test operating
points, which are now pinned by assertions to regimes the soft tyre does not
reach.
