# ADR-0053: the bridge broadcasts a REP 105 TF tree, with ground truth as the map correction

- **Status:** Proposed
- **Date recorded:** 2026-08-20
- **Requirements:** the ROS bridge requirements in spirit; `docs/spec` is
  not present in a fresh clone, so no ID is cited as authority. The
  operative constraint is the ground-truth provability rule the launch
  switch already serves: truth on the wire must be declineable, and the
  manifest must record the choice.
- **Related:** ADR-0050 (the bridge), ADR-0047 (the sensor rig casts from
  the vehicle origin), ADR-0006 (honesty on the wire, in spirit)

## Context

The bridge published no TF, so nothing it said could be placed in any
frame: RViz could show the ground truth odometry (whose frame id is
literally `map`) and nothing else. Worse, the frame ids it stamped on
scans and odometry carried a leading slash copied from the topic
namespace, and tf2 in ROS 2 refuses to look up any frame that starts with
one, so those frames could never have joined a tree at all; the defect was
invisible exactly because no tree existed. Wanting to watch a run forced
both decisions at once.

Three shapes were considered for the dynamic tree:

- **`map -> base_link` directly from the state.** Simplest, but it gives
  `base_link` a second parent the moment any stack dead-reckons or
  localises, which TF forbids, and it puts the truth on the wire with no
  frame in between for a stack to claim.
- **No truth in TF at all**, publishing only the dead reckoner's
  `odom -> base_link` and leaving `map` to the stack's own localiser.
  Honest, but a bridge run without a localiser could then never be
  watched in the world frame, which was the whole complaint.
- **REP 105 as real robots do it**: `odom -> base_link` from the dead
  reckoner, and `map -> odom` as the correction a localiser would
  publish, here computed from the truth the simulator uniquely has.

A separate small choice: the tf2_ros broadcaster classes are thin wrappers
over two publishers with fixed QoS; the bridge publishes `TFMessage`
directly with the same QoS, keeping its imports to message packages.

## Decision

Frame ids lose the leading slash: the prefix is `car_N`, the topic
namespace without its slash, on every message and every TF edge.

The bridge broadcasts, per agent, unless `--no-tf`:

- **`car_N/base_link -> car_N/<mount>`** on `/tf_static`, latched once,
  identity. Identity is the modelled truth, not a default: the sensor
  models cast every ray and sample every sensor from the vehicle origin
  (ADR-0047), and a test asserts the identity on the wire so that the day
  a mount gains a pose, the static edge must carry it or fail loudly.
- **`car_N/odom -> car_N/base_link`** on `/tf`, at the ground truth rate,
  from the dead reckoner: the same belief the `odom` topic publishes,
  drifting by exactly the slip and the lag. It publishes regardless of
  the ground truth switch, because a belief is not truth.
- **`map -> car_N/odom`** on `/tf`, only while ground truth is offered:
  with truth T and belief R as planar poses, the edge is T composed with
  the inverse of R, so the chain `map -> odom -> base_link` lands exactly
  on T while `odom -> base_link` alone stays the honest belief. This is
  the correction a perfect localiser would publish, and it is ground
  truth on the wire, so `--no-ground-truth` removes it and the bridge
  manifest records the choice, as it already did for the topics.

## Consequences

- The wire format changed: frame ids recorded before this record carry
  the slash and will not resolve against the new tree. Nothing had been
  released, and the bags that exist are development artefacts.
- A stack that runs its own localiser publishing `map -> car_N/odom`
  fights the bridge's map edge; such a stack launches with
  `--no-ground-truth` (keeping the belief edge) or `--no-tf` (owning the
  tree outright). The default favours watching a run, because a stack
  sophisticated enough to localise is sophisticated enough to pass a
  flag.
- The belief edge is piecewise constant between encoder readings and is
  republished with fresh sim-time stamps at the ground truth rate, so
  the chain is exact at publication instants and interpolated between
  them; a consumer comparing TF against topic data at millimetre
  precision must interpolate, as with any TF tree.
- TF stamps are simulation time, like `/clock` and every message stamp.
  A viewer that mixes wall-clock data into the same buffer will see the
  jump; that is the `use_sim_time` contract, not a bridge defect.
- The map correction wobbles as the belief drifts, exactly as a particle
  filter's correction does. That is the honest shape: the drift lives in
  the edge whose job is to carry it.
