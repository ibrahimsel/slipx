# ADR-0050: the ROS 2 bridge is rclpy above the Python bindings, pacing a validation-mode run

- **Status:** Proposed
- **Date recorded:** 2026-08-19
- **Requirements:** the ROS bridge requirements and SIM-05 in spirit;
  `docs/spec` is not present in this checkout, so no ID is cited as
  authority (ROS-03, the one ID the code already cites, is the
  ground-truth provability rule the launch switch serves)
- **Related:** ADR-0044 (the barrier and reproducibility from the input
  log), ADR-0047 (the sensor rig), ADR-0049 (the track world), ADR-0025
  (refusal over defaulting)

## Context

The bridge's job is fixed by the milestone: the per-agent topic set under
`/car_N/` (`drive` in; `scan`, `imu`, `odom`, ground truth out), `/clock`
correct under `use_sim_time`, F1TENTH names and QoS where conventions
exist, and an existing stack connecting with a remap file and no code
change. What had to be decided is what the bridge is made of.

An rclcpp node was the alternative, and it fails on the project's own
rules before it fails on convenience. Parsing lives in Python: a C++ node
cannot read a car directory, so it would need either a second parameter
path (the drift ADR-0025 exists to prevent) or a Python front-end handing
structs across a process boundary, which is most of an rclpy bridge plus
an IPC layer. It would also put colcon between a student and their first
simulation. The speed argument for C++ is real but unquantified until the
RMW benchmark that M7.7 runs anyway; meanwhile the physics, the sensors
and every ray are already native (ADR-0047, ADR-0049), so what remains in
Python is message assembly at sensor rates, which is exactly what the
benchmark will measure. The dependency order has said `slipx_ros` sits
above `slipx` (Python) since the layers were first drawn.

## Decision

`slipx_ros` is a Python package using rclpy, run from a sourced ROS 2
environment against an installed (or in-tree) `slipx`. It is not part of
the wheel: rclpy is a ROS package, not a pip dependency, and the extras
table stays honest.

The shape:

- **One node, one simulation, N agents.** The bridge loads the track and
  each car directory through the same loaders every other consumer uses,
  builds the sim (footprints from the car's geometry), the rig (from each
  car's `sensors.yaml` via `sensors_for`) and the `TrackWorld`, and steps
  everything in one thread: spin callbacks, advance, publish what the rig
  delivered. No executor gymnastics; the lockstep design made the single
  thread correct and ADR-0044's mailboxes exist for the day a transport
  needs more.
- **The run is a validation-mode run.** The simulation itself paces the
  wall clock (`RunMode.kValidation`, `real_time_factor` configurable), so
  the manifest says NOT REPRODUCIBLE exactly as M5.6 built it to, and the
  bridge does not reimplement pacing. Input logging is on: a live run is
  reproducible from its input log, not by re-running (ADR-0044's promise,
  inherited unchanged).
- **Commands hold, like a servo.** The bridge keeps each agent's latest
  `AckermannDriveStamped` and a policy closure applies it every step:
  steering angle directly; speed through a proportional acceleration
  demand with a named, configurable gain, which is a mechanisation of the
  speed controller a VESC runs and is labelled one. Before the first
  message, the car coasts. A nonzero `acceleration` field is ignored with
  a single warning: stacks default-construct messages, so refusing a zero
  would refuse everyone, and silently consuming a field the speed loop
  then fights would be worse.
- **Topics and QoS.** Under `/car_N/` (N is the agent index): `scan`
  (LaserScan), `imu` (Imu), `odom` (Odometry), all sensor-data QoS (best
  effort, the F1TENTH default); `drive` in, reliable; `/clock` reliable,
  published every sensor-publishing tick as steps times dt. Frame ids
  come from each sensor's `mount` field, prefixed with the agent
  namespace, which is what that field was carried for. Invalid rays are
  NaN in `ranges`, never zero (ADR-0006 on the wire).
- **Odometry is the encoder's belief.** `odom` dead-reckons a pose from
  the encoder's own speed and the commanded steering angle through the
  kinematic bicycle, the same estimate a VESC driver publishes, so it
  diverges from ground truth by exactly the slip and the actuator lag: an
  honest odometry, wrong the way the real one is wrong. Ground truth
  (pose and twist from the state) lives under `/car_N/ground_truth/odom`,
  is removed entirely by a launch switch, and the bridge manifest records
  whether it was offered.
- **The bridge writes its own manifest** beside the run: the sim's
  manifest plus what the sim cannot know (car directories, track, ground
  truth on or off, the real-time factor, the speed-loop gain, topic
  names), so a bag recorded against the bridge can be traced to its
  configuration.

## Consequences

- The bridge's throughput ceiling is Python message assembly. That is a
  measured question, and M7.7's RMW benchmark answers it before anything
  is promised; if 20 agents at full sensor rates need C++, the bridge's
  seams (structs in, messages out) are where an rclcpp publisher drops in,
  and this record is superseded rather than bent.
- Determinism is not promised for a live bridge run and the manifests say
  so; the input log is the replay path. Lockstep racing over ROS is the
  `race_sync` half of M7.7, built on the mailboxes, and is its own slice.
- The bridge cannot run on Windows (rclpy there is not supported in this
  project's environments); it is developed and tested in the WSL
  distribution, and its tests skip cleanly where rclpy is absent so the
  Windows pytest run stays green.
- `slipx_ros` stays out of the wheel and the extras table; its presence in
  the tree is a directory with its own tests, exercised where ROS exists.
