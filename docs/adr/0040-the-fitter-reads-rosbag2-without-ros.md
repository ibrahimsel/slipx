# ADR-0040: The fitter reads rosbag2 without ROS, and emits a car directory

- **Status:** Proposed
- **Date recorded:** 2026-08-19 (decision taken while building M6.3)
- **Requirements:** ID-02 in spirit; `docs/spec` is not present in this
  checkout, so no ID is cited as authority.
- **Related:** [ADR-0038](0038-the-fitter-is-staged-and-depends-on-nothing-new.md),
  [ADR-0013](0013-provenance-labels-are-printed.md),
  [ADR-0035](0035-track-geometry-is-converted-never-redistributed.md)

## Context

The data a team has is a rosbag2 recording, because that is what their car
already writes. The obvious ingestion path is to depend on ROS
(`rosbag2_py`, `rclpy`), and it fails the people this tool is for twice
over: `pip install slipx` must work on a laptop that has never seen ROS
(the analysis machine is rarely the car), and a ROS dependency under the
fitter would put the entire ROS stack beneath a tool whose inputs are a
few well-known message types in two well-documented containers.

Those containers are genuinely open: rosbag2's sqlite3 storage is an
SQLite database with a two-table schema, its mcap storage is MCAP, and the
payloads are CDR, a fixed little-endian wire format whose rules fit on a
page. The standard library reads SQLite; the `mcap` package is already an
optional extra for the sinks.

On the output side, the roadmap says "emit `dynamics.yaml`", but a
`dynamics.yaml` alone is not loadable: the loader consumes a car directory
(manifest, dynamics, tyres, limits, provenance), and a fit whose output
cannot be handed straight back to `slipx.load_car` would leave the last
step of the identification loop manual.

## Decision

**`slipx_id` reads rosbag2 recordings directly: sqlite3 storage with the
standard library, mcap storage through the existing `mcap` extra, and CDR
decoded by hand for a named message set.** The set is exactly what the
manoeuvre library records: `sensor_msgs/msg/Imu`,
`geometry_msgs/msg/PoseStamped` and `nav_msgs/msg/Odometry` for the pose,
`sensor_msgs/msg/JointState` for the encoders, and
`ackermann_msgs/msg/AckermannDriveStamped` for the commands. A topic whose
type is not in the set is refused by name, never skipped silently: a
recording the fitter half-understands would produce a fit that looks
complete and is missing a signal.

**The same module writes rosbag2, for the self-test and for nothing the
fit depends on.** The synthetic self-test must pass "end to end through
the real bag path", which means writing a real bag; the writer exists so
the test exercises the same decoder a team's recording will meet, and it
writes the sqlite3 storage with a metadata file rosbag2 itself accepts.

**The fit emits a complete car directory**, not a single file: dynamics,
tyres, limits, manifest, and a provenance file whose residuals block
carries every fitted parameter's value and confidence interval. Emission
refuses, naming the fields, when the provenance is not populated
(contributor, source, method, date), and the emitted directory is loaded
back through `slipx_schema` before the tool reports success, so
plausibility warnings surface immediately and an emitted car that cannot
load is an error here rather than a surprise later. Parameters no stage
identified keep their `provisional` label and the per-parameter labels
travel in the file; a session whose launch never saturated the tyre gets a
provisional `mu_x0` and a note, not a guess.

## Consequences

- Hand-written CDR is a real maintenance surface: five message types,
  their alignment rules, and any future additions. Accepted because the
  alternative surface is all of ROS, and because the codecs are pinned by
  byte-level round-trip tests rather than trusted.
- Only the named message set is readable. A team whose encoders publish a
  vendor message (VESC telemetry, for instance) must remap or convert
  first; the refusal names the type so the failure is five minutes rather
  than a silent gap.
- The writer makes it possible to fabricate bags, and provenance is the
  defence, as it always was: a submitted parameter set names its data, its
  contributor and its method, and the registry's review is what checks the
  story holds together (ADR-0013).
- Compressed bags (zstd storage plugins) are out of scope until somebody
  brings one; the refusal names the compression rather than guessing.
