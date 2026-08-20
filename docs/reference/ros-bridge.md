# The ROS 2 bridge

`slipx_ros` speaks the F1TENTH dialect for a SlipX simulation: one node,
one simulation, N agents (ADR-0050). It is not part of the wheel, because
rclpy is a ROS package rather than a pip dependency; it runs from a
sourced ROS 2 environment against an installed or in-tree `slipx`.

```bash
python -m slipx_ros.bridge --track examples/tracks/paddock_stadium \
    --car examples/cars/reference_1_10 --car examples/cars/reference_1_10 \
    --out runs/first_bridge_run
```

## Topics, per agent, under `/car_N/`

| Topic | Type | Direction | QoS |
|---|---|---|---|
| `drive` | `ackermann_msgs/AckermannDriveStamped` | in | reliable |
| `scan` | `sensor_msgs/LaserScan` | out | sensor data |
| `imu` | `sensor_msgs/Imu` | out | sensor data |
| `odom` | `nav_msgs/Odometry` | out | sensor data |
| `ground_truth/odom` | `nav_msgs/Odometry` | out | reliable |

`/clock` carries simulation time (steps times dt) for `use_sim_time`.
Topic names are the sensor names from the car's `sensors.yaml`, which the
schema has always defined as topic names; the reference car names its
sensors `scan`, `imu` and `odom`, so an existing stack connects with a
topic remap file and no code change. Frame ids are each sensor's `mount`,
prefixed with the agent namespace without a leading slash: tf2 refuses to
look up a frame that starts with one.

What the messages mean is SlipX's usual honesty on the wire. An invalid
ray is NaN in `ranges`, never a zero that reads as a wall against the
mast. `odom` is the encoder's own belief, dead-reckoned from its measured
speed and the commanded steering angle through the kinematic bicycle, so
it drifts from ground truth by exactly the slip and the actuator lag, the
way the estimate a VESC driver publishes does. Ground truth is separate,
clearly namespaced, removable at launch with `--no-ground-truth`, and the
bridge manifest records whether it was offered.

Commands hold like a servo: the latest `drive` message is applied every
step, the steering angle directly, the speed through a proportional
acceleration demand whose gain (`--speed-gain`) is a stated mechanisation
of a VESC's speed loop.

## The TF tree

REP 105 shaped, per agent (ADR-0053). `car_N/base_link` to each sensor
mount on `/tf_static`, identity because the sensor models cast from the
vehicle origin; `car_N/odom` to `car_N/base_link` on `/tf`, the dead
reckoner's belief, the same one the `odom` topic carries; and, only while
ground truth is offered, `map` to `car_N/odom`, the correction a perfect
localiser would publish, computed so the chain composes to the true pose.
Declining ground truth declines the map edge with it, and `--no-tf`
removes the broadcast entirely, for a stack that owns its tree; both
choices are recorded in the bridge manifest. A stack that runs its own
localiser publishes that same map edge, and tf2 keeps whichever authority
spoke last: the car snaps between the estimate and the truth, and the
symptom reads as a broken filter. Such a stack launches with
`--no-ground-truth` (keeping the belief edge) or `--no-tf`.

To watch a run: RViz with fixed frame `map` places every scan and pose.
Without the map edge, fix on `car_N/odom` instead, and the scan sits in
the belief's frame, off the true one by exactly as far as the belief has
drifted.

## Two ways to run

**Live** (the default): the simulation paces the wall clock
(`--real-time-factor`), latency and jitter are experienced the way a car
delivers them, and the manifests say plainly that such a run is NOT
REPRODUCIBLE beyond replay: the input log is always on, and
`sim.replay(log)` reproduces the trajectory bit for bit (ADR-0044).

**Lockstep** (`--lockstep`, ADR-0051): the simulator is the sync
authority. It announces each step index on `/race_sync/step` and advances
only when every agent has answered; a miss is ruled by
`--lockstep-timeout` and `--lockstep-policy` (`wait`, `coast`, `freeze`,
`dnf`). Commands become functions of step indices instead of the wall
clock, so the run is reproducible when the stacks are, and a client that
dawdles changes nothing. Joining costs three lines:

```python
from slipx_ros.race_sync import RaceSyncClient

sync = RaceSyncClient(self, "/car_0")   # in the node's __init__
sync.publish(msg)                        # where publisher.publish(msg) was
```

In this wrapping mode the client acknowledges steps the stack has not
answered and a command lands one step after it is made. A stack that
wants its computation itself step-synchronous passes `on_step=` and
computes when announced.

## The transport, measured

The RMW benchmark (`python -m slipx_ros.rmw_bench`) runs the same fully
sensored lockstep race, one client process per agent, over three
transport configurations, and records what a race organiser feels:
how long until everyone is in, and how fast the barrier turns.

The multicast failure mode, first, because it is why the benchmark
exists: Fast-DDS as Jazzy ships it discovers peers over UDP multicast,
and multicast is exactly what competition venues break. Access points
routinely drop or throttle multicast frames, managed switches prune
unregistered groups, and the failure is silent: every node starts
cleanly, `ros2 topic list` looks healthy on each machine separately, and
nothing ever subscribes to anything across the air. A grid that worked
on a bench cable dies on venue Wi-Fi with no error message. Both
alternatives exist to remove that failure: a Fast-DDS discovery server
(`fast-discovery-server`, with `ROS_DISCOVERY_SERVER=host:11811` on
every participant) replaces multicast discovery with unicast to a known
address, and rmw_zenoh replaces the whole transport with sessions to a
router (`rmw_zenohd`), unicast by design.

The measured answer (ADR-0052, loopback on one machine, every row
repeatable within one per cent): all three configurations carry a fully
sensored 20-agent lockstep race at about 200 barrier turns per second, so
the choice is about venue survival, not speed. rmw_zenoh costs 5 to 12
per cent against as-shipped Fast-DDS and removes the multicast dependence
by construction; the discovery server costs measurably more here and
hides the node graph from ordinary tooling unless every shell is
configured as a super client. The recommendation, therefore:

- one bench machine: Fast-DDS as shipped, nothing to configure;
- a race, or anything on venue networking: `rmw_zenohd` on the bridge
  host and `RMW_IMPLEMENTATION=rmw_zenoh_cpp` everywhere;
- a stack pinned to Fast-DDS: the discovery server, accepting its cost
  and its tooling wart.

## Multi-host

Lockstep needed no new mechanism for multiple hosts: the announcement and
the stamped commands are ordinary topics, and the simulator is the sync
authority because only it advances (ADR-0051). What a second host needs
is discovery that does not depend on multicast: run the bridge host's
`fast-discovery-server` (or `rmw_zenohd`) on an address every machine can
reach, point every participant at it, and remap topics as for any stack.
The benchmark's client processes exercise the same code path as a second
host, minus the physical wire; a measured two-machine run is planned
alongside the first external team connection (M5.7's exit condition).

## Support matrix

| ROS 2 | Status |
|---|---|
| Jazzy (Ubuntu 24.04) | tested; the bridge's test suite runs on it |
| Rolling | expected to work, untested; nothing used is Jazzy-specific |

The bridge's tests skip cleanly where rclpy is absent, so the ordinary
pytest run stays green on machines without ROS.
