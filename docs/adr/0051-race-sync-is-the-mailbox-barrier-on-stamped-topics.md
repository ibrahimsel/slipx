# ADR-0051: race_sync is the mailbox barrier spoken over stamped topics

- **Status:** Proposed
- **Date recorded:** 2026-08-19
- **Requirements:** SIM-05 in spirit; `docs/spec` is not present in this
  checkout, so no ID is cited as authority.
- **Related:** ADR-0044 (the barrier is a step-tagged mailbox), ADR-0050
  (the bridge), ADR-0042 (DNF)

## Context

A live bridge run is decided partly by message timing, and its manifests
say so. Racing wants the opposite: twenty stacks on twenty machines whose
race is the same race when it is run again. ADR-0044 built the mechanism
below the transport (step-tagged mailboxes, acknowledgements, a wall-clock
timeout answered by a per-agent policy) precisely so that a transport
could inherit it instead of reinventing it; what was left to decide is the
wire shape, and how a student's control node joins without being rewritten.

The alternatives for the wire: a custom message package carrying explicit
step tags (a colcon build between a student and their first lockstep lap,
for one integer field); services or actions per step (a round trip per
agent per step, and the pattern F1TENTH stacks do not use); or the tag
encoded in the header stamp of the same `AckermannDriveStamped` the stack
already publishes. For the client: requiring the stack to restructure into
a compute-per-step callback (correct, and a rewrite), or wrapping the
stack's existing publish call and answering the barrier on its behalf.

## Decision

Lockstep is a bridge mode plus a small client, speaking vanilla messages.

- **The announcement.** The bridge publishes the next step index on
  `/race_sync/step` (`std_msgs/UInt64`) after every advance. The clients
  answer it; the bridge's agents run ADR-0044 mailboxes (policy `wait` by
  default, or `coast`, `freeze`, `dnf` with a wall-clock timeout), so one
  hung stack holds or forfeits exactly as the recorded design says.
- **The tag is the stamp.** A lockstep command is the same
  `AckermannDriveStamped` on the same `/car_N/drive` topic, with
  `header.stamp` set to the announced step's simulation time. At a
  millisecond step the mapping between stamp nanoseconds and step index is
  exact in both directions, so no custom message exists. An
  acknowledgement ("alive, hold my last command") is the step index on
  `/car_N/drive_ack`. A tag the mailbox refuses (stale or repeated) is
  logged and dropped by the bridge, not smuggled in.
- **The client wraps the stack.** `RaceSyncClient(node, "/car_0")`
  subscribes to the announcements; `client.publish(msg)` replaces the
  stack's `publisher.publish(msg)` and tags it with the current step; any
  announced step the stack has not answered by its next publication is
  acknowledged on its behalf, which is the servo-hold of ADR-0050 made
  step-synchronous. That is the whole integration: an import, a
  constructor, and one substituted call. A stack that wants strict
  compute-per-step semantics passes `on_step=` instead and computes when
  announced; the wrapping mode is honest about its residue (a stack whose
  own timing decides WHICH step a command lands on is deterministic only
  if that timing is, and the callback mode is the cure).
- **Translation happens at the post, safely.** The bridge's subscription
  thread translates speed to an acceleration demand using the current
  state, and the current state is stable at that moment by construction:
  integration for step k begins only after every mailbox holds an entry
  for k, so every post for k reads the state after step k minus one. The
  spin thread and the stepping thread meet only at the mailbox, which is
  the one synchronised doorway ADR-0044 built.
- **The manifest becomes honest the other way.** A lockstep run uses
  deterministic mode: commands are functions of step indices, not of the
  wall clock, so the run is reproducible as a whole when the clients are,
  and always replayable from its input log regardless.

## Consequences

- A lockstep race runs as fast as its slowest client, which is the
  contract, not a defect. The timeout policies are the escape hatch and
  they carry ADR-0042/0044's tested semantics unchanged.
- Encoding the tag in the stamp means a lockstep `drive` message's stamp
  is not a wall-clock time. Stacks do not read their own command stamps,
  and the client owns the encoding; a consumer that wants wall time has
  the announcement and `/clock`.
- The wrapping client acknowledges on the stack's behalf, so a crashed
  stack whose node still spins would coast forever under `wait`. The
  timeout policies exist for races; `wait` is for development, where a
  hang you can see beats a forfeit you cannot.
- Multi-host follows with no new mechanism: the announcement and the
  stamped commands are ordinary topics, and the simulator is the sync
  authority because only it advances. What multi-host adds is discovery
  configuration, which is the RMW benchmark's territory, recorded there.
