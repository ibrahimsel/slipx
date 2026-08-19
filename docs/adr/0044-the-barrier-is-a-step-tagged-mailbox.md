# ADR-0044: The barrier is a step-tagged mailbox, and a miss has three answers

- **Status:** Proposed
- **Date recorded:** 2026-08-19
- **Requirements:** SIM-05 in spirit; `docs/spec` is not present in this
  checkout, so no ID is cited as authority.
- **Related:** [ADR-0004](0004-step-is-const-and-stateless.md),
  [ADR-0042](0042-rollover-is-a-sim-event-that-freezes-the-agent.md),
  [ADR-0043](0043-contact-is-one-impulse-between-declared-footprints.md)

## Context

The orchestrator's policies are synchronous callables: they cannot be late,
so nothing in the sim has ever had to decide what lateness means. Real
control stacks are late. A student node misses a deadline, a process dies
mid-race, a network hiccups, and the racing phase needs an answer to the
question the in-process design never had to ask: what happens to the race
when one car's command does not arrive?

The tempting place to answer it is the ROS bridge, where the lateness will
physically happen. That puts protocol semantics inside one transport, makes
them testable only with that transport installed, and leaves the second
transport (the multi-host race, a plain socket, a test) to reinvent them.
The semantics belong below the transport: what a step needs, when a miss is
declared, and what a miss does to the agent are simulator facts.

The other contender was to leave commands synchronous and give policies a
time budget enforced by the sim. Rejected: it means the sim killing or
abandoning user code mid-call, which no portable mechanism does cleanly,
and it measures the wrong thing anyway. The race does not care why the
command is absent; it cares that the barrier came and the slot was empty.

## Decision

**Commands can arrive through a `CommandMailbox`: a thread-safe queue of
step-tagged entries, the one synchronised doorway into an otherwise
single-threaded simulation. At each step the orchestrator takes the entry
tagged with exactly that step; a missing entry is a miss, and the agent's
configured `TimeoutPolicy` answers it: `kWait`, `kFreeze`, `kCoast` or
`kDnf`.**

The pieces:

1. **Strict step tagging is the acknowledgement.** An entry tagged N says
   "here is my command for step N" (`post`) or "I am alive, hold my last
   command" (`ack`). Tags are strictly monotonic, stale entries are
   discarded on arrival at the barrier, and a controller slower than the
   step rate acknowledges the steps it has no new command for. That
   chattiness is deliberate: the barrier's whole content is one explicit
   answer per agent per step, and smoothing it over (hold-until-replaced
   with a staleness window) is a convenience layer for the `race_sync`
   client to build once the transport's reality is in hand, not a protocol
   default to bake in here.
2. **The four answers to a miss.**
   - `kWait` (the default): the barrier blocks until the entry arrives.
     One hung agent hangs the race, which is what strict lockstep means,
     and in exchange the trajectory is bit-identical whatever the timing:
     grading and RL rollouts want exactly this.
   - `kFreeze`: the agent is not stepped this step; its state, velocities
     included, is untouched, and it resumes exactly where it paused when
     commands return. A pause, not a crash: it is still in the world and
     contact still applies to it.
   - `kCoast`: the agent is stepped with the neutral input, the same
     coasting an agent with no policy gets.
   - `kDnf`: the agent is out, through exactly ADR-0042's machinery, with
     its own cause (`DnfCause::kTimeout`), frozen as a stationary
     obstacle.
   How long the barrier waits before ruling a miss for the non-wait
   policies is `SimulationConfig::barrier_timeout`, a wall-clock duration,
   zero meaning a poll.
3. **The input log is the reproducibility contract for external commands.**
   A live run with a non-wait mailbox agent is decided in part by a wall
   clock, and its manifest says so plainly instead of repeating a
   bit-identity promise that no longer holds: such a run is bit-identical
   when replayed from its input log, and only then. The log records what
   was actually applied. A coasted miss is logged as the zeros it applied.
   A frozen or DNF'd miss is logged as a NaN-tagged slot, and replay
   answers a NaN slot by applying that agent's own timeout policy, which
   reproduces the freeze or the timeout-DNF at the recorded step. NaN can
   carry this meaning because it is refused everywhere else: a mailbox
   refuses a NaN command at `post`, and a policy callable returning NaN is
   a named error, so the only NaN a log can contain is one the simulator
   wrote itself.
4. **The sim stays single-threaded.** `advance()` runs on one thread and
   owns every agent; the mailbox's mutex is the entire concurrency
   surface, and nothing else in `slipx_sim` or below grows a lock. The
   integrator remains thread-free as always.
5. **Snapshots exclude undelivered commands.** A mailbox belongs to the
   controller side of the boundary, like the policy callables it
   generalises; restoring a snapshot restores the simulation, not the
   other party's unsent mail.

## Consequences

- Two reproducibility regimes now exist and the manifest names which one a
  run was in. Policy-only runs keep the old promise. Mailbox runs with
  `kWait` keep it too, by construction. Mailbox runs with a non-wait
  policy promise reproducibility only through their log, and a consumer
  who ignores the manifest's word on this will file a determinism bug that
  is actually a scheduler.
- `kFreeze` is unphysical on its face: a car paused mid-corner holds its
  speed in suspension and resumes as if no time had passed, while the
  world moves on around it. It exists for debugging and for lenient
  practice sessions, not for scoring, and race control is free to forbid
  it.
- The strict tag-per-step contract makes a slow controller's client code
  busier (ack every step). The alternative, implicit zero-order hold,
  quietly converts "my controller crashed" into "my car drives straight
  at full throttle into the wall", which is the kind of default this
  project refuses on principle.
- The NaN log marker is in-band signalling, and it is only sound because
  the door refuses NaN commands. Anyone loosening that refusal breaks
  replay in a way no test of theirs will see; the refusal carries a
  comment saying exactly this.
- A `kWait` deadlock (the controller died and the barrier waits forever)
  is resolved by the operator, not by the sim: choosing `kWait` is
  choosing that failure mode over silent degradation.
