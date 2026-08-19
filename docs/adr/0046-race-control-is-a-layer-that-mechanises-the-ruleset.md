# ADR-0046: Race control is a layer above the sim that mechanises the ruleset

- **Status:** Proposed
- **Date recorded:** 2026-08-19
- **Requirements:** RACE-01 to RACE-04 in spirit; `docs/spec` is not present
  in this checkout, so no ID is cited as authority.
- **Related:** [ADR-0003](0003-dependencies-point-downward.md),
  [ADR-0037](0037-sensing-never-sees-the-scene.md),
  [ADR-0042](0042-rollover-is-a-sim-event-that-freezes-the-agent.md),
  [ADR-0043](0043-contact-is-one-impulse-between-declared-footprints.md)

## Context

A race is not physics. Who won, what a crash costs, where a car restarts and
when a team is disqualified are rules written by a competition, revised
every event, and owned by people who have never read this repository. Two
architectural questions follow. Where does the referee live, and whose rules
does it enforce?

The referee needs the track (laps, limits, restart positions) and the
simulation (states, contacts, DNFs) at once. Putting it inside `slipx_sim`
couples the orchestrator to the track representation, which the LiDAR
composition pattern (ADR-0037) was specifically arranged to avoid, and makes
every embedder who wants lockstep without racing carry the ruleset. Putting
it in the Python layer makes races unavailable to the C++ reference stack
and to CI harnesses that should not need an interpreter.

On whose rules: paraphrasing a ruleset invites drift, and the RoboRacer
rules are revised per competition. Vendoring the rulebook text would
redistribute someone else's document (the licence lesson of ADR-0035).

## Decision

**Race control is a new component, `slipx_race`, above `slipx_sim` and
`slipx_scene` in the dependency order and below the bindings. It implements
the published RoboRacer ruleset,
`https://github.com/f1tenth/roboracer_rules` at revision
`202c3771465b1690c0e28618271cca91d5c842c9` (2025-10-13), tracked as a
versioned dependency: the repository and revision are constants the build
carries and prints, the rule numbers are cited at each implementation site,
and nothing of the rulebook's text is vendored.**

The pieces, and the judgment calls each one mechanises:

1. **The sim reports contacts; race control interprets them.** `Simulation`
   gains a per-step list of `ContactEvent`s (pair, point, normal, impulse,
   and each car's approach contribution), which is bookkeeping the contact
   pass already computes. What a contact MEANS is not the sim's business.
2. **Procedures as classes: `TimeTrial`, `ObstacleTest`, `HeadToHeadRound`
   and `Match`.** Each steps the simulation and applies the rules, emitting
   flat, timestamped `RaceEvent`s; the event stream task turns those into
   MCAP. Grid starts implement the side-by-side, one-car-width standing
   start of rule 2.5.1.9; a rolling start is provided as a labelled
   extension the ruleset does not define.
3. **Referee judgment is mechanised, and each mechanisation is named
   configuration, not hidden behaviour.** The ruleset says referees judge
   fault (2.5.1.14.4); here the at-fault car is the one contributing more
   approach speed at the contact, ties broken against the car behind on
   track, which is the roadmap's "relative geometry and closing velocity"
   and racing's overtaker-responsibility convention. "Light side-bumps and
   slow-speed nudges" (2.5.1.14.2) become a closing-speed threshold.
   "Excessive, repeated touching" of the border (2.5.3.1) becomes: leaving
   the corridor beyond the tolerance is a border crash, and the car is
   placed at rest where it left (2.5.3.3). "Complete stop" in the obstacle
   test (2.5.1.6.3) becomes a speed floor. Every warning follows an
   at-fault crash, and the third disqualifies (2.5.1.14.7-8), which is
   stricter than a human referee reserving warnings for malice; automated
   racing has no way to read intent and says so.
4. **Crash restarts follow 2.5.1.14.5 and .9**: both cars stopped on the
   centreline at the crash, the at-fault car two metres behind, plus one
   more when the victim is still running, which is this simulator's
   operationalisation of "autonomously recovers" (a car nobody can reach
   into either recovers autonomously or is DNF). Lap accounting survives
   restarts because the counters measure progress: a car set back two
   metres genuinely lost two metres.
5. **Walls are rules here, not physics.** Nothing in SlipX collides a car
   with a wall; the track-limits machinery is what makes the wall real, by
   penalty rather than by force. That is stated wherever it matters,
   because a user watching a car clip a wall geometrically and lose nothing
   until the corridor check fires deserves to know it is the design.

## Consequences

- The dependency order gains a layer, and `tools/dep_lint.py`, its
  operative statement, is amended in the same change: core, schema, sense,
  scene, sim, race, bindings.
- The ruleset revision is pinned, and pinning cuts both ways: when the
  competition publishes new rules, updating is a deliberate change to the
  constants and to every cited rule number, not a silent drift. Until then
  the build states which revision it implements and makes no claim about
  any other.
- Mechanised judgment is falsely precise by construction: a referee would
  not warn a car for every at-fault tap above a threshold, and no threshold
  in `RaceConfig` has been calibrated against how human referees actually
  call these races. The defaults are plausible, labelled as such, and a
  competition adopting this is expected to set its own; governance of the
  implementation stays as decided under M7.9, deferred until adoption.
- Round three's side choice is a coin flip in the rulebook (2.5.1.9.4); a
  deterministic simulator has no coins, so it derives the choice from the
  match seed and records it. A replay of the match is therefore exact,
  which is the property this project sells; the cost is that "coin flip"
  is one more mechanisation the word "referee" used to cover.
- The obstacle test and time trial are single-car procedures run by the
  same machinery, so their scenario tests double as the regression suite
  for lap counting and limits under teleports, which is where restart bugs
  would otherwise hide.
