# ADR-0052: the recommended racing transport is rmw_zenoh, measured against both Fast-DDS shapes

- **Status:** Proposed
- **Date recorded:** 2026-08-20
- **Requirements:** the RMW decision the roadmap's M7.7 names; `docs/spec`
  is not present in this checkout, so no ID is cited as authority.
- **Related:** ADR-0050 (the bridge), ADR-0051 (race_sync)

## Context

ROS 2 Jazzy ships Fast-DDS with multicast discovery, and multicast is
exactly what competition venues break: access points drop or throttle it,
managed switches prune it, and the failure is silent, with every node
healthy on its own machine and nothing subscribing across the air. Two
escapes exist. A Fast-DDS discovery server replaces multicast discovery
with unicast to a known address while keeping the default RMW; rmw_zenoh
replaces the transport with sessions to a router, unicast by design. The
roadmap requires the default to be benchmarked and decided rather than
adopted by fashion, at 6 and 20 agents.

The benchmark (`python -m slipx_ros.rmw_bench`) runs the same fully
sensored lockstep race under each configuration: a bridge process plus one
client process per agent, every agent answering every step through the
race_sync barrier, so the row differences are the transport's. Loopback on
one machine cannot exhibit the multicast failure itself (localhost
multicast works); what it measures is what the transports cost when they
work, and the failure mode is documented from its mechanism in
`docs/reference/ros-bridge.md`.

## Decision

The documented default for racing is **rmw_zenoh**; a single bench machine
needs nothing (Fast-DDS as shipped is fine there); the Fast-DDS discovery
server is the fallback for a stack that must stay on the default RMW.

The numbers the decision rests on, measured 2026-08-20 on a Ryzen 7
7800X3D under WSL2 (Ubuntu 24.04, ROS 2 Jazzy), all processes on one
host, 300 lockstep steps, best understood as barrier round trips per
second; a repeat run agreed within one per cent on every row:

| configuration | agents | lockstep steps/s |
|---|---|---|
| Fast-DDS multicast (as shipped) | 6 | 634 |
| Fast-DDS discovery server | 6 | 370 |
| rmw_zenoh | 6 | 556 |
| Fast-DDS multicast (as shipped) | 20 | 210 |
| Fast-DDS discovery server | 20 | 179 |
| rmw_zenoh | 20 | 199 |

Discovery time after the bridge was up was under ten milliseconds for
multicast and zenoh and about half a second for the discovery server, at
both sizes, on loopback.

The reasoning:

- zenoh removes the multicast dependence by construction and costs 5 to
  12 per cent of barrier throughput against multicast Fast-DDS, which
  loses at a venue anyway.
- The discovery server also removes it, but measurably costs more here
  (42 per cent at 6 agents, repeatably), and it carries the operational
  wart that graph tooling (`ros2 topic list` and friends) sees nothing
  unless configured as a super client, which is precisely the kind of
  silent-looking failure a competition pit does not need twice.
- All three carry a 20-agent fully sensored lockstep race at about 200
  barrier turns per second on this machine, which also answers ADR-0050's
  deferred throughput question with a first number: the Python bridge and
  twenty Python client processes together turn the barrier in about five
  milliseconds, transport included, so the transport choice is about
  venue survival, not speed.

## Consequences

- The docs recommend one daemon (`rmw_zenohd`) and one environment
  variable (`RMW_IMPLEMENTATION=rmw_zenoh_cpp`) for race days, and say
  when the plain default is fine. Nothing in `slipx_ros` depends on the
  choice; the bridge runs on whatever RMW the environment names.
- The numbers are loopback numbers from one machine, and they say nothing
  about venue radio; the failure mode itself is documented from mechanism,
  not measured, and a two-machine measurement rides with the first
  external team connection (M5.7's exit condition).
- A future Jazzy or rolling release that changes RMW defaults or fixes
  the discovery server's cost re-opens this record by superseding it; the
  benchmark stays in the tree so the re-measurement is one command.
