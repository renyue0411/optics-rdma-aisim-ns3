# DCI path manager: standalone first experiment

This adds a control service to the existing Qbb/SwitchNode model. It does not
schedule business traffic, create RDMA QPs, terminate RDMA reliability, or change
the existing DCI DATA selector. The old dci-tunnel manager remains separate.

## Scope and inputs

- Each application knows its own control IPv4 address and configured peer address.
- `AddPeer(local, peer, optionalWanInterfaceAllowList)` takes a startup snapshot of
  `SwitchNode::GetRoutingTable()`, before ECMP selection. It deduplicates ports,
  retains Qbb devices, and optionally filters designated WAN interfaces.
- No route yields zero candidates. Re-registering an existing peer is idempotent.
- A candidate is `(peer, interface)`. IDs are local to a manager, not physical-path
  IDs or QP IDs. The manager supports multiple peers and at most 4096 candidates.
- V1 supports directly connected, parallel WAN links only. A reply uses the
  receiving interface. Reserved control UDP is consumed locally at Qbb receive,
  so this version must not be advertised as multihop path discovery/forwarding.
- Control addresses are assigned to loopbacks in the acceptance fixture. The
  existing SwitchNode forwarding table is populated by fixture setup, as ordinary
  topology/routing setup would do. The manager never reads remote apps, channel
  delays, error models, or the global topology to determine health or RTT.

## Probes and state

A real packet contains the fork's 14-byte PPP/Ethernet-like header, IPv4 (20), UDP
(8), and private control payload (40): 82 modeled bytes. Both UDP ports are 49000.
The reserved port pair must not be used for business QPs. Ordinary RoCE/DATA/ACK
header layouts are unchanged. This UDP protocol is private, not interoperable
QUIC, MetaRoCE, STAMP, or a new RoCE extension.

The payload serializes a magic/version, kind, local path ID, session ID, sequence,
and transmit time. A responder echoes the initiator's values. An accepted reply
must match the receiving interface/peer, pending request, session, ID, sequence,
and timestamp and arrive before the deadline. Late replies do not resurrect a
path. Sessions change on service restart.

States are UNVERIFIED, UP, DOWN, plus STOPPED for application shutdown. One valid
reply makes a candidate UP; one timed-out attempt makes it DOWN. DOWN candidates
continue probing. Timers are fixed and configurable. There is at most one request
outstanding per candidate: a tick skips pending candidates, so actual probe
frequency can be lower than `1 / interval` when RTT/timeout exceeds the interval.

All probes and replies use existing Qbb priority queue 0, sharing its real queue,
serialization, channel propagation and receive error model. Priority does not
preempt a frame already being serialized. No packet-arrival event is synthesized
from the topology delay. A bounded work queue models a configurable fixed delay
for each received control message at each end; it is not a serialized CPU server
or hardware processing-throughput model.

Reported control RTT includes local queueing, both link serializations, both
propagation delays, and configured processing delays at both ends. It is for
maintenance/diagnostics, not available bandwidth, data-queue congestion or a
business-scheduling score. There is no packet-size padding, physical Ethernet
preamble/FCS/IFG accounting beyond what Qbb itself models.

## Resource reporting

`DCI_PM_STATS` reports per-candidate state, control RTT, samples, accepted enqueue
packet/byte counts, received packet/bytes, timeouts, enqueue drops and stale replies.
TX bytes are accepted modeled frames, not payload-only bytes or actual Ethernet
wire overhead. The acceptance program independently checks physical-TX and q0
trace counts in these unloaded experiments.

`DCI_PM_RESOURCE` reports candidate count, current/peak outstanding requests,
current/peak pending processing work and overload drops. It also reports raw
snapshot storage (`sizeof(Snapshot) * candidates`) only: this excludes map nodes,
events, packets, allocator overhead, forwarding entries and ASIC memory.

The candidate limit is 4096; pending received-message work is capped at 4096;
requests have one deadline per candidate and there is one periodic tick per app.
Timer/status work is control-traffic driven, with no per-business-packet record.
Total probe traffic scales with the number of candidates; this is not a global
PPS limiter. The operator must choose interval/timeout for deployment size.

## VM application and build

Use the supplied patch for **both** source trees. First dry-run both applications
before editing either tree. From `/home/keith/myrdma/simAI-optics`:

```bash
patch_file="$PWD/experiments/patches/dci-path-manager.patch"
sim_dir="$PWD/astra-sim-alibabacloud/extern/network_backend/ns3-interface/simulation"
patch --dry-run --batch --forward -p2 -d ns-3-alibabacloud/simulation -i "$patch_file"
patch --dry-run --batch --forward -p2 -d "$sim_dir" -i "$patch_file"
```

Only if both dry-runs pass:

```bash
patch --batch --forward -p2 -d ns-3-alibabacloud/simulation -i "$patch_file"
patch --batch --forward -p2 -d "$sim_dir" -i "$patch_file"
cmake -S "$sim_dir" -B "$sim_dir/cmake-cache-nccl"
cmake --build "$sim_dir/cmake-cache-nccl" --target scratch_dci-path-manager-check -j2
pm_bin="$sim_dir/build-nccl/scratch/ns3.36.1-dci-path-manager-check-optimized"
"$pm_bin" --paths=2 --scenario=normal
```

This reuses the configured cache and existing active source; do not run the
repository-wide source-copy/build script, which could replace that active tree.
A patch failure is a source-version mismatch: stop before applying the other tree
and inspect the failing hunk. Do not force application or copy whole old files.
The patch is not intended to be applied twice.

The standalone target links the existing applications library as well as network
libraries (RDMA's SimpleSeqTsHeader currently resides in applications). If this
cache does not enable applications, preserve its existing module list while adding
applications; do not remove the Astra/SimAI application sources.

## Acceptance runs

Default links are 400 Gbps with one-way delays `245 + 500*i` microseconds. The
fixture accepts 1..8 paths per peer and 1..8 peers; these are experiment limits,
not two-path assumptions in the manager. Loopbacks/internal devices precede WAN
interfaces, so WAN indices are not assumed to be 0 or 1.

```bash
"$pm_bin" --paths=2 --scenario=failure
"$pm_bin" --paths=8 --peers=2 --scenario=normal
"$pm_bin" --paths=2 --scenario=late
"$pm_bin" --paths=2 --scenario=skew
"$pm_bin" --paths=2 --scenario=restart
```

Each successful run ends with `CHECK_OK` and exit code 0. Failure gives CHECK_FAILED
or an ns-3 abort and nonzero status. Fixed acceptance checkpoints require the
chosen timers, rate and processing delay to allow validation before 18 ms and
recovery by 55 ms; the defaults satisfy this. Arbitrary CLI timers may deliberately
fail these fixture checks, so they are not an automatic timeout-tuning feature.

- normal: route deduplication, absent route, candidate count, real RTT formula,
  actual Qbb TX/q0 traces, legacy 64-bit DATA sequence parsing, bounded requests.
- failure: at 20 ms blackhole both directions of the first WAN link by receive
  errors, leaving link status UP. At 34 ms only its candidate must be DOWN; at
  40 ms restore delivery; by 55 ms all candidates must be UP.
- late: first candidate has RTT above timeout; stale replies must not make it UP.
- skew: remote services start 5 ms late and must become available without business.
- restart: first remote stops at 20 ms, is detected down, restarts at 40 ms and is
  recovered without duplicating interface callbacks or accepting old-session state.

Use `--ccMode=3` or `--ccMode=10` for an additional guard that control payloads
are not modified as DATA INT headers. `--rate=1Gbps` checks actual serialization
in RTT; these runs are not measurements under competing business traffic.

First establish normal and failure on the VM, then expand candidate/peer counts.
Integration into the 4-GPU AllReduce experiment is a separate next step. The
standalone results must not be described as collective-performance gains.
