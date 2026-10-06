# P1-09AA — Real-Time Frame to FormalRunWriter Binding Design Audit

## Status and boundary

Design-only audit. No code was modified and no MuJoCo, ROS2, benchmark,
formal run, pilot, or real robot was run. P1-09X and P1-09Z remain BLOCKED by
the unreachable GLFW/X11 environment. This document is a proposed binding
contract, not runtime evidence and not a formal VALID run.

## 1. Identity binding

`FormalRunWriter` must allocate the formal `run_id` at recorder startup and
must own it for exactly one run directory. A `StateRL` `session_id` is a
controller-lifecycle identity assigned at `enter()`; it is not interchangeable
with `run_id`. The frame header `sequence` is a seqlock transport sequence and
must not be used as a run identity. `rl_step` is the candidate frame-order
field for formal telemetry after a stable frame has been accepted.

The future recorder should establish this binding in order:

1. allocate one writer-owned `run_id`;
2. create a run-local binding record containing `run_id` and the first accepted
   frame `session_id`;
3. require every subsequent frame and event to carry that same session;
4. derive telemetry sequence from accepted frame order/`rl_step`, while
   retaining the source frame sequence in provenance if needed;
5. close only after a matching shutdown event and complete writer flush.

There is currently no `run_id` field in `RuntimeFrame`, and no runtime recorder
implements this binding. Therefore the binding is **DESIGN REQUIRED**, not
implemented. A recorder must reject a frame if its session changes, if no
session has been bound, if the frame source is not
`AUTHORITATIVE_RUNTIME`, or if the writer run ID is not the sole run-directory
identity. A second session must start a different formal run; it must never be
merged into the first run.

An incomplete shutdown, invalidated final frame, nonzero MuJoCo exit, forced
TERM/KILL, or missing shutdown event makes the episode `INVALID`; it must not
be converted to `SUCCESS` by arrival state.

## 2. P1-02 field mapping

The classifications below describe the current evidence and the minimum
future binding requirement. Presence in the real-time frame does not upgrade a
field whose semantics or provenance are unavailable.

### Manifest

| P1-02 field | Classification | Current evidence / binding note |
|---|---|---|
| `schema_version` | AVAILABLE_BUT_ADAPTER_NEEDED | P1-02 writer/schema; recorder supplies exact version |
| `run_id` | AVAILABLE_BUT_ADAPTER_NEEDED | Writer-owned; bind to one frame session |
| `created_at` | AUTHORITATIVE_FROM_OTHER_RUNTIME_SOURCE | Recorder clock at run creation, subject to provenance capture |
| `variant` | AUTHORITATIVE_FROM_OTHER_RUNTIME_SOURCE | Explicit launch/config selection; must reject absent/ambiguous selection |
| `pairing_key` | AUTHORITATIVE_FROM_OTHER_RUNTIME_SOURCE | Deterministic manifest derivation, after all inputs are bound |
| `git.commit`, `branch`, `dirty_state`, `dirty_patch_sha256` | AUTHORITATIVE_FROM_OTHER_RUNTIME_SOURCE | Process/repository provenance, not in frame |
| `models.*` paths/hashes/provenance | UNKNOWN | P1-01 server-dependent provenance/order remains UNKNOWN |
| `effective_config` path/hash | UNKNOWN | Must come from effective runtime configuration, not a default file path |
| `environment.*` | AUTHORITATIVE_FROM_OTHER_RUNTIME_SOURCE | Binary/MJCF/assets/version/timestep/solver must be captured by runtime launcher; frame does not provide them |
| `scenario.*` | AUTHORITATIVE_FROM_OTHER_RUNTIME_SOURCE | Scenario manifest/hash/metadata must come from launch/scenario source |
| `seeds.*` | UNKNOWN | No authoritative root-seed propagation is present |
| `perception.source/version/sha256/frame_contract_version` | AVAILABLE_BUT_ADAPTER_NEEDED | Frame contract version is available; active ray source/version/hash are not fully authoritative |
| `rates_hz.*` | UNKNOWN | Frame timestamps permit later measurement, but do not themselves prove effective cadence |
| `thresholds.*` | AUTHORITATIVE_FROM_OTHER_RUNTIME_SOURCE | Must be captured from effective controller/config state, not inferred from frame values |

### Telemetry

| P1-02 field/group | Classification | Current frame mapping / gap |
|---|---|---|
| `run_id` | AVAILABLE_BUT_ADAPTER_NEEDED | Not in frame; recorder-side binding required on every row |
| `sequence` | AVAILABLE_BUT_ADAPTER_NEEDED | Use accepted frame order/`rl_step`; header seqlock sequence alone is not episode sequence |
| `simulation_time_s` | UNKNOWN | Not present in `RuntimeFrame`; must come from authoritative simulator time |
| `monotonic_time_ns` | AUTHORITATIVE_FROM_RT_FRAME | `RuntimeFrame.header.monotonic_ns`, if frame is stable and source-authoritative |
| `telemetry_fresh` | AVAILABLE_BUT_ADAPTER_NEEDED | Derived by reader using frame timestamp and a documented freshness clock |
| pose/velocity/command scalars | AUTHORITATIVE_FROM_RT_FRAME | `world_pose`, `lin_vel`, and `command`, with documented frames |
| `policy_state` | AUTHORITATIVE_FROM_RT_FRAME | Frame enum AGILE/RECOVERY/FAULTED |
| `ra_value` | AUTHORITATIVE_FROM_RT_FRAME | Frame value, finite check required |
| RA thresholds | AUTHORITATIVE_FROM_OTHER_RUNTIME_SOURCE | Not carried by frame; capture effective config |
| actual/recovery velocity and constraint margin | AVAILABLE_BUT_ADAPTER_NEEDED | Some frame values exist, but exact P1-02 semantic mapping needs adapter definition |
| `ray_valid`, `ray_age_ns`, `ray_log2[11]` | AUTHORITATIVE_FROM_RT_FRAME | Available when frame is authoritative; `ray_origin` must also be checked |
| `controller_active`, `rl_active` | AUTHORITATIVE_FROM_RT_FRAME | Frame flags, consistency-checked |
| `collision_available`, `collision` | UNKNOWN | Frame explicitly reports collision origin unavailable; bridge/MuJoCo source is not bound |
| `fall`, `arrival_candidate` | UNKNOWN | Not present as authoritative frame fields |
| `action_raw[12]` | AUTHORITATIVE_FROM_RT_FRAME | Frame command chain, policy order documented in contract |
| `action_clipped[12]` | AUTHORITATIVE_FROM_RT_FRAME | Frame command chain, controller order documented |
| `joint_target_rad[12]` | AUTHORITATIVE_FROM_RT_FRAME | Frame command chain, controller order documented |
| `torque_nm[12]` | AUTHORITATIVE_FROM_RT_FRAME | Frame command chain, finite check required |
| `torque_saturated[12]` | UNKNOWN | Frame flag says not computed; zero values cannot be interpreted as unsaturated |

### Events, summary and plots

| P1-02 artifact/field | Classification | Required handling |
|---|---|---|
| event `run_id` | AVAILABLE_BUT_ADAPTER_NEEDED | Recorder adds and validates against writer identity |
| event sequence/times | AVAILABLE_BUT_ADAPTER_NEEDED | Derived from accepted frame/authoritative runtime clock; monotonicity required |
| `episode_start` | AVAILABLE_BUT_ADAPTER_NEEDED | Recorder event after binding and runtime readiness |
| `controller_active`, `rl_entered`, `valid_ready` | AUTHORITATIVE_FROM_RT_FRAME | Emit only from coherent frame state transitions; `valid_ready` also requires all formal fields |
| collision/fall events | UNKNOWN | Need authoritative MuJoCo/bridge safety event source |
| `arrival_accepted`, `timeout`, `terminal`, `shutdown` | UNKNOWN | Need authoritative state/event producer and complete shutdown evidence |
| summary `schema_version`, `run_id`, `validity` | AVAILABLE_BUT_ADAPTER_NEEDED | Writer/validator can enforce identity and validity |
| summary `terminal_outcome` and `invalid_reasons` | UNKNOWN | Must be produced by safety-first reducer, never inferred from arrival alone |
| summary `metrics` | AVAILABLE_BUT_ADAPTER_NEEDED | Reducer can calculate only from complete authoritative telemetry/events |
| summary artifact paths/hashes | AVAILABLE_BUT_ADAPTER_NEEDED | Writer can bind hashes to this run; runtime data must exist and be non-placeholder |
| `ra_switching.svg`, `trajectory_obstacles.svg`, `command_tracking.svg`, `stability.svg`, `recovery_markers.svg` | AVAILABLE_BUT_ADAPTER_NEEDED | Data-driven plots from this run only; no placeholder satisfies validity |

## 3. Formal fail-closed rules

The future recorder/adapter must reject or invalidate when any of these occur:

- frame status is not `LIVE`, source is not `AUTHORITATIVE_RUNTIME`, or the
  frame is stale, incoherent, non-finite, unarmed, or contract-version-invalid;
- no session has been bound, session changes, sequence regresses/skips under
  the chosen continuity rule, or any accepted time is non-monotonic;
- required manifest provenance is absent, including seed, effective config,
  model identity/order, scenario, or active ray source;
- collision availability, collision truth, fall truth, torque saturation,
  measured cadence, or simulation time is unavailable where required by the
  formal schema;
- bridge/controller shutdown is incomplete, MuJoCo exits nonzero, TERM/KILL is
  required, or a matching shutdown event is absent;
- a safety event or telemetry safety transition exists but the reducer claims
  `SUCCESS`; safety outcome always precedes arrival/outcome ordering;
- any artifact is missing, placeholder-only, has an identity/hash mismatch, or
  cannot be traced to the same writer run ID and telemetry/event source.

No default `0`, maximum ray distance, inferred seed, inferred collision=false,
or arrival flag may repair an unresolved field. The only acceptable result is
`INVALID` (or an explicit pre-run `UNKNOWN` that cannot be submitted).

## 4. Minimum future implementation boundary

Allowed observational work: a recorder-side reader for the stable seqlock
frame, session binding, frame-to-telemetry projection, event transition
reduction, manifest provenance capture, data-driven plot generation, and
offline validator invocation. It must not alter controller outputs or policy
semantics.

Before a formal `VALID` run is possible, the project still needs a reachable
MuJoCo runtime (P1-09X/Z are blocked), authoritative simulation time and
scenario/config/seed provenance, collision/fall event wiring, active ray-source
provenance, measured cadence, complete shutdown event, and a reviewer-approved
runtime adapter. P1-01's server-dependent model provenance/order UNKNOWN also
remains unresolved.

An accepted frame/HUD session, a synthetic adapter fixture, or a legacy
evaluator result cannot produce a formal VALID run. This design does not close
P1-09, P1-02 runtime integration, or Phase 1.

## Recommendation

An independent Reviewer should inspect this identity and field-boundary design
before implementation. The first implementation should be limited to the
recorder-side binding and offline fixtures; runtime integration should wait for
the GLFW/clean-shutdown gate and explicit authoritative sources.
