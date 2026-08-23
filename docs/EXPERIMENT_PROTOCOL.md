# Formal Experiment Protocol

This protocol defines what may enter project Acceptance statistics. Visual behavior, interactive smoke tests and historical logs remain useful diagnostics but are not formal evidence unless they satisfy this document.

## Run Identity

Every formal run has a globally unique `run_id` and records:

- schema version;
- run ID and wall-clock timestamp;
- monotonic/simulation start and end time;
- Git commit, branch and complete dirty state or dirty patch hash;
- model names, paths and SHA-256 hashes;
- effective config snapshot and hash;
- MuJoCo binary hash, version, timestep and solver configuration;
- Go2 MJCF and asset version/hash;
- scenario ID, scenario file/hash and obstacle metadata;
- random seed and every downstream random source;
- perception/ray source, source version and model hash;
- controller, PD, policy, RA and perception rates;
- all outcome thresholds.

## Required Runtime Validity

Before an episode may start:

- simulator/robot state is fresh and belongs to this run;
- controller is active;
- FSM has explicitly entered the requested RL state;
- Agile, RA and Recovery models pass load/shape checks;
- perception source is identified, valid and fresh;
- collision telemetry is available under the registered definition;
- configuration seen by the controller matches the run manifest.

## Required Episode Artifacts

### `manifest.json`

Contains all run identity, hashes, rates, thresholds and environment metadata.

### `telemetry.csv` or equivalent columnar data

At minimum:

- simulation/monotonic time and sequence;
- base pose, height, roll, pitch and yaw;
- policy state: Agile or Recovery;
- RA Value and entry/exit thresholds;
- complete goal/policy command and actual base velocity;
- Recovery twist and final RA constraint margin;
- 11 rays plus freshness/validity;
- raw and clipped actions, joint targets and torque/saturation data;
- collision, fall and arrival state.

Sampling must preserve every switching boundary. A throttled text log alone is insufficient.

### `events.jsonl`

Structured events with time, sequence and reason:

- episode start and valid-ready;
- controller active and RL entered;
- Recovery ENTER and EXIT;
- collision START and END;
- fall;
- arrival start and accepted arrival;
- timeout;
- sensor/perception/controller invalidation;
- shutdown.

### `summary.json`

At minimum:

- validity and invalid reason;
- Success/Collision/Fall/Timeout result;
- time to goal;
- path length and efficiency;
- switch count;
- total, mean and maximum Recovery duration;
- minimum clearance;
- command-tracking metrics;
- RA min/max/mean and threshold behavior;
- stability statistics;
- links to all raw artifacts.

### Fixed plots

- RA + thresholds + policy state;
- trajectory + obstacle footprint;
- command versus actual velocity;
- roll/pitch/base height;
- Recovery duration/switch markers.

HUD data is operational only; it never replaces stored telemetry.

## Outcome Ordering

At every sample, process telemetry validity and safety outcomes before accepting arrival. A robot that contacts an obstacle or falls while crossing the goal region is not a Success.

Success requires:

1. fresh valid telemetry;
2. within the registered goal region;
3. no collision or fall;
4. upright stability for the preregistered arrival hold.

## INVALID Episode

An episode is `INVALID` and excluded from numerator and denominator when any required condition is missing, including:

- telemetry stale, malformed or from an earlier run;
- collision telemetry unavailable;
- controller not active or RL not actually entered;
- perception/ray source unknown, stale or invalid;
- seed not recorded or not propagated as declared;
- model/config/scene/binary version or hash unknown;
- startup failure or wrong hardware mode;
- required structured events missing;
- evaluator and controller use different goals or thresholds.

Invalid episodes are reported separately; they are never silently retried until a desired result appears.

## Seeds and Comparisons

- Fixed-seed replay must reproduce scene construction and discrete event sequence within documented deterministic limits.
- Paired variants use the same seed list and scenario list.
- Pilot seeds are separated from formal Acceptance seeds.
- Formal thresholds and analysis rules are frozen before examining formal outcomes.

## Experiment Variants

Every run declares one variant. At minimum:

- `paper-faithful`: paper threshold and logic without unreported path/hysteresis/hold additions;
- `stabilized`: explicitly declared engineering additions;
- `agile-only`: Recovery disabled by a registered ablation mechanism.

Results from different variants are never merged without stratification.

## Historical Data

Runs created before this protocol are retained as `LEGACY / NON-ACCEPTANCE` unless every required field can be reconstructed without assumption. The historical 12/12 arrival baseline, Full 38/40 and Agile-only 30/40 reports remain diagnostic only.

## Storage Policy

- Raw telemetry, videos, bags and large model outputs stay outside ordinary Git.
- Small reviewed Acceptance summaries and plots may be committed with run IDs and manifest hashes.
- Artifact restoration follows `artifacts/manifest.yaml`.
- Credentials, network secrets and machine-specific private data are never included.
