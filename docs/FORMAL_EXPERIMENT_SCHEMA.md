# Formal Experiment Schema

Schema version: `abs-go2-formal-run/v1`  
Normative machine-readable schema: [`../schemas/formal_experiment_run_v1.json`](../schemas/formal_experiment_run_v1.json)  
Writer/validator: [`../scripts/formal_experiment_contract.py`](../scripts/formal_experiment_contract.py)

## Run layout

```text
<run_id>/
├── manifest.json
├── telemetry.csv
├── events.jsonl
├── summary.json
└── plots/
    ├── ra_switching.svg
    ├── trajectory_obstacles.svg
    ├── command_tracking.svg
    ├── stability.svg
    └── recovery_markers.svg
```

`manifest.json` identifies the exact source inputs. `telemetry.csv` holds ordered samples. `events.jsonl` is the authoritative lifecycle and terminal-event stream; text logs are diagnostic only. `summary.json` links every artifact and records the validator classification. Plot files are required outputs, not substitutes for raw telemetry.

The validator loads `formal_experiment_run_v1.json` on every invocation. The schema rejects unexpected nested manifest fields and validates variant, SHA-256 values, seed sources, rates, thresholds and pairing-key shape. The validator then applies the schema's `x-abs-*` telemetry/event/summary requirements; these are part of the same versioned canonical contract, not a second hand-maintained field list.

## Validity model

- The validator returns `VALID`, `INVALID`, or `LEGACY / NON-ACCEPTANCE`.
- Missing, stale, malformed, incoherent, or unknown required runtime fields produce `INVALID`; they are never filled with safe-looking defaults.
- `LEGACY / NON-ACCEPTANCE` is a retained historical classification, never a formal Success.
- Deployed model SHA-256 may be known while training-server source lineage remains `UNKNOWN`; P1-01 provenance remains separately blocked and is not silently converted into a schema failure or a PASS.

## Terminal ordering

The terminal reducer evaluates validity, collision and fall before arrival. A `collision_start` or `fall` at or before `arrival_accepted` vetoes `SUCCESS`. Each run emits exactly one `terminal` event with `SUCCESS`, `COLLISION`, `FALL`, or `TIMEOUT`.

Event `sequence`, `monotonic_time_ns`, and `simulation_time_s` must be parseable, finite, non-negative and monotonic. Invalid, missing, `NaN`, or `Inf` event clocks classify the run `INVALID`; validation never raises an exception for these artifact values.

Safety evidence uses one rule across telemetry, events and terminal outcome. Each telemetry collision/fall `false → true` edge requires its corresponding `collision_start`/`fall` event inside the preceding-to-current telemetry sequence window. An unpaired telemetry edge, an event with no matching telemetry safety state, or any safety evidence with terminal `SUCCESS` is `INVALID`.

## Identity and data binding

- Manifest `run_id` must equal every telemetry row, every event, and the summary `run_id`; a wrong-run artifact is `INVALID`.
- Required telemetry contains all 11 `ray_log2_*` values and, for each of 12 joints, `action_raw_*`, `action_clipped_*`, `joint_target_rad_*`, `torque_nm_*`, and `torque_saturated_*`. Every numeric vector element must be finite.
- Summary records SHA-256 for manifest, telemetry, events and every fixed plot. The validator recomputes them.
- A plot must be a non-empty data-driven SVG carrying its run ID, telemetry/events hashes, point count and deterministic input hash. A placeholder, source-hash mismatch, empty plot, or hash mismatch is `INVALID`.

## Pairing

The pairing key is the canonical SHA-256 of scenario ID/hash, root seed, effective-config hash, and the three deployed model hashes. Each formal comparison uses the same key for `paper-faithful`, `stabilized`, and `agile-only`; mixing labels or mismatched keys is rejected.

Before any formal aggregation or comparison, run:

```bash
rtk python3 scripts/formal_experiment_contract.py --validate-comparison \
  <paper-faithful-manifest.json> <stabilized-manifest.json> <agile-only-manifest.json>
```

This comparison gate requires exactly the three distinct labels, schema-valid manifests, and one shared pairing key. A failed comparison gate prohibits aggregation.

## Validation

```bash
rtk python3 scripts/formal_experiment_contract.py <run-directory>
rtk python3 scripts/test_formal_experiment_contract.py
```

The CLI exits zero only for `VALID`. Fixture source and actual P1-02 results are recorded in [`evidence/P1-02/`](evidence/P1-02/).
