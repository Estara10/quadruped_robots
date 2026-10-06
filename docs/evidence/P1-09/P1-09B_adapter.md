# P1-09B — Minimal Formal Runtime Adapter Boundary

Date: 2026-08-28
Scope: offline, additive implementation increment of P1-09. No runtime process was run.
Status: P1-09 remains **EXECUTING**; this increment is not an Acceptance claim.

## What was implemented

A minimal, additive Python adapter boundary over the already-accepted P1-02
`FormalRunWriter`:

- `scripts/formal_runtime_adapter.py` — `FormalRuntimeAdapter` (plus
  `AdapterValidationError`) with an explicit input-origin classification boundary.
- `scripts/test_formal_runtime_adapter.py` — synthetic adapter-contract tests.

No change was made to `FormalRunWriter`, the P1-02 schema, the validator, the
controller, policy, thresholds, gains, switching, solver, dynamics, or any config.

## Input-origin boundary

Every manifest context and telemetry snapshot must carry an explicit origin:

| Origin | Meaning | Adapter behavior |
|---|---|---|
| `SYNTHETIC_TEST` | test-only fixture | accepted (writable), recorded as `synthetic-test-only` in `adapter_origin.json` |
| `LEGACY_ONLY` | legacy evaluator output | rejected before any write |
| `AUTHORITATIVE_RUNTIME` | reserved for a real producer | rejected — not available until a runtime producer is wired |
| missing / unrecognized | — | rejected before any write |

This closes the legacy-origin gap: a complete, well-formed legacy payload is still
rejected the moment it is declared `LEGACY_ONLY`, so legacy evaluator data cannot
pass through the adapter as formal data.

## Adapter contract

`FormalRuntimeAdapter` guarantees:

1. **Owns one run_id** — wraps a single `FormalRunWriter` and exposes its
   allocated `run_id`; it never accepts a caller-supplied replacement.
2. **Explicit inputs only** — `bind_manifest(context)` and
   `append_telemetry(snapshot)` accept caller-provided values; the adapter
   provides no defaults.
3. **Writes only through `FormalRunWriter`** — manifest and telemetry artifacts
   are written exclusively by the P1-02 writer.
4. **Fails closed before write** — `append_telemetry` validates before buffering,
   and `write_telemetry` flushes nothing when no snapshot was accepted:
   - wrong `run_id` → `AdapterValidationError` (nothing written);
   - missing manifest section or missing telemetry field → rejected;
   - non-finite numeric (all scalar + 11-ray + 5×12 vector columns) → rejected;
   - malformed boolean flag → rejected;
   - non-monotonic `sequence` / `monotonic_time_ns` / `simulation_time_s` → rejected.
5. **Never invents values** — a manifest numeric field supplied as `UNKNOWN` or
   `None` is rejected (`...has no authoritative source and must not be invented`);
   `models.*.source_provenance` `UNKNOWN` is preserved because the schema permits it.
6. **Input-origin boundary** — every input must carry an explicit origin;
   `LEGACY_ONLY`, `AUTHORITATIVE_RUNTIME`, missing, and unrecognized origins are
   rejected before any write; only `SYNTHETIC_TEST` is writable.

## Validation results (all offline)

| Command | Result |
|---|---|
| `python3 scripts/test_formal_experiment_contract.py` | **22/22 PASS** (existing P1-02 suite remains green) |
| `python3 scripts/test_formal_runtime_adapter.py` | **16/16 PASS** |
| `python3 -m py_compile scripts/formal_runtime_adapter.py scripts/test_formal_runtime_adapter.py` | **PASS** |
| `git diff --check` | **clean** |

Adapter test coverage maps to the required checks:

- writer-owned run ID binds manifest and telemetry → `test_writer_owned_run_id_binds_manifest_and_telemetry`
- mismatched run ID rejected before write → `test_mismatched_run_id_is_rejected_before_write`
- missing required fields fail closed → `test_missing_required_manifest_section_fails_closed`, `test_missing_required_telemetry_field_fails_closed`
- non-finite 11-ray values fail closed → `test_nonfinite_11_ray_values_fail_closed`
- non-finite 5×12 command-chain values fail closed → `test_nonfinite_command_chain_values_fail_closed`
- legacy evaluator artifacts cannot be upgraded to VALID → `test_legacy_evaluator_artifact_cannot_be_upgraded_to_valid`, `test_adapter_never_upgrades_partial_artifacts_to_valid`
- legacy-shaped input rejected before write → `test_legacy_origin_rejects_manifest_and_snapshot_before_write`
- missing origin rejected before write → `test_missing_origin_rejects_before_write`
- unrecognized origin rejected before write → `test_unrecognized_origin_rejects_before_write`
- authoritative runtime origin not yet available → `test_authoritative_runtime_origin_not_available_until_wired`
- synthetic origin recorded as test-only → `test_synthetic_origin_is_recorded_as_test_only`
- existing P1-02 suite green → ran the suite above

## Synthetic-only fields (test fixtures, not runtime evidence)

All adapter-test inputs are synthetic fixtures. In particular the following are
supplied by test code only and carry **no runtime authority**:

- `created_at`, `git.commit/branch/dirty_state`, model paths/hashes,
  `effective_config`, `environment.*` hashes/version/solver, `scenario.*`,
  `perception.source/version`, and every numeric `rates_hz.*` value — all fixture
  constants.
- `seeds.root_seed` and `seeds.sources.*` — fixture integers, not a captured root
  seed lineage.
- telemetry `sequence`, `simulation_time_s`, `monotonic_time_ns`,
  `telemetry_fresh`, pose, `ra_value`, command/actual velocity, `ray_log2_*`,
  and the 5×12 command chain — all synthesized finite numbers/booleans in the test
  fixtures.

## Fields that still lack an authoritative runtime producer

The adapter validates shape/finiteness/consistency; it does **not** resolve
provenance. The following remain `UNKNOWN`/`LEGACY_ONLY` per the P1-09A
field-to-runtime-source map and are not produced by any runtime code:

- `simulation_time_s`, per-row `sequence`, per-row `monotonic_time_ns`,
  `telemetry_fresh` — no producer exists (telemetry cannot yet be `VALID`).
- `torque_saturated_*` (12) — no per-joint saturation flag is computed.
- measured cadence (`rates_hz.*`) and active ray source mode — static config is
  not measurement; remain `UNKNOWN`.
- `seeds.root_seed` / downstream `sources.*` — no root seed is captured.
- `models.*.source_provenance` — P1-01 lineage remains `UNKNOWN`.
- structured safety events: `episode_start`, `valid_ready`, `shutdown` (UNKNOWN);
  `fall`, `arrival_accepted`, `timeout`, `terminal`, `summary.metrics`,
  `summary.terminal_outcome` (LEGACY_ONLY).

## Confirmation

No ROS2, MuJoCo, benchmark, pilot, formal episode, or real-robot process was run.
This increment is offline adapter code + synthetic tests only.
