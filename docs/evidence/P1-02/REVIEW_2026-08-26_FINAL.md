# P1-02 Final Independent Review — Formal Experiment Contract

Date: 2026-08-26  
Reviewer: Independent Reviewer  
Scope: read-only review of the P1-02 offline formal-contract implementation,
schema, fixtures, and evidence. No simulation, benchmark, pilot, real-robot
test, controller, policy, threshold, Roadmap, Acceptance criterion, exec plan,
or project-state file was modified by this review.

## Decision

**ACCEPT**

P1-02's offline formal-contract Acceptance is satisfied. This accepts the
versioned schema, writer/validator behavior, comparison gate, and deterministic
fixture evidence only. It is not runtime benchmark evidence and does not imply
Phase 1 Acceptance.

## Independent verification

The Reviewer ran:

| Command / check | Result |
|---|---|
| `rtk python3 scripts/test_formal_experiment_contract.py` | **PASS** — 22/22 mechanical tests. |
| `rtk python3 -m py_compile scripts/formal_experiment_contract.py scripts/test_formal_experiment_contract.py` | **PASS**. |
| `rtk python3 -m json.tool schemas/formal_experiment_run_v1.json` | **PASS**. |
| `rtk git diff --check` | **PASS**. |
| `/tmp` summary-ID check | **PASS** — a mismatched summary ID raises before write; an existing summary is unchanged; a new writer creates no summary. |

## Acceptance review

| P1-02 Acceptance criterion | Result | Evidence |
|---|---|---|
| Every formal episode is mechanically classified `VALID` or `INVALID` with explicit reasons | **PASS** | Validator and malformed/non-finite event-clock fixtures. |
| `VALID` episodes contain required identity, provenance, telemetry, events, summary, and fixed plots | **PASS** | Schema, artifact/hash validation, 11 rays and 5×12 vector fixtures. |
| Tests prove safety precedence and reject corrupted/incomplete/mismatched runs | **PASS** | Telemetry/event/terminal consistency checks veto `SUCCESS` for any collision/fall evidence. |
| Seed lineage and paired-variant keys are recorded and verified | **PASS** | Deterministic derivation and pairing fixtures. |
| `paper-faithful`, `stabilized`, and `agile-only` cannot be silently merged | **PASS** | Comparison CLI requires all three labels, one pairing key, and distinct run IDs; missing/duplicate labels, mismatch, and duplicate IDs exit non-zero. |
| Historical runs remain `LEGACY / NON-ACCEPTANCE` | **PASS** | Legacy fixture and protocol classification. |
| P1-01 `UNKNOWN` remains explicit; no benchmark/Acceptance performance claim is made | **PASS** | Schema permits unresolved provenance as `UNKNOWN`; evidence and protocol retain legacy/runtime boundaries. |
| An independent Reviewer can reproduce the validator decision | **PASS** | Commands above and `p1_02_mechanical_tests.json`. |

## Run-ID closure

`FormalRunWriter` allocates schema-valid `run-<UUID4 hex>` IDs with a
process-local issued-ID registry. It rejects caller-provided manifest or summary
IDs that differ from its allocation, and binds the same ID to manifest,
telemetry, events, summary, and plot provenance. Comparison validation rejects
duplicate run IDs.

The uniqueness boundary is correctly limited: the registry is process-local;
cross-process/cross-machine uniqueness is UUID4 probabilistic uniqueness plus
persisted artifact identity, not a distributed lock or central registry.

## Required continuing boundaries

- No authoritative runtime adapter yet emits the complete run-bound
  telemetry/events/seeds/provenance/plots contract.
- Existing evaluator outputs remain `LEGACY / NON-ACCEPTANCE`.
- P1-01 artifact order/provenance remains `BLOCKED / UNKNOWN`.
- This report and its 22 mechanical tests are fixture-level offline evidence,
  not a runtime benchmark, pilot, or Phase 1 Acceptance result.

