# Current State

## Current Phase

Phase 1 — MuJoCo Simulation Validation

## Current Task

Director-selected next eligible engineering task: P1-09 — Runtime Source Connection to the Formal Contract

P1-09 status: **EXECUTING — P1-09A read-only field-source audit and interface design only**. First increment of [`exec-plans/P1-09.md`](exec-plans/P1-09.md); produces the [field-to-runtime source map](evidence/P1-02/field_to_runtime_source.md) and an [adapter-interface note](evidence/P1-09/adapter_interface_note.md). No code changes and no runtime adapter implementation are authorized.

P1-03 status: **ACCEPTED / COMPLETED** — offline paper-to-code trace accepted by [final independent review](evidence/P1-03/REVIEW_2026-08-27_FINAL.md) on 2026-08-27. 11 records: 1 MATCH, 4 STABILIZED_VARIANT, 4 MISMATCH, 1 UNKNOWN, 1 CONFLICT. Not paper-equivalence proof, runtime validation, benchmark evidence, or Phase 1 Acceptance.

P1-02 status: **ACCEPTED / COMPLETED** — the [final independent review](evidence/P1-02/REVIEW_2026-08-26_FINAL.md) accepts the offline fixture-level formal contract only. Runtime adapter remains incomplete; existing evaluator output remains `LEGACY / NON-ACCEPTANCE`; no benchmark or formal runtime result is claimed.

Open blocked task: **P1-01 — Policy Artifact Provenance and Joint/Contact/Action Order Contract**

Status: **BLOCKED / PARTIALLY COMPLETE — 61/19/49 parity and all local/live P1-01F deployment-contract checks PASS; recovered checkpoint/source→export→deployed weight lineage is verified, while historical artifact order, RA exact Agile binding, historical run metadata and real foot-force semantics remain `UNKNOWN`**

State model and role boundaries: [`PROJECT_STATE_MODEL.md`](PROJECT_STATE_MODEL.md)

## Phase Acceptance

- Phase 1: **NOT ACCEPTED**
- Phase 2: **NO-GO**
- Phase 3: **NOT STARTED**

## Critical Blockers

- Recovered Agile/Recovery checkpoints and exports and the RA source/JIT artifacts close deployed **weight lineage** by exact tensor and byte equality. Targeted run-directory, TensorBoard, embedded-metadata and recovered-Git searches found no immutable historical config/command/seed/commit record and no independent RA loaded-checkpoint record; deployed artifact order and RA exact Agile binding remain `UNKNOWN`.
- Real Go2 `foot_force[0..3]` semantics are not independently captured.
- Isaac Gym Go2 `terminate_after_contacts_on=["base"]` currently matches no runtime body.
- Recovery solver and switching contain known paper mismatches.
- P1-02 formal experiment contract is **ACCEPTED / COMPLETED** for offline schema, writer/validator, comparison-gate, and fixture-level evidence; authoritative runtime event, telemetry, seed and provenance sources are not yet connected; existing evaluator outputs remain `LEGACY / NON-ACCEPTANCE`. This is not runtime benchmark evidence or Phase 1 Acceptance.

## P1-01 Evidence

- Three deployed hashes, installed bindings, executable shapes and deterministic outputs: **PASS**.
- Recovered weight lineage: Agile `model_4000.pt` actor → recovered export → deployed artifact, RA source model → recovered JIT → deployed artifact, and Recovery `model_15000.pt` actor → recovered export → deployed artifact: **PASS** by exact tensor/byte equality.
- Historical evidence retrieval: scoped run directories contain checkpoints plus TensorBoard events but no config/hparams/command/metadata sidecars; checkpoints contain no seed/config/commit/order metadata; recovered Git has no committed Go2 config/export snapshot or run identifier: **artifact order and historical run metadata remain `UNKNOWN`**.
- RA exact executed Agile checkpoint binding: source-code/filename candidate only; no execution log, command record, embedded metadata or dataset manifest: **UNKNOWN**.
- Isaac Gym DOF/body/feet order and ROS2→motor→MuJoCo mapping: **PASS**.
- Current remap is bijective and correct for the captured training order, but correctness for the actual deployed artifacts: **UNKNOWN**.
- P1-02 run-ID closure: duplicate run IDs are rejected at comparison CLI level; FormalRunWriter allocates distinct process-local UUID4 IDs, rejects caller-supplied IDs that differ from the allocation, and `write_summary()` fails before create/overwrite on a mismatch while defaulting omitted IDs to the writer allocation. Evidence: [`p1_02_mechanical_tests.json`](evidence/P1-02/p1_02_mechanical_tests.json).
- P1-02 Acceptance: **ACCEPTED / COMPLETED** — [final independent review](evidence/P1-02/REVIEW_2026-08-26_FINAL.md) accepts the offline formal-contract schema, writer/validator, comparison gate, and 22 fixture-level mechanical tests. This does not constitute runtime benchmark evidence, a formal runtime result, or Phase 1 Acceptance; runtime and legacy limitations remain unchanged.
- P1-03: **ACCEPTED / COMPLETED** — offline paper-to-code trace accepted by [final independent review](evidence/P1-03/REVIEW_2026-08-27_FINAL.md); not paper-equivalence, runtime, benchmark, or Phase 1 evidence. It is not blocked by P1-01 under the Roadmap dependency conclusion.
- P1-01F corrected rolling timer, contact temporal filter, nominal bias, fail-closed ray freshness and finite-value vetoes; helper-level fault tests **PASS**.
- Live ROS2+MuJoCo P1-01F: normal writer, writer freeze/exit, ray NaN/Inf, observation/RA/action/target/final-command non-finite injections all **PASS**. Timing is one `steady_clock` domain; 200 ms freshness + one 20 ms ray-check interval is met. Telemetry proves finite post-veto targets and zero Kp/Kd/torque.
- Contract: [`POLICY_IO_CONTRACT.md`](POLICY_IO_CONTRACT.md). Goal shaping remains **INTENTIONAL ENGINEERING VARIANT**. Historical deployed-artifact order, RA exact Agile binding and real Go2 foot-force slot order remain `UNKNOWN`.
- Latest independent P1-01 re-review: **REJECT** — recovered Agile/RA/Recovery weight lineage is verified, but deployed artifact order, RA exact Agile binding, historical run metadata, real Go2 foot-force semantics and pre-existing semantic gaps remain `UNKNOWN`. Evidence: [`REVIEW_2026-08-26_PROVENANCE_CLOSURE.md`](evidence/P1-01/REVIEW_2026-08-26_PROVENANCE_CLOSURE.md). No P1-01 closure or further re-review is pending until new server-side evidence exists.

## Current Metrics

- All existing simulation results: **LEGACY / NON-ACCEPTANCE**.
- Historical paired report: Full ABS 38/40; Agile-only 30/40. No matched seeds or true-contact metric at that time.
- Later true-contact session: 5/6 with one collision.
- Latest four-scene session: 3/4 with one terrain fall.
- The old 12/12 result is an arrival baseline, not a formal collision-free score.

## Known Dirty Changes

- User deletion of `paper.txt`.
- User path gains changed from 3.0 to 2.5.
- User real-network comment changed to `enp7s0`.
- Untracked legacy report generator with stale claims; intentionally not committed.

## Real Robot Gate

Allowed: `PASSIVE`, `FIXEDDOWN`, `FIXEDSTAND`, emergency-stop checks and software dry-run.

ABS/RL real test: **NO-GO**

## Next

Active Engineering Task: **P1-09 — Runtime Source Connection to the Formal Contract** is **EXECUTING (P1-09A read-only audit only)** under [`exec-plans/P1-09.md`](exec-plans/P1-09.md). P1-03 is **ACCEPTED / COMPLETED** (offline trace only). P1-01 remains `BLOCKED`; no Phase 1 Acceptance, benchmark, formal runtime result or Phase-2 ABS/RL is authorized.
