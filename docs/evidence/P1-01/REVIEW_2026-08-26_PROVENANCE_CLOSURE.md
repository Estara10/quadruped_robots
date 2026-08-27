# P1-01 Independent Re-review — Provenance Closure Evidence

Date: 2026-08-26  
Reviewer scope: read-only review of the cited recovery evidence and artifacts. No
model, controller, threshold, Roadmap, Acceptance criterion, or project-state
file was modified. This report does not replace `REVIEW.md` (2026-08-24).

## Final conclusion

**REJECT** for the complete P1-01 Acceptance.  The newly recovered evidence
credibly closes the three deployed **weight-lineage** chains, but it does not
prove deployed joint/contact/action semantic order, exact RA-to-Agile training
binding, or real Go2 foot-force slot semantics. Weight identity is not order
evidence.

## Independent checks performed

All cited paths were present. SHA-256 and byte size agreed with
`artifacts/manifest.yaml` and `provenance_recovery.json`:

| Chain | Independent result |
|---|---|
| Agile | `model_4000.pt` = 4,743,147 B, SHA `a21419…ead912`; recovered export and deployed `policy.pt` are both 801,726 B, SHA `5a87d6…e0b7cf`. Checkpoint iteration is 4000. All 8 checkpoint `actor.*` tensors exactly equal the deployed JIT tensors; architecture is `61,512,256,128,12`. |
| RA | Recovered source model = 26,047 B, SHA `ed3c75…2aaa92`; recovered JIT export and deployed `ra_value.pt` are both 32,011 B, SHA `05c40f…1a90b7`. All 6 source tensors exactly equal deployed JIT tensors; architecture is `19,64,64,1`. |
| Recovery | `model_15000.pt` = 4,595,761 B, SHA `51ee8e…3eeec5`; recovered export and deployed `policy.pt` are both 775,715 B, SHA `e3047a…b0171`. Checkpoint iteration is 15000. All 8 checkpoint `actor.*` tensors exactly equal the deployed JIT tensors; architecture is `49,512,256,128,12`. |

The recovered checkpoint metadata independently contains only
`model_state_dict`, `optimizer_state_dict`, `iter`, and `infos=None`; it does
not carry a seed, config, commit, command, or order record.

`rtk conda run -n abs python scripts/validate_p1_01_contract.py` reported
`pass=135 known=8 fail=0` and exit code 2. This is the validator's documented
Acceptance-blocked outcome, not a regression failure.

## Historical-retrieval negative conclusion

**Accepted within the stated, targeted search scope.** The two cited run
directories contain model checkpoints and one TensorBoard event file; the RA
directory contains only the source model and JIT export. The targeted recovered
Git queries found the Go2 configs and `export_rec_policy.py` untracked, no
history/object for them, no history for the cited ignored `logs/` paths, and no
commit containing either run identifier. `check-ignore` independently confirms
that the run paths are ignored.

This is negative evidence for the cited run/export directories and recovered
Git history, not proof that no record can exist elsewhere. It correctly
requires `UNKNOWN`; current untracked Go2 config, co-location, filenames,
TensorBoard curves, and visual behavior cannot be used to infer historical
order or RA binding.

RA's online in-memory queues explain why a persisted dataset is not expected
from this training path. The missing persisted sample hash, episode count, seed
and exact executed Agile binding remain `UNKNOWN`; this is not relabeled as a
missing-but-assumed dataset.

## P1-01 Acceptance review

| Criterion | Result | Reviewer basis |
|---|---|---|
| Three deployed artifacts have verified hash, dimensions and provenance status | **PASS** | Independent size/SHA/tensor checks above; remaining historical metadata is explicitly marked `UNKNOWN`. |
| Authoritative Isaac Gym DOF and foot/body capture is preserved | **PASS** | Preserved runtime capture and the contract validator's Isaac evidence checks pass. |
| All 61/19/49 fields have closed source, frame, scale, order and validity | **FAIL** | Artifact semantic order remains unknown; documented goal-shaping, frame/freshness and real-contact limitations remain. |
| Each deployed output traces to one intended Go2 motor without duplicate/omission | **UNKNOWN** | Controller-to-motor map is bijective, but the deployed tensor index semantics are not proven. |
| Contact indices trace to FR/FL/RR/RL unambiguously | **UNKNOWN** | Simulation/controller candidate mapping is proven; deployed-policy contact order and real hardware slots are not. |
| Asymmetric observation/action golden tests pass | **PASS** | Existing production-linked 61/19/49 and action/contact evidence remains recorded; this review found no contrary evidence. |
| ROS2 remap necessity is artifact-evidence-backed | **UNKNOWN** | The permutation is correct conditionally for the captured FL/FR/RL/RR training order, but that order is not bound to the recovered deployed checkpoints. |
| No Critical conclusion relies only on visual locomotion | **PASS** | This review used hashes, tensors, runtime captures and source/Git evidence only. |
| Independent Reviewer accepts evidence | **FAIL** | This re-review is REJECT for the whole Acceptance. |
| Unresolved source data remains explicit `UNKNOWN` with recorded impact | **PASS** | The manifest, recovery evidence, Current State, Gap Matrix and policy contract preserve the relevant UNKNOWNs. |

## Old blockers: status

Closed:

- Agile checkpoint → recovered export → deployed weight lineage.
- RA source model → recovered JIT export → deployed weight lineage.
- Recovery checkpoint → recovered export → deployed weight lineage.
- The old assertion that all source checkpoints/source RA model were absent.

Still open:

- Agile and Recovery deployed joint/contact/action order, and therefore
  unconditional remap necessity.
- RA exact executed Agile-checkpoint binding and exact generated training data.
- Historical run config/seed/commit/execution-environment records.
- Real Go2 `foot_force[0..3]` semantics.
- The pre-existing non-proven 61/19/49 runtime semantic gaps recorded in the
  policy contract and Gap Matrix.

## Decision and minimum action

P1-01 **must not be marked ACCEPTED**. The minimum evidence capable of closing
the P1-01 order blocker is an immutable run-local config/commit association
that binds each recovered checkpoint to the relevant Isaac Gym asset/order, or
an equivalent independent artifact-specific order record. More historical seed
or export-command recovery alone would improve reproducibility but cannot prove
tensor semantic order.

RA binding should remain `UNKNOWN` unless an execution record, embedded
metadata, or dataset/command manifest independently identifies the Agile
checkpoint. Real foot-force slots remain a hardware commissioning/safety item
and must not be asserted from simulation evidence.
