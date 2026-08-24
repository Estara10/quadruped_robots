# P1-01 Independent Review

Date: 2026-08-24  
Result: **REJECT**

The Reviewer performed a read-only inspection of the P1-01 manifest, contract, runtime captures, validator and directly cited training/deployment code. The Reviewer reran the validator and Isaac Gym CPU asset capture. No file was changed by the Reviewer.

## Findings

### Critical

- Agile/Recovery source checkpoint and export lineage is not closed. RA source model, dataset and exact deployed Agile binding are also absent. The actual deployed policy order and unconditional remap necessity therefore remain `UNKNOWN`.

### High

- The asymmetric observation checks validate recorded slices and source literals, but do not invoke independent training and ROS2 observation builders from one physical state. They are not implementation-level 61/49/19 golden parity tests.
- Recovery training subtracts `dof_bias`, while inline and manual deployment Recovery subtract only the default pose. This mismatch was added to the contract after review.
- Deployed-policy contact order and real Go2 foot-force slot semantics remain `UNKNOWN`; foot-force interfaces lack the explicit sorting/assertion used for joints.
- Before this review, the evidence files were untracked. They must be committed before the evidence can survive a clean checkout.

### Medium

- The Isaac Gym capture proves the current environment and asset options, not the unavailable historical training environment; the exact Isaac Gym release is `UNKNOWN`.
- The regression validator can pass while Acceptance remains blocked. It must report this distinction explicitly.
- A ROS2 YAML comment incorrectly claimed the FR-first controller order matched Isaac Gym. The comment was corrected after review; runtime behavior was not changed.

## Acceptance

| Criterion | Result |
|---|---|
| Artifact hashes/dimensions/provenance status recorded | PASS |
| Isaac Gym runtime order preserved | PASS |
| 61/19/49 source/frame/scale/order/validity fully closed | FAIL |
| All deployed outputs trace to intended motor unconditionally | UNKNOWN |
| Contacts unambiguous for deployed policy and real Go2 | UNKNOWN |
| Independent implementation-level observation/action goldens | FAIL |
| Remap necessity backed by deployed-artifact evidence | UNKNOWN |
| No visual-only correctness claim | PASS |
| UNKNOWNs preserved conservatively | PASS |
| Independent Reviewer acceptance | FAIL — REJECT |

## Minimum Blockers for Re-review

1. Recover immutable Agile and Recovery source/checkpoint/export evidence, or equivalent independent artifact-order evidence.
2. Recover the RA source model/dataset and prove exact binding to deployed Agile.
3. Add independent implementation-level 61/49/19 observation and 12-action/contact golden tests.
4. Close missing source/frame/scale/order/validity rules, including Recovery bias and real foot-force semantics.
5. Commit the P1-01 evidence package.

P1-02 must not start while this review remains rejected.
