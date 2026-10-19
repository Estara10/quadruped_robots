# P1-10 Stage-A 2026-09-07 Pair — Evidence Integrity Incident

## Disposition

The pair

`docs/evidence/P1-10/replay_pair_20260907_stage_a_current_instrumented/`

is archived as:

`RUNTIME_EXECUTED / EXACT_REPLAY_FAIL / EVIDENCE_INCOMPLETE`

It is completed historical evidence and must not be retried, rebuilt, or used
as a current comparator input. The frozen `pair_manifest.json` was not changed.

## Read-only inventory

The machine-readable inventory is
[`stage_a_pair_20260907_evidence_integrity_inventory.json`](stage_a_pair_20260907_evidence_integrity_inventory.json).
The pair directory currently contains only:

| Artifact | Status | Bytes | SHA-256 |
|---|---|---:|---|
| `pair_manifest.json` | PRESENT | 9823 | `769e7f6bd148ad3111213af2261cb357a89510b511453c8f406b7f5c3a4edc8c` |

Both `run_A/` and `run_B/` directories are missing. Therefore every original
required run artifact is `MISSING`: resolved manifest, P1-10 context, preflight
evidence, runtime record, sim-clock timing, runtime-frame timing, reader stats,
process facts, orphan inventory, and the three raw logs. The four pair-level
comparison outputs are also missing: canonical input/output, `diff_report.json`,
and the Markdown comparison report. No missing content is inferred or
reconstructed.

## What the remaining evidence supports

The preserved
[`stage_a_pair_difference_diagnosis_20260907.md`](stage_a_pair_difference_diagnosis_20260907.md)
records that Run A and Run B were each executed once, each had a valid record,
process and required-child exit code 0, normal SIGINT shutdown, no forced
termination, and that the saved-record comparator returned `FAIL` with
`difference_count=66604`. It also records the first-frame divergence and the
minimum supported classification `startup alignment`.

That document is preserved historical reporting, not a replacement for the
deleted raw records. The current pair directory no longer independently
supports re-validation of process facts, frame/timing content, hashes, or the
comparator output. There is no current `diff_report.json` or other comparator
output to re-open.

Consequently the pair supports only the archived historical conclusion:
individual execution was reported successful, exact replay comparison failed,
and the evidence set is incomplete. It cannot support formal P1-10 acceptance,
a new comparison, an algorithm conclusion, or common-start runtime validation.

## Static cause trace

The unsafe pre-repair fixture was
`scripts/test_p1_10_stage_a_saved_record_compare.py`. Its `setUp()` called the
real `PAIR_DIR` fixture cleanup; `_remove_fixture()` removed `run_A`, `run_B`,
and the pair-level comparison outputs; `_make_fixture()` then recreated those
directories and wrote production-shaped artifacts directly below the real
`docs/evidence/P1-10/replay_pair_20260907_stage_a_current_instrumented/`.
The observed pre-repair source spans were `setUp()` around line 39,
`_remove_fixture()` around lines 42–58, and `_make_fixture()` around lines
60–89. The failure occurred after cleanup while the stale Stage-A identity was
being validated, so the real pair was left with only its manifest.

The test touched the formal evidence path because it hard-coded the production
pair locator as its fixture (`PAIR_DIR = REPO / comparator.PAIR_DIR_REL`) and
used the production context/preflight writer to build a fixture there. That was
an invalid fixture boundary: a completed evidence directory was treated as
disposable test state.

The current replacement at
[`scripts/test_p1_10_stage_a_saved_record_compare.py`](../../scripts/test_p1_10_stage_a_saved_record_compare.py)
contains only non-mutating historical-boundary checks. Its full filesystem
fixture coverage is in
[`scripts/test_p1_10_saved_record_compare.py`](../../scripts/test_p1_10_saved_record_compare.py),
where each test allocates `tempfile.TemporaryDirectory()` (see lines
125–365). The current tests do not create, delete, or rewrite the real pair.

## Required future boundary

The old pair cannot be rerun or completed by filling the missing artifacts.
Any future attempt requires a new fresh pair, independent review of the
common-start implementation, and a new Director authorization. The common-start
implementation itself has no runtime evidence from this pair. No runtime was
started for this incident investigation, and no MuJoCo, ROS2, A/B, benchmark,
FormalRun, or later P1 task was started.

This incident does not alter P1-08 evidence, the old pair manifest, or the
common-start implementation. It records an evidence-integrity loss and its
fail-closed disposition only.
