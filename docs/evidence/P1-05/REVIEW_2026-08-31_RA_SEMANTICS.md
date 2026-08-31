# P1-05 — Independent Reviewer Disposition (2026-08-31)

Decision: **ACCEPT WITH KNOWN ISSUES.**

## Reviewer result

The independent Reviewer accepted the P1-05 offline evidence audit
(`P1-05_ra_semantics_20260830.md` + `P1-05_ra_19d_matrix_20260830.json`) with
known issues. The Reviewer identified one concrete documentation omission which
this correction closes: the recovered-training switching entry was recorded as
`n/a`, but `testbed.py` actually contains `twist_eps = 0.05` and
`recovery = (v_pred > -twist_eps)` (i.e. strict `v_pred > −0.05`) as an immediate
test/testbed branch with no `−0.08` exit hysteresis and no 30-step forced hold.

## Three-way switching fact (now recorded precisely)

| Source | Recovery entry | Exit | Hold |
|---|---|---|---|
| Paper (ABS_PAPER_NOTES) | `RA >= −0.05` | `RA < −0.05` | none |
| Recovered testbed (`testbed.py:64,324`, both `ABS/` and `ABS_fuwuqi/ABS/`) | `v_pred > −twist_eps` = `v_pred > −0.05` (strict `>`) | immediate (no separate exit threshold) | none |
| Current deployment (`StateRL.cpp:1557-1592`) | `ra_value_ > −0.05` (strict `>`) | `ra_value_ < −0.08` | 30-step forced hold (`REC_HOLD_STEPS`) |

Strict `>` entry (recovered testbed and deployment) vs paper `>=`, and the
deployment `−0.08` exit hysteresis + 30-step hold, are recorded as
**INTENTIONAL_VARIANT**; paper equivalence is not claimed.

## Known issues retained (boundaries)

- P1-05 = **ACCEPT WITH KNOWN ISSUES**.
- **No label numeric fixture exists.** The Bellman `MATCH` is
  **structural / source-level** (γ=0.999999, max/min reach-avoid operator,
  soft collision over 10 steps) — not end-to-end numerical equivalence.
- Paper Eq.18 exact log base, `σ_tight` value, terminal bootstrap, and loss
  scale remain **UNKNOWN** from the local paper materials.
- RA ↔ Agile `model_4000` binding remains **OPERATOR_DECLARED** under DEC-010;
  historical seed/command/Git/dataset are deferred reproducibility, not
  blockers.
- No runtime, benchmark, formal-run, or paper-equivalence claim is made.

## Status

- P1-05: **ACCEPT WITH KNOWN ISSUES — independent review 2026-08-31**.
- No active engineering task after closure.
- Phase 1 remains **NOT ACCEPTED**.
- P1-06/P1-07 must not start automatically.

Evidence: `P1-05_ra_semantics_20260830.md`, `P1-05_ra_19d_matrix_20260830.json`,
`docs/exec-plans/P1-05.md`.
