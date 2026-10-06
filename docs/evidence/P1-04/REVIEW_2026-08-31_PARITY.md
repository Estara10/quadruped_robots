# P1-04 Independent Review — Agile 61-D Observation Parity

Date: 2026-08-31  
Disposition: **ACCEPT WITH KNOWN ISSUES**

## Slot verdicts

| Block | Slots | Verdict |
|---|---:|---|
| Contact | `[0:4]` | MATCH |
| Angular velocity | `[4:7]` | MATCH |
| Gravity | `[7:10]` | MATCH |
| Goal/command | `[10:13]` | INTENTIONAL_VARIANT |
| Timer | `[13:14]` | INTENTIONAL_VARIANT |
| Joint position | `[14:26]` | INTENTIONAL_VARIANT: nominal transform MATCH; training random-bias distribution UNKNOWN |
| Joint velocity | `[26:38]` | MATCH |
| Previous action | `[38:50]` | MATCH |
| Ray2D | `[50:61]` | INTENTIONAL_VARIANT: values MATCH; validity differs fail-closed |

The slot total is 61. No unclassified shift, permutation, default-filled value,
or silent non-finite handling was found.

## Verification

- The oracle is the actual training `LeggedRobotPos.compute_observations()`;
  deployment is exercised through the compiled production adapter calling
  `abs_observation::agile()`.
- The first local-contract invocation failed only because Isaac Gym attempted
  to write its default global extension cache. Re-running the same offline test
  with its extension cache under `/tmp` passed: Agile61 `mismatches=[]`, RA19,
  Recovery49, and semantic helpers all `PASS`.
- `validate_p1_01_contract.py` reports `pass=135`, `known=8`, `fail=0` and
  `CONTRACT REGRESSION: PASS`; its rc=2 is the legacy pre-DEC-010 Acceptance
  summary, not a P1-04 regression.
- `git diff --check` returned rc=0.

## Clamp boundary

Deployment applies `torch::clamp(obs, -clip_obs, clip_obs)` with `clip_obs=100`
to the assembled vector. Training configuration declares the same limit, and
the Reviewer confirmed `LeggedRobot.step()` applies `torch.clip` to `obs_buf`
before returning the training observation. The fixture is inside `[-100,100]`;
the complete downstream policy-library call chain was not inspected. This is a
known evidence boundary, not an unclassified mismatch or a claim of
out-of-domain parity.

## Scope boundary

This is offline observation-parity evidence only. It does not establish
paper-equivalence, historical training reproducibility, hardware contact
semantics, a runtime benchmark, or Phase 1 Acceptance. P1-05 does not start
automatically.
