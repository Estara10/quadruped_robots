# P1-01 — Server-Snapshot Provenance Recovery Re-Audit (2026-08-30)

Dated addendum to the earlier P1-01 audits. Read-only evidence recovery over the
restored server snapshot `ABS_fuwuqi/ABS`. No code, artifact, config, or
training output was changed; no MuJoCo/ROS2/benchmark/robot run. This addendum
**narrows and corrects** the earlier audit where the snapshot provides direct
evidence.

## A. Exact deployed Agile lineage — CONFIRMED

Chain:

```
ABS_fuwuqi/ABS/training/legged_gym/logs/go2_pos_rough/05_27_15-53-31_/model_4000.pt
  → exported/policies/05_27_15-53-31_model_4000.pt
  → current deployed Go2 ABS policy.pt
```

- Deployed `policy.pt` == export `05_27_15-53-31_model_4000.pt`:
  **byte-equal** — both `801726` B, SHA-256 `5a87d692…e0b7cf` (re-verified).
- Checkpoint `model_4000.pt` (4,743,147 B, SHA-256 `a21419…ead912`, `iter=4000`,
  `infos=None`) → export: **weight-equal**. All **8/8** checkpoint
  `actor.{0,2,4,6}.{weight,bias}` tensors exactly equal the exported (and the
  deployed) TorchScript parameters `{0,2,4,6}.{weight,bias}` (arch
  `61,512,256,128,12`). Formats differ (state-dict vs TorchScript), so this is
  tensor/weight equality, not byte equality. Directly verified:
  `p1_01_snapshot_agile_weights_20260830.txt`.

## B. Exact RA lineage and Agile binding — CONFIRMED (chain); PARTIAL / UNKNOWN (data binding)

Chain:

```
loaded Agile checkpoint 05_27_15-53-31_/model_4000.pt
  → named RA artifact  exported/RA/05_27_15-53-31_model_4000_ra.pt
  → RA JIT              exported/RA/ra_value_jit.pt
  → deployed            ra_value.pt
```

- Deployed `ra_value.pt` == `ra_value_jit.pt`: **byte-equal** — both `32011` B,
  SHA-256 `05c40ff7…a90b7` (re-verified).
- Named RA `05_27_15-53-31_model_4000_ra.pt` (26,047 B, SHA-256 `ed3c75…2aaa92`)
  → JIT: **weight-equal** — all **6/6** tensors
  (`{0,2,4}.{weight,bias}`, arch `19,64,64,1`+Tanh) exactly equal the JIT and
  deployed parameters. Directly verified:
  `p1_01_snapshot_ra_weights_20260830.txt`.
- Conversion source: `scripts/convert_ra_to_jit.py` — its **default input path is
  exactly `05_27_15-53-31_model_4000_ra.pt`** and it saves to `ra_value_jit.pt`
  in the same directory, tracing `nn.Sequential(19→64,ReLU,64→64,ReLU,64→1,Tanh)`
  which matches the named-RA architecture. Direct conversion link.
- RA naming mechanism (server snapshot source, verified):
  `testbed.py:197` `policy_name = <run_dir> + <model>`
  (`05_27_15-53-31_` + `model_4000.pt` → `05_27_15-53-31_model_4000.pt`);
  `testbed.py:211,219,568` `RA_name = policy_name[:-3] + "_ra.pt"`
  → `05_27_15-53-31_model_4000_ra.pt`, saved under `logs/<exp>/exported/RA/`.
  The observed named-RA filename is the deterministic output of this source
  logic applied to the Agile checkpoint path.

**RA ↔ Agile binding verdict**: the **naming** link (named RA artifact derived
from the loaded Agile-checkpoint path) is **source-verified** (the exact
`policy_name[:-3]+"_ra.pt"` mechanism is in the snapshot and the observed name
is its deterministic output) — this is **PARTIAL**, not full proof: no
execution log / shell / job record proves the mechanism was run with that exact
path. The **training-data** link (the RA was trained on rollouts of
`model_4000.pt`) remains **UNKNOWN** (ephemeral online queues, no persisted
dataset manifest).

## C. Training-run evidence inventory

| Item | Agile `go2_pos_rough/05_27_15-53-31_` | Recovery `go2_rec_rough/06_04_22-43-20_` |
|---|---|---|
| Checkpoints | 21 (`model_0`..`model_4000` @200), all 4,743,147 B except early 4,743,077 B | 76 (`model_0`..`model_15000`), 4,595,761 B / 4,595,691 B / 4,595,289 B |
| Selected checkpoint | `model_4000.pt` SHA-256 `a21419…ead912`, `iter=4000` | `model_15000.pt` SHA-256 `51ee8e…3eeec5`, `iter=15000` |
| Checkpoint payload | `{model_state_dict, optimizer_state_dict, iter, infos=None}` | same |
| TensorBoard | `events.out.tfevents.1779868413.c1505-R5300-G5.1583700.0` — 34 scalar tags (reward/curves), **no hparams** | `events.out.tfevents.1780584201…` |
| Config / command / seed / commit | **absent** | **absent** |
| Non-`.pt`/tfevents files in run dirs | none | none |

Snapshot `.git` (HEAD `9b95329f` "Updated required numpy version", ~15 commits):
tracks the legged-gym **code** (`play.py`, `testbed.py`, `helpers.py`) but the
**Go2 configs are untracked** (`?? …/envs/go2/`); no run identifier appears in
commit messages; no shell history / job / launch scripts / command logs found in
the declared boundaries.

Classification:
1. **directly observed server-run evidence**: checkpoints (+iter), TensorBoard
   scalar curves, export artifacts, RA artifacts, `.git` HEAD/logs.
2. **source snapshot defaults**: the current `go2_pos_config.py` /
   `go2_rec_config.py` working files (untracked; candidate config, **not** the
   historical runtime config).
3. **missing runtime overrides**: seed, git commit, exact export invocation,
   dataset manifest, hparams — **UNKNOWN**.

The snapshot provides **no immutable run-local config/commit association**
binding the recovered checkpoints to a config — the Reviewer's stated minimum
evidence to close the order blocker is still absent.

## D. Deployed order

- **Training-source candidate order**: FL, FR, RL, RR (captured Isaac Gym asset
  order; candidate 61-D layout contact/ang_vel/gravity/goal/timer/dof_pos/
  dof_vel/prev_action/rays from `legged_robot_pos.py` + `go2_pos_config.py`).
  The config is a **candidate** (untracked, no run-binding).
- **Checkpoint/export/deployed relationship**: Agile and RA weight chains are
  **CONFIRMED** (Sections A/B).
- **Empirically proven deployed observation/action order**: unchanged from the
  2026-08-30 closure audit — deployed **Recovery** `prev_action` block
  empirically at slots 37:49 with 11/12 diagonal coupling (PARTIAL, artifact-side
  only); deployed **Agile** probe inconclusive (7/12).
- **Unresolved dimensions**: the named deployed **joint order** (which output
  index maps to which named Go2 joint) remains **UNKNOWN** for Agile; **PARTIAL**
  (artifact-side only) for Recovery. The snapshot's code/config tracking does not
  change this because no immutable config/commit binding exists.

## E. Non-goals honored

No real foot-force claim, no benchmark, no Phase-1 claim, no P1-09 status
change, no P1-08 dynamics/cadence claim, no weight or training-outcome change.

## Final status (for Reviewer)

P1-01 remains **BLOCKED / PARTIALLY COMPLETE** — not ACCEPTED. This addendum
**strengthens** the provenance: the exact Agile checkpoint→export→deployed and
RA named→JIT→deployed chains are now directly weight/byte-verified (previously
relied on prior-recorded equality), and the RA naming mechanism is
source-verified in the snapshot. The deployed order, RA training-data binding,
historical run config/seed/commit, and real foot-force semantics remain UNKNOWN.

## Evidence files

- `P1-01_server_snapshot_reaudit_20260830.md` (this addendum)
- `P1-01_server_snapshot_inventory_20260830.json` (machine-readable)
- `p1_01_snapshot_agile_weights_20260830.txt`, `p1_01_snapshot_ra_weights_20260830.txt` (comparison outputs)
- `p1_01_snapshot_compare_agile_20260830.py`, `p1_01_snapshot_compare_ra_20260830.py`

Existing P1-01 evidence (incl. `p1_01_closure_audit_20260830.md`) is unchanged.
