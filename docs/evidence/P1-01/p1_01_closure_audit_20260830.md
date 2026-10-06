# P1-01 — Deployed Policy / RA Provenance and Order Closure Audit (2026-08-30)

Status: **BLOCKED / PARTIALLY COMPLETE** — evidence-recovery audit; no code,
artifact, or configuration change. The three deployed weight-lineage chains
remain **CONFIRMED** (byte/weight equality). One **new partial** artifact-side
finding was recovered: the deployed **Recovery** policy's `prev_action`
observation block is empirically located at slots 37:49 with index-aligned
(11/12 diagonal) coupling — consistent with the candidate 49-D FL-first layout.
The deployed **Agile** order, exact **RA ↔ Agile** binding, **historical run
metadata**, and **real foot-force semantics** remain **UNKNOWN**. P1-01 is not
ACCEPTED.

## Method (read-only)

- Inventory of every currently-deployed / recovered-training artifact referenced
  by production config and controller load paths (absolute path, size, SHA-256,
  format, loader).
- TorchScript embedded-metadata inspection (`torch.jit.load` + code dump).
- Asymmetric offline probes on the **actual deployed artifacts** (not synthetic
  fixtures): one-hot input scan, operating-point central-difference Jacobian,
  and a 40-point mean-|coupling| robustness Jacobian. Deterministic and
  repeatable. No MuJoCo / ROS2 / benchmark / robot.
- Searched declared boundaries for historical metadata (run dirs, TensorBoard,
  checkpoint embedded state, recovered Git per prior P1-01 evidence).

## 1. Artifact inventory (all hashes verified this audit)

| Artifact | Role | Path | Size | SHA-256 | I/O | Loader |
|---|---|---|---|---|---|---|
| Agile | currently deployed | `quadruped_ros2_control_humble/.../config/abs/policy.pt` | 801726 | `5a87d6…e0b7cf` | 61→12 | StateRL `config/abs` |
| RA | currently deployed | `.../config/abs/ra_value.pt` | 32011 | `05c40f…a90b7` | 19→1 | StateRL |
| Recovery | currently deployed | `.../config/rec/policy.pt` | 775715 | `e3047a…b0171` | 49→12 | StateRLRec/StateRL |
| Agile checkpoint | training source | `ABS_fuwuqi/ABS/training/legged_gym/logs/go2_pos_rough/05_27_15-53-31_/model_4000.pt` | 4743147 | `a21419…ead912` | — | — |
| Agile export | recovered export | `.../exported/policies/05_27_15-53-31_model_4000.pt` | 801726 | `5a87d6…e0b7cf` | — | byte-equal deployed |
| RA source | training source | `.../exported/RA/05_27_15-53-31_model_4000_ra.pt` | 26047 | `ed3c75…2aaa92` | — | — |
| RA JIT | recovered export | `.../exported/RA/ra_value_jit.pt` | 32011 | `05c40f…a90b7` | — | byte-equal deployed |
| Recovery checkpoint | training source | `ABS_fuwuqi/ABS/training/legged_gym/logs/go2_rec_rough/06_04_22-43-20_/model_15000.pt` | 4595761 | `51ee8e…3eeec5` | — | — |
| Recovery export | recovered export | `.../exported/policies/policy.pt` | 775715 | `e3047a…b0171` | — | byte-equal deployed |

Installed loader symlinks point at these tracked files (byte-identical).
Machine-readable: [`p1_01_closure_inventory_20260830.json`](p1_01_closure_inventory_20260830.json).

## 2. Confirmed provenance links (CONFIRMED, exact hashes)

- Agile: checkpoint `a21419…ead912` → export `5a87d6…e0b7cf` → deployed `5a87d6…e0b7cf` — **byte- and weight-equal**.
- RA: source `ed3c75…2aaa92` → JIT export `05c40f…a90b7` → deployed `05c40f…a90b7` — **byte- and weight-equal**.
- Recovery: checkpoint `51ee8e…3eeec5` → export `e3047a…b0171` → deployed `e3047a…b0171` — **byte- and weight-equal**.

UNKNOWN links (unchanged): exact run config snapshot, training seed, training git
commit, exact export/conversion invocation (Agile/RA), RA dataset + episode
count, Recovery export execution log.

## 3. Deployed order verdict

- **Embedded metadata**: all three deployed TorchScripts are plain feedforward
  networks with **no** embedded names / order / provenance attributes
  (**CONFIRMED** negative).
- **Training runtime order** (captured): `FL,FR,RL,RR` (CONFIRMED).
- **Controller/MuJoCo/motor order**: `FR,FL,RR,RL` (CONFIRMED).
- **Deployed Recovery order**: **PARTIAL** — the actual artifact's `prev_action`
  block is empirically at slots 37:49 and its local coupling is diagonal
  (output `j` ↔ `prev_action[37+j]`; 11/12 over 40 operating points). This is
  consistent with the candidate 49-D FL-first training layout. The **named**
  joint binding (output `j` = which named joint) remains **conditional** on the
  candidate layout's authority for the actual run (still UNKNOWN).
- **Deployed Agile order**: **UNKNOWN** — no embedded metadata and the probe is
  inconclusive (prev_action diagonal persistence 7/12).
- **Remap correctness for deployed artifacts**: **UNKNOWN** (Agile); **PARTIAL**
  artifact-side consistency only (Recovery). Not full proof.

## 4. RA ↔ Agile binding verdict — UNKNOWN

The deployed RA artifact embeds **no** reference to an Agile checkpoint
(CONFIRMED negative). No execution log, dataset manifest, loaded-checkpoint
metadata, seed, or commit record exists in the declared boundaries. The only
candidate link is `testbed.py` deriving the RA output name from the loaded Agile
run path — a source-code candidate, not binding evidence.

## 5. Historical run metadata verdict — UNKNOWN (searched boundary recorded)

Declared boundaries searched: both run directories, the RA export directory,
recovered Git history (prior P1-01 search), checkpoint embedded state.
Available: model checkpoints + TensorBoard scalar curves only. Unavailable:
config snapshot, command, seed, git commit, hparams, dataset manifest, export
invocation log. Checkpoint state is `{model_state_dict, optimizer_state_dict,
iter, infos=None}` — no order/seed/commit.

## 6. Real foot-force semantics verdict — UNKNOWN

No independent hardware capture exists. The simulation/controller chain
(`FR,FL,RR,RL` touch sensors) is SOURCE-VERIFIED for MuJoCo only and must not be
asserted for real Go2 `foot_force[0..3]`. This is a hardware commissioning /
safety item.

## 7. P1-01 final status

**BLOCKED / PARTIALLY COMPLETE** (not ACCEPTED). All three deployed weight
lineages and the runtime/controller orders remain verified; the Recovery
artifact-side order now has partial empirical support. The remaining critical
blockers — deployed Agile order, exact RA ↔ Agile binding, historical run
metadata, real foot-force semantics — remain UNKNOWN with recorded searched
boundaries. P1-01 Acceptance requires an immutable run-local config/commit
association binding each recovered checkpoint to the captured Isaac Gym order,
or an equivalent independent artifact-order record (e.g., a clean asymmetric
probe that uniquely recovers the deployed action order across the operating
regime).

## 8. Remaining blockers and searched boundaries

| Blocker | Status | Searched boundary / evidence |
|---|---|---|
| Deployed Agile action/order | UNKNOWN | Embedded metadata: none; one-hot + Jacobian probe on the actual artifact: 7/12 diagonal (inconclusive) |
| Deployed Recovery named order | PARTIAL | Actual artifact prev_action block 37:49 diagonal 11/12 (artifact-side); named binding conditional on candidate layout authority |
| RA ↔ Agile exact binding | UNKNOWN | Deployed RA embedded metadata (none); run/RA/export dirs; recovered Git; TensorBoard; no execution/dataset manifest |
| Historical run metadata | UNKNOWN | run dirs (checkpoints + tfevents only), checkpoint state, recovered Git |
| Real Go2 foot-force semantics | UNKNOWN | no hardware capture; simulation/controller order only |

## 9. Evidence files

- `p1_01_closure_audit_20260830.md` (this report)
- `p1_01_closure_inventory_20260830.json` (machine-readable inventory)
- `p1_01_probe_results_20260830.json` (one-hot scan + Jacobian + metadata, all three artifacts)
- `p1_01_realistic_jacobian_20260830.json` (operating-point Jacobian)
- `p1_01_probe_deployed_order_20260830.py`, `p1_01_analyze_realistic_20260830.py`, `p1_01_robust_jacobian_20260830.py` (probe scripts)

Prior P1-01 evidence (`isaac_gym_asset_order.json`, `ros2_motor_map.json`,
`provenance_recovery.json`, `REVIEW*`, `p1_01f_local_contract.json`) is
unchanged.
