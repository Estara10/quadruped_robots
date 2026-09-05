# P1-01 — Final Evidence Closure and Direct Mapping Correction (2026-08-30)

Dated addendum. Read-only evidence recovery over **all** mandatory evidence
roots. No code, artifact, config, or training output was modified; no MuJoCo /
ROS2 / benchmark / robot run. **No mapping change is justified** by the
recovered evidence.

## 1. Exact evidence roots searched

- `ABS/` (working-tree legged-gym + configs)
- `ABS_fuwuqi/ABS/` (restored server snapshot; **prioritized**) — run dirs
  `go2_pos_rough/05_27_15-53-31_`, `go2_rec_rough/06_04_22-43-20_`, exported
  policies/RA, TensorBoard events, checkpoint payloads, `train.py`,
  `testbed.py`, `task_registry.py`, `helpers.py` (incl. `get_load_path`),
  snapshot `.git` (HEAD `9b95329f`, reflog = single clone of
  `github.com/LeCAR-Lab/ABS.git`, no tags, one branch), hidden files, archives.
- `quadruped_ros2_control_humble/` (controller remap + config + build logs +
  `commands`)
- `unitree_mujoco/` (bridge touch-sensor mapping)
- `docs/evidence/P1-01/`, `artifacts/`
- Local + official GitHub Unitree SDK (LowState) and unitree_ros2 (via
  DeepWiki; primary file not retrievable)

## 2. Agile semantic-order verdict — UNKNOWN (no direct binding)

Complete static trace (deployed artifact → training obs/action → candidate
config → export → controller remap → ROS order → MuJoCo order):

```
deployed policy.pt (61→12)           [weights = checkpoint model_4000.pt = export]
  → training obs construction: legged_robot_pos.py base (candidate 61-D layout:
    contact[0:4], ang_vel[4:7], gravity[7:10], goal[10:13], timer[13:14],
    dof_pos[14:26], dof_vel[26:38], prev_action[38:50], rays[50:61])
  → candidate go2_pos_config.py (UNTRACKED in snapshot git; not run-bound)
  → export 05_27_15-53-31_model_4000.pt (byte-equal deployed; weight-equal checkpoint)
  → controller remap [3,4,5,0,1,2,9,10,11,6,7,8] (DOF), [1,0,3,2] (contact),
    gated by policy_joint_order="ros1_fl_fr_rl_rr" (config/abs/config.yaml:6,
    StateRL.cpp:204,217,230,243)
  → ROS controller order FR,FL,RR,RL (CONFIRMED)
  → MuJoCo actuator order FR,FL,RR,RL (CONFIRMED)
```

Evidence strength:
- **source-code default**: candidate 61-D layout + `ros1_fl_fr_rl_rr` remap flag.
- **candidate config**: `go2_pos_config.py` (untracked, SHA `ef13f180…`, not
  bound to the run).
- **immutable run-local config**: **absent** — no serialized args, pickle,
  command/job log, TensorBoard hparams, git ref, export metadata, or any record
  binding `model_4000.pt` to its training order in any evidence root.
- **actual deployed artifact evidence**: no embedded order metadata; artifact
  probe inconclusive for Agile (prev_action diagonal 7/12).
- **simulation-only convention**: MuJoCo touch order FR,FL,RR,RL.
- **real hardware contract**: no hardware evidence.

The candidate training order (FL,FR,RL,RR) is a source/training-environment
candidate, not an immutable binding to the deployed artifact. The existing remap
is a **conditional** transformation: correct **iff** the deployed Agile uses the
FL-first candidate order, which is **not proven**. **No direct evidence proves
the remap incorrect**, so per the task rule **no mapping change is justified**.

## 3. RA actual-binding verdict — historical execution binding not recoverable

Traced path in the server snapshot:
`get_load_path(log_root, load_run, checkpoint)` (`helpers.py:103`) → sets
`task_registry.loaded_policy_path` (`task_registry.py:161`) → `testbed.py:197`
`policy_name = <run>+<model>` → `:211,219,568` `RA_name = policy_name[:-3]+"_ra.pt"`
→ `exported/RA/05_27_15-53-31_model_4000_ra.pt` → `convert_ra_to_jit.py`
(default input path = the named RA) → `ra_value_jit.pt` → deployed (byte-equal).

Evidence strength:
- **weight chain (named RA → JIT → deployed)**: **CONFIRMED** (6/6 tensors +
  byte equality + conversion-script default path).
- **naming mechanism**: **source-causal / operational** — the mechanism and
  `get_load_path`'s `load_run=-1 / checkpoint=-1` (last run / last checkpoint)
  defaults are source-verified, and the observed name is consistent with loading
  `05_27_15-53-31_/model_4000.pt`. But the **executed** `load_run`/`checkpoint`
  values are not recorded (no serialized args, command/job log, hparams, event
  metadata, dataset manifest, or immutable run-local record in any root).
- **historical execution proof**: **absent** → **"historical execution binding
  is not recoverable from available artifacts"**. Not fabricated from filenames,
  timestamps, or directory layout.

## 4. Real Go2 foot_force verdict — Phase 2 hardware-only UNKNOWN

- **Unitree MuJoCo bridge mapping**: `foot_force[i] = sensordata[dim+16+i]`
  (`unitree_sdk2_bridge.h:729-739`), following the MJCF touch order
  `FR,FL,RR,RL` (`go2.xml:289-292`) — **simulation convention only**.
- **Local official Unitree SDK** (pinned 2.0.0, SHA-verified; LowState_.hpp) and
  official GitHub unitree_sdk2 LowState: `foot_force = array<int16_t,4>` with
  **no slot-to-foot documentation**.
- **unitree_ros2 (official)**: DeepWiki (third-party AI summary) claims
  `[0]=FL, [1]=FR, [2]=RL, [3]=RR` citing `read_low_state.cpp`, but the primary
  repo file was **not retrievable** to verify it is an official explicit mapping.
- No official primary source explicitly mapping `foot_force[i]` to a named foot
  is verifiable from local artifacts; no real-hardware single-foot capture
  exists. → **real Go2 foot_force[0..3] semantics = Phase 2 hardware-only
  UNKNOWN** (official-doc evidence: none verified; hardware evidence: none).

## 5. Code correction

**No mapping change justified.** No direct evidence proves the existing
controller remap incorrect; the deployed Agile semantic order remains UNKNOWN.
No code or test was changed.

## 6. Original P1-01 Acceptance table

| # | Criterion | Verdict |
|---|---|---|
| 1 | Three deployed artifacts verified hash/dimensions/provenance status | **PASS** |
| 2 | Authoritative Isaac Gym DOF + foot/body capture preserved | **PASS** |
| 3 | All 61/19/49 fields have one defined source/frame/scale/order/validity | **FAIL/UNKNOWN** — deployed order unknown; goal shaping variant; some runtime gaps |
| 4 | All 12 outputs trace to exactly one intended Go2 motor target | **UNKNOWN** — controller→motor bijective; deployed artifact order unproven |
| 5 | Contact indices trace to FR/FL/RR/RL unambiguously | **UNKNOWN** — candidate mapping; deployed order + real slots unproven |
| 6 | Asymmetric observation/action golden tests pass | **PASS** |
| 7 | ROS2 remapping necessity is evidence-backed | **UNKNOWN** — conditional only; deployed order unproven |
| 8 | No Critical conclusion relies only on visual locomotion | **PASS** |
| 9 | Independent Reviewer accepts evidence | **FAIL/REJECTED** (prior reviews) |
| 10 | Unresolved source data remains explicit UNKNOWN | **PASS** |

## 7. Final P1-01 status

**BLOCKED / PARTIALLY COMPLETE** — original mandatory semantic/provenance items
(3, 4, 5, 7) remain UNKNOWN after exhaustive search of all evidence roots.
Not ACCEPTED, not ACCEPT WITH KNOWN ISSUES (mandatory criteria unmet).

## 8. Remaining blockers (restricted to original Acceptance)

- Deployed Agile (and Recovery named) semantic order — no immutable run-local
  config/commit binding; artifact probes not conclusive.
- ROS2 remap necessity for the deployed artifacts — conditional only.
- Contact/foot-order binding to the deployed artifacts and real Go2 slots.
- RA training-data execution binding (rollouts of `model_4000.pt`) — not
  recoverable from available artifacts.
- Real Go2 `foot_force[0..3]` slot order — Phase 2 hardware-only.

## 9. Evidence files (this addendum)

- `P1-01_final_closure_20260830.md` (this report)
- `P1-01_final_closure_inventory_20260830.json` (machine-readable)
- Prior dated evidence (snapshot re-audit, closure audit, probe results) unchanged.

Existing UNKNOWNs are preserved; no fabricated lineage.
