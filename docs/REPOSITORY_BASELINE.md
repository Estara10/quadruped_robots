# Repository Baseline

## Day 0 Git Baseline

- Day 0 start branch: `feat/ray-pred-source-switch`
- Day 0 start HEAD: `cebecad1b8ab4070f34780cfd892c729b96293b3`
- Start relationship to `main`: 8 commits ahead, 0 behind
- Staged changes at start: none
- Objective: known state, not forced cleanliness

## Existing User Changes Preserved

| Path | State | Classification | Day 0 treatment |
|---|---|---|---|
| `paper.txt` | deleted | E/H, legacy status note | Preserve deletion unstaged; ownership/intent is user-side |
| `.../config/abs/config.yaml` | modified gains 3.0 → 2.5 | B, configuration | Preserve unstaged; do not tune or commit on preparation day |
| `.../xacro/ros2_control.xacro` | modified network comment | B, machine-specific configuration note | Preserve unstaged; do not infer real launch correctness |
| `scripts/generate_project_report.py` | untracked | A/I, source with stale technical claims | Keep visible and untracked; review separately before any future commit |
| `/skill` | untracked personal command note | H, temporary/personal | Preserve locally and ignore exactly this path |

## A–I Classification

| Class | Repository content | Management |
|---|---|---|
| A. Source Code | ABS training/reference, ROS 2 controllers/hardware, scripts, MuJoCo simulator source | Normal Git. Required MuJoCo `main.cc`, `param.h`, joystick and viewer source must not be ignored. |
| B. Configuration | ROS controller YAML, ABS/Recovery YAML, launch/xacro, MuJoCo `config.yaml` | Normal Git. Machine-specific secrets or network overrides must remain parameters/local overrides. |
| C. MuJoCo Model / Scene / Asset | Go2 `go2.xml`, mesh assets, height fields and formal scene XMLs | Normal Git for the current 28 MB Go2 set. These are required for clean-checkout simulation. |
| D. Model / Checkpoint | Three deployment TorchScripts; Ray-Pred and training checkpoints | Deployment files under 1 MB: Normal Git + manifest. Large training/Ray-Pred files: external artifact storage + SHA-256 manifest. Git LFS is not installed in this workspace. |
| E. Experiment Summary | Aggregate tables and final Acceptance reports | Small, reviewed summaries may be tracked under `docs/` in future. Existing reports remain legacy and are not promoted on Day 0. |
| F. Raw Experiment Data | `logs/`, telemetry, images, videos, rosbags, training logs | External/local experiment storage; ignored by normal Git. A formal run still requires a tracked protocol and immutable manifest. |
| G. Build / Cache / Generated | `build/`, `install/`, `log/`, Python caches, coverage, generated office files | Ignored. Existing local data is preserved, not deleted. |
| H. Temporary / Debug | editor files, `/skill`, scratch output | Ignored or retained locally; never deleted merely to clean status. |
| I. Unknown | Any file whose source/purpose cannot be proven | Preserve in place, leave visible where practical, and record in `CURRENT_STATE.md`; do not delete or silently ignore. |

## MuJoCo Provenance

- Upstream repository: `https://github.com/legubiao/unitree_mujoco.git`
- Recovered nested upstream commit: `ace942311ffef188d6ce15fc6728a1aafceeba63`
- License: BSD-3-Clause at `unitree_mujoco/LICENSE`
- Project-specific tracked modifications include Go2 defaults, DDS bridge, ray/qpos/collision telemetry, scene selection and MuJoCo compatibility.
- Only Go2 runtime assets are vendored into this top-level repository; unrelated upstream robot assets remain outside this baseline.

## Recovered Training-Server Snapshot

- `ABS_fuwuqi/ABS` is a restored snapshot of the training server and is a
  required evidence root for any investigation of ABS training provenance,
  checkpoints, TensorBoard events, exported policies, RA artifacts, training
  scripts, or recovered Git history.
- It is distinct from the canonical `ABS/` checkout. Audits of training or
  artifact provenance must search both roots before declaring an item absent.
- Snapshot contents are historical evidence, not automatic proof of a training
  run's configuration, seed, command, or execution. Each claimed link still
  requires its own hash, file, source, or runtime evidence.
- Current P1-01 findings derived from this root are recorded in
  `docs/evidence/P1-01/P1-01_server_snapshot_reaudit_20260830.md`.

## Clean-Checkout Dependency Map

After the Day 0 asset commit, Git contains the project source, ROS configs, three core ABS TorchScripts, MuJoCo simulator source, Go2 MJCF, mesh assets and current scene XMLs.

External dependencies still required:

- ROS 2 Humble and ros2_control;
- MuJoCo development package;
- Unitree SDK2 and DDS libraries;
- yaml-cpp, Boost, fmt, GLFW and LibTorch;
- Isaac Gym only for training-side work;
- large Ray-Pred/training artifacts listed in `artifacts/manifest.yaml`.

Exact installed MuJoCo and Unitree SDK2 versions are `UNKNOWN` and remain a reproducibility issue. A real clean-checkout build is scheduled under the roadmap; Day 0 does not claim that build has been executed.

## Remaining Known Repository Issues

- `ABS/training/legged_gym/resources/policy/recover_v4_twist.pt` is a tracked absolute symlink to a workstation-local checkpoint. It is not portable. The separately tracked deployment Recovery TorchScript remains available; P1-01 must establish the source/export relationship before this reference is changed.
- `unitree_mujoco/simulate/config.yaml` contains duplicate `domain_id` and `interface` keys. Their effective values depend on parser behavior. The file is preserved unchanged on Day 0 and the ambiguity is recorded in the Gap Matrix.
- Exact MuJoCo and Unitree SDK2 versions remain `UNKNOWN`.
- Retrieval URIs for the large external Ray-Pred artifacts remain `UNKNOWN`; their local names, purposes, sizes and hashes are recorded in `artifacts/manifest.yaml`.

## Ignored Local Data at Day 0

- root `build/` 46 MB, `install/` 11 MB, `log/` 34 MB;
- evaluation `logs/` 186 MB;
- training logs 806 MB, depth logs 257 MB, depth data 212 MB;
- legacy nested Git backups totaling hundreds of MB.

These paths are preserved locally and excluded from normal Git. No cleanup was performed.
