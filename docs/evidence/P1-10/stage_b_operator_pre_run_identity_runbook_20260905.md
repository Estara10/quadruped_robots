# P1-10 Stage-B Operator Pre-Run Identity Runbook

This is a pre-run contract only. It does not authorize an obstacle run. An
independent Reviewer must approve the Stage-B identity/preflight closure and a
Director must issue a separate, one-time `obstacle_test1` observational-run
authorization before an Operator executes the capture command.

## Frozen identity

Use only the frozen file below; do not create, edit, copy, or hand-edit it:

```text
docs/evidence/P1-10/stage_b_execution_manifest_20260905.json
```

It is a new Stage-B execution identity. Its P1-08 parent is provenance only:
the accepted P1-08 canonical identity is
`59dd13fed5ebd026ec519f2659643237502be8e4d8df5174a65b7d35ceb4f7e0`.
The current instrumented executable is an independent Stage-B artifact and is
not the accepted P1-08 executable.

The frozen binding is `obstacle_test1`, `scene_test1.xml`, the recorded XML/
asset closure and loaded-model fingerprint, `stabilized`,
`root_seed=20260902`, `25.0 s`, `scene_default` / `mj_makeData:qpos0`, fixed
goal `[7.0, 0.0]`, the consumed config/plugin hashes, collision v2 (392 bytes),
and the harness-generated capture-ID contract. Formal collision authority is
limited to the two harness-controlled `main.cc` PhysicsLoop paths. UI reset,
keyframe, step-forward, and teleoperation are prohibited.

## Preflight-only validation

From the repository root, a reviewer or Operator may validate the frozen
identity without launching anything:

```bash
cd /home/lidio/quadruped_robots
python3 scripts/p1_10_stage_b_execution.py \
  --validate docs/evidence/P1-10/stage_b_execution_manifest_20260905.json \
  --executable /home/lidio/quadruped_robots/unitree_mujoco/simulate/build2/unitree_mujoco
```

The capture harness requires `--stage-b-execution-manifest` for
`obstacle_test1`, derives the capture ID internally, and rejects a manually
supplied capture ID. It rejects any binary, scene, closure, fingerprint,
config/plugin, initial-state, goal, window, contract, or path drift. It does
not permit this manifest for `obstacle_test2`–`obstacle_test5` or for flat
replay.

## Future authorized command shape

Only after the two approvals above, use the exact frozen values and a fresh
Director-provided output directory. The Operator must not alter any argument
or environment variable:

```bash
cd /home/lidio/quadruped_robots
source /opt/ros/humble/setup.bash
source /home/lidio/quadruped_robots/quadruped_ros2_control_humble/install/setup.bash
python3 scripts/p1_08_baseline_capture.py \
  --out-dir <DIRECTOR_PROVIDED_FRESH_OBSTACLE_TEST1_DIR> \
  --window-s 25.0 \
  --scene scene_test1.xml \
  --mujoco-bin /home/lidio/quadruped_robots/unitree_mujoco/simulate/build2/unitree_mujoco \
  --manifest docs/evidence/P1-08/P1-08_baseline_manifest.json \
  --stage-b-execution-manifest docs/evidence/P1-10/stage_b_execution_manifest_20260905.json \
  --scenario obstacle_test1 \
  --root-seed 20260902 \
  --variant stabilized \
  --initial-state-source scene_default
```

This command is not an authorization and must not be run under this evidence
state. No obstacle runtime, pair, benchmark, FormalRun, or later P1 task is
claimed here. `obstacle_test1` remains `IMPLEMENTED / AWAITING RUNTIME
VALIDATION`; goal/fall/timeout remain `UNKNOWN`; the five-map suite is not
accepted.

The older flat replay runbook under the historical saved-record pair directory
is historical evidence only. It is not a current Stage-B instruction and must
not be used to run or retry any historical pair.
