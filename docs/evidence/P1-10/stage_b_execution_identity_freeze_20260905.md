# P1-10 Stage B Execution Identity Freeze — 2026-09-05

## Purpose and boundary

This evidence records the offline freeze for a future `obstacle_test1`
observational run. It is not obstacle runtime evidence, collision evidence,
P1-10 acceptance, or an Operator authorization. No MuJoCo, ROS2, controller,
Run A/B, obstacle pair, benchmark, FormalRun, or later P1 task was started.

## Frozen manifest

| Item | Value |
|---|---|
| manifest | `docs/evidence/P1-10/stage_b_execution_manifest_20260905.json` |
| manifest SHA-256 | `31087ffcc52c1535ed2ae0c2b778c70b8732113a7ed16257513bde13cb8e48cf` |
| canonical identity SHA-256 | `01fee9d9fab33467c4899b6cf9b2104c28363afc2588f20c3b5894fa649758a4` |
| status | `FROZEN_OFFLINE_PENDING_INDEPENDENT_REVIEW` |
| scenario | `obstacle_test1` |
| scenario SHA-256 | `872582dea10f911f4f2ba94fe7e40297d54cbd3efb97f8bfd93906526c229cb9` |
| candidate suite SHA-256 | `01e53d66ee9d716ac0d4a6b776417120cdd8997bc672e80d90e7752a95efb286` |
| scene root SHA-256 | `e12a69fa5463e723d115696b8872c27c71b03a9d029a9ef933343ae93ba6dd5e` |
| scene closure SHA-256 | `6ca5da14be6909815ac9c41bf6db0f8108e07082aea5aba22c91e833e6181746` |
| runtime model fingerprint | `3e4d82cf204d5929ff98c11fed5b3918ba3b92f0d698215860097d734d239694` |
| initial qpos SHA-256 | `a604dd11dc57ea655bf6d746dcf068a91e80a0a1eddc73d20c1a3800468f59d8` |
| collision contract | `abs-go2-collision-snapshot/v2`, 392 bytes |

Canonical identity is computed as SHA-256 over compact sorted-key UTF-8 JSON.
The executable absolute path is a runtime locator outside the canonical input;
the identity binds its SHA-256 and byte size plus source and CMake build-facts.
The current instrumented executable is
`unitree_mujoco/simulate/build2/unitree_mujoco`, SHA-256
`e4602a19c60ae8072648c8f113770b8d15a8cc3a0fe5b32ffcfc6e63cb40bc32`,
4,214,888 bytes.

## P1-08 boundary

The accepted P1-08 executable remains
`1e9b330f2b6c39dabaaa8424ee53c41d3be08ea00eb3e69ba71f332de50654e2`; the
accepted P1-08 canonical identity remains
`59dd13fed5ebd026ec519f2659643237502be8e4d8df5174a65b7d35ceb4f7e0`.
The Stage-B identity records these only as parent/provenance and rejects use of
the P1-08 manifest as the Stage-B executable identity. The accepted P1-08
manifest, identity, raw capture, and frozen flat pair were not changed.

## Preflight contract

`p1_08_baseline_capture.py` accepts `--stage-b-execution-manifest` only when
the scenario is exactly `obstacle_test1`; it checks the current binary,
scenario/root/closure, full loaded-model fingerprint, obstacle metadata,
initial state, fixed goal/injection, stabilized consumed config/plugin,
collision contract, and fixed 25.0-second binding. It generates the capture ID
internally, with no CLI override, and places the identity in the resolved
context, controlled environment, process facts, runtime record, and collision
v2 input path. Missing, malformed, substituted, or drifted inputs reject before
launch. Flat P1-08 behavior remains on its existing path.

The formal collision authority scope remains the two harness-controlled
`main.cc` PhysicsLoop paths. `simulate.cc` UI step-forward is excluded from
formal capture. UI reset/keyframe/step-forward/teleop remain prohibited.

## Offline verification

The following completed without launching MuJoCo/ROS2/runtime:

- `test_p1_10_stage_b_execution.py`: **12/12 PASS**;
- `test_p1_10_collision_authority.py`: **11 PASS**;
- `test_p1_10_scenario_suite.py`: **12/12 PASS**;
- `test_p1_10_obstacle_inventory.py`: **10 PASS**;
- `test_p1_10_saved_record_compare.py`: **17/17 PASS**;
- `test_run_record.py`: **54/54 PASS**;
- `test_formal_experiment_contract.py`: **22/22 PASS**;
- `test_formal_runtime_adapter.py`: **16/16 PASS**;
- `test_formal_runtime_binding.py`: **12/12 PASS**;
- `test_formal_rt_frame_recorder.py`: **12/12 PASS**;
- `test_p1_08_sim_clock.py`: **32 checks PASS**;
- `test_p1_08_baseline_identity.py`: **21 checks PASS**;
- `test_p1_08_harness.py`: **93 checks PASS**, including explicit rejection of
  the current Stage-B executable against the accepted P1-08 manifest;
- Stage-B manifest validation: **PASS**;
- CTest `p1_08_sim_clock_test`: **1/1 PASS**;
- Python compilation and JSON validation: **PASS**;
- `git diff --check`: **PASS**.

## State

`obstacle_test1` is **IMPLEMENTED / AWAITING RUNTIME VALIDATION**. Goal,
fall, and controller-timeout remain UNKNOWN. The five historical maps are not
an accepted formal suite. Overall status is **P1-10 Stage B RUNTIME
PREPARATION — IMPLEMENTED / AWAITING INDEPENDENT REVIEW**. Independent review
must pass before a separate Director authorization request for one controlled
observational run.
