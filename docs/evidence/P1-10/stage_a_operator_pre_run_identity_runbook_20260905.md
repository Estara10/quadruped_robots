# P1-10 Stage A Operator Runbook — Current-Instrumented Flat Pair

Director review only. These commands are not authorization to run. Operator
must wait for independent review and a separate Director authorization. This
runbook performs no automatic retry and does not use the superseded pair.

## Frozen identity

- Pair: `P1-10-STAGE-A-REPLAY-20260905-flat_goal_forward-stabilized-current-instrumented`
- Pair directory: `docs/evidence/P1-10/replay_pair_20260905_stage_a_current_instrumented`
- Pair manifest SHA-256: `7de8238e18dccd5bf1ec2554ae0e9969f60f18a008c48b265fad2eab93be8660`
- Stage-A manifest: `docs/evidence/P1-10/stage_a_execution_manifest_20260905.json`
- Stage-A manifest SHA-256: `550e07f0958f46a0a1ec3336736347aa07d25c40f287373af47ea07d4ee09920`
- Current instrumented executable SHA-256: `e4602a19c60ae8072648c8f113770b8d15a8cc3a0fe5b32ffcfc6e63cb40bc32`
- Fixed binding: `flat_goal_forward`, `stabilized`, `20260902`, `25.0 s`, `scene_default / mj_makeData:qpos0`.
- P1-08 canonical identity `59dd13fed5ebd026ec519f2659643237502be8e4d8df5174a65b7d35ceb4f7e0` is parent provenance only.

The old `replay_pair_20260903_saved_record_closure` is
`SUPERSEDED_NOT_RUN`; do not use or retry it.

## 1. Host readiness and exact child environment

From the repository root in a usable graphical terminal:

```bash
cd /home/lidio/quadruped_robots
source /opt/ros/humble/setup.bash
source /home/lidio/quadruped_robots/quadruped_ros2_control_humble/install/setup.bash

printf 'DISPLAY=%s\n' "${DISPLAY-<UNSET>}"
printf 'XAUTHORITY=%s\n' "${XAUTHORITY-<UNSET>}"
printf 'uid=%s gid=%s\n' "$(id -u)" "$(id -g)"
for ns in user mnt pid net ipc uts cgroup time; do
    printf 'ns_%s=' "$ns"
    readlink "/proc/self/ns/$ns"
done

xdpyinfo >/dev/null
direct_xdpyinfo_rc=$?
printf 'direct_xdpyinfo_rc=%s\n' "$direct_xdpyinfo_rc"
[ "$direct_xdpyinfo_rc" -eq 0 ] || exit "$direct_xdpyinfo_rc"

python3 - <<'PY'
import subprocess
import sys
from pathlib import Path
sys.path.insert(0, str(Path("scripts").resolve()))
from p1_08_baseline_capture import child_env
rc = subprocess.run(["xdpyinfo"], env=child_env(), stdout=subprocess.DEVNULL).returncode
print(f"child_env_xdpyinfo_rc={rc}")
raise SystemExit(rc)
PY
```

Both return codes must be `0`. Otherwise stop. Do not manually set or modify
`DISPLAY`, `XAUTHORITY`, `LD_LIBRARY_PATH`, model, configuration, scene, or
arguments. The harness owns the child environment.

## 2. Pair and preflight checks before each run

The Director-frozen pair directory and `pair_manifest.json` must already exist.
The Operator must not create or edit them.

```bash
PAIR=/home/lidio/quadruped_robots/docs/evidence/P1-10/replay_pair_20260905_stage_a_current_instrumented
test -d "$PAIR"
test -f "$PAIR/pair_manifest.json"
test ! -e "$PAIR/run_A"
test ! -e "$PAIR/run_B"
sha256sum "$PAIR/pair_manifest.json"
ps -eo pid=,ppid=,pgid=,stat=,comm=,args=
ls -l /dev/shm/mujoco_sim_clock /dev/shm/mujoco_rt_frame 2>&1
sha256sum \
  scenarios/p1_10/scenario_suite_manifest.json \
  scenarios/p1_10/flat_goal_forward.json \
  docs/evidence/P1-08/P1-08_baseline_manifest.json \
  unitree_mujoco/simulate/build2/unitree_mujoco \
  unitree_mujoco/unitree_robots/go2/scene_flat.xml \
  unitree_mujoco/simulate/config.yaml \
  quadruped_ros2_control_humble/descriptions/unitree/go2_description/config/robot_control.yaml \
  quadruped_ros2_control_humble/descriptions/unitree/go2_description/config/abs/config.yaml \
  quadruped_ros2_control_humble/controllers/rl_quadruped_controller/launch/mujoco.launch.py \
  quadruped_ros2_control_humble/install/rl_quadruped_controller/lib/rl_quadruped_controller/librl_quadruped_controller.so \
  quadruped_ros2_control_humble/install/hardware_unitree_mujoco/lib/libhardware_unitree_mujoco.so \
  /home/lidio/Libraries/mujoco-3.3.3/lib/libmujoco.so.3.3.3
```

The expected current instrumented executable hash is the value above, not the
accepted P1-08 executable hash. Any hash, directory, pair binding, process, or
X11 failure is a stop condition. Do not use broad `pgrep` in place of the
harness identity checks and do not manually delete shared-memory objects.

## 3. Run A — exactly once

Run A is permitted only after the checks above and explicit Director
authorization:

```bash
cd /home/lidio/quadruped_robots
source /opt/ros/humble/setup.bash
source /home/lidio/quadruped_robots/quadruped_ros2_control_humble/install/setup.bash
python3 scripts/p1_08_baseline_capture.py \
  --out-dir /home/lidio/quadruped_robots/docs/evidence/P1-10/replay_pair_20260905_stage_a_current_instrumented/run_A \
  --window-s 25.0 \
  --scene scene_flat.xml \
  --mujoco-bin /home/lidio/quadruped_robots/unitree_mujoco/simulate/build2/unitree_mujoco \
  --manifest docs/evidence/P1-08/P1-08_baseline_manifest.json \
  --stage-a-execution-manifest docs/evidence/P1-10/stage_a_execution_manifest_20260905.json \
  --scenario flat_goal_forward \
  --root-seed 20260902 \
  --variant stabilized \
  --initial-state-source scene_default
```

Run A must return `0` and all harness post-checks and required artifacts must
be valid. Any preflight failure, nonzero return, invalid record, abnormal or
forced shutdown, missing artifact, or residual process is an A failure. Stop
immediately; do not run B and do not retry A.

## 4. Run B — only after clean Run A

After Run A passes every post-check, execute B exactly once with the same
frozen command and `run_B` output directory:

```bash
cd /home/lidio/quadruped_robots
source /opt/ros/humble/setup.bash
source /home/lidio/quadruped_robots/quadruped_ros2_control_humble/install/setup.bash
python3 scripts/p1_08_baseline_capture.py \
  --out-dir /home/lidio/quadruped_robots/docs/evidence/P1-10/replay_pair_20260905_stage_a_current_instrumented/run_B \
  --window-s 25.0 \
  --scene scene_flat.xml \
  --mujoco-bin /home/lidio/quadruped_robots/unitree_mujoco/simulate/build2/unitree_mujoco \
  --manifest docs/evidence/P1-08/P1-08_baseline_manifest.json \
  --stage-a-execution-manifest docs/evidence/P1-10/stage_a_execution_manifest_20260905.json \
  --scenario flat_goal_forward \
  --root-seed 20260902 \
  --variant stabilized \
  --initial-state-source scene_default
```

After B, repeat the process/shm checks and inspect all facts. A B failure is
terminal: stop and do not retry B.

## 5. Required artifacts and comparison boundary

Each successful run directory must contain:

`preflight_evidence.json`, `runtime_record.jsonl`, `process_facts.json`,
`scenario_resolved_manifest.json`, `p1_10_context.json`,
`sim_clock_timing.jsonl`, `rt_frame_timing.jsonl`, `reader_stats.json`,
`orphan_inventory.json`, `mujoco_raw.log`, `ros2_launch_raw.log`, and
`orchestrator_raw.log`.

Only after both A and B succeed, and only after Director separately authorizes
the comparison stage, run:

```bash
python3 scripts/p1_10_stage_a_saved_record_compare.py \
  --pair-dir /home/lidio/quadruped_robots/docs/evidence/P1-10/replay_pair_20260905_stage_a_current_instrumented
```

The comparator reads only the saved `run_A/runtime_record.jsonl` and
`run_B/runtime_record.jsonl` plus their required provenance artifacts. It does
not read live shared memory and refuses external paths, symlinks, missing or
drifted preflight evidence, and output overwrite. It must not be used to turn
flat replay into obstacle-collision or ABS-effectiveness evidence.

## 6. Prohibitions

Do not use the UI for reset, keyframe, step-forward, teleoperation, or any
other intervention. Do not change environment variables, models, configs,
scenes, parameters, or pair manifests. Do not run the superseded or historical
pairs. Do not commit/push or start benchmark, FormalRun, P1-11, P1-12, or
P1-13. A flat Stage-A pass is only infrastructure/repeatability evidence and
does not constitute P1-10 final acceptance.
