# Repository Guidelines

## Project Structure & Module Organization

This workspace reproduces and deploys ABS locomotion for Unitree Go2. Key roots:

- `ABS/`: ABS paper reproduction, with Isaac Gym training in `ABS/training/` and legacy deployment code in `ABS/deployment/`.
- `quadruped_ros2_control_humble/`: ROS 2 Humble workspace. Controllers live in `controllers/`, URDF/config in `descriptions/`, hardware interfaces in `hardwares/`, and input nodes in `commands/`.
- `rl_sar/`: RL sim-to-real reference code and ROS packages.
- `unitree_mujoco/`: Unitree MuJoCo simulator and examples.
- `scripts/`: local helper scripts, such as daily summaries.
- Notes are in `README.md`, `CLAUDE.md`, `仿真部署手册.md`, and `服务器训练指南.md`.

Avoid treating `FileZilla3/` and generated build/install/log directories as source.

## Build, Test, and Development Commands

- Train ABS policy:
  ```bash
  conda activate abs
  cd ABS/training/legged_gym/legged_gym
  python scripts/train.py --task=go2_pos_rough --num_envs=1280 --max_iterations=4000 --headless
  ```
- Build the ROS 2 RL controller:
  ```bash
  cd quadruped_ros2_control_humble
  source /opt/ros/humble/setup.bash
  colcon build --packages-select rl_quadruped_controller --symlink-install
  ```
- Run MuJoCo deployment: start `unitree_mujoco/simulate/build2/unitree_mujoco`, then launch `ros2 launch rl_quadruped_controller mujoco.launch.py`, then run `ros2 run keyboard_input keyboard_input`.
- Run ROS package tests when present:
  ```bash
  colcon test --packages-select <package_name>
  colcon test-result --verbose
  ```

## Coding Style & Naming Conventions

C++ ROS code uses package-local `include/<package>/` headers and `src/` implementations. Keep classes in PascalCase, ROS parameters and YAML keys in `snake_case`, and package names lowercase with underscores. Python training code follows Legged Gym/RSL-RL style: 4-space indentation, `snake_case` functions, and task-grouped config classes.

Preserve the critical ordering convention: joints and contacts are `FR, FL, RR, RL`. `rl_quadruped_controller` now explicitly sorts ros2_control interfaces by YAML joint order, and `HardwareUnitree` uses an explicit `motor_index_map_` (`FR,FL,RR,RL -> motor[0..11]`). Do not mix `rl_quadruped_controller` with `unitree_guide_controller`; ABS deployment uses `rl_quadruped_controller`. Real Go2 must not enter RL/ABS until a real ray2d perception source is connected; current real tests are PASSIVE/FIXEDDOWN/FIXEDSTAND/hard stop only.

## Testing Guidelines

There is no single top-level test suite. Prefer package-scoped checks: `colcon test` for ROS packages and Python smoke tests for training changes. For controller edits, validate build success and a MuJoCo launch when dependencies are available. For RL observation changes, verify tensor dimensions: agile policy is 61-dim; recovery policy is 49-dim.

## Commit & Pull Request Guidelines

Recent commits use Conventional Commit-style prefixes, for example `docs: add AI-compatible project documentation`. Use `docs:`, `fix:`, `feat:`, or `refactor:` with a short imperative summary.

Pull requests should include the affected subsystem, commands run, simulation or training evidence, and changed robot assumptions. Include screenshots or log snippets for MuJoCo, Gazebo, or real-robot behavior changes.

## Security & Configuration Tips

Do not commit trained checkpoints, server credentials, or machine-specific secrets. Keep local library paths such as `/home/lidio/Libraries/libtorch-cpu-2.0.1` documented but configurable. On shared GPU servers, use `CUDA_VISIBLE_DEVICES` and do not kill other users' processes.
