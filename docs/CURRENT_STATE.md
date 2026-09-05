# Current State

### P1-10 offline saved-record closure freeze

The new pair
`P1-10-REPLAY-20260903-saved-record-closure-flat_goal_forward-stabilized`
was frozen offline in
`docs/evidence/P1-10/replay_pair_20260903_saved_record_closure/` with pair
manifest SHA-256
`86ae55914db294d269d6f70909bfad1878c287c644f5a85c4075fa758f923a6c`.
It binds `flat_goal_forward`, `stabilized`, `root_seed=20260902`, fixed
`25.0 s`, `scene_default / mj_makeData:qpos0`, the accepted P1-08 baseline,
and the current P1-10 suite. Its frozen status is
**FROZEN_OFFLINE_PENDING_INDEPENDENT_REVIEW**; Operator Run A/Run B launch is prohibited until the Reviewer
accepts this offline closure. Historical failed pairs remain unchanged and
were not retried.

The offline-only comparator
`scripts/p1_10_saved_record_compare.py` accepts only `--pair-dir`, derives
`run_A`/`run_B` from the frozen manifest, and requires process facts, runtime
record, resolved manifest, and P1-10 context for each run. It never opens live
shared memory and never launches MuJoCo/ROS2. Process facts, terminal
structure, session identity, and all fixed scenario/baseline bindings are
validated fail-closed; terminal domain and artifact origin are fail-closed;
binding fields are never backfilled; exact/numeric/excluded projection rules
remain frozen; outputs refuse overwrite. Production context writer now emits
the complete resolved scene binding without fallback. Offline comparator tests
are **17 test methods PASS**.
No Run A, Run B, saved-record comparison, benchmark, FormalRun, or Phase 1
acceptance result exists yet. Flat replay is currently only the P1-10
infrastructure/repeatability sub-gate; even a successful flat replay would
not be P1-10 final acceptance or authorize P1-11/P1-12. P1-10 remains
**IMPLEMENTED / AWAITING INDEPENDENT REVIEW**, and Phase 1 remains **NOT
ACCEPTED**.

The REJECT and repair scope are recorded in
`docs/evidence/P1-10/replay_pair_20260903_saved_record_closure/comparator_contract_closure_20260903.md`.
The Operator runbook is
`docs/evidence/P1-10/replay_pair_20260903_saved_record_closure/operator_replay_runbook.md`.
The baseline identity fields are recorded distinctly: document SHA-256
`6c3563c25d45cc275db6b083f9f0fc0cc2067b48bc8f4a93dcace9f6d42817ea` and
canonical identity
`59dd13fed5ebd026ec519f2659643237502be8e4d8df5174a65b7d35ceb4f7e0`.

### P1-10 residual-process preflight review

The residual-process repair sub-gate is independently accepted with known
issues. Overall P1-10 remains **IMPLEMENTED / AWAITING INDEPENDENT REVIEW**;
the latest offline pair is frozen pending independent review and the
behavioral-validation Stage B/C gates remain open. The historical
`P1-10-REPLAY-20260902-flat_goal_forward-stabilized` remains
`FAILED_FOR_THIS_PAIR` and was not retried. See
[`REVIEW_2026-09-02_RESIDUAL_PROCESS_PREFLIGHT.md`](evidence/P1-10/REVIEW_2026-09-02_RESIDUAL_PROCESS_PREFLIGHT.md).

The historical pair `P1-10-REPLAY-20260903-flat_goal_forward-stabilized` was
frozen and attempted exactly once. Run A passed residual-process identity
inspection but failed X11 preflight (`xdpyinfo rc=1`) before ldd or child
launch; Run B was not attempted and no retry was made. The new pair is
`FAILED_FOR_THIS_PAIR`; pair manifest SHA-256 is
`b86a19887dee8a441c7a5643eca698ea4a092a92bd549e972d41b1067c8f049e` and
result SHA-256 is
`72caf926e8da7e2c27ef60b7c21eb8699613f1ffc7a35734ea93c00fc226dc3e`.
The historical pair `P1-10-REPLAY-20260902-flat_goal_forward-stabilized` also
remains `FAILED_FOR_THIS_PAIR` and was not retried.

The 2026-09-03 X11 preflight failure is recorded as **ENVIRONMENT BLOCKED**:
the current and exact harness child environments both used `DISPLAY=:0` and
`XAUTHORITY=/run/user/1000/gdm/Xauthority`, and `xdpyinfo` failed identically
with rc=1. The X0 socket node existed but was not reachable; the lower-level
cause remains UNKNOWN. No harness code change was justified. See
[`x11_preflight_failure_diagnosis_20260903.md`](evidence/P1-10/x11_preflight_failure_diagnosis_20260903.md).

The read-only execution-context comparison confirms the operator-supplied
known-good graphical terminal previously had `xdpyinfo rc=0`, while the
current Execution and exact child environment both use `DISPLAY=:0` and
`XAUTHORITY=/run/user/1000/gdm/Xauthority` and both fail with rc=1. The current
X0 socket is not reachable; namespace versus credential sub-cause remains
UNKNOWN because the known-good uid/gid and namespace links were not archived.
The minimal prerequisite is the same reachable desktop namespace/session plus
an exact-child-environment `xdpyinfo rc=0` check. See
[`x11_execution_context_comparison_20260903.md`](evidence/P1-10/x11_execution_context_comparison_20260903.md).

## Current Phase

Phase 1 — MuJoCo Simulation Validation

## Current Task

P1-10 — Runtime-Bound Initial State and Supported-Variant Closure — **IMPLEMENTED / AWAITING INDEPENDENT REVIEW**. The frozen flat replay pair remains offline-only and pending its own final review; no A/B runtime comparison exists. Historical pairs `replay_pair_20260902` and `replay_pair_20260903` remain `FAILED_FOR_THIS_PAIR` and were not retried.

Stage-B offline collision authority for canonical `obstacle_test1` is **ACCEPT WITH KNOWN ISSUES — independent review 2026-09-03**. It binds future collision snapshots to a harness-generated capture identity and to a full loaded-model fingerprint; it covers only the two harness-controlled PhysicsLoop paths, not UI step-forward. The accepted P1-08 executable identity is `1e9b330f...`; the current instrumented Stage-B binary is a distinct artifact and must receive an independent Stage-B execution manifest/identity. See [`REVIEW_2026-09-03_STAGE_B_AUTHORITY.md`](evidence/P1-10/REVIEW_2026-09-03_STAGE_B_AUTHORITY.md).

No obstacle runtime has occurred. `obstacle_test1` remains **IMPLEMENTED / AWAITING RUNTIME VALIDATION**; `obstacle_test2`–`obstacle_test5` remain `UNSUPPORTED`; goal/fall/controller-timeout authority and episode-wide collision-free coverage remain UNKNOWN. Flat replay is only an infrastructure/repeatability sub-gate. P1-11/P1-12/P1-13 were not started; no benchmark, pilot, multi-seed evaluation, or FormalRun exists. Phase 1 remains NOT ACCEPTED.

P1-08 — Freeze MuJoCo Model, Effective Timing, and Dynamics Baseline — **ACCEPT WITH KNOWN ISSUES — final independent review 2026-09-02**. No active engineering task after closure; P1-10 does not start automatically; Phase 1 remains NOT ACCEPTED. Accepted scope: v2 MuJoCo model/config/artifact hash-bound baseline (closure `8d9218de…`, 18 files); authoritative sim-clock/timing capture (run_id `4f14416672244cbfb4af93573bd9d86c`, session `1970665031624`, fixed 25 s; 0 rejected reads); observed physics 0.002 s exactly / wall-clock 500 Hz; Policy/RA tick ≈49.97 Hz (rt_frame, 1250 LIVE, mean 20.014 ms); reproducible identity v2 `59dd13fe…` (old `14e8d14f…`/`bdd47a0d…` superseded); real runtime record VALID with two-phase finalize and real process facts (SIGINT delivered, rc 0, no escalation, `FRAMES_ENDED_RC0`). Retained Known Issues: capture-end orphan inventory UNKNOWN (no capture-end artifact archived); Recovery cadence UNKNOWN/not observed (0 transitions); direct controller-callback cadence UNKNOWN (5.003 ms DERIVED only); corrected reader-stats `generated_at` byte-stability boundary (gap math deterministic, timestamp embedded); no benchmark/paper-equivalence/Sim-to-Real/performance conclusion. Evidence: [`REVIEW_2026-09-02_FINAL.md`](evidence/P1-08/REVIEW_2026-09-02_FINAL.md) + [`P1-08_v2_baseline_capture_20260902.md`](evidence/P1-08/P1-08_v2_baseline_capture_20260902.md) + [`P1-08_stride2_gap_correction_20260902.md`](evidence/P1-08/P1-08_stride2_gap_correction_20260902.md) + `capture_20260901_v2/`. Old v1 capture, its identities, and the 2026-09-01 preflight failure remain superseded/non-acceptance. Historical closures: the 2026-09-01 capture (`capture_20260901_rerun`) used the **v1 sim-clock contract + unhashed timing** and is **NOT an accepted baseline** (superseded). Testability/writer-boundary closure (2026-09-01): v2 C++ test now CTest-registered (`include(CTest)` + `add_test`; built from `test/p1_08_sim_clock_test.cpp`; `ctest` **1/1 PASS**, direct **PASS (39 checks)**; Release `-O3 -DNDEBUG`-safe via CHECK, not assert); hook test uses a **non-capturing static callback** with a real install→publishStep→writer→reader round-trip + no-op case; `SimClockWriter::publish()` is **mutex-serialized** (no reliance on external `sim.mtx`) and construction is **fail-closed** (explicit odd in-progress marker + NaN payload + zero monotonic kept until first publish, invalidating stale frames); hook store/load atomic with documented lifetime; default shm name unchanged; old v1 test retired to `test/p1_08_sim_clock_v1_legacy.cpp` (**NOT registered/run**). **Atomicity + C++→Python integration closure (2026-09-01)**: `SimClockWriter` 构造改为**先原子 release-store odd in-progress 标记、再原子写 magic/version/NaN/0**（无 `memset` 先于 odd 标记）；所有共享字段读写全原子（`sim_time` 经 IEEE-754 bit-pattern `uint64` 视图原子存取，ABI 40B/`<4Qd` 不变）；reader 亦全原子（修复 `snap.sequence` 未初始化 bug）；新增 CMake 构建的 `p1_08_sim_clock_bridge`（无 MuJoCo）在唯一临时 shm 上跑**真实** `SimClockWriter`，Python `read_sim_clock(shm_path=…)` 实际读回同一 v2 snapshot（正向集成非 struct.pack 伪造）；负例在真实 C++ shm 上 fail-closed；`struct.pack` 假字节测试标注为 decode/rejection-only；临时 shm 唯一且清理、生产 `/mujoco_sim_clock` 未触碰。C++ v2 test **PASS (45)**（含 stale valid v2 snapshot 被新 writer 构造失效 + 多线程多调用者压力 0 撕裂）；`ctest` **1/1**；Python `test_p1_08_sim_clock.py` **PASS (32)**。`p1_08_model_probe.cpp` 为离线静态 probe，不承诺其 `mj_step` 发布 sim-clock；三个运行时 `mj_step` 路径覆盖事实保留；`publishStep` 无 hook 为 no-op、非库级 every-step 保证。证据：[`P1-08_repair_v2_atomicity_integration_20260901.md`](evidence/P1-08/P1-08_repair_v2_atomicity_integration_20260901.md)。**Recapture harness closure (2026-09-01，无 runtime run)**：child-env `ldd`（returncode/timeout/exception/not-found 全检查 + 证据归档）、launch identity 绑定（实际 `mujoco_bin` hash + `--scene` 解析到 canonical root_xml + path-escape + artifact/config/plugin hash）、窄化 shm 清理（仅任务精确 shm，无活进程确认 + unlink 失败即 FAIL）、runtime record fail-closed（每个 distinct present 帧原始传入 `record_snapshot`，坏帧使整条 record INVALID）、两阶段 facts（顶层 `exit_code` 仅双 wait=0 才为 0；`shutdown_request_source` 真实 SIGINT）、统一 try/finally cleanup（stop sampling → SIGINT+wait → 仅超时才 TERM 且逐条记录）、固定 25s 窗口（非 25 即 FAIL）、identity **v2**（schema `abs-go2-p1-08-baseline-identity/v2`、generator 2.0，全部输入必需、缺失即失败、旧 v1 capture 被 v2 生成器明确拒绝）。离线测试：C++ `p1_08_sim_clock_test` **PASS (45)** + ctest 1/1；`test_p1_08_sim_clock.py` **32**；`test_p1_08_baseline_identity.py` **21**（mutation/missing/old-v1-reject）；`test_p1_08_harness.py` **29**。证据：[`P1-08_recapture_harness_closure_20260901.md`](evidence/P1-08/P1-08_recapture_harness_closure_20260901.md) + [`P1-08_harness_lifecycle_closure_20260901.md`](evidence/P1-08/P1-08_harness_lifecycle_closure_20260901.md) + [`P1-08_process_signal_truthfulness_20260901.md`](evidence/P1-08/P1-08_process_signal_truthfulness_20260901.md) + [`P1-08_generic_signal_exception_20260901.md`](evidence/P1-08/P1-08_generic_signal_exception_20260901.md) + [`P1-08_cleanup_error_persistence_20260901.md`](evidence/P1-08/P1-08_cleanup_error_persistence_20260901.md)。**Cleanup-error persistence / poll-exception closure (2026-09-01，无 runtime run)**：per-child `cleanup_errors`（stage/exception_type/exception_message/time_s）与 `poll_attempts` 结构化持久化到最终 `process_facts.json` 的对应 child facts + 顶层 `cleanup_error_count`（不替代 per-child）；`proc.poll()` 异常不再早退——记录 poll_attempt 异常事实后仍执行 SIGINT signal-attempt 与 wait-attempt、无法确认时 wait_rc 保持 None/UNKNOWN（不伪造已退出/rc=0），其余 child 继续清理；`wait_pid` poll-异常容忍（retry + deadline 返回 None）；`process_facts.json` 落盘 → `recorder.finalize(same facts)` → stats/logs 不可绕过。新 negative tests 实际读取生成 `process_facts.json`（不 mock `build_process_facts`）；`test_p1_08_harness.py` **PASS (77)**。**Generic signal-exception fail-closed cleanup closure (2026-09-01，无 runtime run)**：`_signal_pg()` 现捕获 `Exception`（非 BaseException/SystemExit/KeyboardInterrupt），任意异常记录 `delivered=false` + `result`/`exception_type`/`exception_message` + signal/目标 PID-PGID/时间，绝不误记为 delivered/SIGINT source/forced；`_finalize_capture()` 每 child 异常隔离（`_handle_child` 捕获 + `cleanup_errors` 记录 + 仍尝试 wait 记 UNKNOWN/失败），单 child 异常不阻断其余 child，`process_facts.json` 落盘与 `recorder.finalize(same facts)` 不可绕过，TERM/KILL 仍仅 timeout 分支且 delivered 语义。新增 negative tests（SIGINT RuntimeError、TERM 普通异常、finalize 连续性、多 child 不互相阻断、facts 不伪造正常关闭）；`test_p1_08_harness.py` **PASS (62)**。**Process-signal fact truthfulness closure (2026-09-01，无 runtime run)**：`shutdown_request_source="SIGINT"` 仅在实际 delivered SIGINT 后写入；`forced_termination=true` 仅在 TERM/KILL 实际 delivered 后写入；`_signal_pg` 每信号记录 `delivered`（killpg 成功才 True，失败保留 `failed:<reason>`）；`_wait_or_escalate` 仅在 TERM/KILL delivered 时置 `escalated`；`build_process_facts` 从 delivered 时间线重算 per-child `escalated` 与顶层 `forced_termination`/`source`，三者一致；非零自然退出永不 forced。新增 negative tests（SIGINT 成功/失败、TERM 成功/失败、KILL 成功/失败、natural nonzero、facts↔recorder 一致），信号 sender 注入（mock `os.killpg` + fake proc，不启动真实 child）。`test_p1_08_harness.py` **PASS (51)**。**Harness lifecycle + full model closure + deterministic clock-test closure (2026-09-01，无 runtime run)**：完整 model-closure 校验（`resolve_closure` 递归发现 include XML + mesh/hfield asset，相对当前 XML 目录解析，escape/cycle/missing 全部 fail-closed，manifest 显式记录 included XML 列表与逐文件 SHA；`verify_manifest_hashes` 重跑整 closure 对比）；preflight 异常/锁/shm fail-closed（preflight 整体 try/except → 结构化 PRECHECK FAIL + 证据归档；`pgrep` 区分 not-found(rc=1)/exec-fail(rc=2/异常)；harness 独占 flock 锁 preflight 前取得、cleanup 后释放，锁被占即 FAIL；窄 shm 清理 before/after/spawn 三查）；统一两阶段 cleanup（单一 `_finalize_capture`：stop_sampling → SIGINT+记录 → wait → 仅超时才 TERM/KILL 且逐信号记录 → 先写 process_facts.json → 用同一 facts finalize → 存 stats/logs）；facts 语义 fail-closed（`forced_termination` 仅真实 TERM/KILL；`shutdown_request_source` 仅真实 SIGINT 否则 UNKNOWN；`exit_code=0` 仅全 required child wait=0，缺失→None 不造 0）；**C++ 压力测试 flake 根因 = test bug（读取前阶段 hook 的 stale-but-consistent snapshot），非 contract bug**；修复 = 跳过 pre-stress snapshot + 收紧为精确相等断言；**50 次连续 ctest + 60 次 direct 全通过**。**v2 recapture run FAILED AT PREFLIGHT (2026-09-01，单次、不重试)**：授权的一次性 v2 采集在 preflight 即中止（orchestrator 的 `ldd` 检查用了当前 env 而非子进程 env，`libddsc` 显示 not-found；子进程 env 下实际可解析——测量脚本缺陷，非环境/hash 失败）。未启动任何进程、未创建 `capture_20260901_v2`；按任务规则判 **BLOCKED / FAILED FOR THIS RUN**，不重试、不产出 accepted baseline。orchestrator 的 ldd preflight 已修复（child-env），后续授权重跑可用。重新采集需 Director 另行授权。证据：[`P1-08_v2_capture_preflight_fail_20260901.md`](evidence/P1-08/P1-08_v2_capture_preflight_fail_20260901.md)。 **Repair increment (no runtime run):** (A) v2 strict odd/even seqlock contract (`abs_sim_clock_contract.h` `kVersion=2`; writer marks odd before payload, even after; acquire/release; reader rejects odd/changed/version-mismatch/non-finite; no torn snapshot accepted); all in-scope `mj_step` sites covered — PhysicsLoop main.cc ×2 (`g_sim_clock.publish`) + UI step-forward simulate.cc (`publishStep` via global hook installed by `main()`); C++ `p1_08_sim_clock_test` **PASS** + Python `test_p1_08_sim_clock.py` **PASS (22)**. (B) canonical baseline identity (`build_p1_08_baseline_identity.py` v1.0): `sha256(json.dumps(INPUT, sort_keys=True, separators=(",",":")).encode("utf-8"))` binding SHA-256 of every raw timing JSONL + process-facts + manifest + asset/binary/config hashes + git commit/dirty + generator version/hash; `test_p1_08_baseline_identity.py` **PASS (6)** (determinism, timing-change, manifest-change, recomputation). Old identity `bdd47a0d…` superseded; demonstration identity over old capture = `99b995b0…` (not accepted). Offline baseline facts from the v1 era remain recorded (closure `8d9218de…`, MuJoCo 3.3.3/timestep 0.002 s/Euler/Newton/iter 100/tol 1e-8/gravity −9.81/impratio 100; controller static 1000/200 Hz/decimation 4/`switching_mode=stabilized_switch`; observed timing from the v1 capture: physics 0.002 s, policy/RA ≈50 Hz, Recovery inactive, controller derived 5.0 ms). Manifest refreshed with **v2** `mujoco_executable` `f51ee432…`. Evidence: [`P1-08_repair_20260901.md`](evidence/P1-08/P1-08_repair_20260901.md) + [`P1-08_simulation_baseline.md`](evidence/P1-08/P1-08_simulation_baseline.md) (superseded record) + [`P1-08_baseline_identity.json`](evidence/P1-08/P1-08_baseline_identity.json). Not paper/Sim-to-Real equivalence, not benchmark, not tuning. Phase 1 remains NOT ACCEPTED.

P1-07 — Separate and Test Paper-Faithful vs Stabilized Switching — **ACCEPT WITH KNOWN ISSUES — final independent review 2026-09-01**. No active engineering task after closure; P1-08 must not start automatically. Acceptance is switching-only: two explicit `abs.switching_mode` modes — `paper_faithful_switch` (paper rule only: `RA >= -0.05` → Recovery, `RA < -0.05` → Agile; no hysteresis, no forced hold, equality at `-0.05` enters) and `stabilized_switch` (**default**, pre-P1-07 behavior byte-for-byte: strict `RA > -0.05` enter, `RA < -0.08` + `recovery_hold_steps` 30-step hold exit); truth tables PASS at all threshold equalities (`p1_07_switching.cpp` **292 checks**, exit 0); invalid mode fails initialization (no silent fallback); NaN/±Inf RA fail-closed (no transition; existing `runRAModel` path unchanged); default-compat regression PASS; `colcon build --packages-select rl_quadruped_controller` **PASS**. Known issues (recorded, not fixed): CTest not registered (executable target only, run directly — no `add_test()` added); `paper_faithful_switch` is switching-only, **NOT** full paper-faithful ABS; MuJoCo/ROS2 runtime switching unmeasured (**UNKNOWN**); P1-06 Eq.21/Eq.22 optimizer MISMATCH unresolved and outside this task. Evidence: [`REVIEW_2026-09-01_FINAL.md`](evidence/P1-07/REVIEW_2026-09-01_FINAL.md) + [`P1-07_switching_modes.md`](evidence/P1-07/P1-07_switching_modes.md) + [`P1-07_switching_decision_table.json`](evidence/P1-07/P1-07_switching_decision_table.json). Phase 1 remains NOT ACCEPTED.

### P1-10 residual-process preflight correction

The previous broad `pgrep -af` substring detector is replaced by offline
implemented `/proc` executable/argv identity inspection. Exact MuJoCo
identity and attributable capture ROS launch/controller identities remain
fail-closed rejects; the harness PID and ancestor chain plus shell/path-only
mentions are excluded. Process-table read failure, malformed identity, or
ambiguous controller attribution returns `uncertain` and rejects preflight.
Focused offline regression is **93 checks PASS**. The residual-process repair
sub-gate is independently accepted with known issues; see
[`REVIEW_2026-09-02_RESIDUAL_PROCESS_PREFLIGHT.md`](evidence/P1-10/REVIEW_2026-09-02_RESIDUAL_PROCESS_PREFLIGHT.md).
Overall P1-10 remains **IMPLEMENTED / AWAITING INDEPENDENT REVIEW**; the
latest offline pair remains pending independent review and the behavioral-
validation Stage B/C gates remain open. No replay was run for this
documentation synchronization, the historical runtime pairs remain
`FAILED_FOR_THIS_PAIR`, and neither was retried. Phase 1 remains NOT ACCEPTED.

P1-06 — Recovery Eq.21/Eq.22 and Safe-Twist Optimizer Parity — **ACCEPT WITH KNOWN ISSUES — final independent review 2026-08-31**. P1-07 was later authorized by the Director (2026-09-01); P1-08 and later tasks must not start automatically. Key results (recorded, not fixed): Eq.22 reference `get_pos_integral` (`testbed.py:55-61`) matches paper (yaw-coupled second-order); deployment `pos_x=vx*tau`/`pos_y=vy*tau` (`StateRL.cpp:623-624`) omits both terms → **MISMATCH** (gap, ABS_PAPER_NOTES:131); first-order goal-penalty consequence **MISMATCH**; gradient clip L2-norm vs per-element **MISMATCH** (not an approved stabilized variant); iteration count testbed 10 (not paper MATCH) vs deployment 3 (paper upper-bound MATCH only), testbed↔deployment **MISMATCH**; objective constants/RA λε/lr/clip-type/feasibility/fallback/RA-input concatenation: paper **UNKNOWN**, testbed↔deployment **MATCH**; twist bounds ±[1.5,0.3,3.0] paper+testbed+deployment **MATCH**. Numeric fixture `test_p1_06_recovery_optimizer.py` **14/14 PASS** (independent arithmetic only; not runtime parity). Not paper equivalence / runtime parity / benchmark / safety claim; runtime parity and feasibility rates UNKNOWN. Evidence: [`P1-06_recovery_optimizer_parity.md`](evidence/P1-06/P1-06_recovery_optimizer_parity.md) + [`P1-06_recovery_optimizer_matrix.json`](evidence/P1-06/P1-06_recovery_optimizer_matrix.json) + [`REVIEW_2026-08-31_FINAL.md`](evidence/P1-06/REVIEW_2026-08-31_FINAL.md). Phase 1 remains NOT ACCEPTED.

P1-05 — RA Label, Model Semantics, and Agile Operational Binding — **ACCEPT WITH KNOWN ISSUES — independent review 2026-08-31**. Key result: recovered RA label (`testbed.py`) = `g=+1 collision/−1`, `l=tanh(log2(d/0.65+1e-8))`, Bellman `V'=γ·max(g̃',min(l,V_next))+(1−γ)·max(l,g̃')` with `γ=0.999999`, 10-step collision softening, terminal bootstrap `1.0`, loss ×100. Paper Eq.16–17 **MATCH** (structural/source-level); Eq.18 **INTENTIONAL_VARIANT/UNKNOWN**; collision definition, terminal bootstrap, loss scale **INTENTIONAL_VARIANT/UNKNOWN**. Deployed 19-D RA order matches paper Eq.14 (lin_vel/ang_vel/goal/rays; goal shaped variant, rays fail-closed). `ra_value.pt` = 19→[64,64,1]→Tanh ∈ (−1,1), higher=higher risk. Switching three-way: paper `RA >= −0.05`; recovered testbed `recovery=(v_pred>-twist_eps=-0.05)` strict, immediate, no hold (`testbed.py:64,324`); deployment ENTER `ra>−0.05`, EXIT `ra<−0.08`+30-step hold — **INTENTIONAL_VARIANT** (paper equivalence not claimed). No MISMATCH. Known issues: no label numeric fixture (Bellman MATCH is structural, not end-to-end numerical); paper Eq.18 exact log base/σ/terminal/loss UNKNOWN; RA↔Agile = `OPERATOR_DECLARED`. Evidence: [`P1-05_ra_semantics_20260830.md`](evidence/P1-05/P1-05_ra_semantics_20260830.md) + [`P1-05_ra_19d_matrix_20260830.json`](evidence/P1-05/P1-05_ra_19d_matrix_20260830.json) + [`REVIEW_2026-08-31_RA_SEMANTICS.md`](evidence/P1-05/REVIEW_2026-08-31_RA_SEMANTICS.md). Phase 1 remains NOT ACCEPTED.

P1-04 — Agile 61-D Observation Parity is **ACCEPTED WITH KNOWN ISSUES** by independent review (2026-08-31): all 61 slots are classified; contact/ang_vel/gravity/velocity/previous-action/ray values are element-wise `MATCH` against the real training oracle; goal/command and timer remain `INTENTIONAL_VARIANT`; nominal dof_pos matches while the training bias distribution remains `UNKNOWN`; ray validity is a fail-closed variant. Deployment's global `±100` post-assembly clamp and the training environment `LeggedRobot.step()` clip are consistent; the fixture is in-domain and the complete downstream policy-library chain was not inspected, so out-of-domain equivalence is not claimed. No MISMATCH or unclassified shift/default exists. See [`exec-plans/P1-04.md`](exec-plans/P1-04.md), [`parity matrix`](evidence/P1-04/p1_04_parity_matrix_20260830.md), and [`Reviewer disposition`](evidence/P1-04/REVIEW_2026-08-31_PARITY.md).

P1-09 status: **ACCEPTED (2026-08-30) — P1-09AE runtime-record subchain remains ACCEPT WITH KNOWN ISSUES**. [`exec-plans/P1-09.md`](exec-plans/P1-09.md). An independent Reviewer accepted the two original remaining Acceptance items after one real controlled formal-closure run: MuJoCo + StateRL → HUD → authoritative JSONL record → two-phase finalize → real process facts → P1-02 `FormalRunWriter` / validator.

P1-09AE runtime-record subchain (2026-08-30) is **ACCEPT WITH KNOWN ISSUES**:
one real controlled MuJoCo + StateRL → HUD → authoritative JSONL record →
two-phase finalize → real process facts → post-run summary chain was verified.

P1-09 original task is **ACCEPTED**. The independent Reviewer confirmed both
former formal blockers in the 2026-08-30 formal-closure run: (1) an
authoritative structured safety/terminal-event source exists
(`scripts/formal_runtime_binding.py` single reducer over the authoritative frame
and orchestrator wait facts; no text-log parsing); and (2) a representative real
episode was classified by the P1-02 validator as **INVALID**
(`validator_completed=true`) because authoritative `simulation_time_s` /
collision / fall / goal / timeout sources are absent — never as fabricated
SUCCESS/VALID. The verdict and run identity are traceable: formal
`run-0cbf4a…` ↔ runtime record `26861e6c…` ↔ session `15818838355107`. See
[`P1-09C_formal_closure_20260830.md`](evidence/P1-09/P1-09C_formal_closure_20260830.md).
All other known UNKNOWNs remain deferred classifications, not P1-09 blockers:
P1-01 provenance/order, P1-08 reproducibility, P1-02 follow-up fields,
Phase-2 hardware, and future observability/benchmark work. Phase 1 remains
**NOT ACCEPTED**.

### P1-09G correction — 2026-08-28 post-unlock attempt

The preceding P1-09G wording describes only the first GLFW-blocked attempt and
is superseded for current state by the [combined P1-09G evidence](evidence/P1-09/P1-09G_controlled_shutdown.md): a separately authorized post-unlock attempt
reached `[REC-ENTER]`, `[REC-EXIT]`, normal controller-manager/plugin shutdown,
and ROS launch `rc=0` with no captured `terminate`/SIGABRT. The observed worker
teardown path is **PASS**. The raw log also records MuJoCo `SIGINT`, wait
timeout, subsequent `SIGTERM`, and final `mujoco_rc=143`; this is **FAIL** for
clean shutdown, so P1-09G is **BLOCKED / REJECTED EVIDENCE** and P1-09 remains
**EXECUTING / NOT ACCEPTED**.

The strict one-run P1-09I record is [`P1-09I_controlled_shutdown.md`](evidence/P1-09/P1-09I_controlled_shutdown.md).

### P1-09J — MuJoCo shutdown static audit

The [P1-09J audit](evidence/P1-09/P1-09J_mujoco_shutdown_audit.md) confirms
that the MuJoCo source has no SIGINT handler, starts a permanent
`UnitreeSdk2BridgeThread` with no stop condition or join, calls worker-thread
`exit(0)`, and ends main with `pthread_exit` after joining only the physics
thread. The P1-09I SIGINT target/PGID was correct; application handling remains
unproven, while inherited ignored SIGINT from the asynchronous shell launch is
`LIKELY`. The primary process-lifecycle defects are **CONFIRMED**. No code was
changed and no runtime was run in P1-09J.

### P1-09K — MuJoCo signal/quit runtime diagnosis

The [P1-09K evidence](evidence/P1-09/P1-09K_signal_quit_diagnosis.md) confirms
the running MuJoCo process had `SigIgn=0x6`, including SIGINT, and therefore
ignored SIGINT; it timed out and was cleaned up with SIGTERM (`rc=143`). The
independent UI-close instance started, but the X11 helper found no MuJoCo
window, so UI Quit/window-close behavior remains `UNKNOWN`. No ROS2 or
controller was started. P1-09 remains **EXECUTING / NOT ACCEPTED**.

### P1-09L — graceful-shutdown lifecycle design

The design-only [P1-09L evidence](evidence/P1-09/P1-09L_graceful_shutdown_design.md)
defines a single request-stop path, stoppable bridge and physics workers,
ordered joins, and normal `main` return. It is **not implemented** and requires
independent design review before any lifecycle code change. It does not alter
the P1-09K runtime result: clean MuJoCo shutdown remains **FAIL / NOT PROVEN**.

### P1-09M — unified controlled-exit lifecycle implementation

P1-09M is **BLOCKED — implementation not started**. The local checkout does
not contain the `RecurrentThread` destructor/stop/join implementation, so it is
not possible to prove that the bridge callback thread stops before
`mjModel`/`mjData` are released. No simulator code, tests, build, or runtime
was changed or executed. See
[P1-09M evidence](evidence/P1-09/P1-09M_graceful_shutdown_implementation.md).

### P1-09N — RecurrentThread lifecycle / binary contract audit

P1-09N resolved the effective SDK to `/home/lidio/Libraries/unitree_sdk2`,
package version 2.0.0, and archive SHA256
`08402aea74150dfbfc3fbfded4ca746916a8d892b54d2bade0cbf392a3be4029`.
Binary evidence confirms `pthread_create` with detached state, `Wait()` setting
`mQuit`, and destructor cancellation without `pthread_join`. Therefore the
SDK has a stop-like operation but does not provide a proven join/completion
barrier for this bridge. P1-09M remains **BLOCKED**; see
[P1-09N evidence](evidence/P1-09/P1-09N_recurrent_thread_contract.md).

P1-03 status: **ACCEPTED / COMPLETED** — offline paper-to-code trace accepted by [final independent review](evidence/P1-03/REVIEW_2026-08-27_FINAL.md) on 2026-08-27. 11 records: 1 MATCH, 4 STABILIZED_VARIANT, 4 MISMATCH, 1 UNKNOWN, 1 CONFLICT. Not paper-equivalence proof, runtime validation, benchmark evidence, or Phase 1 Acceptance.

P1-02 status: **ACCEPTED / COMPLETED** — the [final independent review](evidence/P1-02/REVIEW_2026-08-26_FINAL.md) accepts the offline fixture-level formal contract only. Runtime adapter remains incomplete; existing evaluator output remains `LEGACY / NON-ACCEPTANCE`; no benchmark or formal runtime result is claimed.

P1-01 — Policy Artifact Provenance and Joint/Contact/Action Order Contract — **ACCEPTED WITH KNOWN ISSUES** (independent Reviewer, 2026-08-30; Director-approved scope reconciliation DEC-010)

Status: **ACCEPTED WITH KNOWN ISSUES (2026-08-30 independent review)** — 61/19/49 parity and all local/live P1-01F deployment-contract checks PASS; the exact deployed Agile/RA/Recovery weight-lineage chains are CONFIRMED (hash/tensor/byte equality); current declared policy-order `FL,FR,RL,RR` → remap → controller/MuJoCo `FR,FL,RR,RL` is documented and covered by asymmetric contract evidence; current simulation-only runtime chain demonstrated by P1-09. Historical config/seed/command/Git/export invocation/raw RA dataset are **deferred reproducibility (not P1-01 blockers)**; RA training on `model_4000.pt` is recorded as **OPERATOR_DECLARED**; real Go2 `foot_force[0..3]` is **Phase 2 hardware-only**; no real-robot result is claimed. No remap change is justified. See [`REVIEW_2026-08-30_SCOPE_ALIGNMENT.md`](evidence/P1-01/REVIEW_2026-08-30_SCOPE_ALIGNMENT.md).

State model and role boundaries: [`PROJECT_STATE_MODEL.md`](PROJECT_STATE_MODEL.md)

## Phase Acceptance

- Phase 1: **NOT ACCEPTED**
- Phase 2: **NO-GO**
- Phase 3: **NOT STARTED**

## Critical Blockers

- Recovered Agile/Recovery checkpoints and exports and the RA source/JIT artifacts close deployed **weight lineage** by exact tensor and byte equality. Historical config/command/seed/commit/export-invocation records and an independent RA execution log are **absent** — classified as **deferred reproducibility** (not a P1-01 blocker per the 2026-08-30 Director-approved scope reconciliation); the RA training fact (`model_4000.pt`) is recorded as **OPERATOR_DECLARED**. The current declared policy-order `FL,FR,RL,RR` → remap → controller/MuJoCo `FR,FL,RR,RL` is the **operational mapping accepted conditionally** on the recorded operator-declared training order and existing asymmetric contract evidence; it is **not** claimed to be independently recovered historical artifact metadata.
- Real Go2 `foot_force[0..3]` semantics are **Phase 2 hardware-only** (not independently captured; not a P1-01 blocker per DEC-010).
- Isaac Gym Go2 `terminate_after_contacts_on=["base"]` currently matches no runtime body.
- Recovery solver and switching contain known paper mismatches.
- P1-02 formal experiment contract is **ACCEPTED / COMPLETED** for offline schema, writer/validator, comparison-gate, and fixture-level evidence; authoritative runtime event, telemetry, seed and provenance sources are not yet connected; existing evaluator outputs remain `LEGACY / NON-ACCEPTANCE`. This is not runtime benchmark evidence or Phase 1 Acceptance.

## P1-01 Evidence

- Three deployed hashes, installed bindings, executable shapes and deterministic outputs: **PASS**.
- Recovered weight lineage: Agile `model_4000.pt` actor → recovered export → deployed artifact, RA source model → recovered JIT → deployed artifact, and Recovery `model_15000.pt` actor → recovered export → deployed artifact: **PASS** by exact tensor/byte equality.
- `Historical pre-DEC-010 finding (superseded as a P1-01 Acceptance blocker):` historical evidence retrieval — scoped run directories contain checkpoints plus TensorBoard events but no config/hparams/command/metadata sidecars; checkpoints contain no seed/config/commit/order metadata; recovered Git has no committed Go2 config/export snapshot or run identifier: **artifact order and historical run metadata were reported `UNKNOWN`** (now **deferred reproducibility** per DEC-010).
- `Historical pre-DEC-010 finding (superseded as a P1-01 Acceptance blocker):` RA exact executed Agile checkpoint binding — source-code/filename candidate only; no execution log, command record, embedded metadata or dataset manifest: reported **UNKNOWN** (now recorded as **OPERATOR_DECLARED** linkage + **deferred reproducibility** per DEC-010).
- Isaac Gym DOF/body/feet order and ROS2→motor→MuJoCo mapping: **PASS**.
- Current action-order remap: the operational mapping `FL,FR,RL,RR` → remap → controller/MuJoCo `FR,FL,RR,RL` is **accepted conditionally** on the recorded operator-declared training order and existing asymmetric contract evidence (DEC-010); it is **not** claimed to be independently recovered historical artifact metadata.
- P1-02 run-ID closure: duplicate run IDs are rejected at comparison CLI level; FormalRunWriter allocates distinct process-local UUID4 IDs, rejects caller-supplied IDs that differ from the allocation, and `write_summary()` fails before create/overwrite on a mismatch while defaulting omitted IDs to the writer allocation. Evidence: [`p1_02_mechanical_tests.json`](evidence/P1-02/p1_02_mechanical_tests.json).
- P1-02 Acceptance: **ACCEPTED / COMPLETED** — [final independent review](evidence/P1-02/REVIEW_2026-08-26_FINAL.md) accepts the offline formal-contract schema, writer/validator, comparison gate, and 22 fixture-level mechanical tests. This does not constitute runtime benchmark evidence, a formal runtime result, or Phase 1 Acceptance; runtime and legacy limitations remain unchanged.
- P1-03: **ACCEPTED / COMPLETED** — offline paper-to-code trace accepted by [final independent review](evidence/P1-03/REVIEW_2026-08-27_FINAL.md); not paper-equivalence, runtime, benchmark, or Phase 1 evidence. It is not blocked by P1-01 under the Roadmap dependency conclusion.
- P1-01F corrected rolling timer, contact temporal filter, nominal bias, fail-closed ray freshness and finite-value vetoes; helper-level fault tests **PASS**.
- Live ROS2+MuJoCo P1-01F: normal writer, writer freeze/exit, ray NaN/Inf, observation/RA/action/target/final-command non-finite injections all **PASS**. Timing is one `steady_clock` domain; 200 ms freshness + one 20 ms ray-check interval is met. Telemetry proves finite post-veto targets and zero Kp/Kd/torque.
- Contract: [`POLICY_IO_CONTRACT.md`](POLICY_IO_CONTRACT.md). Goal shaping remains **INTENTIONAL ENGINEERING VARIANT**. Per DEC-010: historical deployed-artifact order and RA exact Agile binding are **deferred reproducibility (not P1-01 blockers)**; real Go2 foot-force slot order is **Phase 2 hardware-only**.
- `Historical pre-DEC-010 finding (superseded as a P1-01 Acceptance blocker):` 2026-08-30 read-only closure audit — verified all three deployed hashes and load paths again; confirmed no deployed TorchScript embeds order/provenance metadata; an asymmetric probe on the **actual deployed artifacts** (one-hot + operating-point Jacobian + 40-point mean-|coupling|) found the deployed **Recovery** policy's `prev_action` block empirically at slots 37:49 with 11/12 diagonal coupling (consistent with the candidate 49-D FL-first layout) — **PARTIAL** order evidence; the deployed **Agile** probe was inconclusive (7/12), so deployed Agile order, RA ↔ Agile binding, historical run metadata and real foot-force semantics were reported `UNKNOWN`. Evidence: [`p1_01_closure_audit_20260830.md`](evidence/P1-01/p1_01_closure_audit_20260830.md) + [`p1_01_closure_inventory_20260830.json`](evidence/P1-01/p1_01_closure_inventory_20260830.json) + probe results.
- 2026-08-30 server-snapshot re-audit (`ABS_fuwuqi/ABS`): **directly** verified the exact Agile checkpoint→export→deployed chain (checkpoint `model_4000.pt` `iter=4000` → export → deployed: weight-equal 8/8 actor tensors; export↔deployed byte-equal SHA `5a87d6…`) and the exact RA named→JIT→deployed chain (named RA → JIT → deployed: weight-equal 6/6; JIT↔deployed byte-equal SHA `05c40f…`; conversion script default path is the named RA). The RA naming mechanism (`policy_name[:-3]+"_ra.pt"` in `testbed.py:197,211,219,568`) is **source-verified** in the snapshot. Snapshot `.git` (HEAD `9b95329f`) tracks legged-gym code but **not** the Go2 configs (untracked); TensorBoard has **no hparams**; no shell/job/command logs. Therefore the deployed **joint order** (no immutable config/commit binding) and RA **training-data** binding remain UNKNOWN. Evidence: [`P1-01_server_snapshot_reaudit_20260830.md`](evidence/P1-01/P1-01_server_snapshot_reaudit_20260830.md) + [`P1-01_server_snapshot_inventory_20260830.json`](evidence/P1-01/P1-01_server_snapshot_inventory_20260830.json).
- `Historical pre-DEC-010 finding (superseded as a P1-01 Acceptance blocker):` 2026-08-30 final evidence closure — exhaustive search of **all** mandatory evidence roots (`ABS/`, `ABS_fuwuqi/ABS/`, `quadruped_ros2_control_humble/`, `unitree_mujoco/`, `docs/evidence/P1-01/`, `artifacts/`, git + nested git). No immutable config/commit/command/seed binding to the deployed Agile training order; RA `get_load_path` defaults (`load_run=-1`/`checkpoint=-1`) are source-causal only — historical execution binding **not recoverable**; real Go2 `foot_force[0..3]` has **no verifiable official slot contract** (local + official unitree_sdk2 LowState has no foot-order; MuJoCo bridge order is simulation convention FR,FL,RR,RL; unitree_ros2 claim via third-party DeepWiki unverified in primary file) → **Phase 2 hardware-only UNKNOWN**. **No remap change justified** (no direct evidence of incorrectness). Evidence: [`P1-01_final_closure_20260830.md`](evidence/P1-01/P1-01_final_closure_20260830.md) + [`P1-01_final_closure_inventory_20260830.json`](evidence/P1-01/P1-01_final_closure_inventory_20260830.json).
- `Historical pre-DEC-010 finding (superseded as a P1-01 Acceptance blocker):` latest independent P1-01 re-review (**REJECT**) — recovered Agile/RA/Recovery weight lineage is verified, but deployed artifact order, RA exact Agile binding, historical run metadata, real Go2 foot-force semantics and pre-existing semantic gaps were reported `UNKNOWN`. Evidence: [`REVIEW_2026-08-26_PROVENANCE_CLOSURE.md`](evidence/P1-01/REVIEW_2026-08-26_PROVENANCE_CLOSURE.md). DEC-010 supersedes the historical-reproducibility portion of that REJECT as a P1-01 Acceptance blocker.

## Current Metrics

- All existing simulation results: **LEGACY / NON-ACCEPTANCE**.
- Historical paired report: Full ABS 38/40; Agile-only 30/40. No matched seeds or true-contact metric at that time.
- Later true-contact session: 5/6 with one collision.
- Latest four-scene session: 3/4 with one terrain fall.
- The old 12/12 result is an arrival baseline, not a formal collision-free score.

## Known Dirty Changes

- User deletion of `paper.txt`.
- User path gains changed from 3.0 to 2.5.
- User real-network comment changed to `enp7s0`.
- Untracked legacy report generator with stale claims; intentionally not committed.

## Real Robot Gate

Allowed: `PASSIVE`, `FIXEDDOWN`, `FIXEDSTAND`, emergency-stop checks and software dry-run.

ABS/RL real test: **NO-GO**

## Next

P1-09G correction: the first GLFW-blocked capture is retained. The post-unlock
capture verified Recovery enter/exit and controller/plugin unload without an
abort signature, but the raw log records MuJoCo SIGINT timeout, SIGTERM, and
`rc=143`; it is not clean shutdown and must not be treated as P1-09G or P1-09
Acceptance.

P1-09K confirmed the MuJoCo runtime `SigIgn=0x6` (SIGINT ignored) and a
SIGINT timeout followed by TERM cleanup (`rc=143`). The UI-close path was not
actually delivered because no MuJoCo X11 window target was found; it remains
`UNKNOWN`. See [P1-09K evidence](evidence/P1-09/P1-09K_signal_quit_diagnosis.md).

P1-09L produced a design-only proposal for a verified `RUNNING →
STOP_REQUESTED → THREADS_JOINED → PROCESS_EXITED` lifecycle. Its independent
review conditionally allowed implementation but required explicit model-reload
safety and external-thread closure; no runtime validation occurred.

P1-09M was blocked by the Unitree `RecurrentThread` completion semantics;
P1-09N confirmed detached creation and cancel-without-join. P1-09O now
implements the approved project-owned joinable bridge replacement, but runtime
clean-shutdown evidence is still absent. P1-09 remains **EXECUTING / NOT
ACCEPTED**.

`Historical pre-DEC-010 finding (superseded):` Active Engineering Task **P1-09** was **EXECUTING**; P1-09O was independently reviewed and rejected as incomplete due to the model-reload race. P1-03 was **ACCEPTED / COMPLETED** (offline trace only). **P1-01 was reported `BLOCKED` at that time**; no Phase 1 Acceptance, benchmark, formal runtime result or Phase-2 ABS/RL was authorized. (P1-09 has since been ACCEPTED; P1-01's historical-reproducibility blockers were superseded by DEC-010.)

P1-09O is implemented with a project-owned joinable RobotBridge worker,
condition-variable stop wakeup, and main-owned final `mjModel`/`mjData` cleanup.
Offline lifecycle test and simulator build pass. Runtime clean shutdown remains
`UNKNOWN` because this task prohibited running MuJoCo; the subsequent
independent Reviewer rejected the implementation as incomplete due to reload
and lifecycle-boundary gaps.

### P1-09O reviewer result and session close — 2026-08-28

The independent Reviewer **REJECTED** P1-09O as a complete lifecycle repair.
Normal final shutdown ordering is supported by source evidence, but PhysicsLoop
can replace `mjModel`/`mjData` while RobotBridge retains and reads cached
pointers. This model-reload race is `HIGH`; concurrent `JoinableThread`
start/stop semantics and external DDS thread teardown also remain incomplete.
P1-09O is therefore not accepted and no further MuJoCo clean-shutdown run is
authorized yet.

Director approved the fail-closed reload safety barrier on 2026-08-29
(DEC-009): while the bridge is active, a model reload must first stop and join
every m/d-accessing worker; without a proven safe rebind/restart protocol, the
reload is refused. P1-09P is implemented with conservative active-lifetime
reload rejection and offline evidence; independent review is required before
any runtime validation. P1-09 remains **EXECUTING / NOT ACCEPTED**.

P1-09P Reviewer disposition: **ACCEPT WITH KNOWN ISSUES** for the offline
barrier increment. P1-09Q static audit retains High issues/UNKNOWN for direct
bridge–PhysicsLoop m/d concurrency and SDK DDS thread lifetime; P1-09 remains
**EXECUTING / NOT ACCEPTED**. See
[P1-09Q audit](evidence/P1-09/P1-09Q_md_concurrency_audit.md).

P1-09S closes the reviewed lock-boundary issues offline: post-`sim.Load()` m/d
replacement is guarded, bridge start waits for initial m/d readiness, bridge
constructor I/O is outside the m/d guard, and `ray_exit` is immediate again.
Mechanical tests/build pass; runtime evidence remains `UNKNOWN`. P1-09 remains
**EXECUTING / NOT ACCEPTED**.

P1-09R is implemented with the existing `sim.mtx` guarding direct RobotBridge
m/d access and with failed reload-candidate cleanup. Offline static, lifecycle,
and simulator build checks pass; runtime and DDS ownership evidence remain
`UNKNOWN`. P1-09 remains **EXECUTING / NOT ACCEPTED**.

P1-09S is recorded as **REJECTED** for lock-internal diagnostics, restart
ambiguity, and insufficient mechanical verification. P1-09T is also **REJECTED**:
its separate active/terminal flags retained a lifecycle race, its concurrent
coverage was not real, and its first U-target build command was stale. P1-09U
replaces those flags with one mutex-protected `INITIAL → RESERVED → ACTIVE →
STOPPING → TERMINAL` state machine, reserves before worker start, rejects all
post-terminal restart attempts, adds a 64-iteration real concurrent mechanical
test, and records every actual command exit code. P1-09U is **AWAITING
INDEPENDENT REVIEW**. No MuJoCo runtime was run; clean process shutdown and DDS
lifetime remain UNKNOWN. P1-09 and Phase 1 remain **NOT ACCEPTED**.

P1-09V closes the terminal-invariant and constructor m/d gaps offline: only
`STOPPING` can transition to `TERMINAL`, main joins the bridge before terminal
completion and m/d release, and `mj_name2id(m, ...)`/`m->nu` are read under
`sim.mtx` into local values. R/S/T/U/V static checks and O/P/U/V lifecycle tests
pass with actual command exit codes recorded; no runtime was run. P1-09V is
**AWAITING INDEPENDENT REVIEW**. P1-09 and Phase 1 remain **NOT ACCEPTED**;
DDS lifetime and clean process shutdown remain UNKNOWN.

P1-09W consolidated self-check is complete offline. It found and corrected the
INITIAL-to-STOPPING allowance, worker-side terminal submission, and uncaptured
scene metadata reads. `beginStop()` now accepts only RESERVED/ACTIVE;
`completeTerminal()` is main-owned after bridge join/reset; and
`printSceneInformation()` snapshots model metadata under `sim.mtx` before any
printing. Final R/S/T/U/V/W tests, simulator build, and O/P/U/V lifecycle tests
all exited 0. P1-09W reviewer disposition is **ACCEPT WITH KNOWN ISSUES**.
No runtime was run; P1-09X is the single bounded MuJoCo-only clean-shutdown
validation. P1-09 remains **EXECUTING / NOT ACCEPTED** and Phase 1 remains
**NOT ACCEPTED**. DDS lifetime and clean process shutdown remain UNKNOWN.

P1-09X was attempted once in the MuJoCo-only boundary and was **BLOCKED**
before runtime initialization by GLFW (`could not initialize GLFW`); no retry
was performed. P1-09 remains **EXECUTING / NOT ACCEPTED**, and Phase 1 remains
**NOT ACCEPTED**. Bridge and physics shutdown evidence remains UNKNOWN.

P1-09Y preflight confirmed `DISPLAY=:0` is not reachable (`xdpyinfo`/`xset`
rc=1) and found no installed Xvfb/headless wrapper or project headless GLFW
path. The exact display failure cause remains UNKNOWN. A new P1-09X run
requires separate Director authorization.

P1-09Z was explicitly authorized as the single retry, but its mandatory
X11 preflight with `DISPLAY=:0` and `/run/user/1000/gdm/Xauthority` failed
(`xdpyinfo` rc=1). MuJoCo was not started. P1-09Z is **BLOCKED**; P1-09 and
Phase 1 remain **NOT ACCEPTED**.

P1-09AA produced a design-only fail-closed binding audit from the authoritative
`/mujoco_rt_frame` contract to the P1-02 writer. No code or runtime was
changed. Runtime adapter integration, authoritative collision/fall/simulation
time/seed/config sources, and P1-01 provenance UNKNOWN remain open.

P1-09AB implemented the offline fixed-source recorder boundary and rejection
fixtures. It never calls FormalRunWriter while required runtime authority is
missing; final recorder tests and existing frame/adapter/formal-contract tests
pass. No MuJoCo runtime or formal VALID run was produced. P1-09AB awaits
independent Reviewer review; P1-09 remains **EXECUTING / NOT ACCEPTED**.

P1-09AC clarified source-frame, `rl_step`, and formal-telemetry sequence
semantics and added an independent rejection index. Rejected samples do not
consume formal sequence numbers; synthetic/legacy/unknown input has no runtime
eligibility. Offline recorder and regression tests pass; no writer or runtime
was used. P1-09AC awaits independent Reviewer review.

P1-09AD's first graphical attempt was blocked at the mandatory `xdpyinfo`
preflight (`DISPLAY=:0` + `/run/user/1000/gdm/Xauthority`, rc=1). In the
user-confirmed graphical terminal the display was reachable: MuJoCo 3.3.3
started, and after `Ctrl+C` exited `rc=0` with no TERM/KILL. The archived raw
log proves startup/initialization only; it does not contain explicit bridge
stop/join, physics join, or m/d release output, so the internal shutdown
ordering remains UNKNOWN. P1-09AD is therefore **PARTIAL PASS**, not a full
clean-shutdown proof; P1-09 remains **EXECUTING / NOT ACCEPTED** and Phase 1
remains **NOT ACCEPTED**. Evidence:
[`P1-09AD_clean_shutdown_graphical.md`](evidence/P1-09/P1-09AD_clean_shutdown_graphical.md).

P1-09AE completes the runtime data chain offline: `scripts/run_record.py` +
`scripts/record_runtime_run.py` save the full payload of the existing real
`/mujoco_rt_frame` into one per-run JSONL record (session identity, frame/step/
time preserved; flag-0 fields recorded unavailable; no mock fill), and
`scripts/post_run_summary.py` computes the post-run summary from that saved
record only (validity, authoritative-source, outcome
SUCCESS/FAILURE/INVALID/UNKNOWN, termination reason, normal shutdown, duration,
velocity, yaw, Recovery usage, RA stats, safety faults). Simulation time,
reached-goal, timeout, collision and fall have no authoritative source today
and are recorded UNKNOWN; SUCCESS is never produced. Process facts (exit code,
forced termination, request source) are accepted explicitly from the run
orchestrator at finalize and never inferred. A Reviewer REJECT fix made the
record fail-closed: any present non-LIVE/malformed/non-authoritative frame,
continuity break (session change, sequence/rl_step/monotonic rollback),
run-identity mismatch, duplicate/misplaced terminal, negative duration, or
malformed process fact invalidates the whole record, with no implicit bool
conversion (`"false"` is never True); frame status is whitelisted to
`{LIVE, MISSING}` (unknown/null/wrong-type statuses fail closed), and every
LIVE payload is fully schema-validated (missing field, wrong type/length,
NaN/Inf, malformed nested structure → INVALID) before any statistics; a
MISSING frame is a legal gap only when it carries no payload and no
availability (a MISSING frame with non-empty payload/availability →
`malformed_missing_frame` → INVALID); the recorder runs a two-phase capture →
stop_sampling → finalize lifecycle (`record_runtime_run.py` stops sampling
before finalize; duplicate finalize and post-stop/post-finalize frame writes
are rejected; missing process facts stay UNKNOWN); 54/54 offline tests pass.
P1-09AE 的两阶段记录器已获独立 Reviewer **PASS**：CAPTURE 阶段只采集
固定真实帧，STOP 后拒绝帧写入，FINALIZE 才以真实 process facts 写唯一
terminal；54/54 离线测试及回归通过。该两阶段运行时采集已于 2026-08-30
完成（见下方”P1-09AE 2026-08-30 env-corrected real capture”），因此 P1-09
仍为 **EXECUTING / NOT ACCEPTED**。
See [`P1-09AE_runtime_record_completion.md`](evidence/P1-09/P1-09AE_runtime_record_completion.md).

P1-09AE then performed one supervisor-controlled MuJoCo-only graphical run.
The recorded process had `SigIgn=0x4` (not SIGINT), received one SIGINT whose
`kill` syscall returned 0, did not time out or receive TERM/KILL, and `wait`
returned `MUJOCO_RC=0`. This is runtime evidence of bounded SIGINT-to-normal
process exit. The simulator's raw log does not expose bridge/physics join,
`mjModel`/`mjData` release, or DDS teardown, so those internal ordering facts
remain UNKNOWN. P1-09 remains **EXECUTING / NOT ACCEPTED** and Phase 1 remains
**NOT ACCEPTED**. Evidence:
[`P1-09AE_clean_shutdown_run.md`](evidence/P1-09/P1-09AE_clean_shutdown_run.md).

### P1-09AE — 2026-08-30 env-corrected real Runtime Record Capture

One Director-authorized, environment-corrected simulation-only run completed
the runtime-record main chain end-to-end. The only change was the launch
environment: every child process (MuJoCo, ros2 launch, HUD, recorder, control
pub) inherited `LD_LIBRARY_PATH` with unitree_sdk2/lib + libtorch (matching
`launch_abs_sim.sh`), resolving the earlier 11:28 `libddsc.so.0` plugin-load
failure that had aborted the controller before it became active. Result: real
MuJoCo + StateRL → HUD LIVE (same session `8049381969251`) → two-phase recorder
(306 LIVE + 176 MISSING frames, continuity OK, one record
`run_id=d9c988223cec4b1385a8fd031abc385f`) → STOP (no terminal yet) → normal
process exit (ros launch SIGINT `rc=0`, MuJoCo SIGINT `rc=0`, no TERM/KILL, no
residue, from actual `wait`s) → FINALIZE (`normal_shutdown=true`,
`FRAMES_ENDED_RC0`) → `post_run_summary` (record VALID, authoritative true,
`outcome=UNKNOWN` — never SUCCESS). `reached_goal`/`timeout`/`collision`/`fall`/
`simulation_time_s` remain UNKNOWN; internal bridge/physics join and DDS
teardown order remain UNKNOWN. Not a benchmark/formal run/FormalRunWriter use;
not P1-09 Acceptance, not P1-02 runtime integration, not Phase 1 Acceptance.
Evidence: [`P1-09AE_record_capture_20260830.md`](evidence/P1-09/P1-09AE_record_capture_20260830.md)
and its raw logs; the pre-run launch failure is recorded in
[`P1-09AE_record_capture_fail_20260830.md`](evidence/P1-09/P1-09AE_record_capture_fail_20260830.md).

### P1-10 Stage B/C historical-map preparation — 2026-09-03

The five historical obstacle roots were formally inventoried offline. The
candidate suite is
[`obstacle_candidate_suite_manifest.json`](../scenarios/p1_10/obstacle_candidate_suite_manifest.json),
with evidence in
[`historical_five_map_formalization_20260903.md`](evidence/P1-10/historical_five_map_formalization_20260903.md)
and
[`historical_five_map_inventory_20260903.json`](evidence/P1-10/historical_five_map_inventory_20260903.json).
`scene_obstacle.xml` is a byte-identical alias of `scene_test1.xml`; the five
formal candidate identities are `obstacle_test1`–`obstacle_test5`, while
`scene_terrain.xml` remains an extra future/generalization candidate. All five
candidate statuses are `UNSUPPORTED`: their XML/closure/obstacle metadata are
hash-bound, but obstacle runtime authority, collision/terminal authority, and
repeatability are not closed. The existing accepted flat suite and its frozen
pair were not changed.

P1-10 remains **IMPLEMENTED / AWAITING INDEPENDENT REVIEW** and is not
Accepted. Flat replay remains only the Stage A infrastructure/repeatability
sub-gate; even a flat pass would not be P1-10 final acceptance. The current
latest frozen flat pair remains
`FROZEN_OFFLINE_PENDING_INDEPENDENT_REVIEW`, manifest SHA
`86ae55914db294d269d6f70909bfad1878c287c644f5a85c4075fa758f923a6c`, with
comparator `scripts/p1_10_saved_record_compare.py` in the same evidence
directory. Historical pairs remain explicitly `FAILED_FOR_THIS_PAIR` and are
not current or active.

The next behavioral-validation gap is obstacle-scenario definition/freeze,
obstacle authority, collision/terminal authority, real runtime record, and
repeatability. No obstacle runtime, benchmark, FormalRun, P1-11, P1-12, or
P1-13 was started. Phase 1 remains **NOT ACCEPTED**.

### P1-10 Stage B minimal collision/terminal authority — 2026-09-03

The minimum offline collision authority for `obstacle_test1` is now implemented
in the simulator physics path and the existing saved runtime recorder. The
versioned `/mujoco_collision_v2` source is bound to the frozen scene root/full
closure and XML-derived obstacle signatures, reads authoritative MuJoCo
`mjData::ncon`/contact pairs after each existing physics step, and records only
robot↔bound-obstacle contact as obstacle collision. Floor, self-contact, other,
unknown, stale, malformed, and scene-mismatched inputs remain fail-closed; the
legacy `/mujoco_collision` diagnostics are not formal authority. An observed
valid contact can enter `runtime_record.jsonl`; a sampled final no-contact value
cannot be promoted to an episode-wide collision-free result without complete
episode coverage.

This is observability/authority work, not an ABS algorithm or control change.
Goal arrival is source-trace-only and not a formal terminal producer; fall and
controller timeout remain UNKNOWN. The fixed 25 s capture window is not relabeled
as a controller timeout. `obstacle_test1` has not been runtime validated, the
five historical maps are not an accepted P1-10 suite, and no obstacle runtime,
benchmark, FormalRun, or later P1 task was started. P1-10 remains
**IMPLEMENTED / AWAITING INDEPENDENT REVIEW**; Phase 1 remains **NOT ACCEPTED**.
Evidence: [`stage_b_collision_terminal_authority_20260903.md`](evidence/P1-10/stage_b_collision_terminal_authority_20260903.md).
The pre-repair obstacle candidate-suite manifest SHA was
`4552fc5b408855174337c7b2d73acf49a92a5b44622a540e0d1b90082ab58cb5`; the
scenario status remains `UNSUPPORTED`, while only `obstacle_test1`'s nested
collision-authority binding is `IMPLEMENTED / AWAITING RUNTIME VALIDATION`.

### P1-10 Stage B capture-binding and runtime-fingerprint repair — 2026-09-03

The independent-review REJECT blockers are repaired offline. Collision snapshot
v2 now carries a harness-generated capture identity and a full loaded-model
fingerprint; the recorder binds the snapshot to the same capture identity and
expected fingerprint recorded in context/process facts. The shared canonical
fingerprint covers every model geom, including non-obstacle contact-relevant
identity. Closure SHA-256 remains the preflight file identity, while the
runtime fingerprint is the loaded MuJoCo contact-model identity.

Formal authority scope is limited to the two harness-controlled `main.cc`
PhysicsLoop paths. `simulate.cc` UI step-forward is interactive debugging only,
not formal capture; the runbook prohibits UI reset/keyframe/step-forward/teleop
intervention. The instrumented executable is distinct from accepted P1-08
executable SHA-256
`1e9b330f2b6c39dabaaa8424ee53c41d3be08ea00eb3e69ba71f332de50654e2`; any
future obstacle run needs an independent Stage-B manifest/identity.

Current obstacle candidate-suite manifest SHA-256 is
`01e53d66ee9d716ac0d4a6b776417120cdd8997bc672e80d90e7752a95efb286`, with
inventory evidence at
[`historical_five_map_inventory_20260903.json`](evidence/P1-10/historical_five_map_inventory_20260903.json).
The latest frozen flat pair remains manifest SHA-256
`86ae55914db294d269d6f70909bfad1878c287c644f5a85c4075fa758f923a6c`, and the
saved-record comparator remains offline-only with **17/17 tests PASS** in
[`replay_pair_20260903_saved_record_closure/`](evidence/P1-10/replay_pair_20260903_saved_record_closure/).
Historical pairs remain `FAILED_FOR_THIS_PAIR`, never current or active.

Status is **P1-10 Stage B AUTHORITY IMPLEMENTATION — REPAIRED / AWAITING
INDEPENDENT REVIEW**. `obstacle_test1` remains
`IMPLEMENTED / AWAITING RUNTIME VALIDATION`; goal/fall/timeout remain UNKNOWN,
the five maps are not an accepted formal suite, and P1-10 is not accepted.
No runtime capture, A/B replay, benchmark, FormalRun, P1-11, P1-12, or P1-13
was started; Operator authorization remains withheld. Phase 1 remains
**NOT ACCEPTED**.
