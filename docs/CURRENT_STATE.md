# Current State

## Current Phase

Phase 1 — MuJoCo Simulation Validation

## Current Task

No active engineering task. P1-09 is formally closed; the next task requires a
separate Director selection under the Roadmap.

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
