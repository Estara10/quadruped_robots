# P1-08 — v2 Sim-Clock Testability and Writer-Boundary Closure (2026-09-01)

Status: **REPAIRED / AWAITING INDEPENDENT REVIEW**.
Phase 1 remains **NOT ACCEPTED**.

Scope: closes the Reviewer gaps on the P1-08 v2 sim-clock repair (test
registration/assertion validity, hook-test callback type, writer
serialization/init boundary, retired v1 test). **No MuJoCo/ROS2 runtime
recapture was run**; no baseline produced or upgraded; no P1-10; no controller/
policy/RA/Recovery/threshold/switching/gains/optimizer/solver/timestep/model/
scene/dynamics change.

## 1. Exact root-cause corrections

| Reviewer issue | Root cause found | Correction |
|---|---|---|
| A. v2 C++ test not registered/compiled/run | CMake built `p1_08_sim_clock_test` from **`test/p1_08_sim_clock.cpp`** (the old v1 test); the v2 file `test/p1_08_sim_clock_test.cpp` was never compiled, and no `add_test`/CTest existed. The reported "PASS" ran the **v1** test. | CMake now builds `p1_08_sim_clock_test` from `test/p1_08_sim_clock_test.cpp` (v2), adds `include(CTest)` + `add_test(NAME p1_08_sim_clock_test …)`. |
| B. `PublishFn` hook test callback type error | The v2 test installed a **capturing lambda** to a function pointer (`installPublishHook([&hook_ns,…](…){…})`), which cannot convert to `PublishFn` — a compile error that was never caught because the file was never compiled. | Test uses a **non-capturing static callback** `route_to_writer`; verified round-trip. |
| C. writer lacks serialization / init fail-closed | `publish()` had no internal lock (relied on external `sim.mtx`); construction used a bare `memset` and left `seq=0` (rejected only by the `seq==0` rule). | `std::mutex` inside `publish()`; fail-closed construction writes an explicit **odd in-progress marker** + NaN payload + zero monotonic, kept until first publish; atomic hook store/load; default shm name unchanged. |
| D. old v1 test treated as v2 evidence | The registered test binary was the v1 test (assert-based, not Release-safe, wrong v2 sequence expectations). | v1 file retired to `test/p1_08_sim_clock_v1_legacy.cpp` with a legacy header and **NOT registered**; the registered test is the v2 test. |

## 2. v2 writer/reader and hook test coverage (C++ `p1_08_sim_clock_test`, 39 checks PASS)

- construction/init state rejected (odd marker + NaN + zero monotonic);
- stable EVEN snapshot accepted (sequence 4/6, values verified);
- ODD (in-progress) sequence rejected;
- sequence changed during copy (before != after) rejected;
- wrong magic / wrong + old version rejected (no silent fallback);
- NaN / +Inf / -Inf `sim_time` rejected;
- zero monotonic timestamp rejected;
- hook dispatch round-trip: install non-capturing `route_to_writer` →
  `publishStep(9000, 0.018)` → writer → `readSnapshot` accepts `{9000, 0.018}`;
- no hook installed → `publishStep` is a **no-op** (shm unchanged);
- multi-threaded multi-caller publish stress: 4 writer threads × 20000
  publishes of `(M, M·1e-9)` + concurrent reader; every accepted snapshot must
  satisfy `sim == mono·1e-9` (cross-field tear detection). **0 inconsistent
  accepted** — the internal mutex prevents any torn snapshot from being valid.

## 3. CTest registration proof and Release-safe failure behavior

- `ctest --output-on-failure` in `unitree_mujoco/simulate/build2`:
  **1/1 Test #1: p1_08_sim_clock_test … Passed (0.02 s)**; `ctest_exit=0`.
- Direct run: **RESULT: PASS (39 checks), exit 0**.
- Release-safety: `CMAKE_BUILD_TYPE=Release` → `-O3 -DNDEBUG`. The test uses a
  custom `CHECK` macro (not `assert`), so every failed check produces a
  non-zero exit under `-DNDEBUG`; CTest catches failures via the exit code.
- The old v1 test `test/p1_08_sim_clock_v1_legacy.cpp` **exists but is NOT
  registered and is NOT run** (marked LEGACY in its header).

## 4. Writer initialization and multi-caller serialization proof

- `SimClockWriter::publish()` is guarded by a `std::mutex` (lock acquired for
  the whole odd→payload→even critical section); the 4-thread stress test proves
  concurrent callers never yield an inconsistent snapshot to a reader.
- Construction: `memset` → `sequence = 1` (odd, in-progress) → `magic`/`version`
  → `sim_time = NaN` → `monotonic_ns = 0`, and the odd marker is **kept** until
  the first publish. At every instant (seq 0, seq odd, odd + NaN) the reader
  rejects; a stale valid v2 frame from a previous process cannot be misread.
- Hook storage/load use `__atomic_load_n`/`__atomic_store_n` with
  acquire/release; lifetime documented (install once before consumer threads;
  the installed function must outlive the last `publishStep`). main.cc installs
  a non-capturing lambda over the namespace-scope `g_sim_clock`.

## 5. Mechanical verification

- `unitree_mujoco` + `p1_08_sim_clock_test` build **exit 0** (v2 object
  `p1_08_sim_clock_test.cpp.o`; stale v1 object removed from the build dir).
- C++ test direct **PASS (39)**; `ctest` **1/1 PASS**.
- `python3 scripts/test_p1_08_sim_clock.py` **PASS (23)** (adds the explicit
  construction-state rejection case).
- `python3 scripts/test_p1_08_baseline_identity.py` **PASS (6)**.
- All P1-08 evidence JSONs **VALID**; `git diff --check` **PASS**.

## 6. Old-capture boundary

- The 2026-09-01 capture (`capture_20260901_rerun`) remains **superseded /
  non-acceptance**: it used the v1 contract and unhashed timing; its identity
  (`bdd47a0d…`, and the demonstration `99b995b0…`) is not an accepted baseline.
- **No v2 recapture was performed** in this increment; an accepted baseline
  requires a Director-authorized v2 recapture under the current contract.

## 7. Remaining UNKNOWN

- Accepted baseline identity (requires v2 recapture — Director authorization).
- Runtime observed timing under the v2 contract (unchanged UNKNOWN until
  recapture); controller per-callback direct cadence; Recovery tick (not
  active in the old capture).

Phase 1 remains **NOT ACCEPTED**; P1-10 not started; P1-08 not self-accepted.
