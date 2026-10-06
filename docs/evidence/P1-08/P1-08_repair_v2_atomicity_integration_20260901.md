# P1-08 — Sim-Clock Initialization Atomicity and C++→Python Integration Closure (2026-09-01)

Status: **REPAIRED / AWAITING INDEPENDENT REVIEW**.
Phase 1 remains **NOT ACCEPTED**.

Scope: closes the two latest Independent-Review blockers — (1) `SimClockWriter`
construction did not atomically publish the odd/in-progress sequence before any
payload cleanup, and (2) the Python tests only fabricated bytes with
`struct.pack` instead of exercising a real C++ writer → Python reader. **No
MuJoCo/ROS2 runtime recapture was run**; no P1-10; no timestep/control/policy/
RA/Recovery/solver/scene/model/gains change; no baseline-identity change; no
upgrade of the old v1 capture; no unrelated sim-clock refactor.

## 1. Exact initialization-order correction

`common/abs_sim_clock_contract.h` `SimClockWriter` constructor is now strictly
ordered and fully atomic:

1. map the shared memory;
2. **FIRST**: `__atomic_store_n(&ptr_->sequence, 1, RELEASE)` — the odd
   (in-progress) marker is published before any payload/header write;
3. **THEN**: atomic release-stores for `magic`, `version`, `sim_time` (NaN) and
   `monotonic_ns` (0);
4. the odd marker is kept until the first `publish()`.

There is **no `memset` before the odd marker** (the previous `memset` was
removed). At every instant (before the marker, at the marker, after payload
writes) a reader sees either seq 0 (rejected), seq odd (rejected), or odd + NaN
(rejected), so a reader racing construction — or a stale valid v2 frame left by
a previous process — is always rejected until the first complete publish.

## 2. ABI / atomic reader-writer safety rationale

- **All** shared-field accesses are atomic: the writer (init + publish) and the
  reader (`readSnapshot`) use `__atomic_load_n`/`__atomic_store_n` with
  acquire/release. There are **no plain non-atomic reads or writes** of the
  shared struct, so a concurrent reader can never form a data race / UB with
  the writer, including during construction.
- The `double sim_time` field is accessed atomically through its **IEEE-754 bit
  pattern via a `uint64_t` view** (`storeSimTime`/`loadSimTime`, `bit_cast` via
  memcpy) because GCC's `__atomic` builtins do not accept floating-point
  operands. Byte layout is unchanged: 8 bytes, little-endian on x86-64, offset
  32, 8-byte aligned → lock-free atomic. The Python reader's `"<4Qd"` unpack
  reads the same 8 bytes as a double, and the finite check is applied to the
  decoded double (`math.isfinite`) — byte layout, finite checks and Python
  decode are consistent.
- The struct ABI/layout is unchanged (40 bytes: 4×uint64 + 1×double, offsets
  0/8/16/24/32), and the default shared-memory name `/mujoco_sim_clock` and
  normal behavior are unchanged. `SimClockWriter` gained an optional
  `shm_name` parameter (default = production name) for the test bridge only.
- `publish()` remains internally serialized by a `std::mutex` over the whole
  odd → payload → even critical section (does not rely on external `sim.mtx`).

Bugs found and fixed while closing this increment:

- `readSnapshot` copied a local `SimClock snap;` whose `sequence` was
  **uninitialized** into `*out` (magic/version/mono/sim were set, sequence was
  not). Fixed: `snap.sequence = seq_before` (the validated stable even value).
- The Python test passed the shm_open NAME to `read_sim_clock`, which opens the
  `/dev/shm/`-prefixed path. Fixed by passing `/dev/shm/<name>` to Python.

## 3. C++→Python real shared-memory integration evidence

New minimal, repeatable, MuJoCo-free bridge `test/p1_08_sim_clock_bridge.cpp`
(built by CMake as `p1_08_sim_clock_bridge`; no MuJoCo, no ROS):

- Creates a **real** `SimClockWriter` on a **unique temporary** shm name and
  publishes known `{monotonic_ns, sim_time}` (e.g. `{3000, 0.006}`), printing
  the shm name and the last published pair.
- Python `read_sim_clock(shm_path="/dev/shm/<name>")` opens that shm and reads
  back the same valid v2 snapshot — the **forward C++-writer → Python-reader
  integration is real**, not fabricated with `struct.pack`.
- Negative fail-closed cases (odd / wrong+old version / NaN / ±Inf / zero
  monotonic / construction state) are applied to the **real C++-created shm**
  and must be rejected.
- The `struct.pack` synthetic-byte tests remain but are explicitly labelled
  **decode/rejection-only** (they do not prove the C++ output format).
- The temporary shm is unique (`mujoco_sim_clock_itest_<pid>`) and unlinked
  after the test; the production `/mujoco_sim_clock` is never created by the
  test and is verified absent afterwards.

## 4. CTest and Release-safe verification

- CMake: `include(CTest)`; `p1_08_sim_clock_test` builds from
  `test/p1_08_sim_clock_test.cpp` (v2) and is registered via `add_test`;
  `p1_08_sim_clock_bridge` is built by CMake and invoked by the Python test.
- `ctest --test-dir unitree_mujoco/simulate/build2 --output-on-failure`:
  **1/1 Test #1: p1_08_sim_clock_test … Passed**.
- Direct run: **RESULT: PASS (45 checks), exit 0**. Release build uses
  `-O3 -DNDEBUG`; the test uses a custom `CHECK` macro (not `assert`), so any
  failed check exits non-zero and CTest catches it.
- Old v1 test `test/p1_08_sim_clock_v1_legacy.cpp`: **exists, NOT registered,
  NOT run** (legacy marker only).

## 5. C++ v2 test coverage (45 checks)

- construction/init state rejected;
- **stale valid v2 snapshot pre-seeded → a new writer's construction invalidates
  it (reader rejects before first publish), then first publish is accepted**;
- stable even snapshots accepted (sequence/values verified);
- odd sequence rejected; sequence changed during copy rejected; wrong magic /
  wrong + old version rejected; NaN / +Inf / -Inf sim_time rejected; zero
  monotonic rejected;
- hook dispatch round-trip (non-capturing static callback → `publishStep` →
  writer → reader);
- no-hook `publishStep` is a no-op;
- multi-threaded multi-caller publish stress (4 writers × 20000, reader) — **0
  inconsistent (torn) payload pairs accepted**.

## 6. model-probe exclusion / runtime mj_step coverage boundary

- `p1_08_model_probe.cpp` is an **offline static probe** (loads the scene, prints
  `mjModel` facts, steps the model a few times) and is **NOT part of the main
  simulator runtime clock contract**; it does **not** promise that its `mj_step`
  publishes the sim clock. Its stepping is a probe-side arithmetic check only.
- The main simulator's **three runtime `mj_step` paths are covered**: PhysicsLoop
  out-of-sync + in-sync (`g_sim_clock.publish` directly) and the UI step-forward
  in `simulate.cc` (`publishStep` → global hook installed by `main()`).
- `publishStep` with no hook installed is a **no-op** — it is not described as a
  library-global every-step guarantee.

## 7. Mechanical verification

- Simulator build (`unitree_mujoco` + `p1_08_sim_clock_test` +
  `p1_08_sim_clock_bridge`) **exit 0**.
- C++ test direct **PASS (45)**; `ctest` **1/1 PASS**.
- `python3 scripts/test_p1_08_sim_clock.py` **PASS (32)** — includes the real
  C++→Python forward integration and real-shm fail-closed negatives.
- `python3 scripts/test_p1_08_baseline_identity.py` **PASS (6)**.
- All P1-08 evidence JSONs **VALID**; all P1-08 scripts `py_compile` OK;
  `git diff --check` **PASS**; production `/mujoco_sim_clock` verified absent.

## 8. Old-capture boundary

- The 2026-09-01 capture (`capture_20260901_rerun`) remains **superseded /
  non-acceptance** (v1 contract + unhashed timing; identities `bdd47a0d…` and
  the demonstration `99b995b0…` are not accepted baselines).
- **No v2 recapture was performed**; an accepted baseline requires a
  Director-authorized v2 recapture under the current contract.

## 9. Remaining UNKNOWN

- Accepted baseline identity (requires v2 recapture — Director authorization);
  runtime observed timing under v2; controller per-callback direct cadence;
  Recovery tick (not active in the old capture).

Phase 1 remains **NOT ACCEPTED**; P1-10 not started; P1-08 not self-accepted.
