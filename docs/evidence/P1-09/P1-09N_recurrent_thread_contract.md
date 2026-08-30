# P1-09N — RecurrentThread Lifecycle / Binary Contract Evidence

Date: 2026-08-28  
Status: **EVIDENCE COMPLETE; P1-09M remains BLOCKED**

## Boundary

This was a read-only source and binary audit. No simulator/controller code or
configuration was modified, and no build, MuJoCo, ROS2, benchmark, or real
robot process was run. A temporary archive extraction was made under `/tmp`
solely to disassemble two archive members; no repository file was changed.

## Resolved dependency

The simulator CMake target uses `find_package(unitree_sdk2 REQUIRED)` and links
the imported `unitree_sdk2` target (`unitree_mujoco/simulate/CMakeLists.txt:14,
28-35`). The existing `build2` metadata resolves it to:

| Item | Evidence |
|---|---|
| SDK include root | `/home/lidio/Libraries/unitree_sdk2/include` (`build2/.../flags.make`) |
| SDK CMake package | `/home/lidio/Libraries/unitree_sdk2/lib/cmake/unitree_sdk2` |
| Static SDK library | `/home/lidio/Libraries/unitree_sdk2/lib/libunitree_sdk2.a` |
| Link mode | Static `unitree_sdk2`; transitive shared `libddsc.so` and `libddscxx.so` (`unitree_sdk2Targets.cmake`) |
| Package version | `2.0.0` (`unitree_sdk2ConfigVersion.cmake`) |
| SDK archive size | 27,666,376 bytes |
| SDK archive SHA256 | `08402aea74150dfbfc3fbfded4ca746916a8d892b54d2bade0cbf392a3be4029` |
| Archive object ABI/build evidence | ELF64 x86-64 relocatable members; `.comment`: GCC Ubuntu 9.4.0-1ubuntu1~20.04.2 |
| Simulator build mode | Release, `/usr/bin/c++`, `-O3 -DNDEBUG` (`build2/CMakeCache.txt`, `flags.make`) |
| compile_commands.json | **UNKNOWN / not present in the inspected simulator build directory** |
| SDK source commit/build ID | **UNKNOWN**; no source implementation or package revision was found in the resolved SDK root |

The two thread headers are present both in the repository vendored SDK tree and
the resolved installation root. The simulator's compile flags point to the
installation root, so the installed headers and archive are the effective
dependency for the captured build.

## Direct use in this project

`unitree_mujoco/simulate/src/unitree_sdk2_bridge.h:546-556` constructs a
`unitree::common::RecurrentThread` with callback `RobotBridge::run` and stores it
as `RecurrentThreadPtr` at lines 657-658. The outer MuJoCo worker is created at
`main.cc:628`, and the `RobotBridge` receives `m` and `d` at line 546. No
`RobotBridge` destructor or explicit `Wait()` call is present in the audited
bridge class.

## Binary findings

The following evidence is from `libunitree_sdk2.a`, members
`recurrent_thread.cpp.o` and `thread.cpp.o`, with symbols and disassembly
retained in the audit notes.

### Native thread ownership

- `Thread::CreateThreadNative()` is a defined symbol in `thread.cpp.o`.
- Its disassembly calls `pthread_create` at offset `0x4cf`.
- Before that call it invokes `pthread_attr_setdetachstate` at offset `0x47f`
  with value `1`. On POSIX this is `PTHREAD_CREATE_DETACHED`.
- `RecurrentThread` constructors call inherited `Thread::Run`, which calls
  `CreateThreadNative` (`recurrent_thread.hpp:16-49`; binary constructor
  relocation and `thread.cpp.o` symbol).

Conclusion: **CONFIRMED** — `RecurrentThread` owns/creates a native pthread,
but the SDK creates it detached, so it is not joinable by the caller.

### Stop/request behavior

- `RecurrentThread` has private `volatile bool mQuit` and no public
  `Stop`/`RequestStop` method (`recurrent_thread.hpp:52-60`).
- `RecurrentThread::Wait(long)` is defined at `recurrent_thread.cpp.o:0x578`.
  Its first operation stores `1` into the `mQuit` field at object offset `0x68`
  (`0x590-0x594`), then calls `FutureWrapper::Wait(long)`.
- `ThreadFunc` tests `mQuit` before invoking the callback and exits its loop
  when true (`recurrent_thread.cpp.o:0x1b4-0x1c4`). `ThreadFunc_0` likewise
  tests `mQuit` before each callback (`0x3aa-0x3c9`).

Conclusion: **PARTIAL** — `Wait()` is a stop-and-wait API in the binary, but it
is not named `Stop`, and this project does not call it from `RobotBridge` or the
outer bridge worker. The exact `FutureImpl` completion signaling is present in
the binary, but it does not turn the detached pthread into a joinable thread.

### Destructor behavior

- `RecurrentThread::~RecurrentThread()` (`recurrent_thread.cpp.o:0x0-0x40`)
  destroys its callback and calls `Thread::~Thread()`; it does not call
  `Wait()` directly.
- `Thread::~Thread()` (`thread.cpp.o:0x76-0x10c`) checks liveness with
  `pthread_kill(thread_id, 0)` at `0xb5`, and if live calls
  `pthread_cancel(thread_id)` at `0xd9`. There is no `pthread_join` call in
  this destructor.
- The detached attribute is established at thread creation, not in the
  destructor.

Conclusion: **CONFIRMED** — destruction attempts cancellation, not join. It
does not itself prove that the callback has stopped before object destruction
or before `m/d` reclamation. Because cancellation is asynchronous at the POSIX
thread level and the thread is detached, a callback touching `mj_data_` may
continue or be interrupted during destruction. Whether this exact callback is
at a cancellation point at the relevant moment is **UNKNOWN**.

### Access after RobotBridge destruction and m/d safety

`RobotBridge::run()` reads and writes `mj_data_` throughout
`unitree_sdk2_bridge.h:560-646`; the callback is bound to `this` at lines
554-556. The current class has no explicit destructor that calls `Wait()`.
The only shared ownership handle is the `shared_ptr` to the SDK thread at
line 657, so destruction of the bridge eventually invokes the SDK destructor
chain above.

| Question | Result |
|---|---|
| Does `RecurrentThread` possess a native thread? | **CONFIRMED** |
| Is there an explicit stop-like operation? | **PARTIAL** — `Wait()` sets `mQuit` and waits; no `Stop`/`RequestStop` exists or is used here |
| Does destructor join? | **CONFIRMED: NO** |
| Is the native thread detached? | **CONFIRMED** |
| Can callback access `mj_data_` after RobotBridge destruction begins? | **UNKNOWN**, with a **CONFIRMED race risk** because destruction cancels without join |
| Can we prove the thread ended before `mjModel/mjData` release? | **CONFIRMED: NO** |

## Consequence for P1-09M

The binary audit resolves the dependency location and proves more behavior than
the header-only audit, but it does **not** satisfy the P1-09M hard constraint.
In fact, it establishes that the current SDK contract is insufficient for safe
`m/d` reclamation by itself:

```text
RequestStop
  → call RecurrentThread::Wait() or an equivalent SDK-supported stop path
  → still no join guarantee because pthread is detached
  → callback completion must be independently acknowledged
  → only then destroy RobotBridge and release m/d
```

Therefore **P1-09M must continue BLOCKED**. The hard constraint “prove
RecurrentThread stops before `mjModel/mjData` release” remains unresolved. The
P1-09L proposed bridge-before-physics ordering is still only **LIKELY** and is
not sufficient without a completion proof.

## Required next evidence before implementation

One of the following must be obtained and independently reviewed:

1. Unitree SDK source/documentation proving that `Wait()` provides a complete
   callback termination barrier despite detached creation; or
2. a version-pinned lifecycle probe against this exact archive proving that
   callback completion is observed before `RecurrentThread` destruction and
   before `m/d` release; or
3. a revised bridge ownership design that does not let the SDK callback retain
   access to `m/d` during teardown, with a mechanical test proving the boundary.

The probe must not use TERM/KILL, detach as a workaround, or a running robot.

P1-09 and Phase 1 remain **NOT ACCEPTED**.
