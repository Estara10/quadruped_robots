// Offline P1-10 common-start state-machine contract test.  It does not load a
// model, call mj_step, start a UI, or start ROS/runtime.
#include <cstdio>

#include "abs_stage_a_common_start_contract.h"

static int g_checks = 0;
static bool g_failed = false;

#define CHECK(condition)                                                   \
  do {                                                                     \
    ++g_checks;                                                            \
    if (!(condition)) {                                                    \
      g_failed = true;                                                     \
      std::printf("FAIL %s:%d %s\n", __FILE__, __LINE__, #condition);     \
    }                                                                      \
  } while (0)

int main() {
  using abs_stage_a_common_start::StateMachine;
  StateMachine gate("p1-10-capture-0123456789abcdef0123456789abcdef");
  CHECK(!gate.stepAllowed());
  CHECK(gate.physics_steps() == 0);
  CHECK(!gate.notePhysicsStep(1));
  CHECK(gate.noteRlEnter(9, 0, 0));
  CHECK(!gate.release("p1-10-capture-ffffffffffffffffffffffffffffffff"));
  CHECK(!gate.stepAllowed());
  CHECK(gate.release("p1-10-capture-0123456789abcdef0123456789abcdef"));
  CHECK(gate.stepAllowed());
  CHECK(!gate.release("p1-10-capture-0123456789abcdef0123456789abcdef"));
  CHECK(gate.notePhysicsStep(1));
  CHECK(gate.physics_steps() == 1);
  CHECK(!gate.notePhysicsStep(2));
  CHECK(gate.noteFirstFrame(2, 0));
  CHECK(!gate.noteFirstFrame(4, 1));
  if (g_failed) return 1;
  std::printf("P1-10 common-start state-machine contract PASS (%d checks)\n", g_checks);
  return 0;
}
