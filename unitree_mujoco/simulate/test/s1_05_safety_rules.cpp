#include <cassert>
#include <cmath>

#include "abs_safety_rules.h"

int main() {
  using abs_collision::kContactGround;
  using abs_collision::kContactRobotObstacle;
  using abs_collision::kContactSelf;
  using abs_collision::kContactUnknown;

  // Normal foot-floor and abnormal non-foot-floor both remain ground contacts;
  // the producer separately records the geom-name based foot/non-foot count.
  assert(abs_safety::classifyContact(true, false, false, false, false, true) == kContactGround);
  assert(abs_safety::classifyContact(false, true, false, false, true, false) == kContactGround);
  assert(abs_safety::classifyContact(false, false, true, false, false, true) == abs_collision::kContactOther);
  assert(abs_safety::classifyContact(true, false, false, true, false, false) == kContactRobotObstacle);
  assert(abs_safety::classifyContact(true, true, false, false, false, false) == kContactSelf);
  assert(abs_safety::classifyContact(false, false, false, false, false, false) == kContactUnknown);

  assert(abs_safety::newCollisionEdge(true, false, false));
  assert(abs_safety::newCollisionEdge(true, true, false));
  assert(!abs_safety::newCollisionEdge(true, true, true));  // persistent contact is one episode
  assert(!abs_safety::newCollisionEdge(false, true, true));
  assert(abs_safety::newCollisionEdge(true, true, false));  // new episode after separation

  abs_safety::FallTracker fall;
  const double h = abs_safety::kFallHeightM - 0.01;
  assert(!abs_safety::updateFall(fall, 0.445, 0.0, 0.0, 1, 0.0, true, true)); // standing
  assert(!fall.candidate && !fall.confirmed);
  assert(!abs_safety::updateFall(fall, h, 0.0, 0.0, 2, 0.01, true, true));
  assert(fall.candidate && !fall.confirmed && fall.start_physics_step == 2);
  assert(!abs_safety::updateFall(fall, h, 0.0, 0.0, 3, 0.309, true, true)); // < 0.30 s
  assert(abs_safety::updateFall(fall, h, 0.0, 0.0, 4, 0.31, true, true)); // equality confirms
  assert(fall.confirmed && fall.confirmed_physics_step == 4);
  assert(fall.generation == 1);
  assert(abs_safety::updateFall(fall, h, 0.0, 0.0, 5, 0.32, true, true));
  assert(fall.generation == 1);  // sustained anomaly is still the same event
  assert(!abs_safety::updateFall(fall, 0.445, 0.0, 0.0, 6, 0.33, true, true));
  assert(!fall.confirmed && !fall.active);  // valid recovery rearms the tracker
  assert(!abs_safety::updateFall(fall, h, 0.0, 0.0, 7, 0.34, true, true));
  assert(abs_safety::updateFall(fall, h, 0.0, 0.0, 8, 0.64, true, true));
  assert(fall.generation == 2);  // a later fall is a distinct confirmation

  abs_safety::FallTracker restarted;
  assert(!abs_safety::updateFall(restarted, h, 0.0, 0.0, 10, 1.0, true, true));
  assert(!abs_safety::updateFall(restarted, h, 0.0, 0.0, 20, 2.0, true, false));
  assert(restarted.start_physics_step == 20 && !restarted.confirmed); // gap restarts hold
  assert(!abs_safety::updateFall(restarted, h, 0.0, 0.0, 21, 2.29, true, true));
  assert(!abs_safety::updateFall(restarted, h, 0.0, 0.0, 22, 2.30, false, true)); // invalid posture is unresolved
  assert(restarted.candidate && !restarted.confirmed);
  assert(!abs_safety::updateFall(restarted, std::nan(""), 0.0, 0.0, 23, 2.31, true, true));
  assert(restarted.candidate);
  assert(!abs_safety::updateFall(restarted, h, 0.0, 0.0, 24, 2.32, true, true));
  assert(!restarted.confirmed && restarted.start_physics_step == 24);

  assert(!abs_safety::updateFall(restarted, 0.445, 0.0, 0.0, 24, 2.32, true, true)); // candidate clears
  assert(!restarted.candidate);
  assert(!abs_safety::updateFall(restarted, 0.445, abs_safety::kFallTiltRad, 0.0,
                                25, 2.33, true, true)); // strict threshold
  assert(!restarted.candidate);
  assert(!abs_safety::updateFall(restarted, 0.445, abs_safety::kFallTiltRad + 0.01, 0.0,
                                26, 2.34, true, true));
  assert(restarted.candidate);
}
