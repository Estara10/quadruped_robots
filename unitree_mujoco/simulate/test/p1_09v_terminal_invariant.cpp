#include <cassert>

#include "../src/bridge_lifecycle.h"

int main() {
  using State = BridgeLifecycle::State;

  BridgeLifecycle initial;
  assert(!initial.completeTerminal());
  assert(!initial.beginStop());
  assert(initial.state() == State::INITIAL);
  assert(initial.reloadAllowed());

  BridgeLifecycle reserved;
  assert(reserved.reserveBridge());
  assert(!reserved.completeTerminal());
  assert(reserved.state() == State::RESERVED);
  assert(!reserved.reloadAllowed());

  BridgeLifecycle active;
  assert(active.reserveBridge());
  assert(active.markBridgeActive());
  assert(!active.completeTerminal());
  assert(active.state() == State::ACTIVE);
  assert(!active.reloadAllowed());
  assert(active.beginStop());
  assert(active.state() == State::STOPPING);
  assert(!active.markBridgeActive());
  assert(!active.reserveBridge());
  assert(active.completeTerminal());
  assert(!active.completeTerminal());
  assert(active.state() == State::TERMINAL);
  assert(active.reloadAllowed());
  assert(!active.markBridgeActive());
  assert(!active.reserveBridge());
}
