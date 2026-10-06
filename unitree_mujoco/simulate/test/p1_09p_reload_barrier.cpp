#include <cassert>

#include "../src/bridge_lifecycle.h"

int main() {
  BridgeLifecycle lifecycle;
  assert(lifecycle.reloadAllowed());
  assert(lifecycle.reserveBridge());
  assert(lifecycle.bridgeActive());
  assert(!lifecycle.reloadAllowed());

  assert(lifecycle.markBridgeActive());
  assert(lifecycle.bridgeActive());
  assert(!lifecycle.reloadAllowed());

  assert(lifecycle.beginStop());
  assert(lifecycle.stopRequested());
  assert(!lifecycle.completeTerminal());
  assert(!lifecycle.beginStop());
  assert(lifecycle.completeTerminal());
  assert(!lifecycle.completeTerminal());
  assert(!lifecycle.bridgeActive());
  assert(lifecycle.reloadAllowed());
  assert(lifecycle.terminal());

  // A stopped bridge is terminal and cannot be restarted.
  assert(!lifecycle.markBridgeActive());
  assert(!lifecycle.bridgeActive());
}
