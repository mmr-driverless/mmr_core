#include "inc/tps.h"
#include "inc/global_state.h"
#include <math.h>
#include <stdbool.h>
#include <delay.h>


static bool areTpsPlausible();

bool MMR_TPS_Check() {
  static MmrDelay timeout = { .ms = 90 };

  bool arePlausible = areTpsPlausible();
  if (arePlausible) {
    MMR_DELAY_Reset(&timeout);
  }

  if (MMR_DELAY_WaitOnceAsync(&timeout)) {
    return false;
  }

  return true;
}


static bool areTpsPlausible() {
  float difference = gs.uThrottleA + gs.uThrottleB - 5;
  return fabs(difference) <= 0.5;
}
