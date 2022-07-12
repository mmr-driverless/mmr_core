#include "inc/apps.h"
#include <delay.h>
#include <math.h>

static const uint32_t APPS_MIN = 1150;
static const uint32_t APPS_SLOPE = 700;

static bool areAppsPlausible(uint32_t apps1, uint32_t apps2);


bool MMR_APPS_Check(uint32_t apps1, uint32_t apps2) {
  static MmrDelay timeout = { .ms = 100 };

  bool arePlausible = areAppsPlausible(apps1, apps2);
  if (arePlausible) {
    MMR_DELAY_Reset(&timeout);
  }

  if (MMR_DELAY_WaitOnceAsync(&timeout)) {
    return false;
  }

  return true;
}


uint32_t MMR_APPS_ComputeSpeed(float percentage) {
  return APPS_SLOPE * percentage + APPS_MIN;
}


static bool areAppsPlausible(uint32_t apps1, uint32_t apps2) {
  double a = 3.3f * (apps1 / 4096);
  double b = 3.3f * (apps2 / 4096);

  return fabs(a - (2 * b)) <= 0.5f;
}
