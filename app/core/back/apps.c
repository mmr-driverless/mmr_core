#include "inc/peripherals.h"
#include "inc/apps.h"
#include "inc/tps.h"
#include <delay.h>
#include <math.h>

static const uint32_t APPS_MIN = 1150;
static const uint32_t APPS_SLOPE = 700;

static bool areAppsPlausible();


void MMR_APPS_TryWrite(uint32_t value) {
  bool canWrite = MMR_APPS_Check() && MMR_TPS_Check();
  *(asp.appsOut) =  canWrite ? value : 0;
}


bool MMR_APPS_Check() {
  static MmrDelay timeout = { .ms = 90 };

  bool arePlausible = areAppsPlausible();
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


static bool areAppsPlausible() {
  double a = 3.3f * (asp.appsIn[0] / 4096.f);
  double b = 3.3f * (asp.appsIn[1] / 4096.f);

  return
    fabs(a - (2 * b)) <= 0.5f &&
    asp.appsIn[0] > 100 &&
    asp.appsIn[1] > 100;
}
