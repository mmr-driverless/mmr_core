#include "inc/apps.h"
#include <math.h>

static const uint32_t APPS_MIN = 1150;
static const uint32_t APPS_SLOPE = 700;


bool MMR_APPS_Check(uint32_t apps1, uint32_t apps2) {
  double a = 3.3f * (apps1 / 4096);
  double b = 3.3f * (apps2 / 4096);

  return fabs(a - (2 * b)) <= 0.5f;
}


uint32_t MMR_APPS_ComputeSpeed(float percentage) {
  return APPS_SLOPE * percentage + APPS_MIN;
}
