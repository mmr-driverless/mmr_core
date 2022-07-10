#include "inc/apps.h"

static const uint32_t APPS_MIN = 650;
static const uint32_t APPS_SLOPE = 716;


bool MMR_APPS_Check(uint32_t apps1, uint32_t apps2) {
  double difference = 3.3f * (apps2 / 4096) - 2 * 3.3f * apps1;
  return fabs(difference) <= 0.5f;
}


uint32_t MMR_APPS_ComputeSpeed(float percentage) {
  return APPS_SLOPE * percentage + APPS_MIN;
}
