#include "inc/apps.h"

static const uint32_t APPS_MIN = 650;
static const uint32_t APPS_SLOPE = 716;


uint32_t MMR_APPS_ComputeSpeed(float percentage) {
  return APPS_SLOPE * percentage + APPS_MIN;
}
