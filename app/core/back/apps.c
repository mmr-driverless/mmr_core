#include "inc/apps.h"

uint32_t MMR_APPS_ComputeSpeed(float percentage) {
  return APPS_SLOPE * percentage + APPS_MIN;
}
