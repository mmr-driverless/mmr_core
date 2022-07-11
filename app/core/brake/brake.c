#include "brake.h"

bool MMR_BRAKE_IsEngaged(float brakePf, float brakePr) {
  if (brakePf >= 3 || brakePr >= 3)  // TODO: check threshold
    return true;
  return false;
}

bool MMR_BRAKE_Check(float brakePf, float brakePr) {
  if (brakePf <= 2 && brakePr <= 2)  // TODO: check pressure threshold
    return true;
  return false;
}