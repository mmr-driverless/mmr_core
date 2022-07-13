#include "inc/tps.h"
#include <math.h>
#include <stdbool.h>


bool MMR_TPS_Check(uint32_t ath1, uint32_t ath2) {
  return fabs(ath1 + ath1 - 5) <= 0.5;
}
