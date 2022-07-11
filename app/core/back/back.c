#include "inc/back.h"

MmrAsPeripherals asp;

void MMR_AS_Init(
  MmrCan *can,
  MmrPin *gearUp,
  MmrPin *gearDown,
  MmrPin *gearN,
  MmrPin *changeMode,
  uint32_t *appsOut,
  uint32_t *appsIn,
  MmrPin *ctrLed1,
  MmrPin *ctrLed2,
  MmrPin *ctrLed3
) {
  asp = (MmrAsPeripherals) {
    .can = can,
    .gearN = gearN,
    .gearUp = gearUp,
    .gearDown = gearDown,
    .appsOut = appsOut,
    .appsIn = appsIn,
    .ctrLed1 = ctrLed1,
    .ctrLed2 = ctrLed2,
    .ctrLed3 = ctrLed3,
  };
}
