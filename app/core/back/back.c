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

  MmrLed *blueAxisLed,
  MmrLed *yellowAxisLed,

  MmrLed *ctrLed1,
  MmrLed *ctrLed2,
  MmrLed *ctrLed3
) {
  asp = (MmrAsPeripherals) {
    .can = can,
    .gearN = gearN,
    .gearUp = gearUp,
    .gearDown = gearDown,
    .appsOut = appsOut,
    .appsIn = appsIn,

    .blueAxisLed = blueAxisLed,
    .yellowAxisLed = yellowAxisLed,

    .ctrLed1 = ctrLed1,
    .ctrLed2 = ctrLed2,
    .ctrLed3 = ctrLed3,
  };
}
