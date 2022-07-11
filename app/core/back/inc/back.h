#ifndef APP_CORE_BACK_INC_BACK_H_
#define APP_CORE_BACK_INC_BACK_H_

#include "peripherals.h"
#include <can.h>
#include <led.h>
#include <pin.h>
#include <stdint.h>

void MMR_BACK_Init(
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
);

#endif // !APP_CORE_BACK_INC_BACK_H_
