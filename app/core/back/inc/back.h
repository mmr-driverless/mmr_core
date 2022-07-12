#ifndef APP_CORE_BACK_INC_BACK_H_
#define APP_CORE_BACK_INC_BACK_H_

#include "peripherals.h"
#include <can.h>
#include <led.h>
#include <pin.h>
#include <button.h>
#include <stdint.h>

void MMR_BACK_Init(
  MmrCan *can,

  MmrPin *gearUp,
  MmrPin *gearDown,
  MmrPin *gearN,

  MmrButton *blueButton,

  uint32_t *appsOut,
  uint32_t *appsIn,

  MmrPin *ebs1,
  MmrPin *ebs2,

  MmrLed *blueAxisLed,
  MmrLed *yellowAxisLed,

  MmrLed *ctrLed1,
  MmrLed *ctrLed2,
  MmrLed *ctrLed3,

  WatchdogStart watchdogStart,
  WatchdogStop watchdogStop
);

#endif // !APP_CORE_BACK_INC_BACK_H_
