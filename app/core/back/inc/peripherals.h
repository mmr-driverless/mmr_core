#ifndef APP_CORE_BACK_INC_PERIPHERALS_H_
#define APP_CORE_BACK_INC_PERIPHERALS_H_

#include <can.h>
#include <pin.h>
#include <delay.h>
#include <stdint.h>

typedef struct MmrAsPeripherals {
  MmrCan *can;

  MmrPin *gearN;
  MmrPin *gearUp;
  MmrPin *gearDown;

  uint32_t *appsOut;
  uint32_t *appsIn;

  MmrPin *blueLed;
  MmrPin *yellowLed;

  MmrPin *ctrLed1;
  MmrPin *ctrLed2;
  MmrPin *ctrLed3;
} MmrAsPeripherals;

extern MmrAsPeripherals asp;

#endif // !APP_CORE_BACK_INC_PERIPHERALS_H_
