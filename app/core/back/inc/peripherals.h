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

  uint32_t *apps;
  uint32_t *adc;

  MmrPin *blueLed;
  MmrPin *yellowLed;
  MmrDelay *__assi_delay;
} MmrAsPeripherals;

extern MmrAsPeripherals asp;

#endif // !APP_CORE_BACK_INC_PERIPHERALS_H_