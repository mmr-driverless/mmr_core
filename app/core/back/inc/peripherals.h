#ifndef APP_CORE_BACK_INC_PERIPHERALS_H_
#define APP_CORE_BACK_INC_PERIPHERALS_H_

#include <can.h>
#include <pin.h>

typedef struct MmrAsPeripherals {
  MmrCan *can;
  MmrPin *gearN;
  MmrPin *gearUp;
  MmrPin *gearDown;
  uint32_t *apps;
  uint32_t *adc;
} MmrAsPeripherals;

extern MmrAsPeripherals asp;

#endif // !APP_CORE_BACK_INC_PERIPHERALS_H_
