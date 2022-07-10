#ifndef APP_CORE_BACK_INC_PERIPHERALS_H_
#define APP_CORE_BACK_INC_PERIPHERALS_H_

#include <can.h>
#include <pin.h>

typedef struct MmrAsPeripherals {
  const MmrCan *can;
  const MmrPin *gearN;
  const MmrPin *gearUp;
  const MmrPin *gearDown;
  const uint32_t *apps;
  const uint32_t *adc;
} MmrAsPeripherals;

extern MmrAsPeripherals asPeripherals;

#endif // !APP_CORE_BACK_INC_PERIPHERALS_H_
