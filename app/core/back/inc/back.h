#ifndef APP_CORE_BACK_INC_BACK_H_
#define APP_CORE_BACK_INC_BACK_H_

#include "peripherals.h"
#include <can.h>
#include <pin.h>
#include <stdint.h>

void MMR_BACK_Init(
  MmrCan *can,
  MmrPin *gearUp,
  MmrPin *gearDown,
  MmrPin *gearN,
  MmrPin *changeMode,
  uint32_t *apps,
  uint32_t *adc
);

#endif // !APP_CORE_BACK_INC_BACK_H_
