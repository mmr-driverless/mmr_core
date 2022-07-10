#ifndef APP_CORE_LAUNCH_CONTROL_INC_MANUAL_H_
#define APP_CORE_LAUNCH_CONTROL_INC_MANUAL_H_

#include <can.h>

typedef enum MmrManualState {
  MMR_MANUAL_WAITING,
  MMR_MANUAL_CLUTCH_SET_MANUAL,
  MMR_MANUAL_SET_LAUNCH_CONTROL,
  MMR_MANUAL_STOP_LAUNCH,
  MMR_MANUAL_DONE,
} MmrManualState;

void MMR_MANUAL_Init(MmrCan *can, uint32_t *apps, uint32_t *adc);
MmrManualState MMR_MANUAL_Run(MmrManualState state);

#endif // !APP_CORE_LAUNCH_CONTROL_INC_MANUAL_H_
