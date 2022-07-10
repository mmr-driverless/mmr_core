#ifndef APP_CORE_LAUNCH_CONTROL_INC_MANUAL_H_
#define APP_CORE_LAUNCH_CONTROL_INC_MANUAL_H_

#include <can.h>

typedef enum MmrManualState {
  MMR_MANUAL_LAUNCH_WAITING,
  MMR_MANUAL_LAUNCH_SET_LAUNCH_CONTROL,
  MMR_MANUAL_LAUNCH_STOP_LAUNCH,
  MMR_MANUAL_LAUNCH_DONE,
} MmrManualState;


MmrManualState MMR_MANUAL_LAUNCH_Run(MmrManualState state);

#endif // !APP_CORE_LAUNCH_CONTROL_INC_MANUAL_H_
