


#include "can.h"

typedef enum MmrManualState {
  MMR_MANUAL_WAITING,
  MMR_MANUAL_SET_LAUNCH_CONTROL,
  MMR_MANUAL_STOP_LAUNCH,
  MMR_MANUAL_DONE,
} MmrAutonomousState;
