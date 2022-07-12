#ifndef APP_BACK_INC_AS_H_
#define APP_BACK_INC_AS_H_

#include "global_state.h"
#include <can.h>
#include <pin.h>
#include "timing.h"
#include "delay.h"


typedef enum MmrAsState {
  MMR_AS_OFF,
  MMR_AS_READY,
  MMR_AS_DRIVING,
  MMR_AS_EMERGENCY,
  MMR_AS_FINISHED,
} MmrAsState;


void MMR_AS_Run();

#endif // !APP_BACK_INC_AS_H_
