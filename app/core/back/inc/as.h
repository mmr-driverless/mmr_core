#ifndef APP_BACK_INC_AS_H_
#define APP_BACK_INC_AS_H_

#include <can.h>
#include <pin.h>
#include "timing.h"
#include "delay.h"


typedef enum MmrAsState {
  AS_IDLE,
  AS_OFF,
  AS_READY,
  AS_DRIVING,
  AS_EMERGENCY,
  AS_FINISHED,
} MmrAsState;


MmrAsState MMR_AS_Run(MmrAsState state);

#endif // !APP_BACK_INC_AS_H_
