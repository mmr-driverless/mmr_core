#ifndef APP_LAUNCH_CONTROL_INC_LAUNCH_CONTROL_H_
#define APP_LAUNCH_CONTROL_INC_LAUNCH_CONTROL_H_

#include "can.h"


typedef struct MmrLaunchControl MmrLaunchControl;


typedef enum MmrLaunchControlMode {
  MMR_LAUNCH_CONTROL_MODE_AUTONOMOUS,
  MMR_LAUNCH_CONTROL_MODE_MANUAL,
} MmrLaunchControlMode;

typedef enum MmrClutchState {
  MMR_CLUTCH_UNKNOWN,
  MMR_CLUTCH_PULLED,
  MMR_CLUTCH_RELEASED,
} MmrClutchState;


void MMR_LAUNCH_CONTROL_Init(MmrCan *can);
void MMR_LAUNCH_CONTROL_Run(MmrLaunchControlMode mode);

void MMR_LAUNCH_CONTROL_SetClutchState(MmrClutchState state);

uint8_t MMR_LAUNCH_CONTROL_GetNmot();
MmrClutchState MMR_LAUNCH_CONTROL_GetClutchState();

#endif // !APP_LAUNCH_CONTROL_INC_LAUNCH_CONTROL_H_
