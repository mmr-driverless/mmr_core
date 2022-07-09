#ifndef APP_CORE_BACK_INC_GLOBAL_STATE_H_
#define APP_CORE_BACK_INC_GLOBAL_STATE_H_

#include <stdint.h>
#include <can.h>

typedef enum MmrClutchState {
  MMR_CLUTCH_UNKNOWN,
  MMR_CLUTCH_PULLED,
  MMR_CLUTCH_RELEASED,
} MmrClutchState;

typedef enum MmrLaunchControlState {
  MMR_LAUNCH_CONTROL_UNKNOWN,
  MMR_LAUNCH_CONTROL_SET,
  MMR_LAUNCH_CONTROL_NOT_SET,
} MmrLaunchControlState;


void MMR_GS_Init();
void MMR_GS_UpdateFromCan(MmrCan *can);

uint16_t MMR_GS_GetRpm();
uint16_t MMR_GS_GetGear();
uint16_t MMR_GS_GetSpeed();
uint8_t MMR_GS_GetLap();
void MMR_GS_SetLap(uint8_t lap);
uint16_t MMR_GS_GetAth();
uint8_t MMR_GS_GetUThrottle();
uint8_t MMR_GS_GetUThrottleB();
int16_t MMR_GS_GetSteeringAngle();
uint32_t MMR_GS_GetInfoSpeed();
uint16_t MMR_GS_GetEbs2();
uint16_t MMR_GS_GetEbs1();
uint16_t MMR_GS_GetBreakP1();
uint16_t MMR_GS_GetBreakP2();

MmrClutchState MMR_GS_GetClutchState();
MmrLaunchControlState MMR_GS_GetLaunchControlState();

#endif // !APP_CORE_BACK_INC_GLOBAL_STATE_H_
