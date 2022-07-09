#ifndef APP_CORE_BACK_INC_GLOBAL_STATE_H_
#define APP_CORE_BACK_INC_GLOBAL_STATE_H_

#include <stdint.h>
#include <can.h>
#include "as.h"

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


typedef struct MmrGlobalState {
  float infoSpeed;
  int16_t steeringAngle;
  uint8_t lap;
  uint16_t ath;//<-- primo valore farfalla
  uint16_t ath2;//<-- secondo valore farfalla
  uint16_t gear;
  uint16_t rpm;
  float speed;
  uint8_t uThrottle;
  uint8_t uThrottleB;
  uint16_t ebs1;
  uint16_t ebs2;
  float brakePf;
  float brakePr;

  MmrClutchState clutch;
  MmrLaunchControlState launchControl;
} MmrGlobalState;


extern MmrGlobalState gs;


void MMR_GS_Init();
void MMR_GS_UpdateFromCan(MmrCan *can);
void MMR_GS_SendToCan(MmrCan* can);


uint8_t MMR_AS_MissionReady();
uint8_t MMR_AS_MissionFinished();
uint8_t MMR_AS_Go_Signal();
uint8_t MMR_AS_ASB_Check();
MmrMission MMR_AS_GetMission();

#endif // !APP_CORE_BACK_INC_GLOBAL_STATE_H_
