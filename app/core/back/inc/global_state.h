#ifndef APP_CORE_BACK_INC_GLOBAL_STATE_H_
#define APP_CORE_BACK_INC_GLOBAL_STATE_H_

#include "mission.h"
#include "as.h"
#include "manual_launch.h"
#include "autonomous_launch.h"
#include <button.h>
#include <can.h>
#include <ebs.h>
#include <stdint.h>

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
  MmrAutonomousState as;
  MmrManualState ms;
  MmrAsState stateAs;

  float infoSpeed;
  int16_t steeringAngle;
  uint8_t lap;
  uint16_t ath;//<-- primo valore farfalla
  uint16_t ath2;//<-- secondo valore farfalla
  float infoAth;
  uint16_t gear;
  uint16_t rpm;
  float speed;
  uint8_t uThrottleA;
  uint8_t uThrottleB;
  uint16_t ebs1Pressure;
  uint16_t ebs2Pressure;
  float brakePressureFront;
  float brakePressureRear;

  bool goSignal;
  bool missionFinished;
  bool missionReady;
  bool asbCheck;
  bool asbEngaged;
  bool readyToDrive;
  bool vehicleStandstill;
  MmrMission currentMission;
  MmrButtonState resEmergencyButton;
  
  MmrClutchState clutch;
  MmrLaunchControlState launchControl;
} MmrGlobalState;

extern MmrGlobalState gs;

void MMR_GS_Init();
void MMR_GS_UpdateFromCan(MmrCan *can);
void MMR_GS_SendToCan(MmrCan* can);

#endif // !APP_CORE_BACK_INC_GLOBAL_STATE_H_
