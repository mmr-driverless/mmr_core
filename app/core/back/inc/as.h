#ifndef APP_BACK_INC_AS_H_
#define APP_BACK_INC_AS_H_

#include <can.h>
#include <pin.h>


typedef struct MmrLaunchControl MmrLaunchControl;


typedef enum MmrLaunchControlMode {
//  MMR_AS_MODE_IDLE = 0,
  MMR_AS_MODE_MANUAL = 0,
  MMR_AS_MODE_GEAR_CHANGE = 1,
  MMR_AS_MODE_AUTONOMOUS = 2,
} MmrLaunchControlMode;

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


typedef enum MmrMission {
  MMR_MISSION_IDLE,
  MMR_MISSION_ACCELERATION,
  MMR_MISSION_SKIDPAD,
  MMR_MISSION_AUTOCROSS,
  MMR_MISSION_TRACKDRIVE,
  MMR_MISSION_EBS_TEST,
  MMR_MISSION_INSPECTION,
  MMR_MISSION_MANUAL,
} MmrMission;


void MMR_AS_Init(MmrCan *can, MmrPin *gearUp, MmrPin *gearDown, MmrPin *gearN, MmrPin *changeMode, uint32_t *apps, uint32_t *adc);
MmrLaunchControlMode MMR_AS_Run(MmrLaunchControlMode mode);

uint16_t MMR_AS_GetRpm();
uint16_t MMR_AS_GetGear();
uint16_t MMR_AS_GetSpeed();
uint8_t MMR_AS_GetLap();
void MMR_AS_SetLap(uint8_t lap);
uint16_t MMR_AS_GetAth();
int16_t MMR_AS_GetSteeringAngle();
MmrClutchState MMR_AS_GetClutchState();
MmrLaunchControlState MMR_AS_GetLaunchControlState();

#endif // !APP_BACK_INC_AS_H_
