#ifndef APP_LAUNCH_CONTROL_INC_LAUNCH_CONTROL_H_
#define APP_LAUNCH_CONTROL_INC_LAUNCH_CONTROL_H_

#include <can.h>
#include <pin.h>


typedef struct MmrLaunchControl MmrLaunchControl;


typedef enum MmrLaunchControlMode {
  MMR_LAUNCH_CONTROL_MODE_IDLE,
  MMR_LAUNCH_CONTROL_MODE_AUTONOMOUS,
  MMR_LAUNCH_CONTROL_MODE_MANUAL,
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


void MMR_LAUNCH_CONTROL_Init(MmrCan *can, MmrPin *gearDown, MmrPin *changeMode, uint32_t *apps);
MmrLaunchControlMode MMR_LAUNCH_CONTROL_Run(MmrLaunchControlMode mode);

uint16_t MMR_LAUNCH_CONTROL_GetRpm();
uint16_t MMR_LAUNCH_CONTROL_GetGear();
uint16_t MMR_LAUNCH_CONTROL_Getspeed();
MmrClutchState MMR_LAUNCH_CONTROL_GetClutchState();
MmrLaunchControlState MMR_LAUNCH_CONTROL_GetLaunchControlState();

#endif // !APP_LAUNCH_CONTROL_INC_LAUNCH_CONTROL_H_
