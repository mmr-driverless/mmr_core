#ifndef APP_LAUNCH_CONTROL_INC_AUTONOMOUS_H_
#define APP_LAUNCH_CONTROL_INC_AUTONOMOUS_H_

#include <can.h>
#include <pin.h>

typedef enum MmrAutonomousState {
  MMR_AUTONOMOUS_WAITING,
  MMR_AUTONOMOUS_PULL_CLUTCH,
  MMR_AUTONOMOUS_SET_LAUNCH_CONTROL,
  MMR_AUTONOMOUS_CHANGE_GEAR,
  MMR_AUTONOMOUS_ACCELERATE,
  MMR_AUTONOMOUS_RELEASE_CLUTCH,
  MMR_AUTONOMOUS_UNSET_LAUNCH,
  MMR_AUTONOMOUS_CLUTCH_SET_MANUAL,
  MMR_AUTONOMOUS_DONE,
} MmrAutonomousState;


void MMR_AUTONOMOUS_Init(MmrCan *can, MmrPin *pin, uint32_t *apps);
MmrAutonomousState MMR_AUTONOMOUS_Run(MmrAutonomousState state);

#endif // !APP_LAUNCH_CONTROL_INC_AUTONOMOUS_H_