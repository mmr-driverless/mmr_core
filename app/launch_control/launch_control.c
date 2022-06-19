#include "inc/launch_control.h"
#include <stdint.h>
#include <stdbool.h>

struct MmrLaunchControl {
  uint16_t gear;
  uint8_t nmot;
  uuint8_t speed;
  MmrClutchState clutch;
  MmrLaunchControlState launchControl;
} __state = {};


static MmrCan *__can;


void MMR_LAUNCH_CONTROL_Init(MmrCan *can) {
  __can = can;
  __state = {};
}

void MMR_LAUNCH_CONTROL_Run(MmrLaunchControlMode mode) {
  switch (mode) {

  }
}


void MMR_LAUNCH_CONTROL_SetClutchState(MmrClutchState state) {
  __state.clutch = state;
}

MmrClutchState MMR_LAUNCH_CONTROL_GetClutchState() {
  return __state.clutch;
}

uint8_t MMR_LAUNCH_CONTROL_GetNmot() {
  return __state.nmot;
}

uint8_t MMR_LAUNCH_CONTROL_Getspeed() {
  return __state.speed;
}
