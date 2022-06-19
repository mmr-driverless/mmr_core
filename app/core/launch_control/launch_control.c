#include "inc/launch_control.h"
#include <stdint.h>
#include <stdbool.h>

struct MmrLaunchControl {
  uint16_t gear;
  uint16_t rpm;
  uint16_t speed;
  MmrClutchState clutch;
  MmrLaunchControlState launchControl;
} __state = {};


static MmrCan *__can;
static MmrPin *__gearDown;


void MMR_LAUNCH_CONTROL_Init(MmrCan *can, MmrPin *gearDown) {
  __can = can;
  __gearDown = gearDown;
  __state = (struct MmrLaunchControl){};
}

void MMR_LAUNCH_CONTROL_Run(MmrLaunchControlMode mode) {
  switch (mode) {

  }
}


MmrClutchState MMR_LAUNCH_CONTROL_GetClutchState() {
  return __state.clutch;
}

MmrLaunchControlState MMR_LAUNCH_CONTROL_GetLaunchControlState() {
  return __state.launchControl;
}

uint16_t MMR_LAUNCH_CONTROL_GetGear() {
  return __state.gear;
}

uint16_t MMR_LAUNCH_CONTROL_GetRpm() {
  return __state.rpm;
}

uint8_t MMR_LAUNCH_CONTROL_GetSpeed() {
  return __state.speed;
}
