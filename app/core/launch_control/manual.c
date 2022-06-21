#include "inc/manual.h"
#include "inc/launch_control.h"
#include <delay.h>
#include "message_id.h"

#include <stdint.h>
#include <stdbool.h>

static MmrManualState waiting(MmrManualState state);
static MmrManualState setLaunchControl(MmrManualState state);
static MmrManualState stopLaunchControl(MmrManualState state);
static MmrManualState done(MmrManualState state);

static MmrCan *__can;

void MMR_MANUAL_Init(MmrCan *can) {
  __can = can;
}


MmrManualState MMR_MANUAL_Run(MmrManualState state) {
  switch (state) {
  case MMR_MANUAL_WAITING: return waiting(state);
  case MMR_MANUAL_SET_LAUNCH_CONTROL: return setLaunchControl(state);
  case MMR_MANUAL_STOP_LAUNCH: return stopLaunchControl(state);
  case MMR_MANUAL_DONE: return done(state);
  default: return state;
  }
}


static MmrManualState waiting(MmrManualState state) {
  return MMR_MANUAL_SET_LAUNCH_CONTROL;
}

static MmrManualState setLaunchControl(MmrManualState state) {
  static MmrDelay delay = { .ms = 25 };
  static MmrCanBuffer buffer = { 0x8 };

  MmrCanHeader header = MMR_CAN_HEADER_FromBits(0x628);
  MmrCanMessage setLaunchMsg = MMR_CAN_OutMessage(header);
  MMR_CAN_MESSAGE_SetStandardId(&setLaunchMsg, true);
  MMR_CAN_MESSAGE_SetPayload(&setLaunchMsg, buffer, 8);

  bool rpmOk = MMR_LAUNCH_CONTROL_GetRpm() >= 1000;
  bool isLaunchSet = MMR_LAUNCH_CONTROL_GetLaunchControlState() == MMR_LAUNCH_CONTROL_SET;
  bool isInFirstGear = MMR_LAUNCH_CONTROL_GetGear() == 1;

  if (rpmOk && isInFirstGear && !isLaunchSet && MMR_DELAY_WaitAsync(&delay)) {
    MMR_CAN_Send(__can, &setLaunchMsg);
  }

  if (isLaunchSet) {
    return MMR_MANUAL_STOP_LAUNCH;
  }

  return state;
}


static MmrManualState stopLaunchControl(MmrManualState state) {
  static MmrDelay delay = { .ms = 25 };
  static MmrCanBuffer buffer = { 0x8 };

  MmrCanHeader header = MMR_CAN_HEADER_FromBits(0x628);
  MmrCanMessage unsetLaunchMsg = MMR_CAN_OutMessage(header);
  MMR_CAN_MESSAGE_SetStandardId(&unsetLaunchMsg, true);
  MMR_CAN_MESSAGE_SetPayload(&unsetLaunchMsg, buffer, 8);

  bool rpmOk = MMR_LAUNCH_CONTROL_GetRpm() >= 6000;
  bool launchUnset = MMR_LAUNCH_CONTROL_GetLaunchControlState() == MMR_LAUNCH_CONTROL_NOT_SET;

  if (rpmOk && !launchUnset && MMR_DELAY_WaitAsync(&delay)) {
    MMR_CAN_Send(__can, &unsetLaunchMsg);
  }

  if (launchUnset) {
    return MMR_MANUAL_DONE;
  }

  return state;
}

static MmrManualState done(MmrManualState state) {
  return state;
}
