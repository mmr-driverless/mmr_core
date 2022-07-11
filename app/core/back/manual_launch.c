#include "inc/manual_launch.h"
#include "inc/global_state.h"
#include "inc/peripherals.h"
#include <delay.h>
#include "message_id.h"

#include <stdint.h>
#include <stdbool.h>

static MmrManualState waiting(MmrManualState state);
static MmrManualState setLaunchControl(MmrManualState state);
static MmrManualState stopLaunchControl(MmrManualState state);
static MmrManualState done(MmrManualState state);


MmrManualState MMR_MANUAL_LAUNCH_Run(MmrManualState state) {
  switch (state) {
  case MMR_MANUAL_LAUNCH_WAITING: return waiting(state);
  case MMR_MANUAL_LAUNCH_SET_LAUNCH_CONTROL: return setLaunchControl(state);
  case MMR_MANUAL_LAUNCH_STOP_LAUNCH: return stopLaunchControl(state);
  case MMR_MANUAL_LAUNCH_DONE: return done(state);
  default: return state;
  }
}


static MmrManualState waiting(MmrManualState state) {
  return MMR_MANUAL_LAUNCH_SET_LAUNCH_CONTROL;
}

static MmrManualState setLaunchControl(MmrManualState state) {
  static MmrDelay delay = { .ms = 250 };
  static MmrCanBuffer buffer = { 0x8 };

  MmrCanHeader header = MMR_CAN_HEADER_FromBits(0x628);
  MmrCanMessage setLaunchMsg = MMR_CAN_OutMessage(header);
  MMR_CAN_MESSAGE_SetStandardId(&setLaunchMsg, true);
  MMR_CAN_MESSAGE_SetPayload(&setLaunchMsg, buffer, 8);

  bool rpmOk = gs.rpm >= 1000;
  bool isLaunchSet = gs.launchControl == MMR_LAUNCH_CONTROL_SET;
  bool isInFirstGear = gs.gear == 1;

  if (rpmOk && isInFirstGear && !isLaunchSet && MMR_DELAY_WaitAsync(&delay)) {
    MMR_CAN_Send(asp.can, &setLaunchMsg);
  }

  if (isLaunchSet) {
    return MMR_MANUAL_LAUNCH_STOP_LAUNCH;
  }

  return state;
}


static MmrManualState stopLaunchControl(MmrManualState state) {
  static MmrDelay delay = { .ms = 250 };
  static MmrCanBuffer buffer = { 0x8 };

  MmrCanHeader header = MMR_CAN_HEADER_FromBits(0x628);
  MmrCanMessage unsetLaunchMsg = MMR_CAN_OutMessage(header);
  MMR_CAN_MESSAGE_SetStandardId(&unsetLaunchMsg, true);
  MMR_CAN_MESSAGE_SetPayload(&unsetLaunchMsg, buffer, 8);

  bool isClutchReleased = gs.clutch == MMR_CLUTCH_RELEASED;
  bool launchUnset = gs.launchControl == MMR_LAUNCH_CONTROL_NOT_SET;

  if (isClutchReleased && !launchUnset && MMR_DELAY_WaitAsync(&delay)) {
    MMR_CAN_Send(asp.can, &unsetLaunchMsg);
  }

  if (launchUnset) {
    return MMR_MANUAL_LAUNCH_DONE;
  }

  return state;
}

static MmrManualState done(MmrManualState state) {
  return state;
}
