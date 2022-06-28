#include "inc/manual.h"
#include "inc/as.h"
#include <delay.h>
#include "message_id.h"

#include <stdint.h>
#include <stdbool.h>

static MmrManualState waiting(MmrManualState state);
static MmrManualState setLaunchControl(MmrManualState state);
static MmrManualState stopLaunchControl(MmrManualState state);
static MmrManualState done(MmrManualState state);

static MmrCan *__can;
static uint32_t *__adc;
static uint32_t *__apps;


void MMR_MANUAL_Init(MmrCan *can, uint32_t *apps, uint32_t *adc) {
  __can = can;
  __adc = adc;
  __apps = apps;
}


MmrManualState MMR_MANUAL_Run(MmrManualState state) {
  *__apps = *__adc;

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
  static MmrDelay delay = { .ms = 250 };
  static MmrCanBuffer buffer = { 0x8 };

  MmrCanHeader header = MMR_CAN_HEADER_FromBits(0x628);
  MmrCanMessage setLaunchMsg = MMR_CAN_OutMessage(header);
  MMR_CAN_MESSAGE_SetStandardId(&setLaunchMsg, true);
  MMR_CAN_MESSAGE_SetPayload(&setLaunchMsg, buffer, 8);

  bool rpmOk = MMR_AS_GetRpm() >= 1000;
  bool isLaunchSet = MMR_AS_GetLaunchControlState() == MMR_LAUNCH_CONTROL_SET;
  bool isInFirstGear = MMR_AS_GetGear() == 1;

  if (rpmOk && isInFirstGear && !isLaunchSet && MMR_DELAY_WaitAsync(&delay)) {
    MMR_CAN_Send(__can, &setLaunchMsg);
  }

  if (isLaunchSet) {
    return MMR_MANUAL_STOP_LAUNCH;
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

  bool isClutchReleased = MMR_AS_GetClutchState() == MMR_CLUTCH_RELEASED;
  bool launchUnset = MMR_AS_GetLaunchControlState() == MMR_LAUNCH_CONTROL_NOT_SET;

  if (isClutchReleased && !launchUnset && MMR_DELAY_WaitAsync(&delay)) {
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
