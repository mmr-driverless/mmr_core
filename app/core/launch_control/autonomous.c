#include "inc/autonomous.h"
#include "inc/launch_control.h"
#include "message_id.h"
#include <delay.h>
#include <pin.h>
#include <can.h>
#include <buffer.h>

#include <stdint.h>
#include <stdbool.h>

static MmrAutonomousState waiting(MmrAutonomousState state);
static MmrAutonomousState pullClutch(MmrAutonomousState state);
static MmrAutonomousState waitBeforeChangingGear(MmrAutonomousState state);

static MmrAutonomousState changeGear(MmrAutonomousState state);
static MmrAutonomousState setLaunchControl(MmrAutonomousState state);
static MmrAutonomousState waitBeforeAccelerating(MmrAutonomousState state);

static MmrAutonomousState accelerate(MmrAutonomousState state);
static MmrAutonomousState releaseClutch(MmrAutonomousState state);
static MmrAutonomousState unsetLaunchControl(MmrAutonomousState state);
static MmrAutonomousState accelerateToMinimum(MmrAutonomousState state);

static MmrAutonomousState clutchSetManual(MmrAutonomousState state);
static MmrAutonomousState done(MmrAutonomousState state);


static MmrCan *__can;
static MmrPin *__gearDown;
static uint32_t *__apps;


void MMR_AUTONOMOUS_Init(MmrCan *can, MmrPin *gearDown, uint32_t *apps) {
  __can = can;
  __gearDown = gearDown;
  __apps = apps;
}


MmrAutonomousState MMR_AUTONOMOUS_Run(MmrAutonomousState state) {
  switch (state) {
  case MMR_AUTONOMOUS_WAITING: return waiting(state);
  case MMR_AUTONOMOUS_PULL_CLUTCH: return pullClutch(state);
  case MMR_AUTONOMOUS_WAIT_BEFORE_CHANGING_GEAR: return waitBeforeChangingGear(state);

  case MMR_AUTONOMOUS_CHANGE_GEAR: return changeGear(state);
  case MMR_AUTONOMOUS_SET_LAUNCH_CONTROL: return setLaunchControl(state);
  case MMR_AUTONOMOUS_WAIT_BEFORE_ACCELERATING: return waitBeforeAccelerating(state);

  case MMR_AUTONOMOUS_ACCELERATE: return accelerate(state);
  case MMR_AUTONOMOUS_RELEASE_CLUTCH: return releaseClutch(state);
  case MMR_AUTONOMOUS_UNSET_LAUNCH: return unsetLaunchControl(state);
  case MMR_AUTONOMOUS_ACCELERATE_TO_MINIMUM: return accelerateToMinimum(state);

  case MMR_AUTONOMOUS_CLUTCH_SET_MANUAL: return clutchSetManual(state);
  case MMR_AUTONOMOUS_DONE: return done(state);
  default: return state;
  }
}


static MmrAutonomousState waiting(MmrAutonomousState state) {
  return MMR_AUTONOMOUS_PULL_CLUTCH;
}


static MmrAutonomousState pullClutch(MmrAutonomousState state) {
  static MmrDelay delay = { .ms = 5 };

  MmrCanHeader header = MMR_CAN_ScsHeader(MMR_CAN_MESSAGE_ID_CS_CLUTCH_PULL);
  MmrCanMessage clutchPullMsg = MMR_CAN_OutMessage(header);

  bool isClutchPulled = MMR_LAUNCH_CONTROL_GetClutchState() == MMR_CLUTCH_PULLED;

  if (MMR_DELAY_WaitAsync(&delay)) {
    MMR_CAN_Send(__can, &clutchPullMsg);
  }

  if (isClutchPulled) {
    return MMR_AUTONOMOUS_WAIT_BEFORE_CHANGING_GEAR;
  }

  return state;
}


static MmrAutonomousState waitBeforeChangingGear(MmrAutonomousState state) {
  static MmrDelay delay = { .ms = 1000 };

  if (MMR_DELAY_WaitAsync(&delay)) {
    return MMR_AUTONOMOUS_CHANGE_GEAR;
  }

  return state;
}


static MmrAutonomousState changeGear(MmrAutonomousState state) {
  static MmrDelay gearNotChangedDelay = { .ms = 2000 };
  static MmrDelay gearChangingDelay = { .ms = 350 };
  static bool changingGear = false;

  bool isGearSet = MMR_LAUNCH_CONTROL_GetGear() == 1;
  if (isGearSet && !changingGear) {
    MMR_PIN_Write(__gearDown, MMR_PIN_LOW);
    return MMR_AUTONOMOUS_SET_LAUNCH_CONTROL;
  }

  if (!changingGear && !isGearSet && MMR_DELAY_WaitAsync(&gearNotChangedDelay)) {
    MMR_DELAY_Reset(&gearChangingDelay);
    changingGear = true;
  }

  if (!MMR_DELAY_WaitAsync(&gearChangingDelay)) {
    MMR_PIN_Write(__gearDown, MMR_PIN_HIGH);
  }
  else {
    MMR_PIN_Write(__gearDown, MMR_PIN_LOW);
    MMR_DELAY_Reset(&gearNotChangedDelay);
    changingGear = false;
  }

  return state;
}



static MmrAutonomousState setLaunchControl(MmrAutonomousState state) {
  static MmrDelay delay = { .ms = 250 };
  static MmrCanBuffer buffer = { 0x8 };

  MmrCanMessage setLaunchMsg = {};
  MMR_CAN_MESSAGE_SetId(&setLaunchMsg, 0x628);
  MMR_CAN_MESSAGE_SetStandardId(&setLaunchMsg, true);
  MMR_CAN_MESSAGE_SetPayload(&setLaunchMsg, buffer, 8);

  bool rpmOk = MMR_LAUNCH_CONTROL_GetRpm() >= 1000;
  bool isLaunchSet = MMR_LAUNCH_CONTROL_GetLaunchControlState() == MMR_LAUNCH_CONTROL_SET;
  bool isInFirstGear = MMR_LAUNCH_CONTROL_GetGear() == 1;

  if (rpmOk && isInFirstGear && !isLaunchSet && MMR_DELAY_WaitAsync(&delay)) {
    MMR_CAN_Send(__can, &setLaunchMsg);
  }

  if (isLaunchSet) {
    return MMR_AUTONOMOUS_WAIT_BEFORE_ACCELERATING;
  }

  return state;
}


static MmrAutonomousState waitBeforeAccelerating(MmrAutonomousState state) {
  static MmrDelay delay = { .ms = 1000 };

  if (MMR_DELAY_WaitAsync(&delay)) {
    return MMR_AUTONOMOUS_ACCELERATE;
  }

  return state;
}


static MmrAutonomousState accelerate(MmrAutonomousState state) {
  static const uint16_t DAC_30 = 865;
  static MmrDelay delay = { .ms = 1000 };

  *__apps = DAC_30;
  if (MMR_DELAY_WaitAsync(&delay)) {
    return MMR_AUTONOMOUS_RELEASE_CLUTCH;
  }

  return state;
}

static MmrAutonomousState releaseClutch(MmrAutonomousState state) {
  static MmrDelay delay = { .ms = 1 };
  
  MmrCanHeader header = MMR_CAN_ScsHeader(MMR_CAN_MESSAGE_ID_CS_CLUTCH_RELEASE);
  MmrCanMessage clutchReleaseMsg = MMR_CAN_OutMessage(header);

  if (MMR_DELAY_WaitAsync(&delay)) {
    MMR_CAN_Send(__can, &clutchReleaseMsg);
  }

  bool isClutchReleased = MMR_LAUNCH_CONTROL_GetClutchState() == MMR_CLUTCH_RELEASED;
  bool rpmOk = MMR_LAUNCH_CONTROL_GetRpm() >= 6000;

  if (isClutchReleased && rpmOk) {
    return MMR_AUTONOMOUS_UNSET_LAUNCH;
  }

  return state;
}


static MmrAutonomousState unsetLaunchControl(MmrAutonomousState state){
  static MmrDelay delay = { .ms = 250 };
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
    return MMR_AUTONOMOUS_ACCELERATE_TO_MINIMUM;
  }

  return state;
}


static MmrAutonomousState accelerateToMinimum(MmrAutonomousState state) {
  static const uint16_t DAC_0 = 500;
  static MmrDelay delay = { .ms = 500 };

  if (MMR_DELAY_WaitAsync(&delay)) {
    *__apps = DAC_0;
    return MMR_AUTONOMOUS_CLUTCH_SET_MANUAL;
  }

  return state;
}



static MmrAutonomousState clutchSetManual(MmrAutonomousState state) {
  static MmrDelay delay = { .ms = 10 };
  static uint8_t clutchMessages = 0;

  MmrCanHeader header = MMR_CAN_ScsHeader(MMR_CAN_MESSAGE_ID_CS_CLUTCH_SET_MANUAL);
  MmrCanMessage clutchManualMsg = MMR_CAN_OutMessage(header);

  if (clutchMessages <= 10 && MMR_DELAY_WaitAsync(&delay)) {
    MMR_CAN_Send(__can, &clutchManualMsg);
  }

  if (clutchMessages > 10) {
    return MMR_AUTONOMOUS_DONE;
  }

  return state;
}

static MmrAutonomousState done(MmrAutonomousState state){
  return MMR_AUTONOMOUS_DONE;
}
