#include "inc/autonomous.h"
#include "inc/as.h"
#include "inc/apps.h"
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
static MmrAutonomousState accelerateTo15(MmrAutonomousState state);
static MmrAutonomousState accelerateToMinimum(MmrAutonomousState state);

static MmrAutonomousState clutchSetManual(MmrAutonomousState state);
static MmrAutonomousState setManualApps(MmrAutonomousState state);
static MmrAutonomousState done(MmrAutonomousState state);


static MmrCan *__can;
static MmrPin *__gearN;
static uint32_t *__apps;
static uint32_t *__adc;


void MMR_AUTONOMOUS_Init(MmrCan *can, MmrPin *gearN, uint32_t *apps, uint32_t *adc) {
  __can = can;
  __gearN = gearN;
  __apps = apps;
  __adc = adc;
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
  case MMR_AUTONOMOUS_ACCELERATE_TO_15: return accelerateTo15(state);
  case MMR_AUTONOMOUS_ACCELERATE_TO_MINIMUM: return accelerateToMinimum(state);

  case MMR_AUTONOMOUS_CLUTCH_SET_MANUAL: return clutchSetManual(state);
  case MMR_AUTONOMOUS_SET_MANUAL_APPS: return setManualApps(state);
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

  if (MMR_DELAY_WaitAsync(&delay)) {
    MMR_CAN_Send(__can, &clutchPullMsg);
  }

  bool isClutchPulled = MMR_AS_GetClutchState() == MMR_CLUTCH_PULLED;
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

  bool isGearSet = MMR_AS_GetGear() == 1;
  if (isGearSet && !changingGear) {
    MMR_PIN_Write(__gearN, MMR_PIN_LOW);
    return MMR_AUTONOMOUS_SET_LAUNCH_CONTROL;
  }

  if (!changingGear && !isGearSet && MMR_DELAY_WaitAsync(&gearNotChangedDelay)) {
    MMR_DELAY_Reset(&gearChangingDelay);
    changingGear = true;
  }

  if (!MMR_DELAY_WaitAsync(&gearChangingDelay)) {
    MMR_PIN_Write(__gearN, MMR_PIN_HIGH);
  }
  else {
    MMR_PIN_Write(__gearN, MMR_PIN_LOW);
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

  bool rpmOk = MMR_AS_GetRpm() >= 1000;
  bool isLaunchSet = MMR_AS_GetLaunchControlState() == MMR_LAUNCH_CONTROL_SET;
  bool isInFirstGear = MMR_AS_GetGear() == 1;

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
  static MmrDelay delay = { .ms = 1000 };

  *__apps = MMR_APPS_ComputeSpeed(0.3);
  if (MMR_DELAY_WaitAsync(&delay)) {
    return MMR_AUTONOMOUS_RELEASE_CLUTCH;
  }

  return state;
}

static MmrAutonomousState releaseClutch(MmrAutonomousState state) {
  static MmrDelay delay = { .ms = 5 };
  
  MmrCanHeader header = MMR_CAN_ScsHeader(MMR_CAN_MESSAGE_ID_CS_CLUTCH_RELEASE);
  MmrCanMessage clutchReleaseMsg = MMR_CAN_OutMessage(header);

  if (MMR_DELAY_WaitAsync(&delay)) {
    MMR_CAN_Send(__can, &clutchReleaseMsg);
  }

  bool isClutchReleased = MMR_AS_GetClutchState() == MMR_CLUTCH_RELEASED;
  if (isClutchReleased) {
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

  bool clutchReleased = MMR_AS_GetClutchState() == MMR_CLUTCH_RELEASED;
  bool launchUnset = MMR_AS_GetLaunchControlState() == MMR_LAUNCH_CONTROL_NOT_SET;

  if (clutchReleased && !launchUnset && MMR_DELAY_WaitAsync(&delay)) {
    MMR_CAN_Send(__can, &unsetLaunchMsg);
  }

  if (launchUnset) {
    return MMR_AUTONOMOUS_ACCELERATE_TO_15;
  }

  return state;
}


static MmrAutonomousState accelerateTo15(MmrAutonomousState state) {
  static const uint16_t DAC_15 = 700;
  static MmrDelay delay = { .ms = 500 };

  if (MMR_DELAY_WaitAsync(&delay)) {
    *__apps = DAC_15;
    return MMR_AUTONOMOUS_ACCELERATE_TO_MINIMUM;
  }

  return state;
}


static MmrAutonomousState accelerateToMinimum(MmrAutonomousState state) {
  static MmrDelay delay = { .ms = 1000 };

  if (MMR_DELAY_WaitAsync(&delay)) {
    *__apps = MMR_APPS_ComputeSpeed(0.0);
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

  if (clutchMessages++ > 10) {
    return MMR_AUTONOMOUS_SET_MANUAL_APPS;
  }

  return state;
}


static MmrAutonomousState setManualApps(MmrAutonomousState state) {
  static MmrDelay delay = { .ms = 5000 };

  if (MMR_DELAY_WaitAsync(&delay)) {
    return MMR_AUTONOMOUS_DONE;
  }

  return state;
}


static MmrAutonomousState done(MmrAutonomousState state) {
  static const uint16_t apps_MIN = 650;

  *__apps = MMR_AS_GetLap() >= 1
    ? MMR_APPS_ComputeSpeed(MMR_AS_GetInfoSpeed())
    : APPS_MIN;

  return MMR_AUTONOMOUS_DONE;
}
