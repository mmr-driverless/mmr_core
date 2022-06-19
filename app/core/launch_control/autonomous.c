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
static MmrAutonomousState setLaunchControl(MmrAutonomousState state);
static MmrAutonomousState changeGear(MmrAutonomousState state);
static MmrAutonomousState accelerate(MmrAutonomousState state);
static MmrAutonomousState releaseClutch(MmrAutonomousState state);
static MmrAutonomousState start(MmrAutonomousState state);
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
  case MMR_AUTONOMOUS_SET_LAUNCH_CONTROL: return setLaunchControl(state);
  case MMR_AUTONOMOUS_CHANGE_GEAR: return changeGear(state);
  case MMR_AUTONOMOUS_ACCELERATE: return accelerate(state);
  case MMR_AUTONOMOUS_RELEASE_CLUTCH: return releaseClutch(state);
  case MMR_AUTONOMOUS_START: return start(state);
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

  bool canPullClutch = MMR_LAUNCH_CONTROL_GetRpm() >= 1000;
  bool isClutchPulled = MMR_LAUNCH_CONTROL_GetClutchState() == MMR_CLUTCH_PULLED;

  if (canPullClutch && MMR_DELAY_WaitAsync(&delay)) {
    MMR_CAN_Send(__can, &clutchPullMsg);
  }

  if (isClutchPulled) {
    return MMR_AUTONOMOUS_SET_LAUNCH_CONTROL;
  }

  return state;
}


static MmrAutonomousState setLaunchControl(MmrAutonomousState state) {
  static MmrDelay delay = { .ms = 25};
  static MmrCanBuffer buffer = { 0x8 };

  MmrCanHeader header = MMR_CAN_HEADER_FromBits(0x628);
  MmrCanMessage setLaunchMsg = MMR_CAN_OutMessage(header);
  MMR_CAN_MESSAGE_SetStandardId(&setLaunchMsg, true);
  MMR_CAN_MESSAGE_SetPayload(&setLaunchMsg, buffer, 8);

  bool rpmOk = MMR_LAUNCH_CONTROL_GetRpm() >= 1000;
  bool launchSet = MMR_LAUNCH_CONTROL_GetLaunchControlState() == MMR_LAUNCH_CONTROL_SET;

  if (rpmOk && !launchSet && MMR_DELAY_WaitAsync(&delay)) {
    MMR_CAN_Send(__can, &setLaunchMsg);
  }

  if (launchSet) {
    return MMR_AUTONOMOUS_CHANGE_GEAR;
  }

  return state;
}

static MmrAutonomousState setGear(MmrAutonomousState state) {
  static MmrDelay delay = { .ms = 5 };

  bool isGearSet = MMR_LAUNCH_CONTROL_GetGear() == 1;
  if (isGearSet) {
    changeGear(true);
    MMR_PIN_Write(__gearDown, MMR_PIN_LOW);
   //GEAR_CHANGED;
   waitMs(true, 1000);
  }

  if (!isGearSet || !changeGear(false)) {
    waitMs(true, 2000);
    // GEAR_NOT_CHANGED;
    if (waitMs(false, 2000)) {
      MMR_PIN_Write(__gearDown, MMR_PIN_LOW);
      // GEAR_NOT_SET;
    }
    return;
  }

  return  MMR_AUTONOMOUS_ACCELERATE;
}

static MmrAutonomousState accelerate(MmrAutonomousState state) {
  const MmrDelay delay = { .ms = 1000 };
  const uint16_t DAC_30 = 865;

  if (MMR_DELAY_WaitAsync(&delay)) {
    __apps = DAC_30;
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
    return MMR_AUTONOMOUS_START;
  }

  return state;
}


static MmrAutonomousState start(MmrAutonomousState state){
  static MmrDelay delay = { .ms = 25};
  static MmrCanBuffer buffer = { 0x8 };

  MmrCanHeader header = MMR_CAN_HEADER_FromBits(0x628);
  MmrCanMessage unsetLaunchMsg = MMR_CAN_OutMessage(header);
  MMR_CAN_MESSAGE_SetStandardId(&unsetLaunchMsg, true);
  MMR_CAN_MESSAGE_SetPayload(&unsetLaunchMsg, buffer, 8);

  bool rpmOk = MMR_LAUNCH_CONTROL_GetRpm() >= 1000;
  bool launchSet = MMR_LAUNCH_CONTROL_GetLaunchControlState() == MMR_LAUNCH_CONTROL_SET;

  if (rpmOk && !launchSet && MMR_DELAY_WaitAsync(&delay)) {
    MMR_CAN_Send(__can, &unsetLaunchMsg);
  }

  if (launchSet) {
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
