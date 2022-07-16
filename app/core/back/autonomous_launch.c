#include "inc/autonomous_launch.h"
#include "inc/global_state.h"
#include "inc/peripherals.h"
#include "inc/apps.h"
#include "message_id.h"
#include "../net/inc/net.h"
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
static MmrAutonomousState done(MmrAutonomousState state);



MmrAutonomousState MMR_AUTONOMOUS_LAUNCH_Run(MmrAutonomousState state) {
  switch (state) {
  case MMR_AUTONOMOUS_LAUNCH_WAITING: return waiting(state);
  case MMR_AUTONOMOUS_LAUNCH_PULL_CLUTCH: return pullClutch(state);
  case MMR_AUTONOMOUS_LAUNCH_WAIT_BEFORE_CHANGING_GEAR: return waitBeforeChangingGear(state);

  case MMR_AUTONOMOUS_LAUNCH_CHANGE_GEAR: return changeGear(state);
  case MMR_AUTONOMOUS_LAUNCH_SET_LAUNCH_CONTROL: return setLaunchControl(state);
  case MMR_AUTONOMOUS_LAUNCH_WAIT_BEFORE_ACCELERATING: return waitBeforeAccelerating(state);

  case MMR_AUTONOMOUS_LAUNCH_ACCELERATE: return accelerate(state);
  case MMR_AUTONOMOUS_LAUNCH_RELEASE_CLUTCH: return releaseClutch(state);
  case MMR_AUTONOMOUS_LAUNCH_UNSET_LAUNCH: return unsetLaunchControl(state);
  case MMR_AUTONOMOUS_LAUNCH_ACCELERATE_TO_15: return accelerateTo15(state);
  case MMR_AUTONOMOUS_LAUNCH_ACCELERATE_TO_MINIMUM: return accelerateToMinimum(state);

  case MMR_AUTONOMOUS_LAUNCH_CLUTCH_SET_MANUAL: return clutchSetManual(state);
  case MMR_AUTONOMOUS_LAUNCH_DONE: return done(state);
  default: return state;
  }
}


static MmrAutonomousState waiting(MmrAutonomousState state) {
  return MMR_AUTONOMOUS_LAUNCH_PULL_CLUTCH;
}


static MmrAutonomousState pullClutch(MmrAutonomousState state) {
  if (MMR_NET_ClutchPullAsync(asp.can)) {
    return MMR_AUTONOMOUS_LAUNCH_WAIT_BEFORE_CHANGING_GEAR;
  }

  return state;
}


static MmrAutonomousState waitBeforeChangingGear(MmrAutonomousState state) {
  static MmrDelay delay = { .ms = 1000 };

  if (MMR_DELAY_WaitAsync(&delay)) {
    return MMR_AUTONOMOUS_LAUNCH_CHANGE_GEAR;
  }

  return state;
}


static MmrAutonomousState changeGear(MmrAutonomousState state) {
  static MmrDelay gearNotChangedDelay = { .ms = 2000 };
  static MmrDelay gearChangingDelay = { .ms = 350 };
  static bool changingGear = false;

  bool isGearSet = gs.gear == 1;
  if (isGearSet && !changingGear) {
    MMR_PIN_Write(asp.gearN, MMR_PIN_LOW);
    return MMR_AUTONOMOUS_LAUNCH_SET_LAUNCH_CONTROL;
  }

  if (!changingGear && !isGearSet && MMR_DELAY_WaitAsync(&gearNotChangedDelay)) {
    MMR_DELAY_Reset(&gearChangingDelay);
    changingGear = true;
  }

  if (!MMR_DELAY_WaitAsync(&gearChangingDelay)) {
    MMR_PIN_Write(asp.gearN, MMR_PIN_HIGH);
  }
  else {
    MMR_PIN_Write(asp.gearN, MMR_PIN_LOW);
    MMR_DELAY_Reset(&gearNotChangedDelay);
    changingGear = false;
  }

  return state;
}



static MmrAutonomousState setLaunchControl(MmrAutonomousState state) {
  static MmrDelay timeout = { .ms = 500 };

  if (MMR_DELAY_WaitAsync(&timeout) || MMR_NET_LaunchControlSetAsync(asp.can)) {
    return MMR_AUTONOMOUS_LAUNCH_WAIT_BEFORE_ACCELERATING;
  }

  return state;
}


static MmrAutonomousState waitBeforeAccelerating(MmrAutonomousState state) {
  static MmrDelay delay = { .ms = 1000 };

  if (MMR_DELAY_WaitAsync(&delay)) {
    return MMR_AUTONOMOUS_LAUNCH_ACCELERATE;
  }

  return state;
}


static MmrAutonomousState accelerate(MmrAutonomousState state) {
  static MmrDelay delay = { .ms = 1000 };

  MMR_APPS_TryWrite(MMR_APPS_ComputeSpeed(0.3));
  if (MMR_DELAY_WaitAsync(&delay)) {
    return MMR_AUTONOMOUS_LAUNCH_RELEASE_CLUTCH;
  }

  return state;
}

static MmrAutonomousState releaseClutch(MmrAutonomousState state) {
  if (MMR_NET_ClutchReleaseAsync(asp.can)) {
    return MMR_AUTONOMOUS_LAUNCH_UNSET_LAUNCH;
  }

  return state;
}


static MmrAutonomousState unsetLaunchControl(MmrAutonomousState state){
  static MmrDelay timeout = { .ms = 500 };

  if (MMR_DELAY_WaitAsync(&timeout) || MMR_NET_LaunchControlUnsetAsync(asp.can)) {
    return MMR_AUTONOMOUS_LAUNCH_ACCELERATE_TO_15;
  }

  return state;
}


static MmrAutonomousState accelerateTo15(MmrAutonomousState state) {
  static MmrDelay delay = { .ms = 500 };

  if (MMR_DELAY_WaitAsync(&delay)) {
    MMR_APPS_TryWrite(MMR_APPS_ComputeSpeed(0.30));
    return MMR_AUTONOMOUS_LAUNCH_ACCELERATE_TO_MINIMUM;
  }

  return state;
}


static MmrAutonomousState accelerateToMinimum(MmrAutonomousState state) {
  static MmrDelay delay = { .ms = 500 };

  if (MMR_DELAY_WaitAsync(&delay)) {
    uint32_t out = gs.currentMission == MMR_MISSION_INSPECTION
      ? MMR_APPS_ComputeSpeed(0.30)
      : MMR_APPS_ComputeSpeed(0.30);

    MMR_APPS_TryWrite(out);
    return MMR_AUTONOMOUS_LAUNCH_CLUTCH_SET_MANUAL;
  }

  return state;
}


static MmrAutonomousState clutchSetManual(MmrAutonomousState state) {
  static MmrDelay delay = { .ms = 10 };
  static uint8_t clutchMessages = 0;

  MmrCanHeader header = MMR_CAN_ScsHeader(MMR_CAN_MESSAGE_ID_CS_CLUTCH_SET_MANUAL);
  MmrCanMessage clutchManualMsg = MMR_CAN_OutMessage(header);

  if (clutchMessages <= 10 && MMR_DELAY_WaitAsync(&delay)) {
    MMR_CAN_Send(asp.can, &clutchManualMsg);
  }

  if (clutchMessages++ > 10) {
    return MMR_AUTONOMOUS_LAUNCH_DONE;
  }

  return state;
}

static MmrAutonomousState done(MmrAutonomousState state) {
  return MMR_AUTONOMOUS_LAUNCH_DONE;
}
