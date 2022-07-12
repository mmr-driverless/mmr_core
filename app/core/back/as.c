#include "inc/as.h"
#include "inc/global_state.h"
#include "inc/gear_change.h"
#include "inc/autonomous_launch.h"
#include "inc/manual_launch.h"
#include "inc/apps.h"
#include "inc/peripherals.h"
#include "inc/ebs.h"
#include "inc/axis_leds.h"
#include <button.h>
#include <buffer.h>
#include <delay.h>
#include <can.h>
#include <led.h>
#include <stdint.h>
#include <stdbool.h>

#include "../net/inc/net.h"

//static MmrEbsCheck ebs = EBS_IDLE;
static MmrAsState stateAs = MMR_AS_OFF;
static bool waitingMissionReady = false;

static MmrAsState computeState();
static void off();
static void ready();
static void driving();
static void emergency();
static void finished();

void MMR_AS_Run() {
  static MmrDelay sendDelay = { .ms = 15 };

  MMR_GS_UpdateFromCan(asp.can);

  if (MMR_APPS_Check(asp.appsIn[0], asp.appsIn[1])) {
    return MMR_SYSTEM_NOT_HEALTHY;
  }

  // Handbook: https://bit.ly/3bRd49t
  stateAs = computeState();
  gs.stateAs = stateAs;
  MMR_AXIS_LEDS_Run(stateAs);

  switch (stateAs) {
  case MMR_AS_OFF: off(); break;
  case MMR_AS_READY: ready(); break;
  case MMR_AS_DRIVING: driving(); break;
  case MMR_AS_EMERGENCY: emergency(); break;
  case MMR_AS_FINISHED: finished(); break;
  }
  

  if (MMR_DELAY_WaitAsync(&sendDelay)) {
    MmrCanHeader header = MMR_CAN_NormalHeader(MMR_CAN_MESSAGE_ID_AS_STATE);
    MmrCanMessage message = MMR_CAN_OutMessage(header);
    MMR_CAN_MESSAGE_SetPayload(&message, (uint8_t*)&stateAs, sizeof(stateAs));
    MMR_CAN_Send(asp.can, &message);

    if (waitingMissionReady) {
      MmrCanHeader header = MMR_CAN_NormalHeader(MMR_CAN_MESSAGE_ID_AS_R2D);
      MmrCanMessage message = MMR_CAN_OutMessage(header);
      MMR_CAN_Send(asp.can, &message);
    }
  }
}

static MmrAsState computeState() {
  if (gs.ebsState == MMR_EBS_ACTIVATED) {
    if (gs.missionFinished && gs.vehicleStandstill) {
      return MMR_AS_FINISHED;
    }
        
    return MMR_AS_EMERGENCY;
  }

  bool canDrive =
    gs.currentMission != MMR_MISSION_IDLE &&
    gs.currentMission != MMR_MISSION_INSPECTION &&
    gs.currentMission != MMR_MISSION_MANUAL &&
    gs.asbCheck
    /*TS_Activation()(**/;

  if (canDrive) {
    if (gs.readyToDrive)
      return MMR_AS_DRIVING;

    if (gs.asbEngaged)
      return MMR_AS_READY;
  }

  return MMR_AS_OFF;
}


void off() {
  static MmrDelay brakeRequestDelay = { .ms = 20 };
  if (MMR_DELAY_WaitAsync(&brakeRequestDelay)) {
    MMR_NET_BrakeCheckRequest();
//    MMR_NET_IsBrakeEngaged();
  }
}

void ready() {
  static MmrDelay readyToDriveDelay = { .ms = 5000 };

  if (MMR_DELAY_WaitOnceAsync(&readyToDriveDelay) && gs.resGoButton == MMR_BUTTON_PRESSED) {
    waitingMissionReady = true;
    if (gs.missionReady) {
      gs.readyToDrive = true;
    }
  }
}

void driving() {
  MMR_MISSION_Run(gs.currentMission);
}

static void emergency() {
  static MmrDelay timeout = { .ms = 10000 };

  if (MMR_DELAY_WaitOnceAsync(&timeout)) {
    asp.buzzerStop();
  }
  else {
    asp.buzzerPlay();
  }
}

static void finished() {
}
