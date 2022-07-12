#include "../net/inc/net.h"
#include "inc/global_state.h"
#include "inc/gear_change.h"
#include "inc/autonomous_launch.h"
#include "inc/manual_launch.h"
#include "inc/as.h"
#include "inc/apps.h"
#include "inc/peripherals.h"
#include "inc/ebs.h"
#include "inc/axis_leds.h"
#include <button.h>
#include <buffer.h>
#include <delay.h>
#include <can.h>
#include <stdint.h>
#include <stdbool.h>

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
  MMR_NET_BRAKE_Init(asp.can);

  // Handbook: https://bit.ly/3bRd49t
  stateAs = computeState();
  gs.stateAs = stateAs;
  MMR_AXIS_LEDS_Run(stateAs);

  switch (stateAs) {
  case MMR_AS_OFF: off();
  case MMR_AS_READY: ready();
  case MMR_AS_DRIVING: driving();
  case MMR_AS_EMERGENCY: emergency();
  case MMR_AS_FINISHED: finished();
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
 MMR_NET_BrakeCheckRequest();

  // MMR_NET_IsBrakeEngaged();
  // ebs = ebsCheck(ebs);
  // if (ebs == EBS_ERROR)
  //   return;

  // if (ebs != EBS_OK)
  //   return;
  
  if (gs.ebsState == EBS_STATE_ACTIVATED) {
    if (gs.missionFinished && gs.vehicleStandstill)
      return MMR_AS_FINISHED;
        
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
  return;
}

void ready() {
  static MmrDelay readyToDriveDelay = { .ms = 5000 };
  if (MMR_DELAY_WaitAsync(&readyToDriveDelay) && gs.goSignal) {
    waitingMissionReady = true;
    if (gs.missionReady)
      gs.readyToDrive = true;
  }
}

void driving() {
  MMR_MISSION_Run(gs.currentMission);
}
