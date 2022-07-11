#include "inc/global_state.h"
#include "inc/gear_change.h"
#include "inc/autonomous_launch.h"
#include "inc/manual_launch.h"
#include "inc/as.h"
#include "inc/apps.h"
#include "inc/peripherals.h"
#include <button.h>
#include <buffer.h>
#include <delay.h>
#include <can.h>
#include <ebs.h>
#include <stdint.h>
#include <stdbool.h>

static MmrEbsCheck ebs = EBS_IDLE; 
static MmrAsState stateAs = MMR_AS_OFF;

static MmrAsState computeState();
static void idle();
static void off();
static void ready();
static void driving();
static void emergency();
static void finished();

void MMR_AS_Run() {
  MMR_GS_UpdateFromCan(asp.can);
  
  // Handbook: https://bit.ly/3bRd49t
  stateAs = computeState();
  gs.stateAs = stateAs;
  switch (stateAs) {
    case MMR_AS_OFF: off();
    case MMR_AS_READY: ready();
    case MMR_AS_DRIVING: driving();
    case MMR_AS_EMERGENCY: emergency();
    case MMR_AS_FINISHED: finished();
  }
 
}

static MmrAsState computeState() {
  // ebs = ebsCheck(ebs);
  // if (ebs == EBS_ERROR)
  //   return;

  // if (ebs != EBS_OK)
  //   return;
  
  if (MMR_AS_GetEbsState() == EBS_STATE_ACTIVATED) {
    if (gs.missionFinished && gs.vehicleStandstill)
      return MMR_AS_FINISHED;

    return MMR_AS_EMERGENCY;
  }

  if (gs.currentMission != MMR_MISSION_IDLE && gs.currentMission != MMR_MISSION_INSPECTION &&
    gs.currentMission != MMR_MISSION_MANUAL && gs.asbCheck && TS_Activation()) {
    
    if (gs.readyToDrive) {
      MmrCanHeader header = MMR_CAN_NormalHeader(MMR_CAN_MESSAGE_ID_AS_R2D);
      MmrCanMessage message = MMR_CAN_OutMessage(header);
      MMR_CAN_Send(asp.can, &message);
      return MMR_AS_DRIVING;
    }

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
  if (gs.goSignal && MMR_DELAY_WaitAsync(&readyToDriveDelay))
    gs.readyToDrive = true;
}

void driving() {
  if (gs.missionReady)
    MMR_MISSION_Run(gs.currentMission);
}

void emergency() {

}

void finished() {
  
}