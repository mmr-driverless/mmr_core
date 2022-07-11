#include "inc/global_state.h"
#include "inc/gear_change.h"
#include "inc/autonomous_launch.h"
#include "inc/manual_launch.h"
#include "inc/as.h"
#include "inc/apps.h"
#include "inc/peripherals.h"

#include "../brake/inc/brake.h"

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

MmrAsState MMR_AS_Get_State() {
  return stateAs;
}

void MMR_AS_Run() {
  MMR_GS_UpdateFromCan(asp.can);

  ebs = ebsCheck(ebs);
  if (ebs == EBS_ERROR)
    return;

  if (ebs != EBS_OK)
    return;

  // ebs == EBS_OK
  MmrEbsState ebsState = MMR_AS_GetEbsState();
  if (ebsState != EBS_STATE_ACTIVATED)
    

  // Handbook: https://bit.ly/3bRd49t
 stateAs = computeState();
 switch (stateAs) {
 case MMR_AS_OFF: off();
 case MMR_AS_READY: ready();
 case MMR_AS_DRIVING: driving();
 case MMR_AS_EMERGENCY: emergency();
 case MMR_AS_FINISHED: finished();
 }
 
}


static MmrAsState computeState() {
 bool VehicleAtStandstill = true;  // TODO: do real checks
 bool TsActive = true;
 bool R2D = true;

 if (MMR_AS_GetEbsState() == EBS_STATE_ACTIVATED) {
   if (gs.missionFinished == ~0x00 && VehicleAtStandstill) {
     return MMR_AS_FINISHED;
   }

   return MMR_AS_EMERGENCY;
 }

 if (gs.currentMission != MMR_MISSION_IDLE &&
  gs.currentMission != MMR_MISSION_MANUAL &&
  // gs.asbCheck == ~0x00 &&  // La ECU conosce solo la pressione dell'asb
  MMR_AS_GetEbsState() == EBS_STATE_ARMED &&
  TsActive) {

   if (R2D) {
     return MMR_AS_DRIVING;
   }

   if (MMR_BRAKE_IsEngaged(gs.brakePf, gs.brakePr)) {
     return MMR_AS_READY;
   }
 }

  return MMR_AS_OFF;

}


void off() {
  return;
}

void ready() {

}

void driving() {

}

void emergency() {

}

void finished() {

}