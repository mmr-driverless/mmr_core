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
#include <EBS.h>
#include <stdint.h>
#include <stdbool.h>

static MmrEbsCheck ebs = EBS_IDLE;


static MmrAsState computeState();
static void idle();
static void off();
static void ready();
static void driving();
static void emergency();
static void finished();


void MMR_AS_Run() {
  MMR_GS_UpdateFromCan(asp.can);

  ebs = ebsCheck(ebs);
  if (ebs == EBS_ERROR)
    return;

  if (ebs != EBS_OK)
    return;

  // Handbook: https://bit.ly/3bRd49t
//  switch (computeState()) {
//  case MMR_AS_OFF: return off();
//  case MMR_AS_READY: return ready();
//  case MMR_AS_DRIVING: return driving();
//  case MMR_AS_EMERGENCY: return emergency();
//  case MMR_AS_FINISHED: return finished();
//  }
}


static MmrAsState computeState() {
//  if (MMR_EBS_Activated) {
//    if (MissionFinished && VehicleAtStandstill) {
//      return MMR_AS_FINISHED;
//    }
//
//    return MMR_AS_EMERGENCY;
//  }
//
//  if (MissionSelected && ASMSOn && ASBchecksOK && TS Active) {
//    if (R2D) {
//      return MMR_AS_DRIVING;
//    }
//
//    if (BrakesEngaged) {
//      return MMR_AS_READY;
//    }
//  }

  return MMR_AS_OFF;
}
