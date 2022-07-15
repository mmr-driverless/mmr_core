#include "inc/back.h"
#include "inc/as.h"
#include "inc/global_state.h"
#include "inc/apps.h"
#include "inc/mock.h"
#include "inc/tps.h"
#include "inc/hwtest.h"
#include "../net/inc/net.h"

MmrAsPeripherals asp;

void MMR_BACK_Init(
  MmrCan *can,
  MmrPin *gearUp,
  MmrPin *gearDown,
  MmrPin *gearN,

  MmrButton *blueButton,

  MmrPin *ebsAsDrivingMode,
  MmrPin *ebsAsCloseSdc,
  MmrPin *ebsAsSdcIsReady,

  uint32_t *appsOut,
  uint32_t *appsIn,

  BuzzerPlay buzzerPlay,
  BuzzerStop buzzerStop,

  MmrPin *ebs1,
  MmrPin *ebs2,

  MmrLed *blueAxisLed,
  MmrLed *yellowAxisLed,

  MmrLed *ebsErrorLed,
  MmrLed *asmsErrorLed,

  MmrLed *ctrLed1,
  MmrLed *ctrLed2,
  MmrLed *ctrLed3,

  WatchdogStart watchdogStart,
  WatchdogStop watchdogStop,

  MmrPin *asms
) {
  asp = (MmrAsPeripherals) {
    .can = can,
    .gearN = gearN,
    .gearUp = gearUp,
    .gearDown = gearDown,

    .blueButton = blueButton,

    .ebsAsDrivingMode = ebsAsDrivingMode,
    .ebsAsCloseSdc = ebsAsCloseSdc,
    .ebsAsSdcIsReady = ebsAsSdcIsReady,

    .appsOut = appsOut,
    .appsIn = appsIn,

    .buzzerPlay = buzzerPlay,
    .buzzerStop = buzzerStop,

    .ebs1 = ebs1,
    .ebs2 = ebs2,

    .blueAxisLed = blueAxisLed,
    .yellowAxisLed = yellowAxisLed,

    .ebsErrorLed = ebsErrorLed,
    .asmsErrorLed = asmsErrorLed,

    .ctrLed1 = ctrLed1,
    .ctrLed2 = ctrLed2,
    .ctrLed3 = ctrLed3,

    .watchdogStart = watchdogStart,
    .watchdogStop = watchdogStop,

    .asms = asms,
  };
}


void MMR_BACK_Run() {
  static int brakeDisableCount = 0;
  static bool isAutonomous;
  
  MMR_GS_UpdateFromCan(asp.can);

  if (gs.ebsCheckState != EBS_CHECK_ERROR && gs.ebsCheckState != EBS_CHECK_READY) {
    MMR_EBS_SetDrivingMode(MMR_EBS_CHECK_DRIVING_MODE_AUTONOMOUS);
    gs.ebsCheckState = MMR_EBS_Check(gs.ebsCheckState);
    return;
  }

  if (gs.ebsCheckState == EBS_CHECK_ERROR) {
    return;
  }

  MMR_EBS_Arm();
  MMR_AS_Run();

//
//  if (gs.currentMission == MMR_MISSION_MANUAL) {
//    MMR_EBS_SetDrivingMode(MMR_EBS_CHECK_DRIVING_MODE_MANUAL);
//
//    MMR_MISSION_Run(gs.currentMission);
//  }
//
//
//  isAutonomous =
//    gs.currentMission != MMR_MISSION_IDLE &&
//    gs.currentMission != MMR_MISSION_MANUAL;
//
//  if (isAutonomous) {
//    MMR_EBS_SetDrivingMode(MMR_EBS_CHECK_DRIVING_MODE_AUTONOMOUS);
//
//    if (gs.ebsCheckState != EBS_CHECK_ERROR && gs.ebsCheckState != EBS_CHECK_READY) {
//      gs.ebsCheckState = MMR_EBS_Check(gs.ebsCheckState);
//      return;
//    }
//
//    if (gs.ebsCheckState == EBS_CHECK_ERROR) {
//      return;
//    }
//
//    if (gs.stateAs < MMR_AS_DRIVING) {
//      MMR_NET_EngageBrakeAsync(asp.can, 0.7);
//    }
//    else {
//      if (brakeDisableCount++ < 3) {
//        MMR_NET_EngageBrakeAsync(asp.can, 0.0);
//      }
//    }
//
//    MMR_AS_Run();
//  }
}
