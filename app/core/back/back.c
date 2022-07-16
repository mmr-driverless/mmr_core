#include "inc/back.h"
#include "inc/as.h"
#include "inc/global_state.h"
#include "inc/apps.h"
#include "inc/mock.h"
#include "inc/tps.h"
#include "inc/hwtest.h"
#include "inc/manual.h"
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

  MMR_GS_UpdateFromCan(asp.can);

  if (gs.currentMission == MMR_MISSION_MANUAL) {
    gs.ms = MMR_MANUAL_Run(gs.ms);
  }


  bool isAutonomous =
    gs.currentMission != MMR_MISSION_IDLE &&
    gs.currentMission != MMR_MISSION_MANUAL;

  if (isAutonomous) {
    MMR_EBS_SetDrivingMode(MMR_EBS_DRIVING_MODE_AUTONOMOUS);

    bool asmsOk = MMR_PIN_Read(asp.asms) == MMR_PIN_HIGH;
    if (asmsOk && gs.ebsCheckState != EBS_CHECK_ERROR && gs.ebsCheckState != EBS_CHECK_READY) {
      if (gs.ebsCheckState == EBS_CHECK_ACTIVATE_TS) {
        MMR_APPS_TryWrite(MMR_APPS_ComputeSpeed(0.3));
      }

      if (gs.ebsCheckState == EBS_CHECK_DISABLE_ACTUATOR_1) {
        MMR_APPS_TryWrite(MMR_APPS_ComputeSpeed(0.0));
      }

      gs.ebsCheckState = MMR_EBS_Check(gs.ebsCheckState);
      return;
    }

    if (gs.ebsCheckState == EBS_CHECK_ERROR) {
      return;
    }

    if (gs.stateAs < MMR_AS_DRIVING) {
      MMR_NET_EngageBrakeAsync(asp.can, 0.7);
    }
    else {
      if (brakeDisableCount < 3 && MMR_NET_EngageBrakeAsync(asp.can, 0.0)) {
        brakeDisableCount++;
      }
    }

    bool isEbsActivated =
        gs.ebsCheckState == EBS_CHECK_READY &&
        (gs.resEmergencyButton == MMR_BUTTON_PRESSED ||
        asmsOk == false);

    if (isEbsActivated) {
      gs.ebsState = MMR_EBS_ACTIVATED;
    }

    MMR_AS_Run();
  }
}
