#include "inc/back.h"
#include "inc/as.h"
#include "inc/global_state.h"

MmrAsPeripherals asp;

void MMR_BACK_Init(
  MmrCan *can,
  MmrPin *gearUp,
  MmrPin *gearDown,
  MmrPin *gearN,

  MmrButton *blueButton,

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
  MMR_GS_UpdateFromCan(asp.can);
  
  if (gs.currentMission == MMR_MISSION_MANUAL) {
    MMR_EBS_SetDrivingMode(MMR_EBS_CHECK_DRIVING_MODE_MANUAL);
  }

  if (gs.currentMission != MMR_MISSION_IDLE && gs.currentMission != MMR_MISSION_MANUAL) {
    MMR_EBS_SetDrivingMode(MMR_EBS_CHECK_DRIVING_MODE_AUTONOMOUS);
    engageBreak();

    if (gs.ebsCheckState != EBS_CHECK_ERROR && gs.ebsCheckState != EBS_CHECK_READY) {
      gs.ebsCheckState = MMR_EBS_Check(gs.ebsCheckState);
    }
  }


  if (!MMR_APPS_Check(asp.appsIn[0], asp.appsIn[1])) {
    // TODO apps error
  }

  if (!MMR_TPS_Check(gs.ath, gs.ath2)) {
    // TODO tps check
  }

  MMR_AS_Run();
}