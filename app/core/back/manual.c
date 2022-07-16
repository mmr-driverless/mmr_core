#include "inc/manual.h"
#include "inc/peripherals.h"
#include "inc/global_state.h"
#include "inc/mission.h"
#include "inc/ebs.h"
#include <pin.h>
#include <delay.h>

typedef enum {
  WaitingTs,
  WaitSdcReady,
  EbsDisarm1,
  EbsArm,
  EbsDisarm2,
  EbsDisarmDone,
} MmrManualEbsDisarmState;

static MmrManualState ManualInit();
static MmrManualState ManualRunning();

MmrManualState MMR_MANUAL_Run(MmrManualState state) {
  switch (state) {
  case MMR_MANUAL_INIT: return ManualInit();
  case MMR_MANUAL_RUNNING: return ManualRunning();
  }
}

static MmrManualState ManualInit() {
  static MmrManualEbsDisarmState disarmState = WaitingTs;
  static MmrDelay delay = { .ms = 2000 };
  static bool watchdogStarted = false;
  bool isDelayWaited = MMR_DELAY_WaitAsync(&delay);
  // bool isDelayWaited = true;

  MMR_EBS_SetDrivingMode(MMR_EBS_DRIVING_MODE_AUTONOMOUS);
  if (!watchdogStarted && asp.watchdogStart()) {
    watchdogStarted = true;
  }

  if (gs.rpm >= 10 && gs.gear == 0) {
    disarmState = WaitSdcReady;
  }

  if (disarmState == WaitSdcReady && MMR_PIN_Read(asp.ebsAsSdcIsReady) == MMR_PIN_HIGH) {
    disarmState = EbsDisarm1;
  }

  if (disarmState == EbsDisarm1) {
    MMR_EBS_Off();
    MMR_DELAY_Reset(&delay);
    disarmState = EbsArm;
  }
  else if (isDelayWaited && disarmState == EbsArm) {
    MMR_EBS_Arm();
    MMR_DELAY_Reset(&delay);
    disarmState = EbsDisarm2;
  }
  else if (isDelayWaited && disarmState == EbsDisarm2) {
    MMR_EBS_Off();
    MMR_DELAY_Reset(&delay);
    disarmState = EbsDisarmDone;
  }


  if (disarmState == EbsDisarmDone) {
    return MMR_MANUAL_RUNNING;
  }
  else {
    return MMR_MANUAL_INIT;
  }
}

static MmrManualState ManualRunning() {
  MMR_MISSION_Run(gs.currentMission);
  return MMR_MANUAL_RUNNING;
}
