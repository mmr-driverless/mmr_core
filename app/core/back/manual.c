#include "inc/manual.h"
#include "inc/peripherals.h"
#include "inc/global_state.h"
#include "inc/mission.h"
#include "inc/ebs.h"
#include <pin.h>
#include <delay.h>

static MmrManualState ManualEntryPoint();
static MmrManualState ManualWaitTs();
static MmrManualState ManualWaitSdc();
static MmrManualState ManualEbsOff();
static MmrManualState ManualRunning();

MmrManualState MMR_MANUAL_Run(MmrManualState state) {
  switch (state) {
  case MMR_MANUAL_ENTRY_POINT: return ManualEntryPoint();
  case MMR_MANUAL_WAIT_TS: return ManualWaitTs();
  case MMR_MANUAL_WAIT_SDC: return ManualWaitSdc();
  case MMR_MANUAL_EBS_OFF: return ManualEbsOff();
  case MMR_MANUAL_RUNNING: return ManualRunning();
  }
}

static MmrManualState ManualEntryPoint() {
  MMR_EBS_SetDrivingMode(MMR_EBS_DRIVING_MODE_MANUAL);
  if (!asp.watchdogStart()) {
    return MMR_MANUAL_RUNNING;
  }
  return MMR_MANUAL_WAIT_TS;
}

static MmrManualState ManualWaitTs() {
  if (gs.gear == 0) {
    return MMR_MANUAL_WAIT_SDC;
  }
  return MMR_MANUAL_WAIT_TS;
}

static MmrManualState ManualWaitSdc() {
  if (MMR_PIN_Read(asp.ebsAsSdcIsReady) == MMR_PIN_HIGH) {
    return MMR_MANUAL_EBS_OFF;
  }
  return MMR_MANUAL_WAIT_SDC;
}

static MmrManualState ManualEbsOff() {
  MMR_EBS_Off();
  return MMR_MANUAL_RUNNING;
}

static MmrManualState ManualRunning() {
  MMR_MISSION_Run(gs.currentMission);
  return MMR_MANUAL_RUNNING;
}
