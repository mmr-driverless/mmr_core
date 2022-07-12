#include "inc/ebs.h"
#include "inc/peripherals.h"
#include "inc/global_state.h"

const uint8_t EBS_MIN_PRESSURE = 40;
const uint8_t BRAKE_MIN_PRESSURE = 30;


static bool checkEbsPressureOk();
static bool checkBrakesPressureOk();


void MMR_EBS_Arm() {
  MMR_PIN_Write(asp.ebsAsCloseSdc, MMR_PIN_HIGH);
}

void MMR_EBS_Disarm() {
  MMR_PIN_Write(asp.ebsAsCloseSdc, MMR_PIN_LOW);
}

void MMR_EBS_SetDrivingMode(MmrEbsDrivingMode mode) {
  MmrPinState out = mode == MMR_EBS_DRIVING_MODE_MANUAL
    ? MMR_PIN_HIGH
    : MMR_PIN_LOW;

  MMR_PIN_Write(asp.ebsAsDrivingMode, out);
}

bool MMR_EBS_SdcIsReady() {
  return MMR_PIN_Read(asp.ebsAsSdcIsReady) == MMR_PIN_HIGH;
}

static MmrEbsState startWatchdog(MmrEbsState state);
static MmrEbsState sdcWaitHigh(MmrEbsState state);
static MmrEbsState stopWatchdog(MmrEbsState state);
static MmrEbsState sdcWaitLow(MmrEbsState state);
static MmrEbsState retoggleWatchdog(MmrEbsState state);
static MmrEbsState ebsPressureOk(MmrEbsState state);
static MmrEbsState brakePressureOk(MmrEbsState state);
static MmrEbsState activateTs(MmrEbsState state);
static MmrEbsState waitTsActivation(MmrEbsState state);
static MmrEbsState disableActuator1(MmrEbsState state);
static MmrEbsState checkAct1BrakePressure(MmrEbsState state);
static MmrEbsState enableActuator1(MmrEbsState state);
static MmrEbsState disableActuator2(MmrEbsState state);
static MmrEbsState checkAct2BrakePressure(MmrEbsState state);
static MmrEbsState enableActuator2(MmrEbsState state);


MmrEbsState MMR_EBS_Check(MmrEbsState state) {
  switch (state) {
  case EBS_START_WATCHDOG: return startWatchdog(state);
  case EBS_SDC_WAIT_HIGH: return sdcWaitHigh(state);
  case EBS_STOP_WATCHDOG: return stopWatchdog(state);
  case EBS_SDC_WAIT_LOW: return sdcWaitLow(state);
  case EBS_RETOGGLE_WATCHDOG: return retoggleWatchdog(state);

  case EBS_EBS_PRESSURE_OK: return ebsPressureOk(state);
  case EBS_BRAKE_PRESSURE_OK: return brakePressureOk(state);

  case EBS_ACTIVATE_TS: return activateTs(state);
  case EBS_WAIT_TS_ACTIVATION: return waitTsActivation(state);

  case EBS_DISABLE_ACTUATOR_1: return disableActuator1(state);
  case EBS_CHECK_ACT_1_BRAKE_PRESSURE: return checkAct1BrakePressure(state);
  case EBS_ENABLE_ACTUATOR_1: return enableActuator1(state);

  case EBS_DISABLE_ACTUATOR_2: return disableActuator2(state);
  case EBS_CHECK_ACT_2_BRAKE_PRESSURE: return checkAct2BrakePressure(state);
  case EBS_ENABLE_ACTUATOR_2: return enableActuator2(state);
  
  case EBS_READY: return EBS_READY;
  case EBS_ERROR: return EBS_ERROR;
  }
}


static MmrEbsState startWatchdog(MmrEbsState state) {
  if (!asp.watchdogStart()) {
    return EBS_ERROR;
  }

  return EBS_SDC_WAIT_HIGH;
}

static MmrEbsState sdcWaitHigh(MmrEbsState state) {
  if (MMR_EBS_SdcIsReady()) {
    return EBS_STOP_WATCHDOG;
  }

  return state;
}

static MmrEbsState stopWatchdog(MmrEbsState state) {
  if (!asp.watchdogStop()) {
    return EBS_ERROR;
  }

  return EBS_SDC_WAIT_LOW;
}

static MmrEbsState sdcWaitLow(MmrEbsState state) {
  static MmrDelay timeout = { .ms = 1000 };

  if (!MMR_EBS_SdcIsReady()) {
    return EBS_RETOGGLE_WATCHDOG;
  }

  if (MMR_DELAY_WaitAsync(&timeout)) {
    return EBS_ERROR;
  }

  return state;
}

static MmrEbsState retoggleWatchdog(MmrEbsState state) {
  if (!asp.watchdogStart()) {
    return EBS_ERROR;
  }

  return EBS_EBS_PRESSURE_OK;
}

static MmrEbsState ebsPressureOk(MmrEbsState state) {
  static MmrDelay timeout = { .ms = 1000 };

  if (checkEbsPressureOk()) {
    return EBS_BRAKE_PRESSURE_OK;
  }

  if (MMR_DELAY_WaitAsync(&timeout)) {
    return EBS_ERROR;
  }

  return state;
}

static MmrEbsState brakePressureOk(MmrEbsState state) {
  static MmrDelay timeout = { .ms = 1000 };

  if (checkBrakesPressureOk()) {
    return EBS_ACTIVATE_TS;
  }

  if (MMR_DELAY_WaitAsync(&timeout)) {
    return EBS_ERROR;
  }

  return state;
}

static MmrEbsState activateTs(MmrEbsState state) {
  MMR_EBS_Arm();
  return EBS_WAIT_TS_ACTIVATION;
}

static MmrEbsState waitTsActivation(MmrEbsState state) {
  if (gs.gear == 0 && gs.rpm >= 1000) {
    return EBS_DISABLE_ACTUATOR_1;
  }

  return state;
}

static MmrEbsState disableActuator1(MmrEbsState state) {
  MMR_PIN_Write(asp.ebs1, MMR_PIN_HIGH);
  return EBS_CHECK_ACT_1_BRAKE_PRESSURE;
}

static MmrEbsState checkAct1BrakePressure(MmrEbsState state) {
  static MmrDelay timeout = { .ms = 1000 };

  if (checkBrakesPressureOk()) {
    return EBS_ENABLE_ACTUATOR_1;
  }

  if (MMR_DELAY_WaitAsync(&timeout)) {
    return EBS_ERROR;
  }

  return state;
}

static MmrEbsState enableActuator1(MmrEbsState state) {
  MMR_PIN_Write(asp.ebs1, MMR_PIN_LOW);
  return EBS_DISABLE_ACTUATOR_2;
}


static MmrEbsState disableActuator2(MmrEbsState state) {
  MMR_PIN_Write(asp.ebs2, MMR_PIN_HIGH);
  return EBS_CHECK_ACT_2_BRAKE_PRESSURE;
}

static MmrEbsState checkAct2BrakePressure(MmrEbsState state) {
  static MmrDelay timeout = { .ms = 1000 };

  if (checkBrakePressureOk()) {
    return EBS_ENABLE_ACTUATOR_2;
  }

  if (MMR_DELAY_WaitAsync(&timeout)) {
    return EBS_ERROR;
  }

  return state;
}

static MmrEbsState enableActuator2(MmrEbsState state) {
  MMR_PIN_Write(asp.ebs2, MMR_PIN_LOW);
  return EBS_READY;
}


static bool checkEbsPressureOk() {
  return
    gs.ebs1Pressure >= EBS_MIN_PRESSURE &&
    gs.ebs2Pressure >= EBS_MIN_PRESSURE;
}

static bool checkBrakesPressureOk() {
  return
    gs.brakePressureFront >= BRAKE_MIN_PRESSURE &&
    gs.brakePressureRear >= BRAKE_MIN_PRESSURE;
}
